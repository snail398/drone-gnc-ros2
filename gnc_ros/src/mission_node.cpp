// Flies an offboard waypoint mission on PX4: arm, take off, visit waypoints, land.
// All decisions live in gnc::mission::MissionStateMachine; this node only
// translates PX4 messages into VehicleState and the state machine output back
// into PX4 messages.
//
// Two ways to follow the mission's position setpoints (parameter `controller`):
//   px4 - send the position to PX4 and let its own position controller fly it;
//   own - run gnc::control::PositionController here and send PX4 an attitude
//         and thrust setpoint, so only PX4's attitude and rate loops remain.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnc_core/control/position_controller.hpp"
#include "gnc_core/guidance/reference_generator.hpp"
#include "gnc_core/mission/mission_state_machine.hpp"

using namespace std::chrono_literals;
using gnc::control::PositionController;
using gnc::control::PositionControllerConfig;
using gnc::guidance::ReferenceGenerator;
using gnc::guidance::ReferenceLimits;
using gnc::mission::MissionConfig;
using gnc::mission::MissionStateMachine;
using gnc::mission::Output;
using gnc::mission::Phase;
using gnc::mission::VehicleState;
using gnc::mission::Waypoint;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// PX4 custom main mode for OFFBOARD, used with VEHICLE_CMD_DO_SET_MODE.
constexpr float kPx4CustomMainModeOffboard = 6.0F;

enum class ControllerType : std::uint8_t { kPx4, kOwn };

}  // namespace

class MissionNode : public rclcpp::Node {
 public:
  MissionNode()
      : Node("mission_node"),
        mission_(load_mission_config()),
        controller_type_(load_controller_type()),
        position_controller_(load_controller_config()),
        shaping_(get_parameter("trajectory_shaping").as_bool()),
        reference_generator_(load_reference_limits()) {
    const auto px4_qos = rclcpp::SensorDataQoS();
    status_sub_ = create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status_v1", px4_qos, [this](const VehicleStatus& msg) {
          vehicle_.armed = msg.arming_state == VehicleStatus::ARMING_STATE_ARMED;
          vehicle_.offboard_active = msg.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          vehicle_.preflight_checks_pass = msg.pre_flight_checks_pass;
          have_status_ = true;
        });
    position_sub_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", px4_qos,
        [this](const VehicleLocalPosition& msg) { on_position(msg); });
    land_sub_ = create_subscription<VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", px4_qos,
        [this](const VehicleLandDetected& msg) { vehicle_.landed = msg.landed; });

    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    attitude_pub_ =
        create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // 20 Hz: comfortably above the 2 Hz PX4 requires to stay in offboard mode.
    timer_ = create_wall_timer(50ms, [this] { on_timer(); });
  }

 private:
  MissionConfig load_mission_config() {
    MissionConfig config;
    config.takeoff_altitude = declare_parameter("takeoff_altitude", config.takeoff_altitude);
    config.acceptance_radius = declare_parameter("acceptance_radius", config.acceptance_radius);
    config.hold_time = declare_parameter("hold_time", config.hold_time);

    // Flat list of (north, east, altitude) triples, metres relative to take-off.
    const auto flat = declare_parameter("waypoints", std::vector<double>{});
    if (flat.size() % 3 != 0) {
      throw std::invalid_argument("waypoints must be a list of (north, east, altitude) triples");
    }
    for (std::size_t i = 0; i < flat.size(); i += 3) {
      config.waypoints.push_back(Waypoint{flat[i], flat[i + 1], flat[i + 2]});
    }
    RCLCPP_INFO(get_logger(), "Mission: take off to %.1f m, %zu waypoint(s)",
                config.takeoff_altitude, config.waypoints.size());
    return config;
  }

  ControllerType load_controller_type() {
    const auto name = declare_parameter("controller", std::string("px4"));
    if (name != "px4" && name != "own") {
      throw std::invalid_argument("controller must be 'px4' or 'own'");
    }
    RCLCPP_INFO(get_logger(), "Position controller: %s", name.c_str());
    return name == "own" ? ControllerType::kOwn : ControllerType::kPx4;
  }

  PositionControllerConfig load_controller_config() {
    PositionControllerConfig config;
    config.hover_thrust = declare_parameter("hover_thrust", config.hover_thrust);
    config.max_velocity_xy = declare_parameter("max_velocity_xy", config.max_velocity_xy);
    config.position_p_xy = declare_parameter("position_p_xy", config.position_p_xy);
    config.velocity_xy.kp = declare_parameter("velocity_xy_p", config.velocity_xy.kp);
    config.velocity_xy.ki = declare_parameter("velocity_xy_i", config.velocity_xy.ki);
    config.velocity_xy.kd = declare_parameter("velocity_xy_d", config.velocity_xy.kd);

    // With a smooth reference the derivative should act on the velocity error:
    // on the measurement it would resist the very acceleration the reference asks for.
    if (declare_parameter("trajectory_shaping", true)) {
      config.velocity_xy.derivative_on = gnc::control::DerivativeOn::kError;
    }
    return config;
  }

  ReferenceLimits load_reference_limits() {
    const auto& controller = position_controller_.config();
    ReferenceLimits limits;
    limits.max_velocity_xy = controller.max_velocity_xy;
    limits.max_velocity_up = controller.max_velocity_up;
    limits.max_velocity_down = controller.max_velocity_down;
    limits.max_acceleration = declare_parameter("max_acceleration", limits.max_acceleration);
    if (controller_type_ == ControllerType::kOwn) {
      RCLCPP_INFO(get_logger(), "Trajectory shaping: %s (max acceleration %.1f m/s^2)",
                  shaping_ ? "on" : "off", limits.max_acceleration);
    }
    return limits;
  }

  void on_position(const VehicleLocalPosition& msg) {
    vehicle_.position_valid = msg.xy_valid && msg.z_valid;
    vehicle_.position_ned = Eigen::Vector3d(msg.x, msg.y, msg.z);
    vehicle_.heading = msg.heading;
    const Eigen::Vector3d velocity(msg.vx, msg.vy, msg.vz);

    // dt from PX4's own timestamps (microseconds): exact sample spacing, no ROS scheduling jitter.
    const double dt = last_position_stamp_
                          ? static_cast<double>(msg.timestamp - *last_position_stamp_) * 1e-6
                          : 0.0;
    last_position_stamp_ = msg.timestamp;

    if (controller_type_ != ControllerType::kOwn || !target_) {
      return;
    }
    // Integrators only run while we are actually flying the vehicle; on the
    // ground or in another mode they would wind up and kick on take-off.
    // Likewise the reference starts from wherever the vehicle is when we take over.
    if (!vehicle_.armed || !vehicle_.offboard_active) {
      position_controller_.reset();
      reference_generator_.reset(vehicle_.position_ned, velocity);
    }
    const auto sp =
        shaping_ ? position_controller_.update(reference_generator_.update(*target_, dt),
                                               target_yaw_, vehicle_.position_ned, velocity, dt)
                 : position_controller_.update(*target_, target_yaw_, vehicle_.position_ned,
                                               velocity, dt);
    publish_attitude_setpoint(sp.attitude, sp.thrust);
  }

  void on_timer() {
    if (!have_status_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for PX4 (is MicroXRCEAgent running?)");
      return;
    }

    const Output out = mission_.step(vehicle_, now().seconds());
    log_progress(out);

    target_ = out.setpoint_ned;
    target_yaw_ = out.yaw;
    // With our controller, setpoints go out from on_position at the estimator rate.
    if (controller_type_ == ControllerType::kPx4 && target_) {
      publish_position_setpoint(*target_, target_yaw_);
    }

    if (out.requests.set_offboard_mode) {
      publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, kPx4CustomMainModeOffboard);
    }
    if (out.requests.arm) {
      publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    }
    if (out.requests.land) {
      publish_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
  }

  void log_progress(const Output& out) {
    if (out.phase != last_phase_) {
      RCLCPP_INFO(get_logger(), "Phase: %s -> %s", gnc::mission::to_string(last_phase_).data(),
                  gnc::mission::to_string(out.phase).data());
      if (out.phase == Phase::kAborted) {
        RCLCPP_ERROR(get_logger(), "Mission aborted: %s", mission_.abort_reason().c_str());
      }
      last_phase_ = out.phase;
    }
    if (out.phase == Phase::kWaypoints && mission_.waypoint_index() != last_waypoint_) {
      last_waypoint_ = mission_.waypoint_index();
      RCLCPP_INFO(get_logger(), "Flying to waypoint %zu", last_waypoint_ + 1);
    }
  }

  void publish_offboard_mode(bool position, bool attitude, uint64_t stamp) {
    OffboardControlMode mode{};
    mode.timestamp = stamp;
    mode.position = position;
    mode.attitude = attitude;
    mode_pub_->publish(mode);
  }

  void publish_position_setpoint(const Eigen::Vector3d& position_ned, double yaw) {
    const uint64_t stamp = timestamp_us();
    publish_offboard_mode(true, false, stamp);

    TrajectorySetpoint sp{};
    sp.timestamp = stamp;
    sp.position = {static_cast<float>(position_ned.x()), static_cast<float>(position_ned.y()),
                   static_cast<float>(position_ned.z())};
    // NaN tells PX4 "not set": only position is controlled, it plans the rest.
    sp.velocity = {kNaN, kNaN, kNaN};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = static_cast<float>(yaw);
    sp.yawspeed = kNaN;
    trajectory_pub_->publish(sp);
  }

  void publish_attitude_setpoint(const Eigen::Quaterniond& attitude, double thrust) {
    const uint64_t stamp = timestamp_us();
    publish_offboard_mode(false, true, stamp);

    VehicleAttitudeSetpoint sp{};
    sp.timestamp = stamp;
    sp.q_d = {static_cast<float>(attitude.w()), static_cast<float>(attitude.x()),
              static_cast<float>(attitude.y()), static_cast<float>(attitude.z())};
    // Multicopter thrust acts along -z of the body FRD frame.
    sp.thrust_body = {0.0F, 0.0F, static_cast<float>(-thrust)};
    sp.yaw_sp_move_rate = 0.0F;
    attitude_pub_->publish(sp);
  }

  void publish_command(uint16_t command, float param1 = 0.0F, float param2 = 0.0F) {
    VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  uint64_t timestamp_us() { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }

  MissionStateMachine mission_;
  ControllerType controller_type_;
  PositionController position_controller_;
  bool shaping_;
  ReferenceGenerator reference_generator_;
  VehicleState vehicle_;
  bool have_status_{false};
  std::optional<Eigen::Vector3d> target_;
  double target_yaw_{0.0};
  std::optional<uint64_t> last_position_stamp_;
  Phase last_phase_{Phase::kWaitForReady};
  std::size_t last_waypoint_{std::numeric_limits<std::size_t>::max()};

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
