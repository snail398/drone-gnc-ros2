// Flies an offboard waypoint mission on PX4: arm, take off, visit waypoints, land.
// All decisions live in gnc::mission::MissionStateMachine; this node only
// translates PX4 messages into VehicleState and the state machine output back
// into PX4 messages.

#include <chrono>
#include <cmath>
#include <limits>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

#include "gnc_core/mission/mission_state_machine.hpp"

using namespace std::chrono_literals;
using gnc::mission::MissionConfig;
using gnc::mission::MissionStateMachine;
using gnc::mission::Output;
using gnc::mission::Phase;
using gnc::mission::VehicleState;
using gnc::mission::Waypoint;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// PX4 custom main mode for OFFBOARD, used with VEHICLE_CMD_DO_SET_MODE.
constexpr float kPx4CustomMainModeOffboard = 6.0F;

}  // namespace

class MissionNode : public rclcpp::Node {
 public:
  MissionNode() : Node("mission_node"), mission_(load_config()) {
    const auto px4_qos = rclcpp::SensorDataQoS();
    status_sub_ = create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status_v1", px4_qos, [this](const VehicleStatus& msg) {
          vehicle_.armed = msg.arming_state == VehicleStatus::ARMING_STATE_ARMED;
          vehicle_.offboard_active = msg.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          vehicle_.preflight_checks_pass = msg.pre_flight_checks_pass;
          have_status_ = true;
        });
    position_sub_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", px4_qos, [this](const VehicleLocalPosition& msg) {
          vehicle_.position_valid = msg.xy_valid && msg.z_valid;
          vehicle_.position_ned = Eigen::Vector3d(msg.x, msg.y, msg.z);
          vehicle_.heading = msg.heading;
        });
    land_sub_ = create_subscription<VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", px4_qos,
        [this](const VehicleLandDetected& msg) { vehicle_.landed = msg.landed; });

    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // 20 Hz: comfortably above the 2 Hz PX4 requires to stay in offboard mode.
    timer_ = create_wall_timer(50ms, [this] { on_timer(); });
  }

 private:
  MissionConfig load_config() {
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

  void on_timer() {
    if (!have_status_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for PX4 (is MicroXRCEAgent running?)");
      return;
    }

    const Output out = mission_.step(vehicle_, now().seconds());

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

    if (out.setpoint_ned) {
      publish_setpoint(*out.setpoint_ned, out.yaw);
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

  void publish_setpoint(const Eigen::Vector3d& position_ned, double yaw) {
    const uint64_t stamp = timestamp_us();

    OffboardControlMode mode{};
    mode.timestamp = stamp;
    mode.position = true;
    mode_pub_->publish(mode);

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
    setpoint_pub_->publish(sp);
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
  VehicleState vehicle_;
  bool have_status_{false};
  Phase last_phase_{Phase::kWaitForReady};
  std::size_t last_waypoint_{std::numeric_limits<std::size_t>::max()};

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
