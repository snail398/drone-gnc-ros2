#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gnc_core/control/pid.hpp"

namespace gnc::control {

constexpr double kGravity = 9.80665;  // m/s^2

// Defaults follow PX4's multicopter position controller (MPC_* parameters),
// so the two controllers can be compared on equal terms.
struct PositionControllerConfig {
  double position_p_xy{0.95};  // (m/s) per m of position error
  double position_p_z{1.0};
  PidGains velocity_xy{.kp = 1.8, .ki = 0.4, .kd = 0.2, .integral_limit = 2.0};
  PidGains velocity_z{.kp = 4.0, .ki = 2.0, .kd = 0.0, .integral_limit = kGravity};
  double max_velocity_xy{5.0};    // m/s
  double max_velocity_up{3.0};    // m/s
  double max_velocity_down{1.5};  // m/s
  double max_tilt{0.785398};      // rad (45 deg)
  double hover_thrust{0.6};       // normalized thrust that holds altitude
  double min_thrust{0.12};
  double max_thrust{1.0};
};

// Everything the attitude controller needs, plus intermediate values for logging.
struct AttitudeThrustSetpoint {
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};  // body FRD -> NED
  double thrust{0.0};  // collective, normalized [0, 1], along -body z
  Eigen::Vector3d velocity_setpoint{Eigen::Vector3d::Zero()};      // NED, m/s
  Eigen::Vector3d acceleration_setpoint{Eigen::Vector3d::Zero()};  // NED, m/s^2
};

// Cascaded multicopter position controller:
//   position error --P--> velocity setpoint --PID--> acceleration setpoint
//   --> thrust direction (body z) + collective thrust --> attitude quaternion.
// All vectors are in the local NED frame.
class PositionController {
 public:
  explicit PositionController(const PositionControllerConfig& config);

  [[nodiscard]] AttitudeThrustSetpoint update(const Eigen::Vector3d& position_setpoint,
                                              double yaw_setpoint, const Eigen::Vector3d& position,
                                              const Eigen::Vector3d& velocity, double dt);

  // Clear integrators, e.g. while the vehicle is on the ground or not under our control.
  void reset();

  [[nodiscard]] const PositionControllerConfig& config() const { return config_; }

 private:
  [[nodiscard]] Eigen::Vector3d velocity_setpoint(const Eigen::Vector3d& position_error) const;

  PositionControllerConfig config_;
  Pid vel_x_;
  Pid vel_y_;
  Pid vel_z_;
};

// Rotate the body z axis (FRD, so pointing "down" through the vehicle) onto
// `body_z` (NED, need not be normalized) with the nose towards `yaw`.
[[nodiscard]] Eigen::Quaterniond attitude_from_body_z(const Eigen::Vector3d& body_z, double yaw);

// Tilt `body_z` towards `world_z` until the angle between them is at most max_angle.
[[nodiscard]] Eigen::Vector3d limit_tilt(const Eigen::Vector3d& body_z,
                                         const Eigen::Vector3d& world_z, double max_angle);

}  // namespace gnc::control
