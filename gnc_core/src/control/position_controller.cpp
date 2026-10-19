#include "gnc_core/control/position_controller.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::control {

namespace {

PidGains with_output_limit(PidGains gains, double limit) {
  gains.output_limit = limit;
  return gains;
}

}  // namespace

PositionController::PositionController(const PositionControllerConfig& config)
    : config_(config),
      // A horizontal acceleration of g * tan(max_tilt) is the most the tilt limit allows.
      vel_x_(with_output_limit(config.velocity_xy, kGravity * std::tan(config.max_tilt))),
      vel_y_(with_output_limit(config.velocity_xy, kGravity * std::tan(config.max_tilt))),
      vel_z_(with_output_limit(config.velocity_z, kGravity)) {}

AttitudeThrustSetpoint PositionController::update(const Eigen::Vector3d& position_setpoint,
                                                  double yaw_setpoint,
                                                  const Eigen::Vector3d& position,
                                                  const Eigen::Vector3d& velocity, double dt) {
  guidance::Reference reference;
  reference.position = position_setpoint;
  return update(reference, yaw_setpoint, position, velocity, dt);
}

AttitudeThrustSetpoint PositionController::update(const guidance::Reference& reference,
                                                  double yaw_setpoint,
                                                  const Eigen::Vector3d& position,
                                                  const Eigen::Vector3d& velocity, double dt) {
  AttitudeThrustSetpoint out;

  // Outer loop: reference velocity (feed-forward) + correction for the position error.
  out.velocity_setpoint = velocity_setpoint(reference.position - position, reference.velocity);

  // Inner loop: reference acceleration (feed-forward) + PID per axis on the velocity error.
  // The feed-forward lets the vehicle follow a moving reference without first building up
  // an error for the feedback to act on.
  Eigen::Vector3d& acc = out.acceleration_setpoint;
  acc.x() = reference.acceleration.x() + vel_x_.update(out.velocity_setpoint.x(), velocity.x(), dt);
  acc.y() = reference.acceleration.y() + vel_y_.update(out.velocity_setpoint.y(), velocity.y(), dt);
  acc.z() = reference.acceleration.z() + vel_z_.update(out.velocity_setpoint.z(), velocity.z(), dt);

  // Acceleration -> thrust. Same formulation as PX4's PositionControl::_accelerationControl:
  // the thrust must cancel gravity and produce `acc`, so its direction (minus body z) is
  // (acc - g). Horizontal acceleration is bought with tilt, vertical with collective thrust.
  Eigen::Vector3d body_z(-acc.x(), -acc.y(), kGravity - acc.z());
  body_z = limit_tilt(body_z.normalized(), Eigen::Vector3d::UnitZ(), config_.max_tilt);

  // Thrust needed along world z, assuming hover_thrust balances gravity. Negative = up.
  const double thrust_ned_z = acc.z() * (config_.hover_thrust / kGravity) - config_.hover_thrust;
  // Tilting reduces the vertical component, so scale up to keep the vertical acceleration.
  const double cos_tilt = body_z.dot(Eigen::Vector3d::UnitZ());
  const double collective = thrust_ned_z / cos_tilt;

  out.thrust = std::clamp(-collective, config_.min_thrust, config_.max_thrust);
  out.attitude = attitude_from_body_z(body_z, yaw_setpoint);
  return out;
}

void PositionController::reset() {
  vel_x_.reset();
  vel_y_.reset();
  vel_z_.reset();
}

Eigen::Vector3d PositionController::velocity_setpoint(
    const Eigen::Vector3d& position_error, const Eigen::Vector3d& velocity_feedforward) const {
  Eigen::Vector3d vel =
      velocity_feedforward + Eigen::Vector3d(config_.position_p_xy * position_error.x(),
                                             config_.position_p_xy * position_error.y(),
                                             config_.position_p_z * position_error.z());

  // Limit horizontal speed as a vector so diagonal moves are not faster.
  const double speed_xy = vel.head<2>().norm();
  if (speed_xy > config_.max_velocity_xy) {
    vel.head<2>() *= config_.max_velocity_xy / speed_xy;
  }
  // NED: negative z velocity is up.
  vel.z() = std::clamp(vel.z(), -config_.max_velocity_up, config_.max_velocity_down);
  return vel;
}

Eigen::Quaterniond attitude_from_body_z(const Eigen::Vector3d& body_z, double yaw) {
  Eigen::Vector3d z = body_z.squaredNorm() > 1e-12 ? body_z.normalized() : Eigen::Vector3d::UnitZ();

  // Desired nose direction rotated by 90 deg in the horizontal plane.
  const Eigen::Vector3d y_c(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d x = y_c.cross(z);
  if (z.z() < 0.0) {
    x = -x;  // keep the nose forward when upside down
  }
  if (x.squaredNorm() < 1e-12) {
    // Thrust exactly horizontal: yaw is undefined, pick any orthogonal axis.
    x = Eigen::Vector3d::UnitZ().cross(z);
  }
  x.normalize();
  const Eigen::Vector3d y = z.cross(x);

  Eigen::Matrix3d rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;
  return Eigen::Quaterniond(rotation).normalized();
}

Eigen::Vector3d limit_tilt(const Eigen::Vector3d& body_z, const Eigen::Vector3d& world_z,
                           double max_angle) {
  const double dot = std::clamp(body_z.dot(world_z), -1.0, 1.0);
  const double angle = std::acos(dot);
  if (angle <= max_angle) {
    return body_z;
  }
  Eigen::Vector3d rejection = body_z - dot * world_z;  // component orthogonal to world_z
  if (rejection.squaredNorm() < 1e-12) {
    rejection = Eigen::Vector3d::UnitX();
  }
  return std::cos(max_angle) * world_z + std::sin(max_angle) * rejection.normalized();
}

}  // namespace gnc::control
