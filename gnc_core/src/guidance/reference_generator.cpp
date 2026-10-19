#include "gnc_core/guidance/reference_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnc::guidance {

ReferenceGenerator::ReferenceGenerator(const ReferenceLimits& limits) : limits_(limits) {}

void ReferenceGenerator::reset(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
  reference_.position = position;
  reference_.velocity = velocity;
  reference_.acceleration.setZero();
}

const Reference& ReferenceGenerator::update(const Eigen::Vector3d& target, double dt) {
  if (dt <= 0.0) {
    return reference_;
  }
  const double a_max = limits_.max_acceleration;
  const Eigen::Vector3d to_target = target - reference_.position;
  const double distance = to_target.norm();

  // Velocity we would like to have: straight at the target, as fast as the
  // limits allow, but slow enough to stop on it, and never past it in one step.
  Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();
  if (distance > 1e-9) {
    const Eigen::Vector3d direction = to_target / distance;
    // The stopping-distance rule is applied once per step, so leave room for the
    // distance covered during this step; otherwise the reference brakes one step
    // late and overshoots by a few centimetres.
    const double braking_distance =
        std::max(0.0, distance - reference_.velocity.dot(direction) * dt);
    const double speed = std::min(
        {max_speed_along(direction), std::sqrt(2.0 * a_max * braking_distance), distance / dt});
    desired_velocity = direction * speed;
  }

  // Move the reference velocity towards it with bounded acceleration.
  Eigen::Vector3d dv = desired_velocity - reference_.velocity;
  const double max_dv = a_max * dt;
  if (dv.norm() > max_dv) {
    dv *= max_dv / dv.norm();
  }
  reference_.acceleration = dv / dt;
  reference_.velocity += dv;
  reference_.position += reference_.velocity * dt;

  // Settled: pin exactly on the target so it does not dither around it.
  if ((target - reference_.position).norm() < 1e-6 && reference_.velocity.norm() < max_dv) {
    reference_.position = target;
    reference_.velocity.setZero();
    reference_.acceleration.setZero();
  }
  return reference_;
}

double ReferenceGenerator::max_speed_along(const Eigen::Vector3d& direction) const {
  // Largest speed along `direction` whose horizontal and vertical components stay in limits.
  double speed = std::numeric_limits<double>::infinity();
  const double horizontal = direction.head<2>().norm();
  if (horizontal > 1e-9) {
    speed = std::min(speed, limits_.max_velocity_xy / horizontal);
  }
  if (std::abs(direction.z()) > 1e-9) {
    // NED: negative z is up.
    const double vertical_limit =
        direction.z() < 0.0 ? limits_.max_velocity_up : limits_.max_velocity_down;
    speed = std::min(speed, vertical_limit / std::abs(direction.z()));
  }
  return speed;
}

}  // namespace gnc::guidance
