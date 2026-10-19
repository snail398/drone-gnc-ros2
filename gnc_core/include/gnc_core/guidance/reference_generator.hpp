#pragma once

#include <Eigen/Core>

namespace gnc::guidance {

// Position, velocity and acceleration the vehicle should have right now (NED).
// Velocity and acceleration are feed-forward terms for the position controller.
struct Reference {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct ReferenceLimits {
  double max_velocity_xy{5.0};    // m/s
  double max_velocity_up{3.0};    // m/s
  double max_velocity_down{1.5};  // m/s
  double max_acceleration{3.0};   // m/s^2, same default as PX4's MPC_ACC_HOR for auto modes
};

// Turns a target that may jump (e.g. the next waypoint) into a smooth reference
// that moves towards it with bounded velocity and acceleration and brakes in
// time to stop on the target: the speed towards the target never exceeds
// sqrt(2 * a_max * distance), the speed from which a_max can still stop it.
// The acceleration may step (no jerk limit yet). Works in 3D and reacts online to target changes.
class ReferenceGenerator {
 public:
  explicit ReferenceGenerator(const ReferenceLimits& limits);

  // Start from the vehicle's current state so the reference does not jump.
  void reset(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);

  [[nodiscard]] const Reference& update(const Eigen::Vector3d& target, double dt);

  [[nodiscard]] const Reference& reference() const { return reference_; }

 private:
  [[nodiscard]] double max_speed_along(const Eigen::Vector3d& direction) const;

  ReferenceLimits limits_;
  Reference reference_;
};

}  // namespace gnc::guidance
