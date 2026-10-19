#include <gtest/gtest.h>

#include <algorithm>

#include "gnc_core/guidance/reference_generator.hpp"

using gnc::guidance::Reference;
using gnc::guidance::ReferenceGenerator;
using gnc::guidance::ReferenceLimits;

namespace {

constexpr double kDt = 0.02;
constexpr double kEps = 1e-9;

// Run the generator towards `target` for `steps` and track extremes.
struct RunStats {
  double max_north{-1e9};
  double max_speed_xy{0.0};
  double max_climb{0.0};
  double max_sink{0.0};
  double max_accel{0.0};
  double max_velocity_jump{0.0};
};

RunStats run(ReferenceGenerator& gen, const Eigen::Vector3d& target, int steps) {
  RunStats r;
  Eigen::Vector3d prev_velocity = gen.reference().velocity;
  for (int i = 0; i < steps; ++i) {
    const Reference& ref = gen.update(target, kDt);
    r.max_north = std::max(r.max_north, ref.position.x());
    r.max_speed_xy = std::max(r.max_speed_xy, ref.velocity.head<2>().norm());
    r.max_climb = std::max(r.max_climb, -ref.velocity.z());
    r.max_sink = std::max(r.max_sink, ref.velocity.z());
    r.max_accel = std::max(r.max_accel, ref.acceleration.norm());
    r.max_velocity_jump = std::max(r.max_velocity_jump, (ref.velocity - prev_velocity).norm());
    prev_velocity = ref.velocity;
  }
  return r;
}

}  // namespace

TEST(ReferenceGenerator, StopsOnTargetWithoutOvershoot) {
  ReferenceGenerator gen(ReferenceLimits{});
  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const RunStats r = run(gen, {10, 0, 0}, 500);

  EXPECT_LE(r.max_north, 10.0 + kEps);
  EXPECT_TRUE(gen.reference().position.isApprox(Eigen::Vector3d(10, 0, 0)));
  EXPECT_NEAR(gen.reference().velocity.norm(), 0.0, kEps);
}

TEST(ReferenceGenerator, RespectsSpeedAndAccelerationLimits) {
  ReferenceLimits limits;
  limits.max_velocity_xy = 4.0;
  limits.max_acceleration = 2.0;
  ReferenceGenerator gen(limits);
  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const RunStats r = run(gen, {30, 30, 0}, 2000);

  EXPECT_LE(r.max_speed_xy, 4.0 + kEps);
  EXPECT_LE(r.max_accel, 2.0 + kEps);
  EXPECT_NEAR(r.max_speed_xy, 4.0, 1e-6);  // long move: reaches cruise speed
}

TEST(ReferenceGenerator, UsesSeparateClimbAndSinkLimits) {
  ReferenceLimits limits;
  limits.max_velocity_up = 3.0;
  limits.max_velocity_down = 1.0;
  ReferenceGenerator gen(limits);

  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const RunStats up = run(gen, {0, 0, -30}, 2000);  // NED: negative z is up
  EXPECT_NEAR(up.max_climb, 3.0, 1e-6);

  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const RunStats down = run(gen, {0, 0, 30}, 4000);
  EXPECT_NEAR(down.max_sink, 1.0, 1e-6);
}

TEST(ReferenceGenerator, StartsFromCurrentMotionWithoutJump) {
  ReferenceLimits limits;
  ReferenceGenerator gen(limits);
  // Moving east at 4 m/s, target is north: velocity must turn gradually.
  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 4, 0));

  const RunStats r = run(gen, {10, 0, 0}, 1000);

  EXPECT_LE(r.max_velocity_jump, limits.max_acceleration * kDt + kEps);
  EXPECT_TRUE(gen.reference().position.isApprox(Eigen::Vector3d(10, 0, 0), 1e-6));
}

TEST(ReferenceGenerator, ReactsToTargetChangeSmoothly) {
  ReferenceLimits limits;
  ReferenceGenerator gen(limits);
  gen.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const RunStats first = run(gen, {20, 0, 0}, 150);  // still cruising
  const RunStats second = run(gen, {0, 20, 0}, 1500);

  EXPECT_LE(first.max_velocity_jump, limits.max_acceleration * kDt + kEps);
  EXPECT_LE(second.max_velocity_jump, limits.max_acceleration * kDt + kEps);
  EXPECT_TRUE(gen.reference().position.isApprox(Eigen::Vector3d(0, 20, 0), 1e-6));
}

TEST(ReferenceGenerator, IgnoresNonPositiveDt) {
  ReferenceGenerator gen(ReferenceLimits{});
  gen.reset(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d::Zero());
  const Reference& ref = gen.update({10, 0, 0}, 0.0);
  EXPECT_TRUE(ref.position.isApprox(Eigen::Vector3d(1, 2, 3)));
}
