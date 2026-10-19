#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "gnc_core/control/position_controller.hpp"

using gnc::control::attitude_from_body_z;
using gnc::control::AttitudeThrustSetpoint;
using gnc::control::kGravity;
using gnc::control::limit_tilt;
using gnc::control::PositionController;
using gnc::control::PositionControllerConfig;

namespace {

constexpr double kDt = 0.01;

// Body z axis (FRD "down") expressed in NED.
Eigen::Vector3d body_z_of(const Eigen::Quaterniond& q) { return q * Eigen::Vector3d::UnitZ(); }

double tilt_of(const Eigen::Quaterniond& q) {
  return std::acos(std::clamp(body_z_of(q).z(), -1.0, 1.0));
}

double yaw_of(const Eigen::Quaterniond& q) {
  const Eigen::Vector3d nose = q * Eigen::Vector3d::UnitX();
  return std::atan2(nose.y(), nose.x());
}

// Point-mass multicopter. The thrust direction follows the commanded attitude
// instantly, or with a first-order lag standing in for the attitude loop.
// `true_hover_thrust` may differ from what the controller assumes.
struct PointMass {
  double true_hover_thrust{0.6};
  double attitude_lag{0.0};  // s
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_dir{0.0, 0.0, -1.0};

  void step(const AttitudeThrustSetpoint& sp, double dt) {
    const Eigen::Vector3d commanded = -body_z_of(sp.attitude);
    if (attitude_lag > 0.0) {
      thrust_dir = (thrust_dir + (commanded - thrust_dir) * (dt / attitude_lag)).normalized();
    } else {
      thrust_dir = commanded;
    }
    const Eigen::Vector3d thrust_acc = thrust_dir * (sp.thrust / true_hover_thrust) * kGravity;
    const Eigen::Vector3d acc = thrust_acc + Eigen::Vector3d(0.0, 0.0, kGravity);
    velocity += acc * dt;
    position += velocity * dt;
  }
};

}  // namespace

TEST(PositionController, HoversLevelWithHoverThrust) {
  PositionController controller(PositionControllerConfig{});
  const auto sp = controller.update({0, 0, -5}, 0.0, {0, 0, -5}, {0, 0, 0}, kDt);

  EXPECT_NEAR(sp.thrust, 0.6, 1e-9);
  EXPECT_NEAR(tilt_of(sp.attitude), 0.0, 1e-9);
  EXPECT_TRUE(sp.attitude.isApprox(Eigen::Quaterniond::Identity(), 1e-9));
}

TEST(PositionController, FollowsYawSetpoint) {
  PositionController controller(PositionControllerConfig{});
  const double yaw = std::numbers::pi / 2.0;
  const auto sp = controller.update({0, 0, -5}, yaw, {0, 0, -5}, {0, 0, 0}, kDt);

  EXPECT_NEAR(yaw_of(sp.attitude), yaw, 1e-9);
  EXPECT_NEAR(tilt_of(sp.attitude), 0.0, 1e-9);
}

TEST(PositionController, PitchesTowardsTarget) {
  PositionController controller(PositionControllerConfig{});
  // Target 2 m north: the thrust vector (-body z) must lean north.
  const auto sp = controller.update({2, 0, -5}, 0.0, {0, 0, -5}, {0, 0, 0}, kDt);

  const Eigen::Vector3d thrust_dir = -body_z_of(sp.attitude);
  EXPECT_GT(thrust_dir.x(), 0.0);
  EXPECT_NEAR(thrust_dir.y(), 0.0, 1e-9);
  // Tilting while holding altitude needs more than hover thrust.
  EXPECT_GT(sp.thrust, 0.6);
}

TEST(PositionController, RespectsTiltLimit) {
  PositionControllerConfig config;
  config.max_tilt = 0.35;  // ~20 deg
  PositionController controller(config);
  // Far target and moving away from it at speed: maximum horizontal demand.
  const auto sp = controller.update({100, 100, -5}, 0.0, {0, 0, -5}, {-5, -5, 0}, kDt);

  EXPECT_LE(tilt_of(sp.attitude), config.max_tilt + 1e-9);
}

TEST(PositionController, KeepsThrustWithinLimits) {
  PositionController controller(PositionControllerConfig{});
  // Far above the target and climbing: wants to drop hard.
  const auto down = controller.update({0, 0, 0}, 0.0, {0, 0, -50}, {0, 0, -5}, kDt);
  EXPECT_DOUBLE_EQ(down.thrust, 0.12);

  controller.reset();
  // Far below the target and sinking: wants full power.
  const auto up = controller.update({0, 0, -50}, 0.0, {0, 0, 0}, {0, 0, 5}, kDt);
  EXPECT_DOUBLE_EQ(up.thrust, 1.0);
}

TEST(PositionController, LimitsVelocitySetpoint) {
  PositionController controller(PositionControllerConfig{});
  const auto sp = controller.update({100, 100, -100}, 0.0, {0, 0, 0}, {0, 0, 0}, kDt);

  EXPECT_NEAR(sp.velocity_setpoint.head<2>().norm(), 5.0, 1e-9);
  EXPECT_NEAR(sp.velocity_setpoint.z(), -3.0, 1e-9);  // max climb rate
}

TEST(PositionController, LimitTiltLeavesSmallTiltUntouched) {
  const Eigen::Vector3d v = Eigen::Vector3d(0.1, 0.0, 1.0).normalized();
  EXPECT_TRUE(limit_tilt(v, Eigen::Vector3d::UnitZ(), 0.5).isApprox(v));
}

TEST(PositionController, AttitudeFromBodyZIsLevelForDown) {
  const Eigen::Quaterniond q = attitude_from_body_z(Eigen::Vector3d::UnitZ(), 0.0);
  EXPECT_TRUE(q.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
}

// Fly to a 3D target on the point-mass model. The model's real hover thrust
// (0.65) differs from the controller's assumption (0.6); the vertical
// integrator has to absorb the difference.
TEST(PositionController, ClosedLoopReachesTargetWithHoverThrustError) {
  PositionControllerConfig config;
  PositionController controller(config);
  PointMass vehicle{.true_hover_thrust = 0.65};
  const Eigen::Vector3d target(10.0, 5.0, -5.0);

  double max_tilt_seen = 0.0;
  for (int i = 0; i < 3000; ++i) {  // 30 s
    const auto sp = controller.update(target, 0.0, vehicle.position, vehicle.velocity, kDt);
    max_tilt_seen = std::max(max_tilt_seen, tilt_of(sp.attitude));
    vehicle.step(sp, kDt);
  }

  EXPECT_LT((vehicle.position - target).norm(), 0.05);
  EXPECT_LT(vehicle.velocity.norm(), 0.05);
  EXPECT_LE(max_tilt_seen, config.max_tilt + 1e-9);
}

// 10 m step with a lagging attitude loop: following a shaped reference (with
// feed-forward and D on the error) must overshoot far less than chasing the
// step target directly, while arriving about as fast.
TEST(PositionController, ShapedReferenceCutsOvershoot) {
  const Eigen::Vector3d start(0.0, 0.0, -5.0);
  const Eigen::Vector3d target(10.0, 0.0, -5.0);
  constexpr double kStep = 0.02;
  constexpr int kSteps = 1500;

  auto fly = [&](bool shaped) {
    PositionControllerConfig config;
    if (shaped) {
      config.velocity_xy.derivative_on = gnc::control::DerivativeOn::kError;
    }
    PositionController controller(config);
    gnc::guidance::ReferenceGenerator generator(gnc::guidance::ReferenceLimits{});
    generator.reset(start, Eigen::Vector3d::Zero());
    PointMass vehicle{.attitude_lag = 0.15, .position = start};

    double max_north = 0.0;
    for (int i = 0; i < kSteps; ++i) {
      const auto sp =
          shaped ? controller.update(generator.update(target, kStep), 0.0, vehicle.position,
                                     vehicle.velocity, kStep)
                 : controller.update(target, 0.0, vehicle.position, vehicle.velocity, kStep);
      vehicle.step(sp, kStep);
      max_north = std::max(max_north, vehicle.position.x());
    }
    EXPECT_LT((vehicle.position - target).norm(), 0.05) << (shaped ? "shaped" : "step");
    return max_north - target.x();
  };

  const double overshoot_step = fly(false);
  const double overshoot_shaped = fly(true);
  EXPECT_GT(overshoot_step, 0.5);  // the M3 behaviour this test guards against
  EXPECT_LT(overshoot_shaped, 0.25);
}
