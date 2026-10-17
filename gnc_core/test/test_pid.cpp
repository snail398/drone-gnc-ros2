#include <gtest/gtest.h>

#include <tuple>

#include "gnc_core/control/pid.hpp"

using gnc::control::Pid;
using gnc::control::PidGains;

TEST(Pid, ProportionalOnly) {
  Pid pid({.kp = 2.0});
  EXPECT_DOUBLE_EQ(pid.update(10.0, 4.0, 0.01), 12.0);
}

TEST(Pid, IntegralAccumulatesError) {
  Pid pid({.ki = 1.0});
  std::ignore = pid.update(1.0, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 0.5), 1.0);
}

TEST(Pid, IntegralIsClamped) {
  Pid pid({.ki = 1.0, .integral_limit = 0.3});
  for (int i = 0; i < 100; ++i) {
    std::ignore = pid.update(1.0, 0.0, 0.1);
  }
  EXPECT_DOUBLE_EQ(pid.integral_term(), 0.3);
}

TEST(Pid, OutputIsSaturated) {
  Pid pid({.kp = 100.0, .output_limit = 5.0});
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 0.01), 5.0);
  EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.0, 0.01), -5.0);
}

TEST(Pid, NoIntegralWindupWhileSaturated) {
  Pid pid({.kp = 10.0, .ki = 1.0, .output_limit = 1.0});
  for (int i = 0; i < 100; ++i) {
    std::ignore = pid.update(1.0, 0.0, 0.1);
  }
  EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
}

TEST(Pid, NoDerivativeKickOnSetpointStep) {
  Pid pid({.kd = 1.0});
  std::ignore = pid.update(0.0, 0.0, 0.01);
  // Setpoint jumps, measurement stays: derivative on measurement gives zero.
  EXPECT_DOUBLE_EQ(pid.update(100.0, 0.0, 0.01), 0.0);
}

TEST(Pid, DerivativeOpposesMeasurementChange) {
  Pid pid({.kd = 1.0});
  std::ignore = pid.update(0.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.5, 0.1), -5.0);
}

TEST(Pid, NonPositiveDtSkipsIntegration) {
  Pid pid({.kp = 1.0, .ki = 1.0});
  EXPECT_DOUBLE_EQ(pid.update(2.0, 0.0, 0.0), 2.0);
  EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
}

TEST(Pid, ResetClearsState) {
  Pid pid({.ki = 1.0, .kd = 1.0});
  std::ignore = pid.update(1.0, 0.0, 0.1);
  std::ignore = pid.update(1.0, 0.2, 0.1);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
  // After reset there is no previous measurement, so no derivative term.
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.0, 0.1), 0.0);
}

// Closed loop on a double integrator (x'' = u), the same plant as one axis of
// a hovering multicopter in the acceleration domain.
TEST(Pid, ClosedLoopReachesSetpoint) {
  Pid pid({.kp = 4.0, .ki = 0.5, .kd = 3.0, .output_limit = 10.0});
  double x = 0.0;
  double v = 0.0;
  const double dt = 0.01;
  for (int i = 0; i < 2000; ++i) {
    const double u = pid.update(5.0, x, dt);
    v += u * dt;
    x += v * dt;
  }
  EXPECT_NEAR(x, 5.0, 0.05);
  EXPECT_NEAR(v, 0.0, 0.05);
}
