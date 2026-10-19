#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "gnc_core/mission/mission_state_machine.hpp"

using gnc::mission::MissionConfig;
using gnc::mission::MissionStateMachine;
using gnc::mission::Output;
using gnc::mission::Phase;
using gnc::mission::VehicleState;

namespace {

constexpr double kDt = 0.05;

// Simulated time at a given step. Steps are counted with an integer so that
// floating point error does not accumulate in loop conditions.
constexpr double time_at(int step) { return step * kDt; }
int steps_for(double seconds) { return static_cast<int>(std::lround(seconds / kDt)); }

VehicleState ready_on_ground() {
  VehicleState v;
  v.preflight_checks_pass = true;
  v.position_valid = true;
  v.position_ned = Eigen::Vector3d(1.0, 2.0, 0.0);
  return v;
}

// Minimal stand-in for PX4: obeys requests and flies straight to the setpoint.
class FakeVehicle {
 public:
  explicit FakeVehicle(VehicleState state) : state_(std::move(state)) {}

  void apply(const Output& out) {
    if (out.requests.set_offboard_mode && out.setpoint_ned) {
      state_.offboard_active = true;  // PX4 only accepts this with an active stream
    }
    if (out.requests.arm) {
      state_.armed = true;
    }
    if (out.requests.land) {
      state_.offboard_active = false;
      landing_ = true;
    }

    if (landing_) {
      state_.position_ned.z() = std::min(state_.position_ned.z() + kSpeed * kDt, ground_z_);
      if (state_.position_ned.z() >= ground_z_) {
        state_.landed = true;
        state_.armed = false;
      }
    } else if (state_.armed && state_.offboard_active && out.setpoint_ned) {
      state_.landed = false;
      const Eigen::Vector3d delta = *out.setpoint_ned - state_.position_ned;
      const double step = kSpeed * kDt;
      state_.position_ned += delta.norm() <= step ? delta : delta.normalized() * step;
    }
  }

  VehicleState& state() { return state_; }

 private:
  static constexpr double kSpeed = 2.0;  // m/s
  VehicleState state_;
  double ground_z_{0.0};
  bool landing_{false};
};

MissionConfig square_mission() {
  MissionConfig config;
  config.takeoff_altitude = 5.0;
  config.waypoints = {{10.0, 0.0, 5.0}, {10.0, 10.0, 5.0}, {0.0, 10.0, 5.0}};
  config.hold_time = 1.0;
  return config;
}

}  // namespace

TEST(MissionStateMachine, WaitsUntilPreflightChecksPass) {
  MissionStateMachine mission(square_mission());
  VehicleState v = ready_on_ground();
  v.preflight_checks_pass = false;

  const Output out = mission.step(v, 0.0);

  EXPECT_EQ(out.phase, Phase::kWaitForReady);
  EXPECT_FALSE(out.setpoint_ned.has_value());
}

TEST(MissionStateMachine, StreamsSetpointsBeforeRequestingOffboard) {
  MissionConfig config = square_mission();
  config.warmup_time = 1.0;
  MissionStateMachine mission(config);
  const VehicleState v = ready_on_ground();

  // Warmup starts at step 0 and lasts until 1.0 s (step 20).
  for (int i = 0; i < steps_for(1.0); ++i) {
    const Output out = mission.step(v, time_at(i));
    ASSERT_TRUE(out.setpoint_ned.has_value()) << "step " << i;
    EXPECT_FALSE(out.requests.set_offboard_mode) << "step " << i;
    EXPECT_FALSE(out.requests.arm) << "step " << i;
  }
  mission.step(v, 1.0);  // warmup elapsed -> ARMING
  const Output out = mission.step(v, 1.0 + kDt);
  EXPECT_EQ(out.phase, Phase::kArming);
  EXPECT_TRUE(out.requests.set_offboard_mode);
  EXPECT_TRUE(out.requests.arm);
}

TEST(MissionStateMachine, TakeoffTargetIsAboveHomeInNed) {
  MissionStateMachine mission(square_mission());
  const Output out = mission.step(ready_on_ground(), 0.0);

  // 5 m up from home (1, 2, 0) is z = -5 in NED. A missing setpoint becomes
  // NaN and fails the comparison.
  const Eigen::Vector3d setpoint =
      out.setpoint_ned.value_or(Eigen::Vector3d::Constant(std::nan("")));
  EXPECT_TRUE(setpoint.isApprox(Eigen::Vector3d(1.0, 2.0, -5.0)));
}

TEST(MissionStateMachine, RetriesCommandsAtConfiguredPeriod) {
  MissionConfig config = square_mission();
  config.warmup_time = 0.0;
  config.command_retry_period = 1.0;
  MissionStateMachine mission(config);
  const VehicleState v = ready_on_ground();  // never arms: PX4 ignores us

  int arm_requests = 0;
  for (int i = 0; i < steps_for(3.5); ++i) {
    arm_requests += mission.step(v, time_at(i)).requests.arm ? 1 : 0;
  }
  EXPECT_EQ(arm_requests, 4);  // at ~0, 1, 2, 3 s
}

TEST(MissionStateMachine, AbortsWhenArmingTimesOut) {
  MissionConfig config = square_mission();
  config.arming_timeout = 2.0;
  MissionStateMachine mission(config);
  const VehicleState v = ready_on_ground();

  Output out;
  for (int i = 0; i < steps_for(5.0); ++i) {
    out = mission.step(v, time_at(i));
  }
  EXPECT_EQ(out.phase, Phase::kAborted);
  EXPECT_FALSE(out.setpoint_ned.has_value());
  EXPECT_FALSE(mission.abort_reason().empty());
}

TEST(MissionStateMachine, FliesWholeMissionAndLands) {
  MissionStateMachine mission(square_mission());
  FakeVehicle vehicle(ready_on_ground());

  std::vector<Phase> phases;
  for (int i = 0; i < steps_for(120.0) && mission.phase() != Phase::kDone; ++i) {
    const Output out = mission.step(vehicle.state(), time_at(i));
    if (phases.empty() || phases.back() != out.phase) {
      phases.push_back(out.phase);
    }
    vehicle.apply(out);
  }

  const std::vector<Phase> expected = {Phase::kWarmup,    Phase::kArming,  Phase::kTakeoff,
                                       Phase::kWaypoints, Phase::kLanding, Phase::kDone};
  EXPECT_EQ(phases, expected);
  EXPECT_TRUE(vehicle.state().landed);
  EXPECT_FALSE(vehicle.state().armed);
  // Landed below the last waypoint (north 0, east 10 from home at (1, 2)).
  EXPECT_NEAR(vehicle.state().position_ned.x(), 1.0, 1e-6);
  EXPECT_NEAR(vehicle.state().position_ned.y(), 12.0, 1e-6);
}

TEST(MissionStateMachine, HoldsAtWaypointBeforeMovingOn) {
  MissionConfig config = square_mission();
  config.hold_time = 2.0;
  MissionStateMachine mission(config);
  FakeVehicle vehicle(ready_on_ground());

  // Fly until the first waypoint is reached.
  const Eigen::Vector3d first_wp(11.0, 2.0, -5.0);
  int i = 0;
  for (; i < steps_for(60.0); ++i) {
    vehicle.apply(mission.step(vehicle.state(), time_at(i)));
    if (mission.phase() == Phase::kWaypoints &&
        (vehicle.state().position_ned - first_wp).norm() <= 0.5) {
      break;
    }
  }
  ASSERT_EQ(mission.waypoint_index(), 0U);

  const int reached_at = i;
  for (++i; i < reached_at + steps_for(1.9); ++i) {
    vehicle.apply(mission.step(vehicle.state(), time_at(i)));
    EXPECT_EQ(mission.waypoint_index(), 0U) << "left waypoint early at step " << i;
  }
  for (; i < reached_at + steps_for(2.5); ++i) {
    vehicle.apply(mission.step(vehicle.state(), time_at(i)));
  }
  EXPECT_EQ(mission.waypoint_index(), 1U);
}

TEST(MissionStateMachine, AbortsWhenOffboardIsLostInFlight) {
  MissionStateMachine mission(square_mission());
  FakeVehicle vehicle(ready_on_ground());

  int i = 0;
  for (; i < steps_for(30.0) && mission.phase() != Phase::kWaypoints; ++i) {
    vehicle.apply(mission.step(vehicle.state(), time_at(i)));
  }
  ASSERT_EQ(mission.phase(), Phase::kWaypoints);

  vehicle.state().offboard_active = false;  // e.g. PX4 failsafe switched to hold
  const Output out = mission.step(vehicle.state(), time_at(i));

  EXPECT_EQ(out.phase, Phase::kAborted);
  EXPECT_FALSE(out.setpoint_ned.has_value());
  EXPECT_FALSE(out.requests.arm || out.requests.set_offboard_mode || out.requests.land);
}

TEST(MissionStateMachine, LandsDirectlyWhenThereAreNoWaypoints) {
  MissionConfig config = square_mission();
  config.waypoints.clear();
  MissionStateMachine mission(config);
  FakeVehicle vehicle(ready_on_ground());

  for (int i = 0; i < steps_for(60.0) && mission.phase() != Phase::kDone; ++i) {
    vehicle.apply(mission.step(vehicle.state(), time_at(i)));
  }
  EXPECT_EQ(mission.phase(), Phase::kDone);
}
