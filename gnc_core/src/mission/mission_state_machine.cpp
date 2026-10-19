#include "gnc_core/mission/mission_state_machine.hpp"

#include <utility>

namespace gnc::mission {

std::string_view to_string(Phase phase) {
  switch (phase) {
    case Phase::kWaitForReady:
      return "WAIT_FOR_READY";
    case Phase::kWarmup:
      return "WARMUP";
    case Phase::kArming:
      return "ARMING";
    case Phase::kTakeoff:
      return "TAKEOFF";
    case Phase::kWaypoints:
      return "WAYPOINTS";
    case Phase::kLanding:
      return "LANDING";
    case Phase::kDone:
      return "DONE";
    case Phase::kAborted:
      return "ABORTED";
  }
  return "UNKNOWN";
}

MissionStateMachine::MissionStateMachine(MissionConfig config) : config_(std::move(config)) {}

Output MissionStateMachine::step(const VehicleState& vehicle, double now) {
  // While airborne under our control, losing arming or offboard mode means
  // PX4 (or a pilot) has taken over. Stop commanding and let PX4 handle it.
  if (phase_ == Phase::kTakeoff || phase_ == Phase::kWaypoints) {
    if (!vehicle.armed) {
      abort("disarmed in flight");
    } else if (!vehicle.offboard_active) {
      abort("offboard mode lost (failsafe or pilot took over)");
    }
  }

  Output out;
  switch (phase_) {
    case Phase::kWaitForReady:
      on_wait_for_ready(vehicle, now, out);
      break;
    case Phase::kWarmup:
      on_warmup(now, out);
      break;
    case Phase::kArming:
      on_arming(vehicle, now, out);
      break;
    case Phase::kTakeoff:
      on_takeoff(vehicle, now, out);
      break;
    case Phase::kWaypoints:
      on_waypoints(vehicle, now, out);
      break;
    case Phase::kLanding:
      on_landing(vehicle, now, out);
      break;
    case Phase::kDone:
    case Phase::kAborted:
      break;
  }

  out.phase = phase_;
  out.yaw = home_yaw_;
  return out;
}

void MissionStateMachine::on_wait_for_ready(const VehicleState& vehicle, double now, Output& out) {
  if (vehicle.position_valid && vehicle.preflight_checks_pass) {
    home_ned_ = vehicle.position_ned;
    home_yaw_ = vehicle.heading;
    enter(Phase::kWarmup, now);
    out.setpoint_ned = takeoff_target();
  }
}

void MissionStateMachine::on_warmup(double now, Output& out) {
  out.setpoint_ned = takeoff_target();
  if (now - phase_start_ >= config_.warmup_time) {
    enter(Phase::kArming, now);
  }
}

void MissionStateMachine::on_arming(const VehicleState& vehicle, double now, Output& out) {
  out.setpoint_ned = takeoff_target();
  if (vehicle.armed && vehicle.offboard_active) {
    enter(Phase::kTakeoff, now);
  } else if (now - phase_start_ > config_.arming_timeout) {
    abort("could not arm and enter offboard mode in time");
    out.setpoint_ned.reset();
  } else if (retry_due(now)) {
    out.requests.set_offboard_mode = !vehicle.offboard_active;
    out.requests.arm = !vehicle.armed;
  }
}

void MissionStateMachine::on_takeoff(const VehicleState& vehicle, double now, Output& out) {
  out.setpoint_ned = takeoff_target();
  if (reached(vehicle, takeoff_target())) {
    waypoint_index_ = 0;
    enter(config_.waypoints.empty() ? Phase::kLanding : Phase::kWaypoints, now);
  }
}

void MissionStateMachine::on_waypoints(const VehicleState& vehicle, double now, Output& out) {
  const Eigen::Vector3d target = waypoint_target(waypoint_index_);
  out.setpoint_ned = target;
  if (!reached(vehicle, target)) {
    hold_start_.reset();
    return;
  }
  if (!hold_start_) {
    hold_start_ = now;
  }
  if (now - *hold_start_ < config_.hold_time) {
    return;
  }
  ++waypoint_index_;
  hold_start_.reset();
  if (waypoint_index_ == config_.waypoints.size()) {
    enter(Phase::kLanding, now);
    out.setpoint_ned.reset();
  }
}

void MissionStateMachine::on_landing(const VehicleState& vehicle, double now, Output& out) {
  // PX4 lands on its own in AUTO_LAND mode; no setpoint stream needed.
  if (vehicle.landed && !vehicle.armed) {
    enter(Phase::kDone, now);
  } else if (vehicle.offboard_active && retry_due(now)) {
    out.requests.land = true;
  }
}

void MissionStateMachine::enter(Phase phase, double now) {
  phase_ = phase;
  phase_start_ = now;
  command_sent_ = false;
  hold_start_.reset();
}

void MissionStateMachine::abort(std::string reason) {
  phase_ = Phase::kAborted;
  abort_reason_ = std::move(reason);
}

Eigen::Vector3d MissionStateMachine::takeoff_target() const {
  return home_ned_ + Eigen::Vector3d(0.0, 0.0, -config_.takeoff_altitude);
}

Eigen::Vector3d MissionStateMachine::waypoint_target(std::size_t index) const {
  const Waypoint& wp = config_.waypoints.at(index);
  return home_ned_ + Eigen::Vector3d(wp.north, wp.east, -wp.altitude);
}

bool MissionStateMachine::reached(const VehicleState& vehicle,
                                  const Eigen::Vector3d& target) const {
  return (vehicle.position_ned - target).norm() <= config_.acceptance_radius;
}

bool MissionStateMachine::retry_due(double now) {
  if (command_sent_ && now - last_command_time_ < config_.command_retry_period) {
    return false;
  }
  command_sent_ = true;
  last_command_time_ = now;
  return true;
}

}  // namespace gnc::mission
