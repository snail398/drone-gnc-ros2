#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace gnc::mission {

// What the mission needs to know about the vehicle. Position is in the local
// NED frame (z points down), as published by PX4.
struct VehicleState {
  bool preflight_checks_pass{false};
  bool armed{false};
  bool offboard_active{false};
  bool landed{true};
  bool position_valid{false};
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  double heading{0.0};  // rad
};

// Waypoint relative to the take-off point: metres north, metres east, altitude
// above home (positive up). Converted to NED internally, so mission files do
// not have to deal with the inverted z axis.
struct Waypoint {
  double north{0.0};
  double east{0.0};
  double altitude{0.0};
};

struct MissionConfig {
  double takeoff_altitude{5.0};      // m above home
  std::vector<Waypoint> waypoints;   // flown in order after take-off
  double acceptance_radius{0.5};     // m, distance at which a target counts as reached
  double hold_time{2.0};             // s to hover at each waypoint
  double warmup_time{1.0};           // s of setpoint streaming before switching to offboard
  double command_retry_period{1.0};  // s between repeated mode/arm requests
  double arming_timeout{10.0};       // s to get armed and into offboard before giving up
};

enum class Phase : std::uint8_t {
  kWaitForReady,  // waiting for valid position and passing pre-flight checks
  kWarmup,        // streaming setpoints; PX4 rejects offboard without an active stream
  kArming,        // requesting offboard mode and arming
  kTakeoff,
  kWaypoints,
  kLanding,
  kDone,
  kAborted,
};

std::string_view to_string(Phase phase);

// Commands to send to PX4 on this step.
struct Requests {
  bool set_offboard_mode{false};
  bool arm{false};
  bool land{false};
};

struct Output {
  Phase phase{Phase::kWaitForReady};
  // Position setpoint (NED) to stream. Empty means: do not stream.
  std::optional<Eigen::Vector3d> setpoint_ned;
  double yaw{0.0};
  Requests requests;
};

// Offboard mission: wait for ready -> warm up -> arm -> take off -> fly
// waypoints -> land. Pure logic: call step() periodically with the latest
// vehicle state and a monotonic time in seconds, then act on the output.
class MissionStateMachine {
 public:
  explicit MissionStateMachine(MissionConfig config);

  Output step(const VehicleState& vehicle, double now);

  [[nodiscard]] Phase phase() const { return phase_; }
  [[nodiscard]] std::size_t waypoint_index() const { return waypoint_index_; }
  [[nodiscard]] const std::string& abort_reason() const { return abort_reason_; }

 private:
  // One handler per phase; each fills `out` and may switch phase.
  void on_wait_for_ready(const VehicleState& vehicle, double now, Output& out);
  void on_warmup(double now, Output& out);
  void on_arming(const VehicleState& vehicle, double now, Output& out);
  void on_takeoff(const VehicleState& vehicle, double now, Output& out);
  void on_waypoints(const VehicleState& vehicle, double now, Output& out);
  void on_landing(const VehicleState& vehicle, double now, Output& out);

  void enter(Phase phase, double now);
  void abort(std::string reason);
  [[nodiscard]] Eigen::Vector3d takeoff_target() const;
  [[nodiscard]] Eigen::Vector3d waypoint_target(std::size_t index) const;
  [[nodiscard]] bool reached(const VehicleState& vehicle, const Eigen::Vector3d& target) const;
  [[nodiscard]] bool retry_due(double now);

  MissionConfig config_;
  Phase phase_{Phase::kWaitForReady};
  double phase_start_{0.0};
  double last_command_time_{0.0};
  bool command_sent_{false};
  std::optional<double> hold_start_;
  std::size_t waypoint_index_{0};
  Eigen::Vector3d home_ned_{Eigen::Vector3d::Zero()};
  double home_yaw_{0.0};
  std::string abort_reason_;
};

}  // namespace gnc::mission
