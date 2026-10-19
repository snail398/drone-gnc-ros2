#pragma once

#include <cstdint>
#include <limits>

namespace gnc::control {

// What the derivative term differentiates.
enum class DerivativeOn : std::uint8_t {
  // -d(measurement)/dt: no "kick" when the setpoint jumps, but it also resists
  // changes the setpoint asks for. Right for step setpoints.
  kMeasurement,
  // d(error)/dt: follows a smoothly moving setpoint. Right when the setpoint is
  // generated smoothly, e.g. by a trajectory generator.
  kError,
};

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  // Bounds on the integral term contribution (ki * integral), anti-windup.
  double integral_limit{std::numeric_limits<double>::infinity()};
  // Symmetric bound on the controller output.
  double output_limit{std::numeric_limits<double>::infinity()};
  DerivativeOn derivative_on{DerivativeOn::kMeasurement};
};

// Single-axis PID controller.
//
// - Derivative on the measurement by default (no "kick" on setpoint steps),
//   or on the error for smooth setpoints, see DerivativeOn.
// - Anti-windup: the integral term is clamped, and integration is paused while
//   the output is saturated in the direction that would increase saturation.
class Pid {
 public:
  explicit Pid(const PidGains& gains);

  // Returns the control output. dt is in seconds; dt <= 0 skips the
  // integral and derivative update (P + stored I is returned).
  [[nodiscard]] double update(double setpoint, double measurement, double dt);

  void reset();

  [[nodiscard]] const PidGains& gains() const { return gains_; }
  [[nodiscard]] double integral_term() const { return integral_term_; }

 private:
  PidGains gains_;
  double integral_term_{0.0};
  double prev_measurement_{0.0};
  double prev_error_{0.0};
  bool has_prev_sample_{false};
};

}  // namespace gnc::control
