#pragma once

#include <limits>

namespace gnc::control {

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  // Bounds on the integral term contribution (ki * integral), anti-windup.
  double integral_limit{std::numeric_limits<double>::infinity()};
  // Symmetric bound on the controller output.
  double output_limit{std::numeric_limits<double>::infinity()};
};

// Single-axis PID controller.
//
// - Derivative acts on the measurement, not on the error, so a step in the
//   setpoint does not produce a derivative "kick".
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
  bool has_prev_measurement_{false};
};

}  // namespace gnc::control
