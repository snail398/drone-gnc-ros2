#include "gnc_core/control/pid.hpp"

#include <algorithm>

namespace gnc::control {

Pid::Pid(const PidGains& gains) : gains_(gains) {}

double Pid::update(double setpoint, double measurement, double dt) {
  const double error = setpoint - measurement;
  const double p_term = gains_.kp * error;
  const double limit = gains_.output_limit;

  if (dt <= 0.0) {
    return std::clamp(p_term + integral_term_, -limit, limit);
  }

  double d_term = 0.0;
  if (has_prev_sample_) {
    d_term = gains_.derivative_on == DerivativeOn::kError
                 ? gains_.kd * (error - prev_error_) / dt
                 : -gains_.kd * (measurement - prev_measurement_) / dt;
  }
  prev_measurement_ = measurement;
  prev_error_ = error;
  has_prev_sample_ = true;

  // Conditional integration: do not grow the integral further into saturation.
  const double unsaturated = p_term + integral_term_ + d_term;
  const bool saturated_high = unsaturated >= limit && error > 0.0;
  const bool saturated_low = unsaturated <= -limit && error < 0.0;
  if (!saturated_high && !saturated_low) {
    integral_term_ += gains_.ki * error * dt;
    integral_term_ = std::clamp(integral_term_, -gains_.integral_limit, gains_.integral_limit);
  }

  return std::clamp(p_term + integral_term_ + d_term, -limit, limit);
}

void Pid::reset() {
  integral_term_ = 0.0;
  prev_measurement_ = 0.0;
  prev_error_ = 0.0;
  has_prev_sample_ = false;
}

}  // namespace gnc::control
