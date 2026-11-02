#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit,
                             float sampling_time)
    : P(P), I(I), D(D), output_ramp(ramp), limit(limit), Ts(sampling_time),
      error_prev(0.0f), output_prev(0.0f), integral_prev(0.0) {
  timestamp_prev = _micros();
}

float PIDController::operator()(float error) {
  float dt = Ts;
  if (!_isset(dt)) {
    unsigned long timestamp_now = _micros();
    dt = (timestamp_now - timestamp_prev) * 1e-6f;
    if (dt <= 0 || dt > 0.5f)
      dt = 1e-3f;
    timestamp_prev = timestamp_now;
  }

  float proportional = P * error;
  float integral = integral_prev + I * dt * 0.5f * (error + error_prev);
  if (_isset(limit))
    integral = _constrain(integral, -limit, limit);
  float derivative = D * (error - error_prev) / dt;

  float output = proportional + integral + derivative;
  if (_isset(limit))
    output = _constrain(output, -limit, limit);

  if (_isset(output_ramp) && output_ramp > 0) {
    float output_rate = (output - output_prev) / dt;
    if (output_rate > output_ramp) {
      output = output_prev + output_ramp * dt;
    } else if (output_rate < -output_ramp) {
      output = output_prev - output_ramp * dt;
    }
  }

  integral_prev = integral;
  output_prev = output;
  error_prev = error;
  return output;
}

void PIDController::reset() {
  integral_prev = 0.0f;
  output_prev = 0.0f;
  error_prev = 0.0f;
}