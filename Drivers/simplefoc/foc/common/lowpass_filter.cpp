#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant, float sampling_time)
    : Tf(time_constant), Ts(sampling_time), y_prev(0.0f) {
  timestamp_prev = _micros();
}

float LowPassFilter::operator()(float x) {
  float dt = Ts;

  if (!_isset(dt)) {
    unsigned long timestamp = _micros();
    dt = (timestamp - timestamp_prev) * 1e-6f;

    if (dt < 0.0f)
      dt = 1e-3f;
    else if (dt > 0.3f) {
      y_prev = x;
      timestamp_prev = timestamp;
      return x;
    }

    timestamp_prev = timestamp;
  }

  float alpha = Tf / (Tf + dt);
  float y = alpha * y_prev + (1.0f - alpha) * x;

  y_prev = y;
  return y;
}