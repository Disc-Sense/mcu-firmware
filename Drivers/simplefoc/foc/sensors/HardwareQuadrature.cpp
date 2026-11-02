#include "HardwareQuadrature.h"
#include <cmath>

HardwareQuadrature::HardwareQuadrature(float _ppr) {
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _ppr;

  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();
}

void HardwareQuadrature::handleUpdate(long _pulse_counter) {
  pulse_counter = _pulse_counter;
  pulse_timestamp = _micros();
}

void HardwareQuadrature::handleIndex() {
  if (index_found)
    return;
  index_found = true;
  long tmp = pulse_counter;
  pulse_counter = round((double)pulse_counter / (double)cpr) * cpr;
  prev_pulse_counter += pulse_counter - tmp;
}

void HardwareQuadrature::update() {
  angle_prev_ts = pulse_timestamp;
  long copy_pulse_counter = pulse_counter;

  full_rotations = copy_pulse_counter / (int)cpr;
  angle_prev = _2PI * ((copy_pulse_counter) % ((int)cpr)) / ((float)cpr);
}

float HardwareQuadrature::getSensorAngle() {
  return _2PI * (pulse_counter) / ((float)cpr);
}

float HardwareQuadrature::getVelocity() {
  long copy_pulse_counter = pulse_counter;
  long copy_pulse_timestamp = pulse_timestamp;

  long timestamp_us = _micros();
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  float Th = (timestamp_us - copy_pulse_timestamp) * 1e-6f;
  long dN = copy_pulse_counter - prev_pulse_counter;

  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts / 2) ? dN / dt : pulse_per_second;

  if (Th > 0.1f)
    pulse_per_second = 0;

  float velocity = pulse_per_second / ((float)cpr * _2PI);

  prev_timestamp_us = timestamp_us;
  prev_Th = Th;
  prev_pulse_counter = copy_pulse_counter;
  return velocity;
}

int HardwareQuadrature::needsSearch() { return hasIndex() && !index_found; }

int HardwareQuadrature::hasIndex() { return true; }

void HardwareQuadrature::init() {
  pulse_counter = 0;
  pulse_timestamp = _micros();
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  cpr = 4 * cpr;
}