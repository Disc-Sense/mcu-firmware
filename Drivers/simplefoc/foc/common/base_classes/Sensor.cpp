#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"

#include <cmath>

void Sensor::update() {
  float val = getSensorAngle();
  if (val < 0)
    return;
  angle_prev_ts = _micros();
  float d_angle = val - angle_prev;
  if (fabsf(d_angle) > (0.8f * _2PI)) {
    full_rotations += (d_angle > 0) ? -1 : 1;
  }

  angle_prev = val;
}

float Sensor::getVelocity() {
  float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
  if (Ts < 0.0f) {
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
  }
  if (Ts < min_elapsed_time)
    return velocity;

  float current_angle = 0.0f;
  float prev_angle = 0.0f;

  if (full_rotations == vel_full_rotations) {
    current_angle = angle_prev;
    prev_angle = vel_angle_prev;
  } else {
    current_angle = (float)full_rotations * _2PI + angle_prev;
    prev_angle = (float)vel_full_rotations * _2PI + vel_angle_prev;
  }
  const float delta_angle = current_angle - prev_angle;

  if (fabsf(delta_angle) > 1e-8f) {
    velocity = delta_angle / Ts;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
  }

  return velocity;
}

void Sensor::init() {
  getSensorAngle();
  _delayMicroseconds(1);
  vel_angle_prev = getSensorAngle();
  vel_angle_prev_ts = _micros();
  _delay(1);
  getSensorAngle();
  _delayMicroseconds(1);
  angle_prev = getSensorAngle();
  angle_prev_ts = _micros();
}

float Sensor::getMechanicalAngle() { return angle_prev; }

float Sensor::getAngle() { return (float)full_rotations * _2PI + angle_prev; }

double Sensor::getPreciseAngle() {
  return (double)full_rotations * (double)_2PI + (double)angle_prev;
}

int32_t Sensor::getFullRotations() { return full_rotations; }

int Sensor::needsSearch() { return 0; }