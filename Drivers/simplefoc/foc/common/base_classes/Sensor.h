#ifndef SENSOR_H
#define SENSOR_H

#include <inttypes.h>

enum Direction : int8_t { CW = 1, CCW = -1, UNKNOWN = 0 };

enum Pullup : uint8_t { USE_INTERN = 0x00, USE_EXTERN = 0x01 };

class Sensor {
public:
  virtual float getMechanicalAngle();

  virtual float getAngle();

  virtual double getPreciseAngle();

  virtual float getVelocity();

  virtual int32_t getFullRotations();

  virtual void update();

  virtual int needsSearch();

  float min_elapsed_time = 0.000100;

protected:
  virtual float getSensorAngle() = 0;

  virtual void init();

  float velocity = 0.0f;
  float angle_prev = 0.0f;
  long angle_prev_ts = 0;
  float vel_angle_prev = 0.0f;
  long vel_angle_prev_ts = 0;
  int32_t full_rotations = 0;
  int32_t vel_full_rotations = 0;
};

#endif /* SENSOR_H */