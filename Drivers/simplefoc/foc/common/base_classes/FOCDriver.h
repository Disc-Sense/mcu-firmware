#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include <cstdint>

enum PhaseState : uint8_t {
  PHASE_OFF = 0,
  PHASE_ON = 1,
  PHASE_HI = 2,
  PHASE_LO = 3
};

enum DriverType {
  UnknownDriver = 0,
  BLDC = 1,
};

class FOCDriver {
public:
  virtual int init() = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;

  long pwm_frequency;
  float voltage_power_supply;
  float voltage_limit;

  bool initialized = false;
  void *params = 0;

  bool enable_active_high = true;

  virtual DriverType type() = 0;
};

#endif /* FOCDRIVER_H */