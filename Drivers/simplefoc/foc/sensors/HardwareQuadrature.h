#ifndef HARDWARE_QUADRATURE_H
#define HARDWARE_QUADRATURE_H

#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

class HardwareQuadrature : public Sensor {
public:
  HardwareQuadrature(float ppr);

  void init() override;

  void handleUpdate(long pulse_counter);
  void handleIndex();

  float cpr;

  float getSensorAngle() override;
  float getVelocity() override;
  virtual void update() override;

  int needsSearch() override;

private:
  int hasIndex();

  volatile long pulse_counter;
  volatile long pulse_timestamp;
  volatile bool index_found = false;

  float prev_Th, pulse_per_second;
  volatile long prev_pulse_counter, prev_timestamp_us;
};

#endif /* HARDWARE_QUADRATURE_H */