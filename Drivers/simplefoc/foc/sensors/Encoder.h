#ifndef FOCENCODER_H
#define FOCENCODER_H

#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

class Encoder : public Sensor {
public:
  Encoder(int encA, int encB, float ppr, int index = 0);

  void init() override;

  void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr,
                        void(doIndex)() = nullptr);

  void handleA();
  void handleB();
  void handleIndex();

  int pinA;
  int pinB;
  int index_pin;

  float cpr;

  float getSensorAngle() override;
  float getVelocity() override;
  virtual void update() override;

  int needsSearch() override;

private:
  int hasIndex();

  volatile long pulse_counter;
  volatile long pulse_timestamp;
  volatile int A_active;
  volatile int B_active;
  volatile int I_active;
  volatile bool index_found = false;

  float prev_Th, pulse_per_second;
  volatile long prev_pulse_counter, prev_timestamp_us;
};

#endif /* FOC_ENCODER_H */