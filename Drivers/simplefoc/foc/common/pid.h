#ifndef PID_H
#define PID_H

#include "foc_utils.h"
#include "time_utils.h"

class PIDController {
public:
  PIDController(float P, float I, float D, float ramp = NOT_SET,
                float limit = NOT_SET, float sampling_time = NOT_SET);
  ~PIDController() = default;

  float operator()(float error);
  void reset();

  float P;
  float I;
  float D;
  float output_ramp;
  float limit;
  float Ts;

protected:
  float error_prev;
  float output_prev;
  float integral_prev;
  unsigned long timestamp_prev;
};

#endif /* PID_H */