#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "foc_utils.h"
#include "time_utils.h"

class LowPassFilter {
public:
  LowPassFilter(float Tf, float Ts = NOT_SET);
  ~LowPassFilter() = default;

  float operator()(float x);
  float Tf;
  float Ts;

protected:
  unsigned long timestamp_prev;
  float y_prev;
};

#endif /* LOWPASS_FILTER_H */