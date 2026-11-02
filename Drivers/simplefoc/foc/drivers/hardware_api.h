#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif

#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true
#endif

#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true
#endif

#define SIMPLEFOC_DRIVER_INIT_FAILED ((void *)-1)

typedef struct GenericDriverParams {
  int pins[6];
  long pwm_frequency;
  float dead_zone;
} GenericDriverParams;

void *_configure3PWM(long pwm_frequency, const int pinA, const int pinB,
                     const int pinC);

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params);

#endif /* HARDWARE_UTILS_DRIVER_H */