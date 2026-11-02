#ifndef FOCUTILS_H
#define FOCUTILS_H

#include <cstdint>

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#ifndef _round
#define _round(x) ((x) >= 0 ? (long)((x) + 0.05f) : (long)((x) - 0.5f))
#endif
#define _constrain(amt, low, high)                                             \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ((a) != (NOT_SET))
#define _UNUSED(v) (void)(v)
#define _powtwo(x) (1 << (x))

#ifndef foc_min
#define foc_min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef foc_max
#define foc_max(a, b) ((a) > (b) ? (a) : (b))
#endif

#define _swap(a, b)                                                            \
  {                                                                            \
    auto temp = a;                                                             \
    a = b;                                                                     \
    b = temp;                                                                  \
  }

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC ((int)NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

struct DQ_s {
  float d;
  float q;
};

typedef DQ_s DQVoltage_s;
typedef DQ_s DQCurrent_s;

struct AB_s {
  float alpha;
  float beta;
};
typedef AB_s ABVoltage_s;
typedef AB_s ABCurrent_s;

struct Phase_s {
  float a;
  float b;
  float c;
};
typedef Phase_s PhaseVoltage_s;
typedef Phase_s PhaseCurrent_s;

float _sin(float a);

float _cos(float a);

void _sincos(float a, float *s, float *c);

float _atan2(float y, float x);

float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _sqrtApprox(float value);

#endif /* FOCUTILS_H */