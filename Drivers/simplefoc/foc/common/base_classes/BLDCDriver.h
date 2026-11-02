#ifndef BLDCDRIVER_H
#define BLDCDRIVER_H

#include "FOCDriver.h"

class BLDCDriver : public FOCDriver {
public:
  float dc_a;
  float dc_b;
  float dc_c;

  virtual void setPwm(float Ua, float Ub, float Uc) = 0;

  virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;

  virtual DriverType type() override { return DriverType::BLDC; };
};

#endif /* BLDCDRIVER_H */