#ifndef BLDCDRIVER3PWM_H
#define BLDCDRIVER3PWM_H

#include "../common/base_classes/BLDCDriver.h"
#include "../common/defaults.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "hardware_api.h"

class BLDCDriver3PWM : public BLDCDriver {
public:
  BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET,
                 int en2 = NOT_SET, int en3 = NOT_SET);

  int init() override;
  void disable() override;
  void enable() override;

  int pwmA;
  int pwmB;
  int pwmC;
  int enableA_pin;
  int enableB_pin;
  int enableC_pin;

  void setPwm(float Ua, float Ub, float Uc) override;

  virtual void setPhaseState(PhaseState sa, PhaseState sb,
                             PhaseState sc) override;

private:
};

#endif /* BLDCDRIVERPWM_H */