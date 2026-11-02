#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H

#include "common/base_classes/BLDCDriver.h"
#include "common/base_classes/FOCDriver.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/Sensor.h"
#include "common/defaults.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

class BLDCMotor : public FOCMotor {
public:
  BLDCMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L_q = NOT_SET,
            float L_d = NOT_SET);

  virtual void linkDriver(BLDCDriver *driver);

  float Ua, Ub, Uc;

  BLDCDriver *driver;

  int init() override;
  void disable() override;
  void enable() override;

  void setPhaseVoltage(float Uq, float Ud, float angle_el) override;

  float estimateBEMF(float velocity) override;

  int characteriseMotor(float voltage) {
    return FOCMotor::characteriseMotor(voltage, 1.5f);
  }
};

#endif /* BLDCMOTOR_H */