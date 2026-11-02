#include "BLDCDriver3PWM.h"

#include "hardware_specific/stm32_gpio.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2,
                               int en3) {
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;
}

void BLDCDriver3PWM::enable() {
  if (_isset(enableA_pin))
    digitalWrite(enableA_pin, enable_active_high);
  if (_isset(enableB_pin))
    digitalWrite(enableB_pin, enable_active_high);
  if (_isset(enableC_pin))
    digitalWrite(enableC_pin, enable_active_high);

  setPwm(0, 0, 0);
}

void BLDCDriver3PWM::disable() {
  setPwm(0, 0, 0);
  if (_isset(enableA_pin))
    digitalWrite(enableA_pin, !enable_active_high);
  if (_isset(enableB_pin))
    digitalWrite(enableB_pin, !enable_active_high);
  if (_isset(enableC_pin))
    digitalWrite(enableC_pin, !enable_active_high);
}

int BLDCDriver3PWM::init() {
  // CubeMX already configures pins

  if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
  return params != SIMPLEFOC_DRIVER_INIT_FAILED;
}

void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb,
                                   PhaseState sc) {
  if (_isset(enableA_pin) && _isset(enableB_pin) && _isset(enableC_pin)) {
    digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high
                                                         : !enable_active_high);
    digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high
                                                         : !enable_active_high);
    digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high
                                                         : !enable_active_high);
  }
}

void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);

  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}