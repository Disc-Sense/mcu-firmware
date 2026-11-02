#include "BLDCMotor.h"

BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _Lq, float _Ld)
    : FOCMotor() {
  pole_pairs = pp;
  phase_resistance = _R;
  KV_rating = _KV;
  axis_inductance = {_Ld, _Lq};
  phase_inductance = _Lq;
  torque_controller = TorqueControlType::voltage;
}

void BLDCMotor::linkDriver(BLDCDriver *_driver) { driver = _driver; }

int BLDCMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    return 0;
  }

  motor_status = FOCMotorStatus::motor_initializing;

  if (voltage_limit > driver->voltage_limit)
    voltage_limit = driver->voltage_limit;
  if (voltage_sensor_align > voltage_limit)
    voltage_sensor_align = voltage_limit;

  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
  updateVelocityLimit(velocity_limit);

  if (_isset(phase_inductance) && !(_isset(axis_inductance.q))) {
    axis_inductance = {phase_inductance, phase_inductance};
  }

  if (!sensor) {
    if ((controller == MotionControlType::angle_openloop ||
         controller == MotionControlType::velocity_openloop) &&
        (sensor_direction == Direction::UNKNOWN)) {
      sensor_direction = Direction::CW;
    }
  }

  _delay(500);
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}

void BLDCMotor::disable() {
  driver->setPwm(0, 0, 0);
  driver->disable();
  enabled = 0;
}

void BLDCMotor::enable() {
  driver->enable();
  driver->setPwm(0, 0, 0);
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  enabled = 1;
}

float BLDCMotor::estimateBEMF(float vel) {
  return vel / (KV_rating * _SQRT3) / _RPM_TO_RADS;
}

void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
  float center;
  int sector;
  float _ca, _sa;

  switch (foc_modulation) {
  case FOCModulationType::SinePWM:
    _sincos(angle_el, &_sa, &_ca);

    Ualpha = _ca * Ud - _sa * Uq;
    Ubeta = _sa * Ud + _ca * Uq;

    Ua = Ualpha;
    Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
    Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

    if (modulation_centered) {
      center = driver->voltage_limit / 2;
      Ua += center;
      Ub += center;
      Uc += center;
    } else {
      float Umin = foc_min(Ua, foc_min(Ub, Uc));
      Ua -= Umin;
      Ub -= Umin;
      Uc -= Umin;
    }
  }

  driver->setPwm(Ua, Ub, Uc);
}