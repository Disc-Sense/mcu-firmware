#include "FOCMotor.h"

#include <cmath>

FOCMotor::FOCMotor() {
  velocity_limit = DEF_VEL_LIM;
  voltage_limit = DEF_POWER_SUPPLY;
  current_limit = DEF_CURRENT_LIM;

  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  foc_modulation = FOCModulationType::SinePWM;

  target = 0;
  voltage.d = 0;
  voltage.q = 0;

  current_sp = 0;
  current.q = 0;
  current.d = 0;

  voltage_bemf = 0;

  Ualpha = 0;
  Ubeta = 0;

  sensor_offset = 0.0f;
  sensor = nullptr;
}

void FOCMotor::linkSensor(Sensor *_sensor) { sensor = _sensor; }

float FOCMotor::shaftAngle() {
  if (!sensor)
    return shaft_angle;
  return (int8_t)sensor_direction * sensor->getAngle() - sensor_offset;
}

float FOCMotor::shaftVelocity() {
  if (!sensor)
    return shaft_velocity;
  return (int8_t)sensor_direction * LPF_velocity(sensor->getVelocity());
}

float FOCMotor::electricalAngle() {
  if (!sensor)
    return electrical_angle;
  return _normalizeAngle((float)(sensor_direction * pole_pairs) *
                             sensor->getMechanicalAngle() -
                         zero_electric_angle);
}

int FOCMotor::characteriseMotor(float voltage, float correction_factor = 1.0f) {
  // no current sense yet
  return 1;
}

float FOCMotor::velocityOpenLoop(float target_velocity) {
  unsigned long now_us = _micros();
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  shaft_velocity = target_velocity;

  open_loop_timestamp = now_us;

  if (torque_controller == TorqueControlType::voltage) {
    return voltage_limit;
  } else {
    return current_limit;
  }
}

float FOCMotor::angleOpenLoop(float target_angle) {
  unsigned long now_us = _micros();
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  if (fabsf(target_angle - shaft_angle) > fabsf(velocity_limit * Ts)) {
    shaft_angle +=
        _sign(target_angle - shaft_angle) * fabsf(velocity_limit) * Ts;
    shaft_velocity = velocity_limit;
  } else {
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  open_loop_timestamp = now_us;

  if (torque_controller == TorqueControlType::voltage) {
    return voltage_limit;
  } else {
    return current_limit;
  }
}

void FOCMotor::updateVelocityLimit(float new_velocity_limit) {
  velocity_limit = new_velocity_limit;
  P_angle.limit = fabsf(velocity_limit);
}

void FOCMotor::updateCurrentLimit(float new_current_limit) {
  current_limit = new_current_limit;
  if (torque_controller != TorqueControlType::voltage) {
    PID_velocity.limit = new_current_limit;
  }
}

void FOCMotor::updateVoltageLimit(float new_voltage_limit) {
  voltage_limit = new_voltage_limit;
  PID_current_q.limit = new_voltage_limit;
  PID_current_d.limit = new_voltage_limit;
  if (torque_controller == TorqueControlType::voltage) {
    PID_velocity.limit = new_voltage_limit;
  }
}

void FOCMotor::updateTorqueControlType(
    TorqueControlType new_torque_controller) {
  torque_controller = new_torque_controller;
  if (torque_controller == TorqueControlType::voltage) {
    updateVoltageLimit(voltage_limit);
  } else {
    updateCurrentLimit(current_limit);
  }
}

void FOCMotor::updateMotionControlType(
    MotionControlType new_motion_controller) {
  if (controller == new_motion_controller)
    return;

  switch (new_motion_controller) {
  case MotionControlType::angle:
    if (controller == MotionControlType::angle_openloop)
      break;
  case (MotionControlType::angle_openloop):
    if (controller == MotionControlType::angle)
      break;
    target = shaft_angle;
    break;
  case MotionControlType::velocity:
    if (controller == MotionControlType::velocity_openloop)
      break;
  case MotionControlType::velocity_openloop:
    if (controller == MotionControlType::velocity)
      break;
    target = 0;
    break;
  case MotionControlType::torque:
    target = 0;
    break;
  default:
    target = 0;
    break;
  }

  controller = new_motion_controller;
  updateVelocityLimit(velocity_limit);
  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
}

int FOCMotor::tuneCurrentController(float bandwidth) {
  if (bandwidth <= 0.0f) {
    return 1;
  }

  if (loopfoc_time_us && bandwidth > 0.5f * (1e6f / loopfoc_time_us)) {
    return 2;
  }

  if (!_isset(phase_resistance) ||
      (!_isset(phase_inductance) && !_isset(axis_inductance.q))) {
    return 3;
  } else if (_isset(phase_inductance) && !(_isset(axis_inductance.q))) {
    axis_inductance = {phase_inductance, phase_inductance};
  }

  PID_current_q.P = axis_inductance.q * (_2PI * bandwidth);
  PID_current_q.I = phase_resistance * (_2PI * bandwidth);
  PID_current_d.P = axis_inductance.d * (_2PI * bandwidth);
  PID_current_d.I = phase_resistance * (_2PI * bandwidth);
  LPF_current_d.Tf = 1.0f / (_2PI * bandwidth * 5.0f);
  LPF_current_q.Tf = 1.0f / (_2PI * bandwidth * 5.0f);

  return 0;
}

void FOCMotor::loopFOC() {
  updateLoopFOCTime();

  if (motor_status == FOCMotorStatus::motor_calibrating)
    return;

  if (sensor)
    sensor->update();

  if (!enabled)
    return;

  if (controller == MotionControlType::angle_openloop ||
      controller == MotionControlType::velocity_openloop) {
    electrical_angle = _electricalAngle((shaft_angle), pole_pairs);
  } else {
    electrical_angle = electricalAngle();
  }

  switch (torque_controller) {
  case TorqueControlType::voltage:
    voltage.q = _constrain(current_sp, -voltage_limit, voltage_limit) +
                feed_forward_voltage.q;
    voltage.d = feed_forward_voltage.d;
    break;
  case TorqueControlType::estimated_current:
    if (!_isset(phase_resistance))
      return;
    current_sp = _constrain(current_sp, -current_limit, current_limit) +
                 feed_forward_current.q;
    if (_isset(KV_rating))
      voltage_bemf = estimateBEMF(shaft_velocity);
    current.q = LPF_current_q(current_sp);
    voltage.q = current.q * phase_resistance + voltage_bemf;
    voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit) +
                feed_forward_voltage.q;

    if (_isset(axis_inductance.q))
      voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs *
                                 axis_inductance.q,
                             -voltage_limit, voltage_limit) +
                  feed_forward_voltage.d;
    else
      voltage.d = feed_forward_voltage.d;
    break;
  case TorqueControlType::dc_current:
    return;
  case TorqueControlType::foc_current:
    return;
  default:
    break;
  }

  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

void FOCMotor::move(float new_target) {
  if (_isset(new_target))
    target = new_target;

  if (motion_cnt++ < motion_downsample)
    return;
  motion_cnt = 0;

  if (motor_status == FOCMotorStatus::motor_calibrating)
    return;

  updateMotionControlTime();

  if (controller != MotionControlType::angle_openloop &&
      controller != MotionControlType::velocity_openloop) {
    shaft_angle = shaftAngle();
    shaft_velocity = shaftVelocity();
  }

  if (!enabled)
    return;

  switch (controller) {
  case MotionControlType::torque:
    current_sp = target;
    break;
  case MotionControlType::angle:
    shaft_angle_sp = target;
    shaft_velocity_sp = feed_forward_velocity +
                        P_angle(shaft_angle_sp - LPF_angle(shaft_angle));
    shaft_velocity_sp =
        _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
    break;
  case MotionControlType::velocity:
    shaft_velocity_sp = target;
    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
    break;
  case MotionControlType::velocity_openloop:
    shaft_velocity_sp = target;
    current_sp = velocityOpenLoop(shaft_velocity_sp);
    break;
  case MotionControlType::angle_openloop:
    shaft_angle_sp = target;
    current_sp = angleOpenLoop(shaft_angle_sp);
    break;
  }
}

extern void debug_print(const char *s);

int FOCMotor::initFOC() {
  int exit_flag = 1;

  motor_status = FOCMotorStatus::motor_calibrating;

  if (sensor) {
    debug_print("Align sensor\n");
    exit_flag *= alignSensor();
    debug_print("Update sensor\n");
    sensor->update();
    debug_print("Shaft angle\n");
    shaft_angle = shaftAngle();
  } else {
    if ((controller == MotionControlType::angle_openloop ||
         controller == MotionControlType::velocity_openloop)) {
      exit_flag = 1;
    } else {
      exit_flag = 0;
    }
  }

  if (exit_flag) {
    // no current sense
  }

  if (exit_flag) {
    debug_print("Motor ready\n");
    motor_status = FOCMotorStatus::motor_ready;
  } else {
    debug_print("Calibration failed\n");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}

int FOCMotor::alignSensor() {
  int exit_flag = 1;

  if (sensor->needsSearch())
    exit_flag = absoluteZeroSearch();
  if (!exit_flag)
    return exit_flag;

  float voltage_align = voltage_sensor_align;

  if (sensor_direction == Direction::UNKNOWN) {
    for (int i = 0; i <= 500; i++) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      sensor->update();
      _delay(2);
    }

    sensor->update();
    float mid_angle = sensor->getAngle();
    for (int i = 500; i >= 0; i--) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      sensor->update();
      _delay(2);
    }

    sensor->update();
    float end_angle = sensor->getAngle();
    _delay(200);

    float moved = fabs(mid_angle - end_angle);
    if (moved < MIN_ANGLE_DETECT_MOVEMENT) {
      return 0; // failed calib
    } else if (mid_angle < end_angle) {
      sensor_direction = Direction::CCW;
    } else {
      sensor_direction = Direction::CW;
    }

    pp_check_result = !(fabs(moved * pole_pairs - _2PI) > 0.5f);
    if (pp_check_result == false) {
      // PP check failed
    } else {
      // PP check OK
    }
  } else {
    // skip dir calib
  }

  if (!_isset(zero_electric_angle)) {
    setPhaseVoltage(voltage_align, 0, _3PI_2);
    _delay(700);
    sensor->update();
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    _delay(20);
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else {
    // skip offset calib
  }

  return exit_flag;
}

int FOCMotor::absoluteZeroSearch() {
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while (sensor->needsSearch() && shaft_angle < _2PI) {
    angleOpenLoop(1.5f * _2PI);
    sensor->update();
  }

  setPhaseVoltage(0, 0, 0);
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;

  return !sensor->needsSearch();
}