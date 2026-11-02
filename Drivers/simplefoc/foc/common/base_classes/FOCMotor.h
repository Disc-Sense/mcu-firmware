#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include <cstdint>

#include "Sensor.h"

#include "../defaults.h"
#include "../foc_utils.h"
#include "../lowpass_filter.h"
#include "../pid.h"
#include "../time_utils.h"

enum MotionControlType : uint8_t {
  torque = 0x00,
  velocity = 0x01,
  angle = 0x02,
  velocity_openloop = 0x03,
  angle_openloop = 0x04,
};

enum TorqueControlType : uint8_t {
  voltage = 0x00,
  dc_current = 0x01,
  foc_current = 0x02,
  estimated_current = 0x03
};

enum FOCModulationType : uint8_t { SinePWM = 0x00 };

enum FOCMotorStatus : uint8_t {
  motor_uninitialized = 0x00,
  motor_initializing = 0x01,
  motor_uncalibrated = 0x02,
  motor_calibrating = 0x03,
  motor_ready = 0x04,
  motor_error = 0x08,
  motor_calib_failed = 0x0E,
  motor_init_failed = 0x0F,
};

class FOCMotor {
public:
  FOCMotor();

  virtual int init() = 0;
  virtual void disable() = 0;
  virtual void enable() = 0;

  virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;

  virtual float estimateBEMF(float velocity) { return 0.0f; };

  virtual int initFOC();

  virtual void loopFOC();

  virtual void move(float target = NOT_SET);

  void linkSensor(Sensor *sensor);

  float shaftAngle();

  float shaftVelocity();

  float electricalAngle();

  int characteriseMotor(float voltage, float correction_factor);

  int tuneCurrentController(float bandwidth);

  float target;
  float feed_forward_velocity = 0.0f;
  float shaft_angle;
  float electrical_angle;
  float shaft_velocity;
  float current_sp;
  float shaft_velocity_sp;
  float shaft_angle_sp;
  DQVoltage_s voltage;
  DQVoltage_s current;
  float voltage_bemf;
  float Ualpha, Ubeta;

  DQCurrent_s feed_forward_current;
  DQVoltage_s feed_forward_voltage;

  float voltage_sensor_align;
  float velocity_index_search;

  float phase_resistance;
  int pole_pairs;
  float KV_rating;
  float phase_inductance;
  DQ_s axis_inductance{NOT_SET, NOT_SET};

  float voltage_limit;
  float current_limit;
  float velocity_limit;

  int8_t enabled = 0;
  FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized;

  FOCModulationType foc_modulation;
  int8_t modulation_centered = 1;

  TorqueControlType torque_controller;
  MotionControlType controller;

  PIDController PID_current_q{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D,
                              DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
  PIDController PID_current_d{DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D,
                              DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};
  LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};
  LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};
  PIDController PID_velocity{DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D,
                             DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT};
  PIDController P_angle{DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM};
  LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};
  LowPassFilter LPF_angle{0.0};
  unsigned int motion_downsample = DEF_MOTION_DOWNSAMPLE;
  unsigned int motion_cnt = 0;

  float sensor_offset;
  float zero_electric_angle = NOT_SET;
  Direction sensor_direction = Direction::UNKNOWN;
  bool pp_check_result = false;

  Sensor *sensor;

  uint32_t loopfoc_time_us = 0;
  uint32_t move_time_us = 0;

  void updateVelocityLimit(float new_velocity_limit);

  void updateCurrentLimit(float new_current_limit);

  void updateVoltageLimit(float new_voltage_limit);

  void updateTorqueControlType(TorqueControlType new_torque_controller);

  void updateMotionControlType(MotionControlType new_motion_controller);

  float velocityOpenLoop(float target_velocity);

  float angleOpenLoop(float target_angle);

protected:
  void updateLoopFOCTime() {
    updateTime(loopfoc_time_us, last_loopfoc_time_us,
               last_loopfoc_timestamp_us);
  }

  void updateMotionControlTime() {
    updateTime(move_time_us, last_move_time_us, last_move_timestamp_us);
  }

  int alignSensor();
  int alignCurrentSense();
  int absoluteZeroSearch();

  uint32_t last_loopfoc_timestamp_us = 0;
  uint32_t last_loopfoc_time_us = 0;
  uint32_t last_move_timestamp_us = 0;
  uint32_t last_move_time_us = 0;

private:
  void updateTime(uint32_t &elapsed_time_filtered, uint32_t &elapsed_time,
                  uint32_t &last_timestamp_us, float alpha = 0.1f) {
    uint32_t now = _micros();
    elapsed_time = now - last_timestamp_us;
    elapsed_time_filtered =
        (1 - alpha) * elapsed_time_filtered + alpha * elapsed_time;
    last_timestamp_us = now;
  }

  uint32_t open_loop_timestamp;
};

#endif /* FOCMOTOR_H */