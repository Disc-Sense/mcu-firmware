#include "MagneticSensorI2C.h"

#include "stm32_i2c.h"

MagneticSensorI2CConfig_s MT6701_I2C = {.chip_address = 0x06,
                                        .bit_resolution = 14,
                                        .angle_register = 0x03,
                                        .msb_mask = 0xFF,
                                        .msb_shift = 6,
                                        .lsb_mask = 0xFC,
                                        .lsb_shift = 2};

MagneticSensorI2C::MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution,
                                     uint8_t _angle_register_msb,
                                     int _bits_used_msb,
                                     bool lsb_right_aligned) {
  _conf.chip_address = _chip_address;
  _conf.bit_resolution = _bit_resolution;
  _conf.angle_register = _angle_register_msb;
  _conf.msb_mask = (uint8_t)((1 << _bits_used_msb) - 1);

  uint8_t lsb_used = _bit_resolution - _bits_used_msb;
  _conf.lsb_mask = (uint8_t)((1 << (lsb_used)) - 1);
  if (!lsb_right_aligned) {
    _conf.lsb_shift = 8 - lsb_used;
  } else {
    _conf.lsb_shift = 0;
  }
  _conf.msb_shift = lsb_used;

  cpr = _powtwo(_bit_resolution);
}

MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config) {
  _conf = config;
  cpr = _powtwo(config.bit_resolution);
}

void MagneticSensorI2C::init() { this->Sensor::init(); }

float MagneticSensorI2C::getSensorAngle() {
  return (getRawCount() / (float)cpr * _2PI);
}

int MagneticSensorI2C::getRawCount() {
  uint8_t readArray[2];
  uint16_t readValue = 0;

  i2c_write(_conf.chip_address, _conf.angle_register);
  i2c_read(_conf.chip_address, readArray, 2);

  readValue = (readArray[0] & _conf.msb_mask) << _conf.msb_shift;
  readValue |= (readArray[1] & _conf.lsb_mask) >> _conf.lsb_shift;
  return readValue;
}