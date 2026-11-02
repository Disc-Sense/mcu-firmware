#include "MT6701.h"
#include <cmath>

MT6701_I2C::MT6701_I2C(I2C_HandleTypeDef *hi2c) { this->hi2c = hi2c; }

HAL_StatusTypeDef MT6701_I2C::init() {
  return HAL_I2C_IsDeviceReady(this->hi2c, MT6701_I2C::I2C_ADDR, 10, 500);
}

float MT6701_I2C::getSensorAngle() {
  float rawAngle = this->readRawAngle();
  return M_TWOPI * (rawAngle / MT6701_I2C::CPR);
}

uint16_t MT6701_I2C::readRawAngle() {
  uint8_t low = 0;
  uint8_t high = 0;
  HAL_I2C_Mem_Read(this->hi2c, MT6701_I2C::I2C_ADDR,
                   static_cast<uint16_t>(MT6701_Register::ANGLE_HIGH),
                   I2C_MEMADD_SIZE_8BIT, &high, 1, 1);
  HAL_I2C_Mem_Read(this->hi2c, MT6701_I2C::I2C_ADDR,
                   static_cast<uint16_t>(MT6701_Register::ANGLE_LOW),
                   I2C_MEMADD_SIZE_8BIT, &low, 1, 1);
  return (static_cast<uint16_t>(high) << 8) | (low & 0xFC);
}