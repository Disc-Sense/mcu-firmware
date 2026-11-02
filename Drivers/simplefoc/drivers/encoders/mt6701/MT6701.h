#ifndef MT6701_H
#define MT6701_H

#include "stm32g0xx_hal.h"

#include <cstdint>

enum MT6701_Register : uint8_t {
  ANGLE_HIGH = 0x03,
  ANGLE_LOW = 0x04,
};

class MT6701_I2C {
  I2C_HandleTypeDef *hi2c;

public:
  static constexpr uint8_t I2C_ADDR = (0b0000110 << 1);
  static constexpr float CPR = 16384.0f;

  MT6701_I2C(I2C_HandleTypeDef *hi2c);

  HAL_StatusTypeDef init();

  float getSensorAngle();

protected:
  uint16_t readRawAngle();
};

#endif /* MT6701_H */
