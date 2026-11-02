#ifndef SIMPLEFOC_STM32_I2C_H
#define SIMPLEFOC_STM32_I2C_H

#include <cstdint>

void i2c_write(uint8_t address, uint8_t byte);

void i2c_read(uint8_t address, uint8_t *pData, uint16_t size);

#endif /* SIMPLEFOC_STM32_I2C_H */