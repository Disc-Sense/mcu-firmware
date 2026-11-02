#ifndef MAGNETICSENSORI2C_H
#define MAGNETICSENSORI2C_H

#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

struct MagneticSensorI2CConfig_s {
  uint8_t chip_address;
  uint8_t bit_resolution;
  uint8_t angle_register;
  uint8_t msb_mask;
  uint8_t msb_shift;
  uint8_t lsb_mask;
  uint8_t lsb_shift;
};

extern MagneticSensorI2CConfig_s MT6701_I2C;

class MagneticSensorI2C : public Sensor {
public:
  MagneticSensorI2C(uint8_t _chip_address, int _bit_resolution,
                    uint8_t _angle_register_msb, int _msb_bits_used,
                    bool lsb_right_aligned = true);

  MagneticSensorI2C(MagneticSensorI2CConfig_s config);

  void init();

  float getSensorAngle() override;

private:
  float cpr;
  MagneticSensorI2CConfig_s _conf;

  int read(uint8_t angle_register_msb);

  int getRawCount();
};

#endif /* MAGNETICSENSORI2C_H */