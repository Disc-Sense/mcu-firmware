#ifndef SIMPLEFOC_STM32_H
#define SIMPLEFOC_STM32_H

#include <cstdint>

enum STM32Pin : uint8_t { D2 = 2, D7 = 7, D8 = 8 };

void digitalWrite(int pin, bool set);

#endif /* SIMPLEFOC_STM32_H */