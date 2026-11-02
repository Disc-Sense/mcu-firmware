# Bugs

- C++ function mangling prevented ISRs in `stm32g0xx_it.cpp` from linking correctly. Adding `extern "C"` fixes this and allows interrupts to execute as expected.

- FOC loop runs too slowly at default system clock frequency (16 MHz). Reconfiguring sysclk source to PLLCLK at 64 MHz allows for higher speeds and lower torque ripple.

- CubeMX shows PLL values in clock configuration but doesn't actually configure the PLL when it isn't used as a clock source. This caused `HAL_RCC_ClockConfig` to error. Enabling and configuring the PLL in `RCC_OscInitStruct` fixed the issue.

- At higher clock speeds, `HAL_RCC_ClockConfig(..., FLASH_LATENCY_0)` fails. 64 MHz requires more flash wait cycles (`FLASH_LATENCY_2`).

- Motor surges briefly when the 16-bit encoder timer overflows
