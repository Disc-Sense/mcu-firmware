# Disc-Sense FOC PoC

This repository contains firmware for the disc-sense gimbal proof-of-concept for the STM32G01BRE Nucleo.
Simplefoc is configured for the 4015 BLDC motor and MT6700 (ABZ output) with a 12V power supply.

See `BUGS.md` for a list of bugs encountered during testing.

## Building and Running

Building the firmware requires `arm-none-eabi-binutils`, `cmake`, and a programming/debugging tool.

1. Open the `mcu-firmware` directory

2. Run `cmake --preset Debug`

3. Run `cmake --build --preset Debug`

4. Program the generated `build/Debug/disc-sense-mcu.bin` file with CubeMXProgrammer or a compatible STLink programming tool

## Disclaimer

Significant portions of the FOC (located in `Drivers/simplefoc`) are derived from [simplefoc/Arduino-FOC](https://github.com/simplefoc/Arduino-FOC).
The relevant LICENSE.md files are provided, and all use is permitted by the Arduino-FOC MIT license.
