# Disc-Sense FOC

This repository contains firmware for the disc-sense gimbal proof-of-concept for the STM32G01BRE Nucleo.
Simplefoc is configured for the 4015 BLDC motor and MT6700 (ABZ output) with a 12V power supply.

See `BUGS.md` for a list of bugs encountered during testing.

## Design Details

This proof-of-concept uses field-oriented control (adapted from SimpleFOC) to control the angle, velocity, or torque of a 4015 BLDC gimbal motor. FOC replaces simpler trapezoidal or V/f control to achieve more precise control. Sinusoidal commutation minimizes torque ripple and maximizes efficiency by maintaining constant torque-producing current and reducing magnetizing current.

![motion control diagram](https://docs.simplefoc.com/extras/Images/a_b_i.drawio.png "Motion Control Diagram")
*Motion control diagram from [simplefoc](https://docs.simplefoc.com/motion_control_implementation).*

The FOC library, in position control, calculates phase voltages from the desired angle. It factors in angle error, velocity error, velocity feed-forward, current error, and current feed-forward. In this demo, the motor angle follows a sine wave to represent the panning motion of the phone mount.

Motor angle feedback is provided by an MT6701 magnetic encoder fixed to the stator. It connects to a timer on the STM MCU configured in encoder mode along with a GPIO pin for the index pulse.

## Building and Running

Building the firmware requires `arm-none-eabi-binutils`, `cmake`, and a programming/debugging tool.

1. Open the `mcu-firmware` directory

2. Run `cmake --preset Debug`

3. Run `cmake --build --preset Debug`

4. Program the generated `build/Debug/disc-sense-mcu.bin` file with CubeMXProgrammer or a compatible STLink programming tool

## Disclaimer

Significant portions of the FOC (located in `Drivers/simplefoc`) are derived from [simplefoc/Arduino-FOC](https://github.com/simplefoc/Arduino-FOC).
The relevant LICENSE.md files are provided, and all use is permitted by the Arduino-FOC MIT license.
