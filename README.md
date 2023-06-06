# OpenUAV
This project aims to develop an open source hardware flight controller for multirotor aircraft using modern components and keeping construction cost to a minimum.

**Warning: this project is still a work in progress and is awaiting testing**
**Warning: this design currently required simultaneous use of ADC1 and ADC2 which is not supported by betaflight**

## Components
The primary components will be as follows.
* STM32G474 - 170MHz microcontroller
* BMI270 - Gyro and accelerometer
* BMP280 - Barometric pressure sensor
* W25N01GVZEIG - 128MB flash
* TPS54202DDC - 5V SMPS

## Goals
* Free open source hardware design. It is hoped that everyone will be able to benefit from the published design.
* Low cost production. The goal is to limit production cost by design, for example by limiting the design to 4 layer PCB, single sided assembly.

## Software
The initial goal will be to run Betaflight.

## Form Factor
Form factor will initially be 30x30.
