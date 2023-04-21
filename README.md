
# OpenUAV
This project aims to develop an open source hardware flight controller for multirotor aircraft using modern components while keeping construction cost to a minimum.

## Components
The primary components will be as follows.
* STM32G4 - 170MHz microcontroller
* LSM6DSR - Gyro and accelerometer
* LPS22HH - Barometric pressure sensor
* W25N01GVZEIG - 128MB flash
* L78L05 - 5V linear regulator
* L78L33 - 3.3V linear regulator

If possible, the following will be added. Space constraints are likely to make this challenging.
* AT7456E - Video overlay
* L78L09 - 9V linear regulator


## Goals
* Free open source hardware design. It is hoped that both hobbyists and mass producers will be able to produce this hardware at minimal cost and with no obligation with no expectation to pay for either the hardware or software design.
* Low cost production. The goal is to limit production cost by design, for example by limiting the design to 4 layer PCB, single sided assembly.

## Software
The initial goal will be to run Betaflight.
