# OpenUAV Firmware
Here you will find the beginnings of a simple custom flight controller
firmware. It is now flyable in acro and angle mode with configurable PIDs.

# Progress

# Hardware Drivers
* LED     - DONE
* USB     - DONE
* DSHOT   - DONE
* UART    - DONE
* ELRS    - DONE - needs checksum
* SPI     - DONE
* Gyro    - DONE
* Accel   - DONE
* Baro    - DONE
* I2C     - DONE
* Compass - DONE
* Voltage - DONE
* Servo   -
* Flash   -
* GPS     -

# Software
* Gyro flight          - DONE
* Accelerometer flight - DONE
* OSD                  - Voltage only
* Compass heading hold -
* Baro altitude hold   -
* GPS flight           -
* Black box            -
* USB configuration    -

# Dependencies

## ArchLinux-based distributions

```sh
sudo pacman -Sy arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-newlib \
                dfu-util \
                stlink
```

# Basic Tuning Guide

Starting from the default values in `firmware/src/main.c`, increase _P_ (`RATE_P`), and keep increasing it until either (1) it feels right, in which case, stop here. Or (2), it starts to oscillate fast, in which case back it off. Once you've done that, try flying in some wind. If it drifts slowly in the wind, increase _I_ (`RATE_I`). Unstable behaviour in wind tends to mean you just have _P_ set way too low. In other words, if it feels like wind knocks it off course suddenly, that's P, and if it's a slow, annoying, but controllable drift, that's I. Should you experience left-yaw without stick input (on `PROPS_IN`, that is) on punchouts, try increasing vertical _P_ (`RATE_ZP`) and _I_ (`RATE_ZI`).