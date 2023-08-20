# OpenUAV Firmware
Here you will find the beginnings of a simple custom flight controller
firmware. It is now flyable in acro and angle mode with configurable PIDs.

# Features
* Status LEDs
* DSHOT
* ELRS
* Gyro
* Accelerometer
* Blackbox
* USB configuration
* Canvas OSD
* 40V SMPS with 9V 2A output

# Dependencies

## ArchLinux-based distributions

```sh
sudo pacman -Sy arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-newlib \
                dfu-util \
                stlink
```

# Roadmap
* Tidy up and comment all current code
* Improved USB interface for configuration and data log download
* Independent software filtering for gyro P and D terms
* More OSD options

# Basic Tuning Guide

Starting from the default values in `firmware/src/main.c`, increase _P_ (`RATE_P`), and keep increasing it until either (1) it feels right, in which case, stop here. Or (2), it starts to oscillate fast, in which case back it off. Once you've done that, try flying in some wind. If it drifts slowly in the wind, increase _I_ (`RATE_I`). Unstable behaviour in wind tends to mean you just have _P_ set way too low. In other words, if it feels like wind knocks it off course suddenly, that's P, and if it's a slow, annoying, but controllable drift, that's I. Should you experience left-yaw without stick input (on `PROPS_IN`, that is) on punchouts, try increasing vertical _P_ (`RATE_ZP`) and _I_ (`RATE_ZI`).

# Tuning Examples

## TBS Source One V5 5" - 2206 Motors at 4S

THIS TUNE IS STILL UNDERGOING TESTING
```
Angle rate: 800
Acro rate: 1500 (861 dps)
P: 100
I: 100
D: 200
Yaw P: 200
Yaw I: 200
Expo: 60
Yaw expo: 60
Throttle gain: 80
Throttle min: 200
```

## iFlight iX3 Racer V1 3" - 1104/5250KV Motors at 3S

THIS TUNE IS STILL UNDERGOING TESTING

```
Angle rate (sR): 670
Acro rate (sr): 670
P (sp): 180
I (si): 120
D (sd): 400
Yaw P (sy): 220
Yaw I (sY): 200
Expo (se): 25
Yaw expo (sE): 30
Throttle gain (st): 80
Throttle min (sT): 100
```
