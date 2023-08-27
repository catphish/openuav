# OpenUAV
OpenUAV is an open source flight controller for multirotors with
a focus on FPV. It is currently in active development.

# Features
* ACRO and ANGLE flight modes
* DSHOT
* ELRS
* Blackbox flight recorder
* USB configuration tool
* Canvas OSD

# Roadmap
* Tidy up and comment all current code
* USB configuration tool
* More OSD options

# Dependencies

## ArchLinux-based distributions

```sh
sudo pacman -Sy arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-newlib \
                dfu-util \
                stlink
```

# Basic Tuning Guide

Starting from the default values in, increase _P_  and keep increasing it until either (1) it feels right, in which case, stop here. Or (2), it starts to oscillate fast, in which case back it off. Once you've done that, try flying in some wind. If it drifts in the wind, increase _I_  as much as is necessary.

# Tuning Examples

## TBS Source One V5 5" - 2206 Motors at 4S

THIS TUNE IS STILL UNDERGOING TESTING!

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
