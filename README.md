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

Tuning should be done in acro mode at a rate you are comfortable with. I use a rate
of 1200 (840 dps) for this process.

## Part 1 - P term

Set the I and D terms to zero so that they don't interfere. Start with a low value (P=50).
Increase P until the quad is sufficiently responsive to stick inputs. You can expect a
little bounceback.
In the 5 inch development quad, the P value is 120.

## Part 2 - D term

Increase the D term only as much as is required to prevent the bounceback.
In the 5 inch development quad, the D value is 150.

## Part 3 - I term

Increase the I term only as much as is required to prevent drift. You may find that this
needs to be increased more when flying in strong wind. Higher values will make the quad
more stable but will introduce some bouncenback when performing flips.
In the 5 inch development quad, the I value is 50.

## Part 4 - Z rates

There is currently no specific process for tuning the Z rates. Values of 200 and 200
for P and I seem to work in most cases. If there is yaw drift then it may be necessary
to increase one or the other. Increase the I term in the case of slow drift or the P
term in the case of sudden yaw disruption.

# Tuning Examples

## TBS Source One V5 5" - 2306/2555KV Motors at 4S

```
Angle rate: 1500
Acro rate: 1500
P: 120
I: 50
D: 150
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
P (sp): x
I (si): x
D (sd): x
Yaw P (sy): 220
Yaw I (sY): 200
Expo (se): 25
Yaw expo (sE): 30
Throttle gain (st): 80
Throttle min (sT): 100
```
