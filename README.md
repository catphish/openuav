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

# Configurator

The USB configurator can be used to configure the quad, and also to download blackbox
logs.

## Configuration

You can view the current configuration by running `configurator get`.

The commands to set configuration are a little cryptic. Use command `configurator set x y z`, where `x` is the
categry number, `y` is the specific entry number, and `z` is the value to set. Please refer to the following file
for the category and entry numbers: https://github.com/catphish/openuav/blob/master/configurator/usb.h

## Motor configuration

The motor configuration is also not very intuitive, but works as follows. First select "props in" `configurator set 3 5 0`
or "props out" `configurator set 3 5 1`.

Next, map the motor outputs using the command: `configurator set 3 x y`, where `x` is the motor output pin (1-4), and `y` is
a corner of the quad according to the following table:

```
0: no output
1: rear left
2: front right
3: front left
4: rear right
```

* NOTE: Due to the way air mode currently works, motors set to zero outout will still spin at minimum RPM. This needs fixing.

## Black Box

Data will be recorded to the black box any time the quad is armed and will continue until the memory
is full (approx 10 minutes flight time). You can erase the black box with `configurator erase`.

You can download the blackbox data with `configurator read > data.csv`. The CSV can be processed
with any tool of your choosing but I find kst2 to be the best.

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
Angle rate: 1000
Acro rate: 1000
P: 100
I: 80
D: 100
Yaw P: 200
Yaw I: 100
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
