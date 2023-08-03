#pragma once

#include "gyro.h"

void imu_init(void);
void imu_update_from_gyro(struct gyro_data *gyro);
void imu_update_from_accel(struct gyro_data *accel);
void imu_get_xy_tilt(float *pitch, float *roll);
