#pragma once
#include <stdint.h>

#define GYRO_REG_OUTX_L_G 0x25
#define GYRO_REG_OUTX_L_A 0x1F

struct gyro_data {
  float x;
  float y;
  float z;
};

void gyro_init(void);
uint8_t gyro_ready(void);
void gyro_zero(void);
void gyro_read_raw(struct gyro_data * d);
void gyro_read(struct gyro_data * d);
void accel_read(struct gyro_data * d);
void gyro_calibrate(void);
