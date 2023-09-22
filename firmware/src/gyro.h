#pragma once
#include <stdint.h>

#define GYRO_REG_INT_CONFIG         0x14
#define GYRO_REG_ACCEL_DATA         0x1F
#define GYRO_REG_GYRO_DATA          0x25
#define GYRO_REG_INT_CONFIG0        0x63
#define GYRO_REG_INT_CONFIG1        0x64
#define GYRO_REG_INT_SOURCE0        0x65
#define GYRO_REG_GYRO_ACCEL_CONFIG0 0x52
#define GYRO_REG_PWR_MGMT0          0x4E
#define GYRO_REG_GYRO_CONFIG0       0x4F
#define GYRO_REG_ACCEL_CONFIG0      0x50

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
