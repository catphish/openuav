#include <stdint.h>
#include "mag.h"
#include "i2c.h"
#include "util.h"

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

void mag_init(void) {
  msleep(10);
  I2C2_Write(0x0D, 0x0B, (uint8_t[]){0x01}, 1);
  msleep(10);
  I2C2_Write(0x0D, 0x09, (uint8_t[]){Mode_Continuous|ODR_200Hz|RNG_2G|OSR_512}, 1);
}

void mag_read(struct mag_data *mag) {
  uint8_t data[6] = {0,0,0,0,0,0};
  I2C2_Read(0x0D, 0x00, data, 6);
  mag->x = (data[1] << 8) | data[0];
  mag->y = (data[3] << 8) | data[2];
  mag->z = (data[5] << 8) | data[4];
}