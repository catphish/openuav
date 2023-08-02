#pragma once
#include <stdint.h>

void i2c_init(void);
uint8_t I2C2_Read(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes);
uint8_t I2C2_Write(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes);
