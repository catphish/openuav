#pragma once
#include <stdint.h>

#define BARO_REG_CTRL_REG1 0x10
#define BARO_REG_PRESS_OUT_XL 0x28
#define BARO_REG_PRESS_OUT_L 0x29
#define BARO_REG_PRESS_OUT_H 0x2A

void baro_init(void);
uint32_t baro_read_pressure();
