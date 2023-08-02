#pragma once
#include <stdint.h>

struct mag_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

void mag_init(void);
void mag_read(struct mag_data *mag);
