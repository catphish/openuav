#pragma once
#include <stdint.h>

void adc_init();
uint32_t adc_read_mv();
uint32_t adc_read_ma();
