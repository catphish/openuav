#pragma once
#include <stdint.h>

void gps_process_char(uint8_t received);
int32_t gps_lat();
int32_t gps_lon();
