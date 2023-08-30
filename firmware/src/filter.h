#pragma once
#include "gyro.h"

void filter_gyro_p(struct gyro_data *input, struct gyro_data *output);
void filter_gyro_d(struct gyro_data *input, struct gyro_data *output);
