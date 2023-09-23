#include <stdint.h>
#include "dshot.h"

// This funtion looks at the motor outputs. If any of them are outside the
// allowed range (currently 100-2000) it will adjust the other motors to
// compensate. This allows a differential thrust to be applied even if the
// throttle is close the the minimum or maximum.

void air_mode(int32_t *motor_outputs) {
  if(motor_outputs[1] > 2000) {
    int32_t motor_error = motor_outputs[1] - 2000;
    motor_outputs[1] = 2000;
    motor_outputs[2] -= motor_error;
    motor_outputs[3] -= motor_error;
    motor_outputs[4] -= motor_error;
  }
  if(motor_outputs[2] > 2000) {
    int32_t motor_error = motor_outputs[2] - 2000;
    motor_outputs[2] = 2000;
    motor_outputs[1] -= motor_error;
    motor_outputs[3] -= motor_error;
    motor_outputs[4] -= motor_error;
  }
  if(motor_outputs[3] > 2000) {
    int32_t motor_error = motor_outputs[3] - 2000;
    motor_outputs[3] = 2000;
    motor_outputs[1] -= motor_error;
    motor_outputs[2] -= motor_error;
    motor_outputs[4] -= motor_error;
  }
  if(motor_outputs[4] > 2000) {
    int32_t motor_error = motor_outputs[4] - 2000;
    motor_outputs[4] = 2000;
    motor_outputs[1] -= motor_error;
    motor_outputs[2] -= motor_error;
    motor_outputs[3] -= motor_error;
  }

  if(motor_outputs[1] < 100) {
    int32_t motor_error = 100 - motor_outputs[1];
    motor_outputs[1] = 100;
    motor_outputs[2] += motor_error;
    motor_outputs[3] += motor_error;
    motor_outputs[4] += motor_error;
  }
  if(motor_outputs[2] < 100) {
    int32_t motor_error = 100 - motor_outputs[2];
    motor_outputs[2] = 100;
    motor_outputs[1] += motor_error;
    motor_outputs[3] += motor_error;
    motor_outputs[4] += motor_error;
  }
  if(motor_outputs[3] < 100) {
    int32_t motor_error = 100 - motor_outputs[3];
    motor_outputs[3] = 100;
    motor_outputs[1] += motor_error;
    motor_outputs[2] += motor_error;
    motor_outputs[4] += motor_error;
  }
  if(motor_outputs[4] < 100) {
    int32_t motor_error = 100 - motor_outputs[4];
    motor_outputs[4] = 100;
    motor_outputs[1] += motor_error;
    motor_outputs[2] += motor_error;
    motor_outputs[3] += motor_error;
  }
}
