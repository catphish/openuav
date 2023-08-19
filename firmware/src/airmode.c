#include <stdint.h>
#include "dshot.h"

// This funtion looks at the motor outputs. If any of them are outside the
// allowed range (currently 100-2000) it will adjust the other motors to
// compensate. This allows a differential thrust to be applied even if the
// throttle is close the the minimum or maximum.

void air_mode(struct dshot_data *dshot) {
  // TODO: Make the limits configurable.
  if(dshot->motor1 < 100) {
    int32_t motor_error = 100 - dshot->motor1;
    dshot->motor1 = 100;
    dshot->motor2 += motor_error;
    dshot->motor3 += motor_error;
    dshot->motor4 += motor_error;
  }
  if(dshot->motor2 < 100) {
    int32_t motor_error = 100 - dshot->motor2;
    dshot->motor2 = 100;
    dshot->motor1 += motor_error;
    dshot->motor3 += motor_error;
    dshot->motor4 += motor_error;
  }
  if(dshot->motor3 < 100) {
    int32_t motor_error = 100 - dshot->motor3;
    dshot->motor3 = 100;
    dshot->motor1 += motor_error;
    dshot->motor2 += motor_error;
    dshot->motor4 += motor_error;
  }
  if(dshot->motor4 < 100) {
    int32_t motor_error = 100 - dshot->motor4;
    dshot->motor4 = 100;
    dshot->motor1 += motor_error;
    dshot->motor2 += motor_error;
    dshot->motor3 += motor_error;
  }
  if(dshot->motor1 > 2000) {
    int32_t motor_error = dshot->motor1 - 2000;
    dshot->motor1 = 2000;
    dshot->motor2 -= motor_error;
    dshot->motor3 -= motor_error;
    dshot->motor4 -= motor_error;
  }
  if(dshot->motor2 > 2000) {
    int32_t motor_error = dshot->motor2 - 2000;
    dshot->motor2 = 2000;
    dshot->motor1 -= motor_error;
    dshot->motor3 -= motor_error;
    dshot->motor4 -= motor_error;
  }
  if(dshot->motor3 > 2000) {
    int32_t motor_error = dshot->motor3 - 2000;
    dshot->motor3 = 2000;
    dshot->motor1 -= motor_error;
    dshot->motor2 -= motor_error;
    dshot->motor4 -= motor_error;
  }
  if(dshot->motor4 > 2000) {
    int32_t motor_error = dshot->motor4 - 2000;
    dshot->motor4 = 2000;
    dshot->motor1 -= motor_error;
    dshot->motor2 -= motor_error;
    dshot->motor3 -= motor_error;
  }
}
