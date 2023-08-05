#include <stdint.h>

struct settings {
  uint32_t version;
  uint32_t angle_rate;
  uint32_t acro_rate;
  uint32_t p;
  uint32_t i;
  uint32_t d;
  uint32_t yaw_p;
  uint32_t yaw_i;
  uint32_t throttle_gain;
  uint32_t throttle_min;
  uint32_t motor_direction;
  uint32_t motor1;
  uint32_t motor2;
  uint32_t motor3;
  uint32_t motor4;
  uint32_t checksum;
};

void settings_load();
void settings_print();
