#include <stdint.h>

// as an undocumented quirk, the STM32 flash won't save anything unless
// it has 64 bytes to write. therefore, the number of these settings must
// be EVEN and each setting must be 32bits long, so e.g. uint32_t...
struct settings {
  uint32_t version;
  uint32_t angle_rate; // 2
  uint32_t acro_rate;
  uint32_t p; // 4
  uint32_t i;
  uint32_t d; // 6
  uint32_t yaw_p;
  uint32_t yaw_i; // 8
  uint32_t expo;
  uint32_t yaw_expo; // 10
  uint32_t throttle_gain;
  uint32_t throttle_min; // 12
  uint32_t motor_direction;
  uint32_t motor1; // 14
  uint32_t motor2;
  uint32_t motor3; // 16
  uint32_t motor4;
  uint32_t adc_coefficient; // 18
  uint32_t chemistry;
  // MUST be the last value, and MUST be even
  uint32_t checksum; // 20
};

struct setting_name {
  char *name;
  uint32_t *value;
};

void settings_read();
void settings_save();
void settings_default();
struct settings *settings_get();
