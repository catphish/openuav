#include <stdint.h>
#include "gyro.h"
#include "dshot.h"

struct __attribute__((__packed__)) blackbox_frame {
    uint32_t timestamp;   // 4 bytes
    int16_t gyro_data[3]; // 6 bytes
    int16_t setpoint[3];  // 6 bytes
    int16_t motor[4];     // 8 bytes
    int16_t p[3];         // 6 bytes
    int16_t i[3];         // 6 bytes
    int16_t d[3];         // 6 bytes
    // 42 bytes
};

void blackbox_init();
uint16_t blackbox_find_free_page();
void blackbox_write(struct blackbox_frame * frame);
