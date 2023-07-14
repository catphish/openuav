#include <stdint.h>

struct dshot_data {
    int32_t armed;
    int32_t motor1;
    int32_t motor2;
    int32_t motor3;
    int32_t motor4;
};

void dshot_init(void);
void dshot_write(struct dshot_data * data);
