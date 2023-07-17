#include <stdint.h>
#include "gpio.h"
#include "spi.h"
#include "gyro.h"

// Set up the gyro
void gyro_init(void)
{
    gpio_init();
    // Set PA4 as input
    gpio_pin_mode(GPIOA, 4, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, 0);
    spi_write_register(GYRO_REG_CTRL2_G, (6<<4)|(3<<2)); // Enable gyro, 2000 dps, 416Hz
    spi_write_register(GYRO_REG_CTRL1_XL, (6<<4)); // Enable accelerometer, 416 Hz
    spi_write_register(GYRO_REG_INT1_CTRL, (1<<1)); // Enable Gyro data ready interrupt
}

// The gyro is ready when PA4 is high
uint8_t gyro_ready(void)
{
    return gpio_get_pin(GPIOA, 4);
}

void gyro_read(struct gyro_data * d)
{
    uint8_t data[6];
    for(int i=0; i<6; i++) {
        data[i] = spi_read_register(GYRO_REG_OUTX_L_G+i);
    }

    d->x = (data[1]<<8)|data[0];
    d->y = (data[3]<<8)|data[2];
    d->z = (data[5]<<8)|data[4];
    d->y = -d->y;
}

void accel_read(struct gyro_data * d)
{
    uint8_t data[6];
    for(int i=0; i<6; i++) {
        data[i] = spi_read_register(GYRO_REG_OUTX_L_A+i);
    }

    d->x = (data[1]<<8)|data[0];
    d->y = (data[3]<<8)|data[2];
    d->z = (data[5]<<8)|data[4];
}
