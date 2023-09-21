#include <stdint.h>
#include "gpio.h"
#include "spi.h"
#include "gyro.h"
#include "util.h"

struct gyro_data gyro_offset;
float gyro_calibration_total[3];
float gyro_calibration_count;

static void gyro_spi_write_register(uint8_t reg, uint8_t value) {
    // Set CS low
    GPIOC->BSRR = GPIO_BSRR_BR_4;
    spi_transmit(SPI1, reg);
    spi_transmit(SPI1, value);
    // Set CS high
    GPIOC->BSRR = GPIO_BSRR_BS_4;
}

static uint8_t gyro_spi_read_register(uint8_t reg) {
    // Set CS low
    GPIOC->BSRR = GPIO_BSRR_BR_4;
    spi_transmit(SPI1, reg | 0x80);
    uint8_t value = spi_receive(SPI1);
    // Set CS high
    GPIOC->BSRR = GPIO_BSRR_BS_4;
    return value;
}

static void gyro_bank_select(uint8_t bank) {
    gyro_spi_write_register(0x76, bank);
}

void gyro_init(void)
{
    gpio_init();
    // Set PA4 as input.
    gpio_pin_mode(GPIOA, 4, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, 0);

    // Configure registers in bank 0
    gyro_bank_select(0);
    // Deassert reset on interrupt system
    gyro_spi_write_register(0x64, 0);
    // Set INT1 to push-pull, active high, latching
    gyro_spi_write_register(0x14, (1<<0) | (1<<1) | (1<<2));
    // Set DRDY to clear on data read
    gyro_spi_write_register(0x63, (2<<4));
    // Enable EN1 on DRDY
    gyro_spi_write_register(0x65, (1<<3));
    // Disable gyro UI filter
    gyro_spi_write_register(0x52, (15<<4) | (15<<0));
    // Enable gyro and accel
    gyro_spi_write_register(0x4E, (3<<2) | (3<<0));
    // Wait for gyro and accel to be ready
    msleep(1);
    // Set gyro ODR to 8kHz
    gyro_spi_write_register(0x4F, (3<<0));
    // Set accel ODR to 8kHz
    gyro_spi_write_register(0x50, (3<<0));
}

// The gyro is ready when PA4 is high
uint8_t gyro_ready(void)
{
    return gpio_get_pin(GPIOA, 4);
}

void gyro_zero(void) {
    gyro_calibration_count = 0;
    gyro_calibration_total[0] = 0;
    gyro_calibration_total[1] = 0;
    gyro_calibration_total[2] = 0;
    gyro_offset.x = 0;
    gyro_offset.y = 0;
    gyro_offset.z = 0;
}

void gyro_calibrate(void) {
    gyro_calibration_count += 1.f;
    struct gyro_data d;
    gyro_read_raw(&d);
    gyro_calibration_total[0] += d.x;
    gyro_calibration_total[1] += d.y;
    gyro_calibration_total[2] += d.z;
    gyro_offset.x = gyro_calibration_total[0] / gyro_calibration_count;
    gyro_offset.y = gyro_calibration_total[1] / gyro_calibration_count;
    gyro_offset.z = gyro_calibration_total[2] / gyro_calibration_count;
}

void gyro_read_raw(struct gyro_data * d)
{
    uint8_t data[6];
    for(int i=0; i<6; i++) {
        data[i] = gyro_spi_read_register(GYRO_REG_OUTX_L_G+i);
    }
    int16_t x = (data[0]<<8)|data[1];
    int16_t y = (data[2]<<8)|data[3];
    int16_t z = (data[4]<<8)|data[5];
    d->x =  x;
    d->y = -y;
    d->z =  z;
}

void gyro_read(struct gyro_data * d)
{
    struct gyro_data raw;
    gyro_read_raw(&raw);
    d->x = raw.x - gyro_offset.x;
    d->y = raw.y - gyro_offset.y;
    d->z = raw.z - gyro_offset.z;
}

void accel_read(struct gyro_data * d)
{
    uint8_t data[6];
    for(int i=0; i<6; i++) {
        data[i] = gyro_spi_read_register(GYRO_REG_OUTX_L_A+i);
    }
    int16_t x = (data[0]<<8)|data[1];
    int16_t y = (data[2]<<8)|data[3];
    int16_t z = (data[4]<<8)|data[5];
    d->x = x;
    d->y = y;
    d->z = z;
}
