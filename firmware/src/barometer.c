#include <stdint.h>
#include "gpio.h"
#include "spi.h"
#include "barometer.h"

static void baro_spi_write_register(uint8_t reg, uint8_t value) {
  // Set CS low
  GPIOC->BSRR = GPIO_BSRR_BR_6;
  spi_transmit(SPI2, reg);
  spi_transmit(SPI2, value);
  // Set CS high
  GPIOC->BSRR = GPIO_BSRR_BS_6;
}

static uint8_t baro_spi_read_register(uint8_t reg) {
  // Set CS low
  GPIOC->BSRR = GPIO_BSRR_BR_6;
  spi_transmit(SPI2, reg | 0x80);
  uint8_t value = spi_receive(SPI2);
  // Set CS high
  GPIOC->BSRR = GPIO_BSRR_BS_6;
  return value;
}

void baro_init(void) {
  // Enable baro, 50Hz update, 2.5Hz filter
  baro_spi_write_register(BARO_REG_CTRL_REG1, (4<<4)|(1<<3)|(1<<2));
}

uint32_t baro_read_pressure() {
  uint32_t out = baro_spi_read_register(BARO_REG_PRESS_OUT_XL);
  out |= baro_spi_read_register(BARO_REG_PRESS_OUT_L) << 8;
  out |= baro_spi_read_register(BARO_REG_PRESS_OUT_H) << 16;
  return out;
}
