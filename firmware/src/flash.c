#include <stm32g4xx.h>
#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "usb.h"
#include "util.h"

void flash_init(void) {
  // Set CS low (A15)
  GPIOA->BSRR = GPIO_BSRR_BR_15;
  // Reset the flash chip.
  spi_transmit(SPI3, 0xFF);
  // Set CS high (A15)
  GPIOA->BSRR = GPIO_BSRR_BS_15;
  // Wait for reset
  msleep(2);
}
