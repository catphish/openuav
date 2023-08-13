#include <stm32g4xx.h>
#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "usb.h"
#include "util.h"

// Reset the flash chip
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

// Wait for busy flag to clear
void busy_wait(void) {
  int busy = 1;
  while(busy) {
    spi_transmit(SPI3, 0x0F); // Read register
    spi_transmit(SPI3, 0xC0); // Status register 3
    busy = spi_receive(SPI3) & 1;
  }
}

// Enable write operations
// This must be done before each write or erase operation
void write_enable(void) {
  // Set CS low (A15)
  GPIOA->BSRR = GPIO_BSRR_BR_15;
  // Send write enable command
  spi_transmit(SPI3, 0x06);
  // Set CS high (A15)
  GPIOA->BSRR = GPIO_BSRR_BS_15;
}

// Erase a 128KB block
void flash_erase_block(uint16_t block) {
  // Wait for busy flag to clear
  busy_wait();

  // Enable writing
  write_enable();

  // Set CS low (A15)
  GPIOA->BSRR = GPIO_BSRR_BR_15;
  // Send erase command
  spi_transmit(SPI3, 0xD8);
  // Send dummy byte
  spi_transmit(SPI3, 0x00);
  // Send address
  spi_transmit(SPI3, (block >> 8) & 0xFF);
  spi_transmit(SPI3, block & 0xFF);
  // Set CS high (A15)
  GPIOA->BSRR = GPIO_BSRR_BS_15;
}

// Erase whole 128MB flash chip (1024 blocks of 128KB)
void flash_erase(void) {
  for(int n=0; n<1024; n++) {
    flash_erase_block(n);
  }
}

// Load data into write buffer
void flash_program_load(uint32_t offset, uint32_t length, uint8_t *data) {
  // Wait for busy flag to clear
  busy_wait();
}

// Write buffer to flash
void flash_program_execute(uint32_t page_address) {
  // Wait for busy flag to clear
  busy_wait();
}
