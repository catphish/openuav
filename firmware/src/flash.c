#include <stm32g4xx.h>
#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "usb.h"
#include "util.h"

// Wait for busy flag to clear
void busy_wait(void) {
  int busy = 1;
  while(busy) {
    // Set CS low (B9)
    GPIOB->BSRR = GPIO_BSRR_BR_9;
    spi_transmit(SPI3, 0x0F); // Read register
    spi_transmit(SPI3, 0xC0); // Status register 3
    busy = spi_receive(SPI3) & 1;
    // Set CS high (B9)
    GPIOB->BSRR = GPIO_BSRR_BS_9;
    usleep(10);
  }
}

void flash_init(void) {
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Reset the flash chip.
  spi_transmit(SPI3, 0xFF);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
  // Wait for reset
  msleep(2);
  // Unprotect all blocks
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send write status register command
  spi_transmit(SPI3, 0x1F);
  // Send register address
  spi_transmit(SPI3, 0xA0);
  // Send value
  spi_transmit(SPI3, 0x00);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
}

// Enable write operations
// This must be done before each write or erase operation
void write_enable(void) {
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send write enable command
  spi_transmit(SPI3, 0x06);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
}

// Erase a 128KB block
void flash_erase_block(uint16_t block) {
  // Enable writing
  write_enable();
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send erase command
  spi_transmit(SPI3, 0xD8);
  // Send dummy byte
  spi_transmit(SPI3, 0x00);
  // Send address
  uint32_t page = block * 64;
  spi_transmit(SPI3, (page >> 8) & 0xFF);
  spi_transmit(SPI3, page & 0xFF);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
  // Wait for erase to complete
  busy_wait();
}

// Load data from the software buffer into the hardware write buffer
void flash_program_load(uint8_t * buffer, uint32_t offset, uint32_t length) {
  // Enable writing
  write_enable();
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send random program load command
  spi_transmit(SPI3, 0x84);
  // Send offset
  spi_transmit(SPI3, (offset >> 8) & 0xFF);
  spi_transmit(SPI3, offset & 0xFF);
  // Send data
  for(uint32_t n=0; n<length; n++) {
    spi_transmit(SPI3, buffer[n]);
  }
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
}

// Write buffer to flash
void flash_program_execute(uint32_t page_address) {
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send program execute command
  spi_transmit(SPI3, 0x10);
  // Send dummy byte
  spi_transmit(SPI3, 0x00);
  // Send address
  spi_transmit(SPI3, (page_address >> 8) & 0xFF);
  spi_transmit(SPI3, page_address & 0xFF);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
}

uint16_t page_cache = 0xFFFF;
void flash_page_read(uint16_t page_address) {
  if(page_cache == page_address) return;
  page_cache = page_address;
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send read command
  spi_transmit(SPI3, 0x13);
  // Send dummy byte
  spi_transmit(SPI3, 0x00);
  // Send address
  spi_transmit(SPI3, (page_address >> 8) & 0xFF);
  spi_transmit(SPI3, page_address & 0xFF);
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
  // Wait for page read to complete
  busy_wait();
}

void flash_read(uint8_t * data, uint16_t length, uint16_t offset) {
  // Set CS low (B9)
  GPIOB->BSRR = GPIO_BSRR_BR_9;
  // Send read command
  spi_transmit(SPI3, 0x03);
  // Send address
  spi_transmit(SPI3, (offset >> 8) & 0xFF);
  spi_transmit(SPI3, offset & 0xFF);
  // Send dummy byte
  spi_transmit(SPI3, 0x00);
  // Read data
  for(int n=0; n<length; n++) {
    data[n] = spi_receive(SPI3);
  }
  // Set CS high (B9)
  GPIOB->BSRR = GPIO_BSRR_BS_9;
}
