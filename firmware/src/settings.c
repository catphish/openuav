#include <stm32g4xx.h>
#include "settings.h"
#include "usb.h"
#include "adc.h"

#define VERSION 2

volatile struct settings settings;

void settings_default() {
  settings.version         = VERSION;
  settings.angle_rate      = 800;   // 8.0
  settings.acro_rate       = 800;   // 8.0
  settings.p               = 100;   // 0.1
  settings.i               = 200;   // 0.0002
  settings.d               = 100;   // 0.1
  settings.yaw_p           = 100;   // 0.1
  settings.yaw_i           = 200;   // 0.0001
  settings.expo            = 50;    // 0.5
  settings.yaw_expo        = 50;    // 0.5
  settings.throttle_gain   = 80;    // 0.8
  settings.throttle_min    = 200;   // 200
  settings.motor_direction = 0;     // Props IN
  settings.motor1          = 0;     // Default disabled
  settings.motor2          = 0;     // Default disabled
  settings.motor3          = 0;     // Default disabled
  settings.motor4          = 0;     // Default disabled
  settings.adc_coefficient = 565;   // 565 as calibrated by catphish on v3 board
  settings.cell_count      = 0;
  settings.checksum        = 0;
}

void settings_read() {
  // Load the settings from flash at (FLASH_BASE + 31 * 0x800) into the settings struct.
  // If the settings are invalid, set them to default values.
  struct settings *flash = (struct settings *)(FLASH_BASE + 31 * 0x800);
  // Calculate the checksum of the settings struct.
  uint32_t checksum = 0;
  for(uint32_t i = 0; i < sizeof(struct settings) - 4; i++) {
    checksum += ((uint8_t *)flash)[i];
  }
  // If the checksum is valid, apply the settings, else set the defaults.
  if(flash->version == VERSION && checksum == flash->checksum) {
    settings = *flash;
  } else {
    settings_default();
  }
}

void settings_save() {
  settings.checksum = 0;
  for(uint32_t i = 0; i < sizeof(struct settings) - sizeof(uint32_t); i++) {
    settings.checksum += ((uint8_t *)&settings)[i];
  }

  __disable_irq();
  // Wait for flash to be ready
  while(FLASH->SR & FLASH_SR_BSY);
  // Unlock flash memory
  if(FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123U;
    FLASH->KEYR = 0xCDEF89ABU;
  }
  // Wait for flash to be ready
  while(FLASH->SR & FLASH_SR_BSY);
  // Erase page 31
  FLASH->CR |= FLASH_CR_PER;
  FLASH->CR |= (31 << FLASH_CR_PNB_Pos);
  FLASH->CR |= FLASH_CR_STRT;
  // Wait for flash to be ready
  while(FLASH->SR & FLASH_SR_BSY);
  // Write settings to flash
  FLASH->CR &= ~FLASH_CR_PER;
  FLASH->CR |= FLASH_CR_PG;
  for(uint32_t i = 0; i < sizeof(struct settings) / sizeof(uint32_t); i++) {
    *(volatile uint32_t *)(FLASH_BASE + 31 * 0x800 + i * sizeof(uint32_t)) = ((uint32_t *)&settings)[i];
    // Wait for flash to be ready
    while(FLASH->SR & FLASH_SR_BSY);
  }
  // Lock flash memory
  FLASH->CR = FLASH_CR_LOCK;
  __enable_irq();
}

volatile struct settings *settings_get() {
  return &settings;
}
