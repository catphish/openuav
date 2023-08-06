#include <stm32g4xx.h>
#include "settings.h"
#include "usb.h"

#define VERSION 2

struct settings settings;

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

void settings_print() {
  // Print the settings to the USB serial port.
  usb_printf("\nVersion: %d\n", settings.version);
  usb_printf("Angle rate: %d\nAcro rate: %d\n", settings.angle_rate, settings.acro_rate);
  usb_printf("P: %d\nI: %d\nD: %d\n", settings.p, settings.i, settings.d);
  usb_printf("Yaw P: %d\nYaw I: %d\n", settings.yaw_p, settings.yaw_i);
  usb_printf("Expo: %d\nYaw expo: %d\n", settings.expo, settings.yaw_expo);
  usb_printf("Throttle gain: %d\nThrottle min: %d\n", settings.throttle_gain, settings.throttle_min);
  usb_printf("Motor direction: %d\n", settings.motor_direction);
  usb_printf("Motor 1: %d\nMotor 2: %d\nMotor 3: %d\nMotor 4: %d\n", settings.motor1, settings.motor2, settings.motor3, settings.motor4);
  usb_printf("Checksum: %d\n", settings.checksum);
}

struct settings *settings_get() {
  return &settings;
}