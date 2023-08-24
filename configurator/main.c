#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <libusb.h>
#include <time.h>

#include "usb.h"

libusb_device_handle *devh;

uint32_t get_setting(uint8_t category, uint8_t index) {
  uint8_t response[7];
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_SETTING_GET, category, index}, 3, NULL, 0);
  libusb_bulk_transfer(devh, 0x81, response, 7, NULL, 0);
  return(response[3] | (response[4] << 8) | (response[5] << 16) | (response[6] << 24));
}

void set_setting(uint8_t category, uint8_t index, uint32_t value) {
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_SETTING_SET, category, index, value, (value >> 8), (value >> 16), (value >> 24)}, 7, NULL, 0);
  uint8_t response[3];
  libusb_bulk_transfer(devh, 0x81, response, 3, NULL, 0);
}

void save_settings() {
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_SETTING_SAVE}, 1, NULL, 0);
  uint8_t response[1];
  libusb_bulk_transfer(devh, 0x81, response, 1, NULL, 0);
}

void load_settings() {
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_SETTING_LOAD}, 1, NULL, 0);
  uint8_t response[1];
  libusb_bulk_transfer(devh, 0x81, response, 1, NULL, 0);
}

void read_flash(uint32_t address, uint8_t *data) {
  uint8_t response[64];
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_FLASH_READ, address, (address >> 8), (address >> 16), (address >> 24)}, 5, NULL, 0);
  libusb_bulk_transfer(devh, 0x81, response, 32+5, NULL, 0);
  memcpy(data, response+5, 32);
}

void erase_flash(uint16_t block) {
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_FLASH_ERASE}, 1, NULL, 0);
  uint8_t response[1];
  libusb_bulk_transfer(devh, 0x81, response, 1, NULL, 0);
}

int main(void)
{
  srand(time(NULL));
  libusb_init(NULL);
  libusb_set_option( NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING );
  devh = libusb_open_device_with_vid_pid(NULL, 0x1209, 0xFC00);

  if (!devh)
  {
    printf("Could not find/open device\n");
    return 1;
  }

  libusb_claim_interface(devh, 0);

  int32_t value;

  uint32_t random_p = rand();
  printf("\nSetting random P value: %d\n", random_p);
  set_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_P, random_p);

  printf("            TUNING\n");
  value = get_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_P);
  printf("            P: %d\n", value);
  value = get_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_I);
  printf("            I: %d\n", value);
  value = get_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_D);
  printf("            D: %d\n", value);
  value = get_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_YAW_P);
  printf("        Yaw P: %d\n", value);
  value = get_setting(USB_SETTING_CAT_TUNE, USB_SETTING_TUNE_YAW_I);
  printf("        Yaw I: %d\n", value);

  printf("\n           CONTROL\n");
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_ACRO_RATE);
  printf("    Acro Rate: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_ANGLE_RATE);
  printf("   Angle Rate: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_EXPO);
  printf("         Expo: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_YAW_EXPO);
  printf("     Yaw Expo: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_THROTTLE_GAIN);
  printf("Throttle Gain: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_THROTTLE_MIN);
  printf(" Throttle Min: %d\n", value);

  printf("\n            MOTOR\n");
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_DIRECTION);
  printf("    Direction: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_1);
  printf("       Motor1: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_2);
  printf("       Motor2: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_3);
  printf("       Motor3: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_4);
  printf("       Motor4: %d\n", value);

  printf("\n            BATTERY\n");
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_ADC_COEFFICIENT);
  printf("    ADC Coeff: %d\n", value);
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_CHEMISTRY);
  printf("    Chemistry: %d\n", value);

  printf("\nFLASH\n");
  printf("=====\n");
  uint8_t flash_data[32];
  for(int n=0; n<256; n+=32) {
    read_flash(n, flash_data);
    printf("%06X: ", n);
    for(int i=0;  i<16; i++) printf("%02X ", flash_data[i]); printf("\n");
    printf("%06X: ", n + 0x10);
    for(int i=16; i<32; i++) printf("%02X ", flash_data[i]); printf("\n");
  }


  libusb_close(devh);
  libusb_exit(NULL);
}
