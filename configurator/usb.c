#include <stdlib.h>
#include <stdio.h>
#include <libusb.h>
#include <string.h>

#include "usb.h"

libusb_device_handle *devh;

int usb_init() {
  if(libusb_init(NULL) != LIBUSB_SUCCESS) {
    printf("Error initializing libusb\n");
    return 1;
  }

  if(libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING) != LIBUSB_SUCCESS) {
    printf("Error setting libusb log level\n");
    return 2;
  }

  devh = libusb_open_device_with_vid_pid(NULL, 0x1209, 0xFC00);
  if(!devh) {
    printf("Could not find / open OpenUAV device\n");
    return 3;
  }


  if(libusb_claim_interface(devh, 0) != LIBUSB_SUCCESS) {
    printf("Could not claim interface\n");
    return 4;
  }

  return 0;
}

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
  libusb_bulk_transfer(devh, 0x01, (uint8_t[]){USB_COMMAND_FLASH_ERASE, block, block>>8}, 3, NULL, 0);
  uint8_t response[1];
  libusb_bulk_transfer(devh, 0x81, response, 1, NULL, 0);
}

void usb_close() {
  libusb_close(devh);
  libusb_exit(NULL);
}
