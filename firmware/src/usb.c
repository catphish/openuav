#include <stm32g4xx.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "usb_private.h"
#include "usb.h"
#include "gpio.h"
#include "util.h"
#include "led.h"
#include "settings.h"
#include "flash.h"
#include "blackbox.h"

uint8_t pending_addr = 0;
uint32_t buffer_pointer;

void usb_init() {
  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // Set PA11 and PA12 to analog mode
  gpio_pin_mode(GPIOA, 11, GPIO_MODE_ANALOG, 10, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  gpio_pin_mode(GPIOA, 12, GPIO_MODE_ANALOG, 10, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

  // Enable Power control clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  // Enable USB clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;
  // Enable clock recovery system clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;

  // Enable HSI48
  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  // Wait for HSI48 to be ready
  while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) != RCC_CRRCR_HSI48RDY);

  // Enable USB Power
  PWR->CR2 |= (1<<10);
  // Wait for USB Power to be ready
  usleep(10);

  // Enable USB and reset
  USB->CNTR = USB_CNTR_FRES;
  // Hold reset for 10us
  usleep(10);
  // Deassert reset
  USB->CNTR = 0;
  // Wait for reset to complete
  usleep(10);
  // Activate DP pullup
  USB->BCDR |= USB_BCDR_DPPU;
}

// Types: 0=Bulk,1=Control,2=Iso,3=Interrupt
void usb_configure_ep(uint8_t ep, uint32_t type) {
  uint8_t in = ep & 0x80;
  ep &= 0x7f;

  uint32_t old_epr = USB_EPR(ep);
  uint32_t new_epr = 0; // Always write 1 to bits 15 and 8 for no effect.
  new_epr |= ((type & 3) << 9); // Set type
  new_epr |= ep;                // Set endpoint number

  if(in || ep == 0) {
    USBBUFTABLE->ep_desc[ep].txBufferAddr = buffer_pointer;
    USBBUFTABLE->ep_desc[ep].txBufferCount = 0;
    buffer_pointer += 64;
    new_epr |= (old_epr & 0x0030) ^ 0x0020; // NAK
  }

  if(!in) {
    USBBUFTABLE->ep_desc[ep].rxBufferAddr = buffer_pointer;
    USBBUFTABLE->ep_desc[ep].rxBufferCount = (1<<15) | (1 << 10) | 0xff;
    buffer_pointer += 64;
    new_epr |= (old_epr & 0x3000) ^ 0x3000; // Ready
  }

  USB_EPR(ep) = new_epr;
}

// Check the endpoint is in a NACK TX state waiting for data.
uint32_t ep_tx_ready(uint32_t ep) {
  ep &= 0x7f;
  return((USB_EPR(ep) & 0x30) == 0x20);
}

// Read 64 bytes from an endpoint buffer.
uint8_t usb_read(uint8_t ep, char * buffer) {
  ep &= 0x7f;
  uint32_t rxBufferAddr = USBBUFTABLE->ep_desc[ep].rxBufferAddr;
  uint8_t len = USBBUFTABLE->ep_desc[ep].rxBufferCount & 0x3ff;
  if(buffer) {
    for(int n=0; n<64; n+=2) {
      *(uint16_t *)(buffer + n) = *(uint16_t *)(USBBUFRAW+rxBufferAddr+n);
    }
  }
  // Clear pending state
  USB_EPR(0) &= 0x078f;
  // RX NAK->VALID, Clear CTR
  USB_EPR(ep) = (USB_EPR(ep) & 0x370f) ^ 0x3000;
  return len;
}

// Write data to an endpoint buffer.
void usb_write(uint8_t ep, char * buffer, uint32_t len) {
  ep &= 0x7f;
  while(!ep_tx_ready(ep));
  uint32_t txBufferAddr = USBBUFTABLE->ep_desc[ep].txBufferAddr;
  for(uint32_t n=0; n<len; n+=2) {
    *(uint16_t *)(USBBUFRAW+txBufferAddr+n) = *(uint16_t *)(buffer + n);
  }
  USBBUFTABLE->ep_desc[ep].txBufferCount = len;

  // TX NAK->VALID
  USB_EPR(ep) = (USB_EPR(ep) & 0x87bf) ^ 0x30;
}

void usb_reset() {
  USB->ISTR &= ~USB_ISTR_RESET;

  buffer_pointer = 64;
  usb_configure_ep(0x00, 0x01);
  usb_configure_ep(0x01, 0x10);
  usb_configure_ep(0x81, 0x10);
  USB->BTABLE = 0;

  // Enable the peripheral
  USB->DADDR = USB_DADDR_EF;
}

void usb_handle_ep0() {
  char request[64];
  usb_read(0, request);

  uint8_t bmRequestType = request[0];
  uint8_t bRequest      = request[1];

  // Descriptor request
  if(bmRequestType == 0x80 && bRequest == 0x06) {
    uint8_t descriptor_type = request[3];
    uint8_t descriptor_idx  = request[2];

    if(descriptor_type == 1 && descriptor_idx == 0) {
      uint32_t len = 18;
      if(len > request[6]) len = request[6];
      usb_write(0, device_descriptor, len);
    }
    if(descriptor_type == 2 && descriptor_idx == 0) {
      uint32_t len = config_descriptor[2];
      if(len > request[6]) len = request[6];
      usb_write(0, config_descriptor, len);
    }
    if(descriptor_type == 3 && descriptor_idx == 0) {
      uint32_t len = string_descriptor[0];
      if(len > request[6]) len = request[6];
      usb_write(0, string_descriptor, len);
    }
  }
  // Set address
  if(bmRequestType == 0x00 && bRequest == 0x05) {
    pending_addr = request[2];
    usb_write(0,0,0);
  }
  // Set configuration
  if(bmRequestType == 0x00 && bRequest == 0x09) {
    usb_write(0,0,0);
  }
}
volatile uint8_t usb_enabled = 0;

void usb_handle_ep1() {
  struct settings *settings = settings_get();
  uint8_t request[64];
  uint8_t response[64];
  uint8_t len = usb_read(1, (char*)request);
  if(len) {
    if(request[0] == USB_COMMAND_SETTING_GET) {
      uint32_t value;
      if(request[1] == USB_SETTING_CAT_TUNE) {
        if(request[2] == USB_SETTING_TUNE_P) {
          value = settings->p;
        } else if(request[2] == USB_SETTING_TUNE_I) {
          value = settings->i;
        } else if(request[2] == USB_SETTING_TUNE_D) {
          value = settings->d;
        } else if(request[2] == USB_SETTING_TUNE_YAW_P) {
          value = settings->yaw_p;
        } else if(request[2] == USB_SETTING_TUNE_YAW_I) {
          value = settings->yaw_i;
        }
      } else if(request[1] == USB_SETTING_CAT_CONTROL) {
        if(request[2] == USB_SETTING_CONTROL_ANGLE_RATE) {
          value = settings->angle_rate;
        } else if(request[2] == USB_SETTING_CONTROL_ACRO_RATE) {
          value = settings->acro_rate;
        } else if(request[2] == USB_SETTING_CONTROL_EXPO) {
          value = settings->expo;
        } else if(request[2] == USB_SETTING_CONTROL_YAW_EXPO) {
          value = settings->yaw_expo;
        } else if(request[2] == USB_SETTING_CONTROL_THROTTLE_GAIN) {
          value = settings->throttle_gain;
        } else if(request[2] == USB_SETTING_CONTROL_THROTTLE_MIN) {
          value = settings->throttle_min;
        }
      } else if(request[1] == USB_SETTING_CAT_MOTOR) {
        if(request[2] == USB_SETTING_MOTOR_DIRECTION) {
          value = settings->motor_direction;
        } else if(request[2] == USB_SETTING_MOTOR_1) {
          value = settings->motor1;
        } else if(request[2] == USB_SETTING_MOTOR_2) {
          value = settings->motor2;
        } else if(request[2] == USB_SETTING_MOTOR_3) {
          value = settings->motor3;
        } else if(request[2] == USB_SETTING_MOTOR_4) {
          value = settings->motor4;
        }
      } else if(request[1] == USB_SETTING_CAT_BATT) {
        if(request[2] == USB_SETTING_BATT_ADC_COEFFICIENT) {
          value = settings->adc_coefficient;
        } else if(request[2] == USB_SETTING_BATT_CHEMISTRY) {
          value = settings->chemistry;
        }
      }
      response[0] = USB_COMMAND_SETTING_GET;
      response[1] = request[1];
      response[2] = request[2];
      response[3] = value;
      response[4] = value >> 8;
      response[5] = value >> 16;
      response[6] = value >> 24;
      usb_write(1, (char*)response, 7);
    } else if(request[0] == USB_COMMAND_SETTING_SET) {
      uint32_t value = request[3] | (request[4] << 8) | (request[5] << 16) | (request[6] << 24);
      if(request[1] == USB_SETTING_CAT_TUNE) {
        if(request[2] == USB_SETTING_TUNE_P) {
          settings->p = value;
        } else if(request[2] == USB_SETTING_TUNE_I) {
          settings->i = value;
        } else if(request[2] == USB_SETTING_TUNE_D) {
          settings->d = value;
        } else if(request[2] == USB_SETTING_TUNE_YAW_P) {
          settings->yaw_p = value;
        } else if(request[2] == USB_SETTING_TUNE_YAW_I) {
          settings->yaw_i = value;
        }
      } else if(request[1] == USB_SETTING_CAT_CONTROL) {
        if(request[2] == USB_SETTING_CONTROL_ANGLE_RATE) {
          settings->angle_rate = value;
        } else if(request[2] == USB_SETTING_CONTROL_ACRO_RATE) {
          settings->acro_rate = value;
        } else if(request[2] == USB_SETTING_CONTROL_EXPO) {
          settings->expo = value;
        } else if(request[2] == USB_SETTING_CONTROL_YAW_EXPO) {
          settings->yaw_expo = value;
        } else if(request[2] == USB_SETTING_CONTROL_THROTTLE_GAIN) {
          settings->throttle_gain = value;
        } else if(request[2] == USB_SETTING_CONTROL_THROTTLE_MIN) {
          settings->throttle_min = value;
        }
      } else if(request[1] == USB_SETTING_CAT_MOTOR) {
        if(request[2] == USB_SETTING_MOTOR_DIRECTION) {
          settings->motor_direction = value;
        } else if(request[2] == USB_SETTING_MOTOR_1) {
          settings->motor1 = value;
        } else if(request[2] == USB_SETTING_MOTOR_2) {
          settings->motor2 = value;
        } else if(request[2] == USB_SETTING_MOTOR_3) {
          settings->motor3 = value;
        } else if(request[2] == USB_SETTING_MOTOR_4) {
          settings->motor4 = value;
        }
      } else if(request[1] == USB_SETTING_CAT_BATT) {
        if(request[2] == USB_SETTING_BATT_ADC_COEFFICIENT) {
          settings->adc_coefficient = value;
        } else if(request[2] == USB_SETTING_BATT_CHEMISTRY) {
          settings->chemistry = value;
        }
      }
      response[0] = USB_COMMAND_SETTING_SET;
      response[1] = request[1];
      response[2] = request[2];
      usb_write(1, (char*)response, 3);
    } else if(request[0] == USB_COMMAND_SETTING_SAVE) {
      settings_save();
      response[0] = USB_COMMAND_SETTING_SAVE;
      usb_write(1, (char*)response, 1);
    } else if(request[0] == USB_COMMAND_SETTING_LOAD) {
      settings_read();
      response[0] = USB_COMMAND_SETTING_LOAD;
      usb_write(1, (char*)response, 1);
    } else if(request[0] == USB_COMMAND_FLASH_READ) {
      uint32_t addr = request[1] | (request[2] << 8) | (request[3] << 16) | (request[4] << 24);
      uint16_t page = addr / 2048;
      uint16_t offset = addr % 2048;
      response[0] = USB_COMMAND_FLASH_READ;
      response[1] = request[1];
      response[2] = request[2];
      response[3] = request[3];
      response[4] = request[4];      
      flash_page_read(page);
      flash_read(response + 5, 32, offset);
      usb_write(1, (char*)response, 32+5);
    } else if(request[0] == USB_COMMAND_FLASH_ERASE) {
      uint16_t block = request[1] | (request[2] << 8);
      flash_erase_block(block);
      response[0] = USB_COMMAND_FLASH_ERASE;
      usb_write(1, (char*)response, 1);
    }
  }
}

void usb_main() {
  // USB reset
  if(USB->ISTR & USB_ISTR_RESET) {
    usb_reset();
  }

  if(USB_EPR(0) & (1<<15)) {
    // EP0 RX
    usb_handle_ep0();
  }

  if(USB_EPR(1) & (1<<15)) {
    // EP1 RX
    usb_handle_ep1();
  }

  if(USB_EPR(0) & (1<<7)) {
    // EP0 TX
    if(pending_addr) {
      // If we're waiting to set an address, do it now.
      USB->DADDR = USB_DADDR_EF | pending_addr;
      pending_addr = 0;
    }
    // Clear pending state
    USB_EPR(0) &= 0x870f;
  }
}
