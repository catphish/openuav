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

uint8_t pending_addr = 0;
uint32_t buffer_pointer;

struct usb_packet {
  char data[64];
  uint8_t len;
};

struct usb_ring_buffer {
  struct usb_packet packet[16];
  uint8_t head;
  uint8_t tail;
} usb_ring_buffer[2];

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

// Check the endpoint CTR_RX, indicating that a packet has been received.
uint32_t ep_rx_ready(uint32_t ep) {
  ep &= 0x7f;
  return(USB_EPR(ep) & (1<<15));
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

// Copy data into the ring buffer in 64 byte chunks. If the buffer is full,
// retun immediately.
void usb_write_string(uint8_t ep, char * data, uint32_t len) {
  while(len) {
    if((usb_ring_buffer[ep].head + 1) % 16 == usb_ring_buffer[ep].tail) {
      return;
    }
    uint32_t chunk = len;
    if(chunk > 64) chunk = 64;
    for(uint32_t n=0; n<chunk; n++) {
      usb_ring_buffer[ep].packet[usb_ring_buffer[ep].head].data[n] = data[n];
    }
    usb_ring_buffer[ep].packet[usb_ring_buffer[ep].head].len = chunk;
    usb_ring_buffer[ep].head = (usb_ring_buffer[ep].head + 1) % 16;
    data += chunk;
    len -= chunk;
  }
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
  char packet[64];
  usb_read(0, packet);

  uint8_t bmRequestType = packet[0];
  uint8_t bRequest      = packet[1];

  // Descriptor request
  if(bmRequestType == 0x80 && bRequest == 0x06) {
    uint8_t descriptor_type = packet[3];
    uint8_t descriptor_idx  = packet[2];

    if(descriptor_type == 1 && descriptor_idx == 0) {
      uint32_t len = 18;
      if(len > packet[6]) len = packet[6];
      usb_write_string(0, device_descriptor, len);
    }
    if(descriptor_type == 2 && descriptor_idx == 0) {
      uint32_t len = config_descriptor[2];
      if(len > packet[6]) len = packet[6];
      usb_write_string(0, config_descriptor, len);
    }
    if(descriptor_type == 3 && descriptor_idx == 0) {
      uint32_t len = string_descriptor[0];
      if(len > packet[6]) len = packet[6];
      usb_write_string(0, string_descriptor, len);
    } else if (descriptor_type == 3){
      usb_write(0,0,0);
    }
  }
  if(bmRequestType == 0x21) {
    usb_write(0,0,0);
  }
  // Set address
  if(bmRequestType == 0x00 && bRequest == 0x05) {
    pending_addr = packet[2];
    usb_write(0,0,0);
  }
  // Set configuration
  if(bmRequestType == 0x00 && bRequest == 0x09) {
    usb_write(0,0,0);
  }
}

void usb_handle_ep1() {
  char packet[64];
  uint8_t len = usb_read(1, packet);
  if(len) {
    if(packet[0] == 'p') {
      uint8_t setting1 = *(volatile uint32_t *)(FLASH_BASE + 31 * 0x800);
      usb_printf("Setting 1: %d\n", setting1);
    } else if(packet[0] == 's') {
      uint8_t setting1 = atoi(packet+1);
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
      // Write setting 1
      FLASH->CR &= ~FLASH_CR_PER;
      FLASH->CR |= FLASH_CR_PG;
      *(volatile uint32_t *)(FLASH_BASE + 31 * 0x800) = setting1;
      *(volatile uint32_t *)(FLASH_BASE + 31 * 0x800 + 4) = setting1;
      // Wait for flash to be ready
      while(FLASH->SR & FLASH_SR_BSY);
      // Lock flash memory
      FLASH->CR |= FLASH_CR_LOCK;
      __enable_irq();
      usb_printf("Setting 1 saved\n");
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

  if(ep_tx_ready(0)) {
    // Copy data from the ring buffer into the endpoint buffer.
    if(usb_ring_buffer[0].head != usb_ring_buffer[0].tail) {
      usb_write(0, usb_ring_buffer[0].packet[usb_ring_buffer[0].tail].data, usb_ring_buffer[0].packet[usb_ring_buffer[0].tail].len);
      usb_ring_buffer[0].tail = (usb_ring_buffer[0].tail + 1) % 16;
    }
  }
  if(ep_tx_ready(1)) {
    // Copy data from the ring buffer into the endpoint buffer.
    if(usb_ring_buffer[1].head != usb_ring_buffer[1].tail) {
      usb_write(1, usb_ring_buffer[1].packet[usb_ring_buffer[1].tail].data, usb_ring_buffer[1].packet[usb_ring_buffer[1].tail].len);
      usb_ring_buffer[1].tail = (usb_ring_buffer[1].tail + 1) % 16;
    }
  }
}

void usb_printf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[64];
  vsprintf(buffer, format, args);
  usb_write_string(1, buffer, strlen(buffer));
  va_end( args );
}
