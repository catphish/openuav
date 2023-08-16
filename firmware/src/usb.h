#pragma once
#include <stdint.h>

void usb_init();
void usb_main();
uint32_t ep_tx_ready(uint32_t ep);
uint32_t ep_rx_ready(uint32_t ep);
uint8_t usb_read(uint8_t ep, char * buffer);
void usb_write(uint8_t ep, char * buffer, uint32_t len);
void usb_printf(const char* format, ...);
uint8_t usb_write_ready(uint8_t ep);
void usb_write_string(uint8_t ep, char * data, uint32_t len);