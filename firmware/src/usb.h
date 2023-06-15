#include <stdint.h>

struct USBBufTable {
  struct USBBufDesc {
    uint16_t txBufferAddr ;
    uint16_t txBufferCount ;
    uint16_t rxBufferAddr ;
    uint16_t rxBufferCount ;
  }
  ep_desc[8];
};

#define USBBUFTABLE ((volatile struct USBBufTable *)0x40006000)
#define USBBUFRAW ((volatile uint8_t *)0x40006000)
#define USB_EPR(n) (*(volatile uint16_t *)(USB_BASE + 4 * n))

void usb_init();
void usb_main();
uint32_t ep_tx_ready(uint32_t ep);
uint32_t ep_rx_ready(uint32_t ep);
void usb_read(uint8_t ep, char * buffer);
void usb_write(uint8_t ep, char * buffer, uint32_t len);
