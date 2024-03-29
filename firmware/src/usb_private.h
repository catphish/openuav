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

// Totally generic device descriptor
char device_descriptor[] = {
  0x12,      /* bLength: Device Descriptor size */
  0x01,      /* bDescriptorType: Device */
  0x10,0x01, /* bcdUSB: USB 1.1 */
  0xFF,      /* bDeviceClass: Vendor Specific */
  0,         /* bDeviceSubClass: */
  0,         /* bDeviceProtocol: */
  0x40,      /* bMaxPacketSize0: 64 Bytes */
  0x09,0x12, /* idVendor  (0x1209) */
  0x00,0xFC, /* idProduct (0xFC00) */
  0,1,       /* bcdDevice: 1.00 */
  0,         /* iManufacturer: NULL */
  0,         /* iProduct: NULL */
  0,         /* Serial number: NULL */
  1          /* bNumConfigurations: 1 */
};

// A simple config descriptor with one IN endpoint.
char config_descriptor[] = {
/*Configuation Descriptor*/
  0x09,      /* bLength: Configuation Descriptor size */
  0x02,      /* bDescriptorType: Configuration */
  9+9+7+7,0, /* wTotalLength: 9+9+7+7 */
  0x01,      /* bNumInterfaces: 1 interface */
  0x01,      /* bConfigurationValue: Configuration value */
  0x00,      /* iConfiguration: NULL */
  0x80,      /* bmAttributes: bus powered */
  50,        /* MaxPower 100 mA */

  /* Data interface */
  0x09 ,  /* bLength: Endpoint Descriptor size */
  0x04,   /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting zero */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: Vendor specific */
  0x00,   /* bInterfaceSubClass */
  0x00,   /* bInterfaceProtocol */
  0x00,   /* iInterface: NULL */

  /* Endpoint 1 Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  0x05,   /* bDescriptorType: Endpoint */
  0x01,   /* bEndpointAddress: (OUT2) */
  0x02,   /* bmAttributes: Bulk */
  0x40,0, /* wMaxPacketSize: 64 bytes */
  0x00,   /* bInterval: Ignored */
  /* Endpoint 1 Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  0x05,   /* bDescriptorType: Endpoint */
  0x81,   /* bEndpointAddress: (IN2) */
  0x02,   /* bmAttributes: Bulk */
  0x40,0, /* wMaxPacketSize: 64 bytes */
  0x00    /* bInterval: Ignored */
};

// String descriptor zero
char string_descriptor[] = {
  0x04, /* bLength: String Descriptor size */
  0x03, /* bDescriptorType: DeviStringce */
  0x09,0x04
};
