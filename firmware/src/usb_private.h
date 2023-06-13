// Totally generic device descriptor
char device_descriptor[] = {
  0x12,
  0x01,
  0x10,0x01,
  2, 0, 0,
  0x40,
  0x09,0x12,
  0x01,0x00,
  0,1,
  1,2,3,
  1
};

// Describes a standard virtual COM port
char config_descriptor[] = {
/*Configuation Descriptor*/
  0x09,   /* bLength: Configuation Descriptor size */
  2,      /* bDescriptorType: Configuration */
  9+9+7,     /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0x80,   /* bmAttributes: bus powered */
  50,     /* MaxPower 100 mA */

/* Serial Data interface */
  0x09 ,   /* bLength: Endpoint Descriptor size */
  4,   /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */

/*TX Endpoint Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  5,   /* bDescriptorType: Endpoint */
  0x81,   /* bEndpointAddress: (IN2) */
  0x02,   /* bmAttributes: Bulk */
  64,     /* wMaxPacketSize: */
  0x00,
  0x01,   /* bInterval: */
};