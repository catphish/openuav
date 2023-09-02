#pragma once
#include <stdint.h>

void usb_init();
void usb_main();
uint32_t ep_tx_ready(uint32_t ep);
uint32_t ep_rx_ready(uint32_t ep);
uint8_t usb_read(uint8_t ep, char * buffer);
void usb_write(uint8_t ep, char * buffer, uint32_t len);

#define USB_COMMAND_SETTING_GET     0x01
#define USB_COMMAND_SETTING_SET     0x02
#define USB_COMMAND_SETTING_SAVE    0x03
#define USB_COMMAND_SETTING_LOAD    0x04
#define USB_COMMAND_FLASH_READ      0x11
#define USB_COMMAND_FLASH_ERASE     0x12

#define USB_SETTING_CAT_TUNE        0x01
#define USB_SETTING_CAT_CONTROL     0x02
#define USB_SETTING_CAT_MOTOR       0x03
#define USB_SETTING_CAT_BATT        0x04

#define USB_SETTING_TUNE_P          0x01
#define USB_SETTING_TUNE_I          0x02
#define USB_SETTING_TUNE_D          0x03
#define USB_SETTING_TUNE_YAW_P      0x04
#define USB_SETTING_TUNE_YAW_I      0x05

#define USB_SETTING_CONTROL_ANGLE_RATE    0x01
#define USB_SETTING_CONTROL_ACRO_RATE     0x02
#define USB_SETTING_CONTROL_EXPO          0x03
#define USB_SETTING_CONTROL_YAW_EXPO      0x04
#define USB_SETTING_CONTROL_THROTTLE_GAIN 0x05
#define USB_SETTING_CONTROL_THROTTLE_MIN  0x06

#define USB_SETTING_MOTOR_DIRECTION 0x01
#define USB_SETTING_MOTOR_1         0x02
#define USB_SETTING_MOTOR_2         0x03
#define USB_SETTING_MOTOR_3         0x04
#define USB_SETTING_MOTOR_4         0x05

#define USB_SETTING_BATT_ADC_COEFFICIENT 0x01
#define USB_SETTING_BATT_CELL_COUNT      0x02
