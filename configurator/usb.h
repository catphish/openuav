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
#define USB_SETTING_BATT_CHEMISTRY       0x02

int usb_init();
uint32_t get_setting(uint8_t category, uint8_t index);
void set_setting(uint8_t category, uint8_t index, uint32_t value);
void save_settings();
void load_settings();
void read_flash(uint32_t address, uint8_t *data);
void erase_flash(uint16_t block);
void usb_close();
