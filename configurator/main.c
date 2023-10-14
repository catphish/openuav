#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <libusb.h>

#include "usb.h"

void get_all(void) {
  int value, raw_value;

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
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_ANGLE_RATE);
  printf("   Angle Rate: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_ACRO_RATE);
  printf("    Acro Rate: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_EXPO);
  printf("         Expo: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_YAW_EXPO);
  printf("     Yaw Expo: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_THROTTLE_GAIN);
  printf("Throttle Gain: %d\n", value);
  value = get_setting(USB_SETTING_CAT_CONTROL, USB_SETTING_CONTROL_THROTTLE_MIN);
  printf(" Throttle Min: %d\n", value);

  printf("\n            MOTOR\n");
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_1);
  printf("       Motor1: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_2);
  printf("       Motor2: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_3);
  printf("       Motor3: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_4);
  printf("       Motor4: %d\n", value);
  value = get_setting(USB_SETTING_CAT_MOTOR, USB_SETTING_MOTOR_DIRECTION);
  printf("    Direction: %d\n", value);

  printf("\n            BATTERY\n");
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_ADC_COEFFICIENT);
  printf("    ADC Coeff: %d\n", value);
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_CELL_COUNT);
  printf("   Cell Count: %d\n", value);
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_ADC1_COEFFICIENT);
  raw_value = get_setting(USB_SETTING_CAT_BATT, USB_VALUE_ADC1_RAW);
  printf("   ADC1 coeff: %d (current raw value: %d)\n", value, raw_value);
  value = get_setting(USB_SETTING_CAT_BATT, USB_SETTING_BATT_ADC2_COEFFICIENT);
  raw_value = get_setting(USB_SETTING_CAT_BATT, USB_VALUE_ADC2_RAW);
  printf("   ADC2 coeff: %d (current raw value: %d)\n", value, raw_value);
  printf("(raw value * coefficient / 1000 will give the mV/mA figure)");
}

int main(int argc, char *argv[])
{
  if (usb_init()) return 1;

  if (argc == 1) {
    printf("Usage: %s <command> [args]\n", argv[0]);
    printf("Commands:\n");
    printf("  get <category> <index>\n");
    printf("  set <category> <index> <value>\n");
    printf("  save\n");
    printf("  load\n");
    printf("  read <address>\n");
    printf("  erase\n");
    return 0;
  }

  if(strcmp(argv[1], "get") == 0) {
    if(argc == 4) {
      uint8_t category = atoi(argv[2]);
      uint8_t index = atoi(argv[3]);
      uint32_t value = get_setting(category, index);
      printf("%d\n", value);
    } else {
      get_all();
    }
  } else if(strcmp(argv[1], "set") == 0) {
    if(argc != 5) {
      printf("Usage: %s set <category> <index> <value>\n", argv[0]);
      return 1;
    }
    uint8_t category = atoi(argv[2]);
    uint8_t index = atoi(argv[3]);
    uint32_t value = atoi(argv[4]);
    set_setting(category, index, value);
  } else if(strcmp(argv[1], "save") == 0) {
    if(argc != 2) {
      printf("Usage: %s save\n", argv[0]);
      return 1;
    }
    save_settings();
  } else if(strcmp(argv[1], "load") == 0) {
    if(argc != 2) {
      printf("Usage: %s load\n", argv[0]);
      return 1;
    }
    load_settings();
  } else if(strcmp(argv[1], "read") == 0) {
    uint32_t prev_time = 0;
    printf("time,gyro[0],gyro[1],gyro[2],target[0],target[1],target[2],m[0],m[1],m[2],m[3],p[0],p[1],p[2],i[0],i[1],i[2],d[0],d[1],d[2]\n");
    for(int address=0; address<1024*1024*128; address+=64) {
      uint8_t data[64];
      read_flash(address, data);
      read_flash(address+32, data+32);
      uint32_t time = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
      if(time == 0xFFFFFFFF) break;
      if(time > prev_time) {
        float timef = time;
        timef /= 8000.f;
        printf("%f", timef);
        for(int n=0; n<19;n++) {
          int16_t v = data[2*n+4] | (data[2*n+5] << 8);
          int32_t value = v;
          if(n<3) value *= -1;
          if(n>=7 && n<=10) value *= 10;
          if(n>9) value *= 10;
          printf(",%i", value);
        }
        printf("\n");
        prev_time = time;
      }
    }
  } else if(strcmp(argv[1], "erase") == 0) {
    printf("Erasing flash ");
    for(int block=0; block<1024; block++) {
      printf(".");
      fflush(stdout);
      erase_flash(block);
    }
    printf(" Done\n");
  } else {
    printf("Unknown command: %s\n", argv[1]);
    return 1;
  }

  usb_close();
}
