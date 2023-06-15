#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "led.h"
#include "util.h"
#include "dfu.h"

void SystemInit(void) {
  gpio_init();
  led_init();
  usb_init();
  dshot_init();
  dfu_init();
}

int main(void) {
  while(1) {
    usb_main();
    dfu_main();
    usb_printf("Hello World: %d!\n", 1234);
    msleep(100);
    led0_toggle();
  }
  return 0;
}