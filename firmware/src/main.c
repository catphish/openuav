#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "led.h"
#include "util.h"

void SystemInit(void) {
  gpio_init();
  led_init();
  usb_init();
  dshot_init();
}

int main(void) {
  while(1) {
    usb_main();
    usb_printf("Hello World: %d!\n", 1234);
    led0_toggle();
    led1_toggle();
  }
  return 0;
}