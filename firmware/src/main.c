#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "led.h"
#include "util.h"
#include "spi.h"
#include "elrs.h"
#include "uart.h"
#include "clock.h"
#include "gyro.h"

void SystemInit(void) {
  gpio_init();
  led_init();
  clock_init();
  usb_init();
  dshot_init();
  spi_init();
  uart_init();
  gyro_init();
}

int main(void) {
  while(1) {
    usb_main();
    struct dshot_data dshot;
    struct gyro_data gyro;
    if(elrs_valid() && elrs_channel(4) > 0)
      dshot.armed = 1;
    else
      dshot.armed = 0;
    if(gyro_ready()) {
      elrs_tick();
      gyro_read(&gyro);
      dshot.motor1 = elrs_channel(2) + 820 - gyro.x/4 - gyro.y/4 - gyro.z/2 + elrs_channel(0)/2 - elrs_channel(1)/2 - elrs_channel(3);
      dshot.motor2 = elrs_channel(2) + 820 - gyro.x/4 + gyro.y/4 + gyro.z/2 - elrs_channel(0)/2 - elrs_channel(1)/2 + elrs_channel(3);
      dshot.motor3 = elrs_channel(2) + 820 + gyro.x/4 - gyro.y/4 + gyro.z/2 + elrs_channel(0)/2 + elrs_channel(1)/2 + elrs_channel(3);
      dshot.motor4 = elrs_channel(2) + 820 + gyro.x/4 + gyro.y/4 - gyro.z/2 - elrs_channel(0)/2 + elrs_channel(1)/2 - elrs_channel(3);
      dshot_write(&dshot);
    }
  }
  return 0;
}
