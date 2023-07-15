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
  struct dshot_data dshot;
  struct gyro_data gyro;
  int32_t x_integral = 0;
  int32_t y_integral = 0;
  int32_t z_integral = 0;
  while(1) {
    usb_main();
    if(elrs_valid() && elrs_channel(4) > 0)
      dshot.armed = 1;
    else {
      dshot.armed = 0;
      x_integral = 0;
      y_integral = 0;
      z_integral = 0;
    }
    if(gyro_ready()) {
      elrs_tick();
      gyro_read(&gyro);
      // Calculate the error for each axis
      int32_t error_x = gyro.x/4 + elrs_channel(1);
      int32_t error_y = gyro.y/4 - elrs_channel(0);
      int32_t error_z = gyro.z/1 + elrs_channel(3)*4;

      // Integrate the error in each axis
      x_integral += error_x;
      y_integral += error_y;
      z_integral += error_z;

      // Add the P and I terms to calculate the final motor outputs
      dshot.motor1 = elrs_channel(2) + 820 - error_x - error_y - error_z - x_integral/512 - y_integral/512 - z_integral/512;
      dshot.motor2 = elrs_channel(2) + 820 - error_x + error_y + error_z - x_integral/512 + y_integral/512 + z_integral/512;
      dshot.motor3 = elrs_channel(2) + 820 + error_x - error_y + error_z + x_integral/512 - y_integral/512 + z_integral/512;
      dshot.motor4 = elrs_channel(2) + 820 + error_x + error_y - error_z + x_integral/512 + y_integral/512 - z_integral/512;

      dshot_write(&dshot);
    }
  }
  return 0;
}
