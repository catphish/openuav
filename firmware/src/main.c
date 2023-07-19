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
#include "imu.h"
#include "quaternion.h"
#include "barometer.h"

void SystemInit(void) {
  gpio_init();
  led_init();
  clock_init();
  usb_init();
  dshot_init();
  spi_init();
  uart_init();
  gyro_init();
  imu_init_zero();
  baro_init();
}

struct dshot_data dshot;
struct gyro_data gyro;
struct gyro_data accel;

int32_t x_integral = 0;
int32_t y_integral = 0;
int32_t z_integral = 0;

int main(void) {
  while(1) {
    // Poll the USB peripherand to transmit and receive data.
    usb_main();
    // If we have valid ELRS data, allow the ESCs to be armed.
    if(elrs_valid() && elrs_channel(4) > 0)
      dshot.armed = 1;
    else {
      // If we are not armed, reset the integral terms.
      dshot.armed = 0;
      x_integral = 0;
      y_integral = 0;
      z_integral = 0;
      // Recalibrate the IMU.
      imu_init(&gyro);
    }
    if(gyro_ready()) {
      // Call elrs_tick() regularly to allow a fialsafe timeout.
      elrs_tick();
      // Read the raw gyro and accelerometer data.
      gyro_read(&gyro);
      accel_read(&accel);
      // Fetch barometer data.
      uint32_t pressure = baro_read_pressure();

      // Update the IMU using the gyro and accelerometer data.
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      // Get the requested angles from the transmitter.
      int32_t angle_request_x = elrs_channel(1);
      int32_t angle_request_y = elrs_channel(0);

      // Get the current X and Y tilt angles from the IMU.
      float x_tilt, y_tilt;
      imu_get_xy_tilt(&x_tilt, &y_tilt);

      // Subtract the tilt angle from the requested angle to get the angle error.
      // The units here are arbitrary, but 800 permits a good range of motion.
      int32_t angle_error_x = angle_request_x - x_tilt * 800.f;
      int32_t angle_error_y = angle_request_y - y_tilt * 800.f;

      // Calculate requested angular velocity from the attitude error.
      // The requested angular velocity of the Z axix is taken directly from the
      // transmitter.
      int32_t rotation_request_x = angle_error_x;
      int32_t rotation_request_y = angle_error_y;
      int32_t rotation_request_z = elrs_channel(3) * 4;

      // Calculate the error in angular velocity by adding the (nagative) gyro
      // readings from the requested angular velocity.
      int32_t error_x = gyro.x/4 + rotation_request_x;
      int32_t error_y = gyro.y/4 + rotation_request_y;
      int32_t error_z = gyro.z/1 + rotation_request_z;

      // Integrate the error in each axis.
      x_integral += error_x;
      y_integral += error_y;
      z_integral += error_z;

      // For each motor, add the appropriate error and integral terms together
      // to get the final motor output.
      dshot.motor1 = elrs_channel(2) + 820 - error_x + error_y - error_z - x_integral/512 + y_integral/512 - z_integral/512;
      dshot.motor2 = elrs_channel(2) + 820 - error_x - error_y + error_z - x_integral/512 - y_integral/512 + z_integral/512;
      dshot.motor3 = elrs_channel(2) + 820 + error_x + error_y + error_z + x_integral/512 + y_integral/512 + z_integral/512;
      dshot.motor4 = elrs_channel(2) + 820 + error_x - error_y - error_z + x_integral/512 - y_integral/512 - z_integral/512;

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);
    }
  }
  return 0;
}
