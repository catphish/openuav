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
#include "i2c.h"
#include "adc.h"
#include "msp.h"

#define ANGLE_RATE 4.0f
#define RATE 6.0f

#define RATE_P 0.1f
#define RATE_I 0.0001f

#define RATE_ZP 0.1f
#define RATE_ZI 0.0001f

void SystemInit(void) {
  clock_init();
  msleep(200);
  gpio_init();
  led_init();
  usb_init();
  dshot_init();
  spi_init();
  uart_init();
  gyro_init();
  imu_init();
  baro_init();
  i2c_init();
  adc_init();
}

struct dshot_data dshot;
struct gyro_data gyro;
struct gyro_data accel;

// Gyro integral terms.
static float ix = 0;
static float iy = 0;
static float iz = 0;

char osd_mv[10];

int main(void) {
  while(1) {
    // Poll the USB peripherand to transmit and receive data.
    usb_main();
    // Poll the UART peripheral to transmit pending data.
    uart_tx();

    msp_send_response();

    if(gyro_ready()) {
      // If we have valid ELRS data, allow the ESCs to be armed.
      if(elrs_valid() && elrs_channel(4) > 0)
        dshot.armed = 1;
      else {
        // If we are not armed, reset the integral terms.
        dshot.armed = 0;
        ix = 0;
        iy = 0;
        iz = 0;
        // Recalibrate the IMU.
        gyro_zero();
        imu_init();
      }

      // Call elrs_tick() regularly to allow a fialsafe timeout.
      elrs_tick();

      // Read the raw gyro and accelerometer data.
      gyro_read(&gyro);
      accel_read(&accel);

      // Update the IMU using the gyro and accelerometer data.
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      // Get the current X and Y tilt angles, and the z rotation offset from the IMU.
      // TODO: This should be broken out into two functions, one for tilt and one for rotation.
      float x_tilt, y_tilt;
      imu_get_xy_tilt(&x_tilt, &y_tilt);

      int32_t rotation_request_x = 0;
      int32_t rotation_request_y = 0;
      int32_t rotation_request_z = 0;

      if(elrs_channel(5) > 0) {
        // Angle mode
        // Subtract the tilt angle from the requested angle to get the angle error.
        // The units here are arbitrary, but 800 permits a good range of motion.
        int32_t angle_error_x = elrs_channel(1) - x_tilt * 800.f;
        int32_t angle_error_y = elrs_channel(0) - y_tilt * 800.f;
        rotation_request_x = angle_error_x * ANGLE_RATE;
        rotation_request_y = angle_error_y * ANGLE_RATE;
        rotation_request_z = elrs_channel(3) * RATE;
      } else {
        // Rate mode. Get angle requests from the controller.
        rotation_request_x = elrs_channel(1) * RATE;
        rotation_request_y = elrs_channel(0) * RATE;
        rotation_request_z = elrs_channel(3) * RATE;
      }

      // Calculate the error in angular velocity by adding the (nagative) gyro
      // readings from the requested angular velocity.
      int32_t error_x = RATE_P * (gyro.x + rotation_request_x);
      int32_t error_y = RATE_P * (gyro.y + rotation_request_y);
      int32_t error_z = RATE_ZP * (gyro.z + rotation_request_z);

      // Integrate the error in each axis.
      ix += RATE_I * (gyro.x + rotation_request_x);
      iy += RATE_I * (gyro.y + rotation_request_y);
      iz += RATE_ZI * (gyro.z + rotation_request_z);

      // Set the throttle. This will ultimately use the controller input, altitude, and attitude.
      // Currently I use 50% throttle and add 200 for "air mode".
      int32_t throttle = (elrs_channel(2) + 820)/2 + 200;

      // For each motor, add all appropriate terms together to get the final output.
      dshot.motor1 = throttle + error_x + error_y - error_z + ix + iy - iz;
      dshot.motor2 = throttle - error_x + error_y + error_z - ix + iy + iz;
      dshot.motor3 = throttle + error_x - error_y + error_z + ix - iy + iz;
      dshot.motor4 = throttle - error_x - error_y - error_z - ix - iy - iz;

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);
    }
  }
  return 0;
}
