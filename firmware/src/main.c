#include <stm32g4xx.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"
#include "led.h"
#include "util.h"
#include "spi.h"
#include "elrs.h"
#include "uart.h"
#include "clock.h"
#include "gyro.h"
#include "imu.h"
#include "quaternion.h"
#include "adc.h"
#include "msp.h"
#include "settings.h"

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
  adc_init();
}

struct dshot_data dshot;
struct gyro_data gyro;
struct gyro_data accel;
struct gyro_data prev_gyro;

uint8_t arming_allowed = 0;

// Gyro integral terms.
static float i_pitch = 0;
static float i_roll  = 0;
static float i_yaw   = 0;

int main(void) {
  led1_on();
  settings_load();
  while(1) {
    // Poll the USB peripherand to transmit and receive data.
    usb_main();
    // Poll the UART peripheral to transmit pending data.
    uart_tx();

    msp_send_response();

    if(gyro_ready()) {
      // Fetch settings
      float angle_rate    = 0.01f     * settings_get()->angle_rate;
      float acro_rate     = 0.01f     * settings_get()->acro_rate;
      float p             = 0.001f    * settings_get()->p;
      float i             = 0.000001f * settings_get()->i;
      float d             = 0.001f    * settings_get()->d;
      float yaw_p         = 0.001f    * settings_get()->yaw_p;
      float yaw_i         = 0.000001f * settings_get()->yaw_i;
      float throttle_gain = 0.01f     * settings_get()->throttle_gain;
      float throttle_min  =             settings_get()->throttle_min;

      // If we're disarmed and we have valid ELRS data, allow the ESCs to be armed.
      if(elrs_valid() && elrs_channel(4) < 0) {
        arming_allowed = 1;
      }

      // If we have valid ELRS data, allow the ESCs to be armed.
      if(elrs_valid() && elrs_channel(4) > 0 && arming_allowed) {
        dshot.armed = 1;
        led0_on();
      } else {
        // If we are not armed, reset the integral terms.
        dshot.armed = 0;
        i_pitch = 0;
        i_roll  = 0;
        i_yaw   = 0;
        // Recalibrate the IMU.
        gyro_zero();
        imu_init();
        led0_off();
      }

      // Call elrs_tick() regularly to allow a fialsafe timeout.
      elrs_tick();

      // Read the raw gyro and accelerometer data.
      gyro_read(&gyro);
      accel_read(&accel);

      // Update the IMU using the gyro and accelerometer data.
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      int32_t rotation_request_pitch = 0;
      int32_t rotation_request_roll  = 0;
      int32_t rotation_request_yaw   = 0;

      // Set the throttle. This will ultimately use the controller input, altitude, and attitude.
      // Currently I use 50% throttle and add 200 for "air mode".
      int32_t throttle = throttle_gain * (elrs_channel(2) + 820) + throttle_min;

      if(elrs_channel(5) > 0) {
        // Angle mode

        // Get the current X and Y tilt angles, and the z rotation offset from the IMU.
        float tilt_pitch, tilt_roll;
        imu_get_xy_tilt(&tilt_pitch, &tilt_roll);

        // Subtract the tilt angle from the requested angle to get the angle error.
        // The units here are arbitrary, but 800 permits a good range of motion.
        int32_t angle_error_pitch = elrs_channel(1) - tilt_pitch * 800.f;
        int32_t angle_error_roll  = elrs_channel(0) - tilt_roll  * 800.f;

        rotation_request_pitch = angle_error_pitch * angle_rate;
        rotation_request_roll  = angle_error_roll  * angle_rate;
        rotation_request_yaw   = elrs_channel(3)   * acro_rate;
      } else {
        // Rate mode. Get angle requests from the controller.
        rotation_request_pitch = elrs_channel(1) * acro_rate;
        rotation_request_roll  = elrs_channel(0) * acro_rate;
        rotation_request_yaw   = elrs_channel(3) * acro_rate;
      }

      // Calculate the error in angular velocity by adding the (nagative) gyro
      // readings from the requested angular velocity.
      int32_t error_pitch = p     * (gyro.x + rotation_request_pitch);
      int32_t error_roll  = p     * (gyro.y + rotation_request_roll);
      int32_t error_yaw   = yaw_p * (gyro.z + rotation_request_yaw);

      // Integrate the error in each axis.
      i_pitch += i     * (gyro.x + rotation_request_pitch);
      i_roll  += i     * (gyro.y + rotation_request_roll);
      i_yaw   += yaw_i * (gyro.z + rotation_request_yaw);

      // Calculate the difference between the current and previous gyro readings.
      int32_t d_pitch = d * (gyro.x - prev_gyro.x);
      int32_t d_roll  = d * (gyro.y - prev_gyro.y);
      // Store the current gyro readings for the next loop.
      prev_gyro = gyro;

      // TODO: This should be configurable.
      #define PROPS_OUT

      // For each motor, add all appropriate terms together to get the final output.
      #ifdef PROPS_IN
      int32_t motor_rear_left   = throttle + error_pitch + error_roll + error_yaw + i_pitch + i_roll + i_yaw + d_pitch + d_roll;
      int32_t motor_front_right = throttle - error_pitch - error_roll + error_yaw - i_pitch - i_roll + i_yaw - d_pitch - d_roll;
      int32_t motor_front_left  = throttle - error_pitch + error_roll - error_yaw - i_pitch + i_roll - i_yaw - d_pitch + d_roll;
      int32_t motor_rear_right  = throttle + error_pitch - error_roll - error_yaw + i_pitch - i_roll - i_yaw + d_pitch - d_roll;
      #endif

      #ifdef PROPS_OUT
      int32_t motor_rear_left   = throttle + error_pitch + error_roll - error_yaw + i_pitch + i_roll - i_yaw + d_pitch + d_roll;
      int32_t motor_front_right = throttle - error_pitch - error_roll - error_yaw - i_pitch - i_roll - i_yaw - d_pitch - d_roll;
      int32_t motor_front_left  = throttle - error_pitch + error_roll + error_yaw - i_pitch + i_roll + i_yaw - d_pitch + d_roll;
      int32_t motor_rear_right  = throttle + error_pitch - error_roll + error_yaw + i_pitch - i_roll + i_yaw + d_pitch - d_roll;
      #endif

      // Configure motor mappings
      dshot.motor1 = motor_rear_left;
      dshot.motor2 = motor_front_left;
      dshot.motor3 = motor_rear_right;
      dshot.motor4 = motor_front_right;

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);
    }
  }
  return 0;
}
