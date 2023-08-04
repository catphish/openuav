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

// ANGLE_RATE is a measure of how fast the quad will rotate in angle mode.
#define ANGLE_RATE 7.0f
// RATE is a measure of how fast the quad will rotate in rate mode.
// Rates are currently linear only.
// A value of 1.0 represents 57.4 degrees per second.
// A value of 7.0 represents 402 degrees per second.
#define RATE 7.0f

// These are regular PI gains
#define RATE_P 0.12f
#define RATE_I 0.0002f

// These are the PI gains for the Z axis.
#define RATE_ZP 0.12f
#define RATE_ZI 0.0004f

// THROTGAIN is a throttle multiplier. Useful values are between 0.5 and 1.0
#define THROTGAIN 0.8f
// AIRBOOST is a minimum throttle to be applied when the quad is armed.
#define AIRBOOST 200

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

uint8_t arming_allowed = 0;

// Gyro integral terms.
static float i_pitch = 0;
static float i_roll  = 0;
static float i_yaw   = 0;

int main(void) {
  led1_on();
  while(1) {
    // Poll the USB peripherand to transmit and receive data.
    usb_main();
    // Poll the UART peripheral to transmit pending data.
    uart_tx();

    msp_send_response();

    if(gyro_ready()) {
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
      int32_t throttle = THROTGAIN * (elrs_channel(2) + 820) + AIRBOOST;

      if(elrs_channel(5) > 0) {
        // Angle mode

        // Get the current X and Y tilt angles, and the z rotation offset from the IMU.
        float tilt_pitch, tilt_roll;
        imu_get_xy_tilt(&tilt_pitch, &tilt_roll);

        // Subtract the tilt angle from the requested angle to get the angle error.
        // The units here are arbitrary, but 800 permits a good range of motion.
        int32_t angle_error_pitch = elrs_channel(1) - tilt_pitch * 800.f;
        int32_t angle_error_roll  = elrs_channel(0) - tilt_roll  * 800.f;

        rotation_request_pitch = angle_error_pitch * ANGLE_RATE;
        rotation_request_roll  = angle_error_roll  * ANGLE_RATE;
        rotation_request_yaw   = elrs_channel(3)   * RATE;
      } else {
        // Rate mode. Get angle requests from the controller.
        rotation_request_pitch = elrs_channel(1) * RATE;
        rotation_request_roll  = elrs_channel(0) * RATE;
        rotation_request_yaw   = elrs_channel(3) * RATE;
      }

      // Calculate the error in angular velocity by adding the (nagative) gyro
      // readings from the requested angular velocity.
      int32_t error_pitch = RATE_P  * (gyro.x + rotation_request_pitch);
      int32_t error_roll  = RATE_P  * (gyro.y + rotation_request_roll);
      int32_t error_yaw   = RATE_ZP * (gyro.z + rotation_request_yaw);

      // Integrate the error in each axis.
      i_pitch += RATE_I  * (gyro.x + rotation_request_pitch);
      i_roll  += RATE_I  * (gyro.y + rotation_request_roll);
      i_yaw   += RATE_ZI * (gyro.z + rotation_request_yaw);

      // TODO: This should be configurable.
      #define PROPS_OUT

      // For each motor, add all appropriate terms together to get the final output.
      #ifdef PROPS_IN
      int32_t motor_rear_left   = throttle + error_pitch + error_roll + error_yaw + i_pitch + i_roll + i_yaw;
      int32_t motor_front_right = throttle - error_pitch - error_roll + error_yaw - i_pitch - i_roll + i_yaw;
      int32_t motor_front_left  = throttle - error_pitch + error_roll - error_yaw - i_pitch + i_roll - i_yaw;
      int32_t motor_rear_right  = throttle + error_pitch - error_roll - error_yaw + i_pitch - i_roll - i_yaw;
      #endif

      #ifdef PROPS_OUT
      int32_t motor_rear_left   = throttle + error_pitch + error_roll - error_yaw + i_pitch + i_roll - i_yaw;
      int32_t motor_front_right = throttle - error_pitch - error_roll - error_yaw - i_pitch - i_roll - i_yaw;
      int32_t motor_front_left  = throttle - error_pitch + error_roll + error_yaw - i_pitch + i_roll + i_yaw;
      int32_t motor_rear_right  = throttle + error_pitch - error_roll + error_yaw + i_pitch - i_roll + i_yaw;
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
