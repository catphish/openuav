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
#include "barometer.h"
#include "i2c.h"
#include "adc.h"
#include "msp.h"
#include "gps.h"
#include "mag.h"

// ANGLE_RATE is a measure of how fast the quad will rotate in angle mode.
#define ANGLE_RATE 5.0f
// RATE is a measure of how fast the quad will rotate in rate mode.
#define RATE 6.0f

// These are regular PI gains
#define RATE_P 0.25f
#define RATE_I 0.0002f

// These are the PI gains for the Z axis.
#define RATE_ZP 1.0f
#define RATE_ZI 0.0008f

// THROTGAIN is a throttle multiplier. Useful values are between 0.5 and 1.0
#define THROTGAIN 0.8f
// AIRBOOST is a minimum throttle to be applied when the quad is armed.
#define AIRBOOST 200

// Enable magnetometer support (QMC5883L).
#define USE_MAG
// Enable GPS support (UBX protocol).
#define USE_GPS
// Enable barometer for altitude hold.
#define USE_BARO

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
  #ifdef USE_BARO
  baro_init();
  #endif
  #ifdef USE_MAG
  i2c_init();
  mag_init();
  #endif
  imu_init();
  adc_init();
}

struct dshot_data dshot;
struct gyro_data gyro;
struct gyro_data accel;
struct mag_data mag;
float heading = 0;

// Gyro integral terms.
static float i_pitch = 0;
static float i_roll  = 0;
static float i_yaw   = 0;

int32_t gps_zero_lat = 0;
int32_t gps_zero_lon = 0;
uint32_t loop_count = 0;
int32_t target_pressure = 0;
int32_t prev_pressure = 0;

char osd_mv[10];

int main(void) {
  while(1) {
    // Poll the USB peripherand to transmit and receive data.
    usb_main();
    // Poll the UART peripheral to transmit pending data.
    uart_tx();

    msp_send_response();

    if(gyro_ready()) {
      // Increment the loop counter.
      loop_count++;
      // If we have valid ELRS data, allow the ESCs to be armed.
      if(elrs_valid() && elrs_channel(4) > 0)
        dshot.armed = 1;
      else {
        // If we are not armed, reset the integral terms.
        dshot.armed = 0;
        i_pitch = 0;
        i_roll  = 0;
        i_yaw   = 0;
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

      #ifdef USE_MAG
      // Update the IMU from the magnetometer.
      mag_read(&mag);
      heading = imu_heading(&mag);
      #endif

      int32_t rotation_request_pitch = 0;
      int32_t rotation_request_roll  = 0;
      int32_t rotation_request_yaw   = 0;

      // Set the throttle. This will ultimately use the controller input, altitude, and attitude.
      // Currently I use 50% throttle and add 200 for "air mode".
      int32_t throttle = THROTGAIN * (elrs_channel(2) + 820) + AIRBOOST;

      if(elrs_channel(5) > 0) {
        // Angle mode

        #ifdef USE_BARO
        // Fetch barometer data.
        int32_t pressure = baro_read_pressure();
        // Set the pressure at ground level.
        if(target_pressure == 0) target_pressure = pressure;
        // Calculate a P error term
        int32_t pressure_error = pressure - target_pressure;
        throttle += pressure_error / 10;
        // Calculate a D term
        int32_t pressure_d = pressure - prev_pressure;
        throttle += pressure_d * 10;
        prev_pressure = pressure;
        #endif

        // Get the current X and Y tilt angles, and the z rotation offset from the IMU.
        float tilt_pitch, tilt_roll;
        imu_get_xy_tilt(&tilt_pitch, &tilt_roll);

        // Subtract the tilt angle from the requested angle to get the angle error.
        // The units here are arbitrary, but 800 permits a good range of motion.
        int32_t angle_error_pitch = elrs_channel(1) - tilt_pitch * 800.f;
        int32_t angle_error_roll  = elrs_channel(0) - tilt_roll  * 800.f;

        #ifdef USE_GPS
        gps_filter();
        // See if GPS position hold is enabled and we have a lock.
        if(elrs_channel(7) > 0 && gps_lat() && gps_lon()) {
          // If we haven't already done so, record the home position.
          if(!gps_zero_lat || !gps_zero_lon) {
            gps_zero_lat = gps_lat();
            gps_zero_lon = gps_lon();
          }
          // If we see control inputs, reset the counter to disable GPS hold and reset the home position after some time.
          if(elrs_channel(0) > 20 || elrs_channel(0) < -20 || elrs_channel(1) > 20 || elrs_channel(1) < -20) {
            loop_count = 0;
          }
          if(loop_count < 800) {
            // If we have just enabled GPS hold, reset the home position.
            gps_zero_lat = gps_lat();
            gps_zero_lon = gps_lon();
          }
          
          // Calculate the absolute position error (GPS P term).
          float gps_error_lat = gps_lat() - gps_zero_lat;
          float gps_error_lon = 0.65f * (gps_lon() - gps_zero_lon);
          // Scale the P term.
          gps_error_lat *= 0.4f;
          gps_error_lon *= 0.4f;

          // Limit the P term.
          float magnitude = sqrtf(gps_error_lat * gps_error_lat + gps_error_lon * gps_error_lon);
          if(magnitude > 250.f) {
            gps_error_lat = gps_error_lat * 250.f / magnitude;
            gps_error_lon = gps_error_lon * 250.f / magnitude;
          }

          // Scale the D term
          float gps_delta_lat = gps_lat_d() * 10.f;
          float gps_delta_lon = gps_lon_d() * 10.f * 0.65f;

          // Combine the P and D terms to create a correction (ground frame).
          int32_t angle_error_lat = gps_error_lat + gps_delta_lat;
          int32_t angle_error_lon = gps_error_lon + gps_delta_lon;

          // Rotate the angle error into the quad frame.
          int32_t gps_angle_error_pitch = angle_error_lat * cosf(heading) + angle_error_lon * sinf(heading);
          int32_t gps_angle_error_roll  = angle_error_lon * cosf(heading) - angle_error_lat * sinf(heading);

          // Add the angle error to the requested angle.
          angle_error_pitch += gps_angle_error_pitch;
          angle_error_roll  += gps_angle_error_roll;
        } else {
          // GPS hold is disabled, reset the home position.
          gps_zero_lat = 0;
          gps_zero_lon = 0;
        }
        #endif

        rotation_request_pitch = angle_error_pitch * ANGLE_RATE;
        rotation_request_roll  = angle_error_roll  * ANGLE_RATE;
        rotation_request_yaw   = elrs_channel(3)   * RATE;
      } else {
        // Rate mode. Get angle requests from the controller.
        rotation_request_pitch = elrs_channel(1) * RATE;
        rotation_request_roll  = elrs_channel(0) * RATE;
        rotation_request_yaw   = elrs_channel(3) * RATE;
        // GPS hold is always disabled in rate mode. Reset the home position.
        gps_zero_lat = 0;
        gps_zero_lon = 0;
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
      #define PROPS_IN

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
      dshot.motor1 = motor_front_left;
      dshot.motor2 = motor_front_right;
      dshot.motor3 = motor_rear_left;
      dshot.motor4 = motor_rear_right;

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);
    }
  }
  return 0;
}
