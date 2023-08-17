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
#include "flash.h"
#include "blackbox.h"

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
  flash_init();
}

struct dshot_data dshot;
struct gyro_data gyro;
struct gyro_data accel;
struct gyro_data prev_gyro;

uint8_t arming_allowed = 0;

volatile uint32_t dump_flash = 0;
uint32_t dump_flash_page = 0;
uint32_t dump_flash_offset = 0;

// Gyro integral terms.
static float i_pitch = 0;
static float i_roll  = 0;
static float i_yaw   = 0;

// D filter terms
static float d_pitch_filter = 0;
static float d_roll_filter  = 0;

// Air mode.
void air_mode() {
  // TODO: Make the limits configurable.
  if(dshot.motor1 < 100) {
    int32_t motor_error = 100 - dshot.motor1;
    dshot.motor1 = 100;
    dshot.motor2 += motor_error;
    dshot.motor3 += motor_error;
    dshot.motor4 += motor_error;
  }
  if(dshot.motor2 < 100) {
    int32_t motor_error = 100 - dshot.motor2;
    dshot.motor2 = 100;
    dshot.motor1 += motor_error;
    dshot.motor3 += motor_error;
    dshot.motor4 += motor_error;
  }
  if(dshot.motor3 < 100) {
    int32_t motor_error = 100 - dshot.motor3;
    dshot.motor3 = 100;
    dshot.motor1 += motor_error;
    dshot.motor2 += motor_error;
    dshot.motor4 += motor_error;
  }
  if(dshot.motor4 < 100) {
    int32_t motor_error = 100 - dshot.motor4;
    dshot.motor4 = 100;
    dshot.motor1 += motor_error;
    dshot.motor2 += motor_error;
    dshot.motor3 += motor_error;
  }
  if(dshot.motor1 > 2000) {
    int32_t motor_error = dshot.motor1 - 2000;
    dshot.motor1 = 2000;
    dshot.motor2 -= motor_error;
    dshot.motor3 -= motor_error;
    dshot.motor4 -= motor_error;
  }
  if(dshot.motor2 > 2000) {
    int32_t motor_error = dshot.motor2 - 2000;
    dshot.motor2 = 2000;
    dshot.motor1 -= motor_error;
    dshot.motor3 -= motor_error;
    dshot.motor4 -= motor_error;
  }
  if(dshot.motor3 > 2000) {
    int32_t motor_error = dshot.motor3 - 2000;
    dshot.motor3 = 2000;
    dshot.motor1 -= motor_error;
    dshot.motor2 -= motor_error;
    dshot.motor4 -= motor_error;
  }
  if(dshot.motor4 > 2000) {
    int32_t motor_error = dshot.motor4 - 2000;
    dshot.motor4 = 2000;
    dshot.motor1 -= motor_error;
    dshot.motor2 -= motor_error;
    dshot.motor3 -= motor_error;
  }
}

int main(void) {
  led1_on();
  settings_read();
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
      float expo          = 0.01f     * settings_get()->expo;
      float yaw_expo      = 0.01f     * settings_get()->yaw_expo;
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
        // Rate mode.
        // Apply expo to roll input.
        float transmitter_roll = elrs_channel(0);
        if(transmitter_roll >  820) transmitter_roll =  820;
        if(transmitter_roll < -820) transmitter_roll = -820;
        transmitter_roll = expo * transmitter_roll * transmitter_roll * transmitter_roll / 820.f / 820.f + (1.f - expo) * transmitter_roll;
        // Apply expo to pitch input.
        float transmitter_pitch = elrs_channel(1);
        if(transmitter_pitch >  820) transmitter_pitch =  820;
        if(transmitter_pitch < -820) transmitter_pitch = -820;
        transmitter_pitch = expo * transmitter_pitch * transmitter_pitch * transmitter_pitch / 820.f / 820.f + (1.f - expo) * transmitter_pitch;
        // Apply expo to yaw input.
        float transmitter_yaw = elrs_channel(3);
        if(transmitter_yaw >  820) transmitter_yaw =  820;
        if(transmitter_yaw < -820) transmitter_yaw = -820;
        transmitter_yaw = yaw_expo * transmitter_yaw * transmitter_yaw * transmitter_yaw / 820.f / 820.f + (1.f - yaw_expo) * transmitter_yaw;
        // Apply the requested rotation rate.
        rotation_request_pitch = transmitter_pitch * acro_rate;
        rotation_request_roll  = transmitter_roll  * acro_rate;
        rotation_request_yaw   = transmitter_yaw   * acro_rate;
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

      // Filter D term
      d_pitch_filter = d_pitch_filter * 0.99 + 0.01 * (gyro.x - prev_gyro.x);
      d_roll_filter  = d_roll_filter  * 0.99 + 0.01 * (gyro.y - prev_gyro.y);
      int d_pitch = d_pitch_filter * d;
      int d_roll  = d_roll_filter  * d;

      // Store the current gyro readings for the next loop.
      prev_gyro = gyro;

      // Calculate the motor outputs.
      // 0: no output
      // 1: rear left
      // 2: front right
      // 3: front left
      // 4: rear right
      int32_t motor_outputs[5];
      motor_outputs[0] = 0;
      if(settings_get()->motor_direction) {
        // Props out
        motor_outputs[1] = throttle + error_pitch + error_roll - error_yaw + i_pitch + i_roll - i_yaw + d_pitch + d_roll;
        motor_outputs[2] = throttle - error_pitch - error_roll - error_yaw - i_pitch - i_roll - i_yaw - d_pitch - d_roll;
        motor_outputs[3] = throttle - error_pitch + error_roll + error_yaw - i_pitch + i_roll + i_yaw - d_pitch + d_roll;
        motor_outputs[4] = throttle + error_pitch - error_roll + error_yaw + i_pitch - i_roll + i_yaw + d_pitch - d_roll;
      } else {
        // Props in
        motor_outputs[1] = throttle + error_pitch + error_roll + error_yaw + i_pitch + i_roll + i_yaw + d_pitch + d_roll;
        motor_outputs[2] = throttle - error_pitch - error_roll + error_yaw - i_pitch - i_roll + i_yaw - d_pitch - d_roll;
        motor_outputs[3] = throttle - error_pitch + error_roll - error_yaw - i_pitch + i_roll - i_yaw - d_pitch + d_roll;
        motor_outputs[4] = throttle + error_pitch - error_roll - error_yaw + i_pitch - i_roll - i_yaw + d_pitch - d_roll;
      }

      // Configure motor mappings
      dshot.motor1 = motor_outputs[settings_get()->motor1];
      dshot.motor2 = motor_outputs[settings_get()->motor2];
      dshot.motor3 = motor_outputs[settings_get()->motor3];
      dshot.motor4 = motor_outputs[settings_get()->motor4];

      // Activate air mode.
      air_mode();

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);

      // A continuously increasing frame count for logging.
      static uint32_t frame_count = 0;
 
      // Populate a blackbox frame.
      struct blackbox_frame frame;
      frame.timestamp = frame_count++;
      frame.gyro_data[0] = gyro.x;
      frame.gyro_data[1] = gyro.y;
      frame.gyro_data[2] = gyro.z;
      frame.setpoint[0] = rotation_request_pitch;
      frame.setpoint[1] = rotation_request_roll;
      frame.setpoint[2] = rotation_request_yaw;
      frame.p[0] = error_pitch;
      frame.p[1] = error_roll;
      frame.p[2] = error_yaw;
      frame.i[0] = i_pitch;
      frame.i[1] = i_roll;
      frame.i[2] = i_yaw;
      frame.d[0] = d_pitch;
      frame.d[1] = d_roll;
      frame.d[2] = 0;
      frame.motor[0] = dshot.motor1;
      frame.motor[1] = dshot.motor2;
      frame.motor[2] = dshot.motor3;
      frame.motor[3] = dshot.motor4;

      if(dshot.armed) blackbox_write(&frame);
      
      if(dump_flash) {
        if(usb_write_ready(1)) {
          uint8_t data[32];
          flash_page_read(dump_flash_page);
          flash_read(data, 32, dump_flash_offset);
          dump_flash_offset += 32;
          if(dump_flash_offset == 2048) {
            dump_flash_offset = 0;
            dump_flash_page++;
          }
          if(dump_flash_page == dump_flash) {
            dump_flash = 0;
            dump_flash_page = 0;
          }
          usb_write_string(1, (char*)data, 32);
        }
      }

    }
  }
  return 0;
}
