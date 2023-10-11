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
#include "airmode.h"
#include "main.h"
#include "barometer.h"

// Set up all the hardware. Each part of the hardware has its own
// module of code with its own init function.
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
  imu_zero();
  baro_init();
  adc_init();
  flash_init();
  blackbox_init();
}

// This struct contains the data that will be output to the four motors.
static struct dshot_data dshot;

// Create a ring buffer of the last GYRO_BUFFER_SIZE gyro readings.
// This will be used to calculate the D term.
#define GYRO_BUFFER_SIZE 60
static struct gyro_data gyro_buffer[GYRO_BUFFER_SIZE];
static uint8_t gyro_buffer_index = 0;

int main(void) {
  // This flag inhibits arming at power-on, and is set to 1 when it's safe to arm.
  uint8_t arming_allowed = 0;

  // These variables are used to store the accumulated integral term for each axis.
  float i_pitch = 0;
  float i_roll  = 0;
  float i_yaw   = 0;

  // A counter to keep track of the number of frames since arming. Used only by the gyro calibration routine.
  uint32_t arm_counter = 0;

  // A continuously increasing counter that is used to timestamp blackbox frames.
  uint32_t frame_count = 0;

  // LED2 is the power LED, turn it on before we do anything else.
  led2_on();

  // Load settings from flash.
  settings_read();

  // Main loop. This runs as fast as possible, but the majority of the code is located
  // within an if block that only runs each time data is received from the gyro.
  while(1) {
    // Poll the USB peripheral to transmit and receive data as needed.
    usb_main();
    // Poll the UART peripheral to transmit pending data. Receiving of data is handled by interrupts.
    uart_tx();
    // Process MSP requests and transmit responses. This also transmits OSD data.
    msp_send_response();

    // Each time the gyro has new data, we run the main control loop.
    if(gyro_ready()) {
      // Fetch settings and scale them to the appropriate units.
      volatile struct settings *settings = settings_get();
      float angle_rate    = 10.f      * settings->angle_rate;
      float acro_rate     = 0.01f     * settings->acro_rate;
      float p             = 0.001f    * settings->p;
      float i             = 0.000001f * settings->i;
      float d             = 0.001f    * settings->d;
      float yaw_p         = 0.001f    * settings->yaw_p;
      float yaw_i         = 0.000001f * settings->yaw_i;
      float expo          = 0.01f     * settings->expo;
      float yaw_expo      = 0.01f     * settings->yaw_expo;
      float throttle_gain = 0.01f     * settings->throttle_gain;
      float throttle_min  =             settings->throttle_min;

      // If we have valid ELRS data and it indicates that we're currently disarmed, then we
      // set a flag to allow arming. This prevents accidental arming at power-on.
      if(elrs_valid() && elrs_channel(4) < 0) {
        arming_allowed = 1;
      }

      // If we have valid ELRS data, and the arming switch is set,
      // and we've prevously disarmed, go ahead and start the motors.
      if(elrs_valid() && elrs_channel(4) > 0 && arming_allowed) {
        // LED1 indicates the arming state.
        led1_on();
        // If we've been armed for less than 200 cycles, don't actually arm the
        // motors yet. Instead, spend a a short time calibrating the gyro.
        if(arm_counter < 200) {
          gyro_calibrate();
          arm_counter++;
          dshot.armed = 0;
        } else {
          // Gyro calibration is finished.
          // Set the armed flag in the DSHOT output to 1.
          dshot.armed = 1;
        }
      } else {
        // LED1 indicates the arming state.
        led1_off();
        // Set the armed flag in the DSHOT output to 0.
        dshot.armed = 0;
        // If we are not armed, reset the integral terms.
        i_pitch = 0;
        i_roll  = 0;
        i_yaw   = 0;
        // Reset the arm counter and gyro calibration.
        arm_counter = 0;
        gyro_zero();
        // Zero the IMU orientation.
        // We assume that when we're disarmed, we're sitting on a level surface.
        imu_zero();
      }

      // Call elrs_tick() at regular intervals. This allows it to count down an internal
      // timer to implement failsafe behavior if no ELRS data is received.
      elrs_tick();

      // These structs will contain the gyro and accelerometer data.
      struct gyro_data gyro;
      struct gyro_data accel;

      // Read the gyro and accelerometer data.
      gyro_read(&gyro);
      accel_read(&accel);

      // The IMU uses input from both the gyro and accelerometer to estimate the orientation.
      // Update the IMU using the gyro and accelerometer data.
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      // These variables will store the requested rotation rate for each axis.
      // The units of these variables are gyro units. 16.4 units = 1 degree per second.
      int32_t rotation_request_pitch = 0;
      int32_t rotation_request_roll  = 0;
      int32_t rotation_request_yaw   = 0;

      // Set the throttle. We take the throttle channel from the transmitter, offset it
      // by 820 so that it's zero at the bottom rather than the midpoint. Then we scale
      // it by the throttle gain and add the minimum throttle.
      int32_t throttle = throttle_gain * (elrs_channel(2) + 820) + throttle_min;

      // AUX2 is the mode switch. If it's high, we're in angle mode, otherwise we're in rate mode.
      if(elrs_channel(5) > 0) {
        // Angle mode

        // Get the current X and Y tilt angles from the IMU.
        float tilt_pitch, tilt_roll;
        imu_get_xy_tilt(&tilt_pitch, &tilt_roll);

        // Normalise the elrs input to +/- 1.
        float target_pitch = elrs_channel(1) / 820.f;
        float target_roll  = elrs_channel(0) / 820.f;
        if(target_pitch >  1.f) target_pitch =  1.f;
        if(target_pitch < -1.f) target_pitch = -1.f;
        if(target_roll  >  1.f) target_roll  =  1.f;
        if(target_roll  < -1.f) target_roll  = -1.f;

        // Map the input angles from a square to a circle.
        target_pitch = target_pitch * sqrtf(1.f - target_roll * target_roll / 2.f);
        target_roll  = target_roll  * sqrtf(1.f - target_pitch * target_pitch / 2.f);

        // Multiply the input angles by the configured angle limit to get the target angles.
        target_pitch *= 1.f;
        target_roll  *= 1.f;

        // Subtract the tilt angle from the requested angle to get the angle error.
        float angle_error_pitch = target_pitch - tilt_pitch;
        float angle_error_roll  = target_roll  - tilt_roll;

        // Multiply the angle error by the configured angle rate to get the required rotation rate.
        rotation_request_pitch = angle_error_pitch * angle_rate;
        rotation_request_roll  = angle_error_roll  * angle_rate;
        rotation_request_yaw   = elrs_channel(3)   * acro_rate;
      } else {
        // Rate mode.
        // Apply expo to roll input.
        // TODO: split expo into a separate function.
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

        // Multiply the transmitter input by the configured acro rate to get the final rotation rate.
        rotation_request_pitch = transmitter_pitch * acro_rate;
        rotation_request_roll  = transmitter_roll  * acro_rate;
        rotation_request_yaw   = transmitter_yaw   * acro_rate;
      }

      // Calculate the error in angular velocity by subtracting the gyro readings from the requested
      // angular velocity. It looks like an additon but the gyro values are negative.
      int32_t error_pitch = p     * (gyro.x + rotation_request_pitch);
      int32_t error_roll  = p     * (gyro.y + rotation_request_roll);
      int32_t error_yaw   = yaw_p * (gyro.z + rotation_request_yaw);

      // Put the current gyro value into the ring buffer. This allows us to compare
      // it with previous gyro values to calculate the D term below.
      gyro_buffer[gyro_buffer_index] = gyro;
      // Increment the ring buffer index, and wrap it around if necessary.
      gyro_buffer_index++; if(gyro_buffer_index >= GYRO_BUFFER_SIZE) gyro_buffer_index = 0;

      // Calculate the difference between the current gyro value and the value from GYRO_BUFFER_SIZE loops
      // ago to find the rate of change in the gyro data, and multiply by the D gain to get the D term.
      float d_pitch = (gyro.x - gyro_buffer[gyro_buffer_index].x) * d;
      float d_roll  = (gyro.y - gyro_buffer[gyro_buffer_index].y) * d;

      // Integrate the error in each axis.
      // Don't integrate pitch and roll if the D term exceeds a specified limit.
      if(d_pitch < 100 && d_pitch > -100)
        i_pitch += i * (gyro.x + rotation_request_pitch);

      if(d_roll < 100 && d_roll > -100)
        i_roll  += i     * (gyro.y + rotation_request_roll);

      i_yaw   += yaw_i * (gyro.z + rotation_request_yaw);

      // Calculate the motor outputs. The motor_outputs array contains the following elements.
      // 0: no output
      // 1: rear left
      // 2: front right
      // 3: front left
      // 4: rear right
      int32_t motor_outputs[5];
      // Motor zero is always off. This provides a convenient way to disable motors during configuration and testing.
      motor_outputs[0] = 0;
      // The yaw calculations need to be different for a props-in configuration vs a props-out configuration.
      if(settings->motor_direction) {
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

      // Activate air mode to keep motor outputs within a good range while maintaining differential.
      air_mode(motor_outputs);

      // Map the appropriate motor output to each DSHOT channel.
      dshot.motor1 = motor_outputs[settings->motor1];
      dshot.motor2 = motor_outputs[settings->motor2];
      dshot.motor3 = motor_outputs[settings->motor3];
      dshot.motor4 = motor_outputs[settings->motor4];

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);

      // Prepare a blackbox data frame to log the data for this loop iteration.
      struct blackbox_frame blackbox_data;
      blackbox_data.timestamp = frame_count;
      blackbox_data.setpoint[0] = rotation_request_pitch;
      blackbox_data.setpoint[1] = rotation_request_roll;
      blackbox_data.setpoint[2] = rotation_request_yaw;
      blackbox_data.gyro_data[0] = gyro.x;
      blackbox_data.gyro_data[1] = gyro.y;
      blackbox_data.gyro_data[2] = gyro.z;
      blackbox_data.p[0] = error_pitch;
      blackbox_data.p[1] = error_roll;
      blackbox_data.p[2] = error_yaw;
      blackbox_data.i[0] = i_pitch;
      blackbox_data.i[1] = i_roll;
      blackbox_data.i[2] = i_yaw;
      blackbox_data.d[0] = d_pitch;
      blackbox_data.d[1] = d_roll;
      blackbox_data.d[2] = 0;
      blackbox_data.motor[0] = dshot.motor1;
      blackbox_data.motor[1] = dshot.motor2;
      blackbox_data.motor[2] = dshot.motor3;
      blackbox_data.motor[3] = dshot.motor4;

      // If we're armed, write the log data to the blackbox.
      if(dshot.armed) blackbox_write(&blackbox_data);

      // Increment the loop counter
      frame_count++;
    }
  }
  // This should never be reached.
  return 0;
}

// Return whether the quad is currently armed.
int main_get_armed_state() {
  return dshot.armed;
}
