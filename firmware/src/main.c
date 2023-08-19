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

// Set up all the hardware. Each devide has its own init function.
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

// This struct contains the data that will be output to the four motors.
struct dshot_data dshot;

// These structs contain the raw gyro and accelerometer data.
struct gyro_data gyro;
struct gyro_data accel;
// This struct contains the gyro data from the previous loop, which is used to calculate the D term.
struct gyro_data prev_gyro;

// This flag is set when it's safe to arm and inhibits arming at power-on.
uint8_t arming_allowed = 0;

// This flag is set when we are dumping flash memory over USB.
// This is a terrible implementation and will be replaced with a new USB interface soon.
volatile uint32_t dump_flash = 0;
uint32_t dump_flash_page = 0;
uint32_t dump_flash_offset = 0;

// These variables are used to store the accumulated integral terms for each axis.
static float i_pitch = 0;
static float i_roll  = 0;
static float i_yaw   = 0;

// A continuously increasing counter that is used to timestamp blackbox frames.
static uint32_t frame_count = 0;


// These variables are used to filter the D term. This will likely be replaced with a
// proper filter on both P and D terms within the gyro code.
static float d_pitch_filter = 0;
static float d_roll_filter  = 0;

int main(void) {
  // LED1 is the power LED, turn it on before we do anything else.
  led1_on();
  // Load settings from flash.
  settings_read();

  // Main loop. This runs as fast as possible, but the majority of the code is located
  // within an if block that only runs each time data is received from the gyro.
  while(1) {
    // Poll the USB peripheral to transmit and receive data as needed.
    usb_main();
    // Poll the UART peripheral to transmit pending data. Receive data is handled by interrupts.
    uart_tx();
    // Process MSP requests and transmit responses. This also transmits unsolicited canvas updates.
    msp_send_response();

    // Each time the gyro has new data, we run the main control loop.
    if(gyro_ready()) {
      // Fetch settings and scale them to the appropriate units.
      struct settings *settings = settings_get();
      float angle_rate    = 0.01f     * settings->angle_rate;
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

      // If we have valid ELRS data and it indicated that we're currently disarmed, then we
      // set a flag to allow arming. This prevents accidental arming at power-on.
      if(elrs_valid() && elrs_channel(4) < 0) {
        arming_allowed = 1;
      }

      // If we have valid ELRS data, and the arming switch is set, arm the motors.
      if(elrs_valid() && elrs_channel(4) > 0 && arming_allowed) {
        dshot.armed = 1;
        // LED0 indicated the arming state.
        led0_on();
      } else {
        // If we are not armed, reset the integral terms.
        dshot.armed = 0;
        i_pitch = 0;
        i_roll  = 0;
        i_yaw   = 0;
        // Zero the gyro and recalibrate the IMU wenever we're not armed.
        gyro_zero();
        imu_init();
        led0_off();
      }

      // Call elrs_tick() at regular intervals. This allows it to count down an internal
      // timer to implement failsafe behavior if no ELRS data is received.
      elrs_tick();

      // Read the raw gyro and accelerometer data.
      gyro_read(&gyro);
      accel_read(&accel);

      // Update the IMU using the gyro and accelerometer data.
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      // These variables will store the requested rotation rate for each axis.
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
        // Get the current X and Y tilt angles, and the z rotation offset from the IMU.
        float tilt_pitch, tilt_roll;
        imu_get_xy_tilt(&tilt_pitch, &tilt_roll);

        // Subtract the tilt angle from the requested angle to get the angle error.
        // The units here are arbitrary, but 800 permits a good range of motion.
        // This max angle should probably be configurable.
        int32_t angle_error_pitch = elrs_channel(1) - tilt_pitch * 800.f;
        int32_t angle_error_roll  = elrs_channel(0) - tilt_roll  * 800.f;

        // Multiply the angle error by the configured angle rate to get the required rotation rate.
        rotation_request_pitch = angle_error_pitch * angle_rate;
        rotation_request_roll  = angle_error_roll  * angle_rate;
        rotation_request_yaw   = elrs_channel(3)   * acro_rate;
      } else {
        // Rate mode. TODO: split expo into a separate function.
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

      // Calculate the error in angular velocity by adding the (negative) gyro
      // readings from the requested angular velocity.
      int32_t error_pitch = p     * (gyro.x + rotation_request_pitch);
      int32_t error_roll  = p     * (gyro.y + rotation_request_roll);
      int32_t error_yaw   = yaw_p * (gyro.z + rotation_request_yaw);

      // Integrate the error in each axis.
      i_pitch += i     * (gyro.x + rotation_request_pitch);
      i_roll  += i     * (gyro.y + rotation_request_roll);
      i_yaw   += yaw_i * (gyro.z + rotation_request_yaw);

      // Calculate the change in angular velocity since the last iteration and filter it.
      d_pitch_filter = d_pitch_filter * 0.99 + 0.01 * (gyro.x - prev_gyro.x);
      d_roll_filter  = d_roll_filter  * 0.99 + 0.01 * (gyro.y - prev_gyro.y);
      // Multiply the filtered change in angular velocity by the D gain to get the D term.
      int d_pitch = d_pitch_filter * d;
      int d_roll  = d_roll_filter  * d;

      // Store the current gyro readings for the next loop.
      prev_gyro = gyro;

      // Calculate the motor outputs. The motor_outputs array contains the following elements.
      // 0: no output
      // 1: rear left
      // 2: front right
      // 3: front left
      // 4: rear right
      int32_t motor_outputs[5];
      // Motor zero is always off. This provides a convenient way to disable a motor.
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

      // Map the appropriate motor output to each DSHOT channel.
      dshot.motor1 = motor_outputs[settings->motor1];
      dshot.motor2 = motor_outputs[settings->motor2];
      dshot.motor3 = motor_outputs[settings->motor3];
      dshot.motor4 = motor_outputs[settings->motor4];

      // Activate air mode to keep motor outputs within a good range while maintaining differential.
      air_mode(&dshot);

      // Write the motor outputs to the ESCs.
      dshot_write(&dshot);

      // Populate a blackbox frame with current data.
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

      // If we're armed, write the log data to the blackbox.
      if(dshot.armed) blackbox_write(&frame);
      
      // This is temporary code to dump the flash contents to USB.
      // It will soon be replaced with a better USB ineterface.
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
