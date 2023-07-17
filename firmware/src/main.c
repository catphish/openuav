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
}

extern Quaternion q;

int main(void) {
  struct dshot_data dshot;
  struct gyro_data gyro;
  struct gyro_data accel;

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
      imu_init(&gyro);
    }
    if(gyro_ready()) {
      elrs_tick();
      gyro_read(&gyro);
      accel_read(&accel);

      // Update the IMU
      imu_update_from_gyro(&gyro);
      imu_update_from_accel(&accel);

      // Get the requested angles from the transmitter
      int32_t requested_x = elrs_channel(1);
      int32_t requested_y = elrs_channel(0);
      int32_t requested_z = elrs_channel(3) * 4;

      // Create a vector to hold the current orientation
      double orientation_vector[3];
      // Rotate the up vector by the current orientation to get the orientation vector
      Quaternion_rotate(&q, (double[]){0, 0, 1}, orientation_vector);

      // Calculate the shortest path from the orientation vector back to the up vector
      Quaternion shortest_path;
      Quaternion_from_unit_vecs(orientation_vector, (double[]){0, 0, 1}, &shortest_path);

      // Fetch the rotation axis for the shortest path
      double orientation_correction_axes[3];
      double angle = Quaternion_toAxisAngle(&shortest_path, orientation_correction_axes);

      // Multiply the components of the rotation axis by the angle to get the magnitude for corrction in each axis
      double x_angle_correction = orientation_correction_axes[0] * angle;
      double y_angle_correction = orientation_correction_axes[1] * angle;

      // Add the angle corrections to the requested angles
      requested_x -= x_angle_correction * 1000.0;
      requested_y -= y_angle_correction * 1000.0;

      // Print final requested angles
      //usb_printf("-1000,1000,%d,%d,%d\n", requested_x, requested_y, requested_z);
      // Print accelerometer data
      //usb_printf("-1000,1000,%d,%d,%d\n", accel.x, accel.y, accel.z);

      // Calculate the error for each axis
      int32_t error_x = gyro.x/4 + requested_x;
      int32_t error_y = gyro.y/4 + requested_y;
      int32_t error_z = gyro.z/1 + requested_z;

      // Integrate the error in each axis
      x_integral += error_x;
      y_integral += error_y;
      z_integral += error_z;

      // Add the P and I terms to calculate the final motor outputs
      dshot.motor1 = elrs_channel(2) + 820 - error_x + error_y - error_z - x_integral/512 + y_integral/512 - z_integral/512;
      dshot.motor2 = elrs_channel(2) + 820 - error_x - error_y + error_z - x_integral/512 - y_integral/512 + z_integral/512;
      dshot.motor3 = elrs_channel(2) + 820 + error_x + error_y + error_z + x_integral/512 + y_integral/512 + z_integral/512;
      dshot.motor4 = elrs_channel(2) + 820 + error_x - error_y - error_z + x_integral/512 - y_integral/512 - z_integral/512;

      dshot_write(&dshot);
    }
  }
  return 0;
}
