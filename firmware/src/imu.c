#include <stm32g4xx.h>
#include "gyro.h"
#include "quaternion.h"
#include "usb.h"

// A quaternion to represent the orientation of the sensor.
Quaternion q;

// Magnetometer offset vector.
float mag_offset[3] = {0, 0, 0};
float heading_offset = 0;

// Set the IMU orientation to zero. We assume the IMU starts level.
void imu_zero(void)
{
  // Zero the orientation
  Quaternion_setIdentity(&q);
}

// Rotate the quaternion by the given angular velocity.
void imu_update_from_gyro(struct gyro_data *gyro)
{
  // Convert the gyro readings to radians and multiply by the time since the last update (1/8000).
  float gyro_x = gyro->x * 0.00106524f * 0.000125f;
  float gyro_y = gyro->y * 0.00106524f * 0.000125f;
  float gyro_z = gyro->z * 0.00106524f * 0.000125f;

  // Create quaternions for each axis rotation.
  Quaternion q_x, q_y, q_z;
  Quaternion_fromXRotation(gyro_x, &q_x);
  Quaternion_fromYRotation(gyro_y, &q_y);
  Quaternion_fromZRotation(gyro_z, &q_z);

  // Multiply the current orientation by each of the rotation quaternions.
  Quaternion_multiply(&q_x, &q, &q);
  Quaternion_multiply(&q_y, &q, &q);
  Quaternion_multiply(&q_z, &q, &q);

  // Normalize the result
  Quaternion_normalize(&q, &q);
}

// Update the quaternion from the accelerometer.
void imu_update_from_accel(struct gyro_data *accel)
{
  // Create a vector to hold the current orientation.
  float orientation_vector[3];

  // Rotate the up vector by the orientation quaternion to populate the orientation vector.
  Quaternion_rotate(&q, (float[]){0, 0, 1}, orientation_vector);

  // Convert accelerometer data to a unit vector.
  float accel_vector[3] = {accel->x, -accel->y, accel->z};
  float accel_vector_length = sqrtf(accel_vector[0]*accel_vector[0] + accel_vector[1]*accel_vector[1] + accel_vector[2]*accel_vector[2]);
  if(accel_vector_length == 0) accel_vector_length = 1;
  accel_vector[0] /= accel_vector_length;
  accel_vector[1] /= accel_vector_length;
  accel_vector[2] /= accel_vector_length;

  // Calculate the path from the current orientation vector to accelerometer vector.
  Quaternion shortest_path;
  Quaternion_from_unit_vecs(orientation_vector, accel_vector, &shortest_path);

  // Convert the path to a single axis rotation.
  float axis[3];
  float angle = Quaternion_toAxisAngle(&shortest_path, axis);

  // Limit the angle to 0.0001 radians to create a very small correction.
  if(angle > 0.0001f) angle = 0.0001f;

  // Generate a quaternion for the correction.
  Quaternion correction;
  Quaternion_fromAxisAngle(axis, angle, &correction);

  // Rotate the current orientation by the correction.
  Quaternion_multiply(&correction, &q, &q);

  // Normalize the result.
  Quaternion_normalize(&q, &q);
}

void imu_get_xy_tilt(float *pitch, float *roll)
{
  // Create a vector to hold the current orientation.
  float orientation_vector[3];

  // Rotate the up vector by the orientation quaternion to populate the orientation vector.
  Quaternion_rotate(&q, (float[]){0, 0, 1}, orientation_vector);

  // Calculate the shortest path from the orientation vector back to the up vector.
  Quaternion shortest_path;
  Quaternion_from_unit_vecs(orientation_vector, (float[]){0, 0, 1}, &shortest_path);

  // Fetch the rotation axis for the shortest path.
  float orientation_correction_axes[3];
  float angle = Quaternion_toAxisAngle(&shortest_path, orientation_correction_axes);

  // Multiply the x and y components of the rotation axis by the angle
  // to get the magnitude of tilt in each axis.
  *pitch = orientation_correction_axes[0] * angle;
  *roll  = orientation_correction_axes[1] * angle;
}
