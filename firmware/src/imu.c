#include <stm32g4xx.h>
#include "gyro.h"
#include "mag.h"
#include "quaternion.h"

// A quaternion to represent the orientation of the sensor.
Quaternion q;
// A gyro reading to use as the offset after calibration.
struct gyro_data gyro_offset;
float mag_offset[3];

// Initialize the IMU and calibrate the gyro.
// This currently uses only a single gyro reading to calibrate.
void imu_init(struct gyro_data *gyro, struct mag_data *mag)
{
    // Zero the orientation
    Quaternion_setIdentity(&q);

    // Set the gyro offset to the provided reading.
    gyro_offset.x = gyro->x;
    gyro_offset.y = gyro->y;
    gyro_offset.z = gyro->z;

    // Convert magnetometer data to a unit vector.
    mag_offset[0] = -mag->y+4250;
    mag_offset[1] = mag->x-5250;
    mag_offset[2] = -mag->z+2125;
    float mag_vector_length = sqrtf(mag_offset[0]*mag_offset[0] + mag_offset[1]*mag_offset[1] + mag_offset[2]*mag_offset[2]);
    if(mag_vector_length == 0) mag_vector_length = 1;
    mag_offset[0] /= mag_vector_length;
    mag_offset[1] /= mag_vector_length;
    mag_offset[2] /= mag_vector_length;
}

// Initialize the IMU without a gyro reading.
void imu_init_zero(void)
{
    // Zero the orientation
    Quaternion_setIdentity(&q);

    // Set the gyro offset to zero.
    gyro_offset.x = 0;
    gyro_offset.y = 0;
    gyro_offset.z = 0;

    // Set the magnetometer offset to North.
    mag_offset[0] = 0;
    mag_offset[1] = 1;
    mag_offset[2] = 0;
}

// Rotate the quaternion by the given angular velocity.
void imu_update_from_gyro(struct gyro_data *gyro)
{
    // Use the systick counter to calculate the time since the last update.
    int dt = 0x00FFFFFF - SysTick->VAL;

    // Reset the systick counter.
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;

    // Convert the gyro readings to radians and multiply by the time since the last update.
    float gyro_x = ((float)gyro->x - (float)gyro_offset.x) * 0.00122173f * (float)dt / 160000000;
    float gyro_y = ((float)gyro->y - (float)gyro_offset.y) * 0.00122173f * (float)dt / 160000000;
    float gyro_z = ((float)gyro->z - (float)gyro_offset.z) * 0.00122173f * (float)dt / 160000000;

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
    if(angle > 0.001f) angle = 0.001f;

    // Generate a quaternion for the correction.
    Quaternion correction;
    Quaternion_fromAxisAngle(axis, angle, &correction);

    // Rotate the current orientation by the correction.
    Quaternion_multiply(&correction, &q, &q);

    // Normalize the result.
    Quaternion_normalize(&q, &q);
}

void imu_update_from_mag(struct mag_data *mag)
{
    // Convert magnetometer data to a unit vector.
    float mag_vector[3] = {-mag->y+4250, mag->x-5250, -mag->z+2125};
    float mag_vector_length = sqrtf(mag_vector[0]*mag_vector[0] + mag_vector[1]*mag_vector[1] + mag_vector[2]*mag_vector[2]);
    if(mag_vector_length == 0) mag_vector_length = 1;
    mag_vector[0] /= mag_vector_length;
    mag_vector[1] /= mag_vector_length;
    mag_vector[2] /= mag_vector_length;

    // Rotate the magnetometer vector by the reverse of the current orientation.
    Quaternion con_q;
    Quaternion_conjugate(&q, &con_q);
    Quaternion_rotate(&con_q, mag_vector, mag_vector);

    // Calculate the path from the current magnetometer vector to the offset vector.
    Quaternion shortest_path;
    Quaternion_from_unit_vecs(mag_offset, mag_vector, &shortest_path);

    // Convert the path to a single axis rotation.
    float axis[3];
    float angle = Quaternion_toAxisAngle(&shortest_path, axis);

    // Limit the angle to 0.0001 radians to create a very small correction.
    if(angle > 0.0005f) angle = 0.0005f;

    // Generate a quaternion for the correction.
    Quaternion correction;
    Quaternion_fromAxisAngle(axis, angle, &correction);

    // Rotate the current orientation by the correction.
    Quaternion_multiply(&correction, &q, &q);

    // Normalize the result.
    Quaternion_normalize(&q, &q);
}

void imu_get_xy_tilt(float *x, float *y, float *z)
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
    *x = orientation_correction_axes[0] * angle;
    *y = orientation_correction_axes[1] * angle;

    // Rotate the north vector by the orientation quaternion to populate the orientation vector.
    Quaternion_rotate(&q, (float[]){0, 1, 0}, orientation_vector);

    // Calculate the shortest path from the orientation vector back to the north vector.
    Quaternion_from_unit_vecs(orientation_vector, (float[]){0, 1, 0}, &shortest_path);

    // Fetch the rotation axis for the shortest path.
    angle = Quaternion_toAxisAngle(&shortest_path, orientation_correction_axes);
    *z = orientation_correction_axes[2] * angle;
}
