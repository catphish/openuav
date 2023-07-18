#include <stm32g4xx.h>
#include "gyro.h"
#include "quaternion.h"

// A quaternion to represent the orientation of the sensor
Quaternion q;
struct gyro_data gyro_offset;

// Initialize and zero the IMU
void imu_init(struct gyro_data *gyro)
{
    Quaternion_setIdentity(&q);
    gyro_offset.x = gyro->x;
    gyro_offset.y = gyro->y;
    gyro_offset.z = gyro->z;
}
void imu_init_zero(void)
{
    Quaternion_setIdentity(&q);
    gyro_offset.x = 0;
    gyro_offset.y = 0;
    gyro_offset.z = 0;
}

// Rotate the quaternion by the given angular velocity
void imu_update_from_gyro(struct gyro_data *gyro)
{
    int dt = 0x00FFFFFF - SysTick->VAL;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;

    // Convert the gyro readings to radians
    float gyro_x = ((float)gyro->x - (float)gyro_offset.x) * 0.00122173 * (float)dt / 160000000;
    float gyro_y = ((float)gyro->y - (float)gyro_offset.y) * 0.00122173 * (float)dt / 160000000;
    float gyro_z = ((float)gyro->z - (float)gyro_offset.z) * 0.00122173 * (float)dt / 160000000;

    Quaternion q_x, q_y, q_z;

    // Generate quaternion for x axis rotation
    Quaternion_fromXRotation(gyro_x, &q_x);
    // Generate quaternion for y axis rotation
    Quaternion_fromYRotation(gyro_y, &q_y);
    // Generate quaternion for z axis rotation
    Quaternion_fromZRotation(gyro_z, &q_z);
    // Multiple the current orientation by the rotation quaternions
    Quaternion_multiply(&q_x, &q, &q);
    Quaternion_multiply(&q_y, &q, &q);
    Quaternion_multiply(&q_z, &q, &q);
    // Normalize the result
    Quaternion_normalize(&q, &q);
}

// Update the quaternion from the accelerometer
void imu_update_from_accel(struct gyro_data *accel)
{
    // Create a vector to hold the current orientation
    double orientation_vector[3];
    // Rotate the up vector by the current orientation to get the orientation vector
    Quaternion_rotate(&q, (double[]){0, 0, 1}, orientation_vector);
    // Convert accel_vector to a unit vector.
    double accel_vector[3] = {accel->x, -accel->y, accel->z};
    double accel_vector_length = sqrt(accel_vector[0]*accel_vector[0] + accel_vector[1]*accel_vector[1] + accel_vector[2]*accel_vector[2]);
    accel_vector[0] /= accel_vector_length;
    accel_vector[1] /= accel_vector_length;
    accel_vector[2] /= accel_vector_length;
    // Calculate the path from the current orientation to accelerometer vector
    Quaternion shortest_path;
    Quaternion_from_unit_vecs(orientation_vector, accel_vector, &shortest_path);
    // Convert the path to a single axis rotation
    double axis[3];
    double angle = Quaternion_toAxisAngle(&shortest_path, axis);
    if(angle > 0.0001) angle = 0.0001;
    // Generate a quaternion for the rotation
    Quaternion q_accel;
    Quaternion_fromAxisAngle(axis, angle, &q_accel);
    // Multiply the current orientation by the rotation quaternion
    Quaternion_multiply(&q_accel, &q, &q);
    // Normalize the result
    Quaternion_normalize(&q, &q);
}
