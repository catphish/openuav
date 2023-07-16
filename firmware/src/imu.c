#include "gyro.h"
#include "quaternion.h"

// A quaternion to represent the orientation of the sensor
Quaternion q;

void imu_init(void)
{
    Quaternion_setIdentity(&q);
}

// Rotate the quaternion by the given angular velocity
void imu_update_from_gyro(struct gyro_data *gyro)
{
    // Convert the gyro readings to radians
    // Divide by 32768 and multiply by 2000 to get degrees per second
    // Divide by 360 and multiply by 2 pi to get radians per second
    // Divide by 416 to get radians per tick
    float gyro_x = (float)gyro->x * 2000.0f / 360.0f * 3.14159265358979323846f * 2.0f / 32768.0f / 833.0f;
    float gyro_y = (float)gyro->y * 2000.0f / 360.0f * 3.14159265358979323846f * 2.0f / 32768.0f / 833.0f;
    float gyro_z = (float)gyro->z * 2000.0f / 360.0f * 3.14159265358979323846f * 2.0f / 32768.0f / 833.0f;

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
