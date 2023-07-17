#include <stm32g4xx.h>
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
    float gyro_x = (float)gyro->x * 0.00122173 / 416.0f;
    float gyro_y = (float)gyro->y * 0.00122173 / 416.0f;
    float gyro_z = (float)gyro->z * 0.00122173 / 416.0f;

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
