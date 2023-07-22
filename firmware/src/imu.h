void imu_init(struct gyro_data *gyro, struct mag_data *mag);
void imu_init_zero(void);
void imu_update_from_gyro(struct gyro_data *gyro);
void imu_update_from_accel(struct gyro_data *accel);
void imu_update_from_mag(struct mag_data *mag);
void imu_get_xy_tilt(float *x, float *y, float *z);
