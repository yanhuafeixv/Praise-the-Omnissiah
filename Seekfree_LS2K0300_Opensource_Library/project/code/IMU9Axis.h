// IMU.h
#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

// 角度结构体（全局变量）
extern struct {
    float yaw;
    float roll;
    float pitch;
} imu_angle;

// 传感器原始数据（模拟用）
extern int16_t imu963ra_gyro_x;
extern int16_t imu963ra_gyro_y;
extern int16_t imu963ra_gyro_z;
extern int16_t imu963ra_acc_x;
extern int16_t imu963ra_acc_y;
extern int16_t imu963ra_acc_z;
extern int16_t imu963ra_mag_x;
extern int16_t imu963ra_mag_y;
extern int16_t imu963ra_mag_z;

// 传感器转换函数（返回物理量）
float imu963ra_acc_transition(int16_t raw);
float imu963ra_gyro_transition(float raw);
float imu963ra_mag_transition(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif // IMU_H