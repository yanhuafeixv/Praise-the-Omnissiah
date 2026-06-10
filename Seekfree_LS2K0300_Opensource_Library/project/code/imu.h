#ifndef IMU_H
#define IMU_H

// 包含必要头文件（让 timer_fd、imu_type 等类型可见）
#include "zf_common_headfile.h"

// ==================== 卡尔曼滤波器类（完整定义） ====================
class KalmanFilter {
public:
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];

    KalmanFilter(float q_angle = 0.001f, float q_bias = 0.003f, float r_measure = 0.03f);

    float update(float gyro_rate, float acc_angle, float dt);
    void setAngle(float a) { angle = a; }
};

// ==================== 外部全局变量声明 ====================
extern KalmanFilter kf_roll;
extern KalmanFilter kf_pitch;
extern KalmanFilter kf_yaw;

extern float roll_rad, pitch_rad, yaw_rad;
extern float acc_x_g, acc_y_g, acc_z_g;
extern float gyro_x_rad, gyro_y_rad, gyro_z_rad;
extern float mag_x_gauss, mag_y_gauss, mag_z_gauss;

// ==================== 函数声明 ====================
void calibrate_gyro_bias(int samples = 200);  // 默认参数只在声明处写一次
void convert_to_physical();
void compute_acc_angles(float &roll_acc, float &pitch_acc);
void compute_mag_yaw(float roll, float pitch, float &yaw_mag);
void get_angle(float *pitch, float *yaw, float *roll);
void update_attitude(float dt);
void pit_callback();

// 定时器对象指针（供 init.cpp 使用）
extern timer_fd *pit_timer;

#endif // IMU_H