#include "zf_common_headfile.h"
#include <cmath>
#include <ctime>
#include "imu.h"

// ==================== 类成员函数实现 ====================
KalmanFilter::KalmanFilter(float q_angle, float q_bias, float r_measure) {
    Q_angle = q_angle;
    Q_bias = q_bias;
    R_measure = r_measure;
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
}

float KalmanFilter::update(float gyro_rate, float acc_angle, float dt) {
    float gyro_rate_unbias = gyro_rate - bias;
    angle += gyro_rate_unbias * dt;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = acc_angle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// ==================== 全局变量定义 ====================
timer_fd *pit_timer = nullptr;          // 定时器指针

char send_buf[256];                     // 调试用

// 物理量转换后的数据（单位见注释）
float acc_x_g, acc_y_g, acc_z_g;        // 加速度，单位 g
float gyro_x_rad, gyro_y_rad, gyro_z_rad; // 角速度，单位 rad/s
float mag_x_gauss, mag_y_gauss, mag_z_gauss; // 磁力计，单位 Gauss（仅部分型号）

// 姿态角（弧度）
float roll_rad = 0.0f;
float pitch_rad = 0.0f;
float yaw_rad = 0.0f;

// 卡尔曼滤波器实例
KalmanFilter kf_roll (0.05f, 0.003f, 0.1f);
KalmanFilter kf_pitch(0.05f, 0.003f, 0.1f);
KalmanFilter kf_yaw  (0.01f, 0.0001f, 0.1f);

// 用于计算 dt 的时间记录
struct timespec last_time;
bool first_time = true;

// ==================== 内部辅助函数 ====================

/**
 * 根据不同 IMU 型号返回物理量转换系数
 */
static void get_scale_factors(float &acc_scale, float &gyro_scale, float &mag_scale) {
    switch(imu_type) {
        case DEV_IMU660RA:
            acc_scale = 1.0f / 2048.0f;
            gyro_scale = (3.1415926f / 180.0f) / 16.4f;
            mag_scale = 1.0f;       // 无磁力计，系数任意
            break;
        case DEV_IMU660RB:
            acc_scale = 1.0f / 2048.0f;
            gyro_scale = (3.1415926f / 180.0f) / 16.4f;
            mag_scale = 1.0f;
            break;
        case DEV_IMU963RA:
            acc_scale = 1.0f / 4098.0f;
            gyro_scale = (M_PI / 180.0f) / 14.29f;
            mag_scale = 0.15f;
            break;
        default:
            acc_scale = 1.0f;
            gyro_scale = 1.0f;
            mag_scale = 1.0f;
            break;
    }
}

/**
 * 将原始 int 值转换为物理单位
 */
void convert_to_physical() {
    float acc_scale, gyro_scale, mag_scale;
    get_scale_factors(acc_scale, gyro_scale, mag_scale);

    acc_x_g = (float)imu_acc_x * acc_scale;
    acc_y_g = (float)imu_acc_y * acc_scale;
    acc_z_g = (float)imu_acc_z * acc_scale;

    gyro_x_rad = (float)imu_gyro_x * gyro_scale;
    gyro_y_rad = (float)imu_gyro_y * gyro_scale;
    gyro_z_rad = (float)imu_gyro_z * gyro_scale;

    if(imu_type == DEV_IMU963RA) {
        mag_x_gauss = (float)imu_mag_x * mag_scale;
        mag_y_gauss = (float)imu_mag_y * mag_scale;
        mag_z_gauss = (float)imu_mag_z * mag_scale;
    }
}

/**
 * 由加速度计数据计算 Roll 和 Pitch（弧度）
 */
void compute_acc_angles(float &roll_acc, float &pitch_acc) {
    roll_acc = atan2f(acc_y_g, acc_z_g);
    pitch_acc = atan2f(-acc_x_g, sqrtf(acc_y_g * acc_y_g + acc_z_g * acc_z_g));
}

/**
 * 用磁力计计算 Yaw（弧度），使用当前的 Roll/Pitch 进行倾斜补偿
 */
void compute_mag_yaw(float roll, float pitch, float &yaw_mag) {
    if(imu_type != DEV_IMU963RA) return;
    float bx = mag_x_gauss * cosf(pitch) +
               mag_y_gauss * sinf(pitch) * sinf(roll) +
               mag_z_gauss * sinf(pitch) * cosf(roll);
    float by = mag_y_gauss * cosf(roll) - mag_z_gauss * sinf(roll);
    yaw_mag = atan2f(by, bx);
}

// ==================== 对外接口函数 ====================

/**
 * 陀螺仪零偏校准（静止采样取平均值）
 * 注意：头文件中声明了默认参数 samples=200，此处定义不写默认值
 */
void calibrate_gyro_bias(int samples) {
    printf("Calibrating gyro bias, keep IMU still...\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for(int i = 0; i < samples; i++) {
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
        sum_x += (float)imu_gyro_x;
        sum_y += (float)imu_gyro_y;
        sum_z += (float)imu_gyro_z;
        system_delay_ms(5);
    }
    float gyro_scale;
    float dummy;
    get_scale_factors(dummy, gyro_scale, dummy);
    float bias_x_rad = (sum_x / samples) * gyro_scale;
    float bias_y_rad = (sum_y / samples) * gyro_scale;
    float bias_z_rad = (sum_z / samples) * gyro_scale;
    kf_roll.bias  = bias_x_rad;
    kf_pitch.bias = bias_y_rad;
    kf_yaw.bias   = bias_z_rad;
    printf("Gyro bias (rad/s): x=%.4f, y=%.4f, z=%.4f\n", bias_x_rad, bias_y_rad, bias_z_rad);
}

/**
 * 姿态解算（在定时器回调中调用）
 */
void update_attitude(float dt) {
    convert_to_physical();

    float roll_acc, pitch_acc;
    compute_acc_angles(roll_acc, pitch_acc);

    // Roll 与 Pitch 通过卡尔曼滤波融合陀螺仪与加速度计
    roll_rad  = kf_roll.update(gyro_x_rad, roll_acc, dt);
    pitch_rad = kf_pitch.update(gyro_y_rad, pitch_acc, dt);

    // Yaw 目前仅使用陀螺仪积分（无磁力计修正），长时间会漂移
    yaw_rad = kf_yaw.update(gyro_z_rad, yaw_rad, dt);

    // 角度归一化到 [-pi, pi]
    if(yaw_rad > M_PI)      yaw_rad -= 2.0f * M_PI;
    if(yaw_rad < -M_PI)     yaw_rad += 2.0f * M_PI;
}

/**
 * 定时器回调函数（每 10ms 调用一次，由硬件定时器触发）
 */
void pit_callback() {
    // 1. 读取原始传感器数据
    if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
    } else if(DEV_IMU963RA == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
        imu_mag_x = imu_get_raw(imu_file_path[MAG_X_RAW]);
        imu_mag_y = imu_get_raw(imu_file_path[MAG_Y_RAW]);
        imu_mag_z = imu_get_raw(imu_file_path[MAG_Z_RAW]);
    }

    // 2. 计算时间间隔 dt
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    float dt = 0.01f;
    if(!first_time) {
        dt = (now.tv_sec - last_time.tv_sec) + (now.tv_nsec - last_time.tv_nsec) * 1e-9f;
        if(dt < 0.001f) dt = 0.001f;
        if(dt > 0.05f)  dt = 0.05f;
    }
    last_time = now;
    first_time = false;

    // 3. 姿态更新
    update_attitude(dt);
}

/**
 * 获取当前姿态角（单位：度）
 * 该函数可在任意位置调用，读取最新的姿态角。
 */
void get_angle(float *pitch, float *yaw, float *roll) {
    if (pitch) *pitch = pitch_rad * 180.0f / M_PI;
    if (yaw)   *yaw   = yaw_rad   * 180.0f / M_PI;
    if (roll)  *roll  = roll_rad  * 180.0f / M_PI;
}