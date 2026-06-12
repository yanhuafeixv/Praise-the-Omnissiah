#include "zf_common_headfile.h"
#include <cmath>
#include <ctime>

#include "encoder.h"
#include "motor_control.h"
#include "IMU_Analysis.h"

// ==================== 磁力计开关 ====================
// 如果你的 IMU 模式使用磁力计（DEFINE_IMU_ANALYSIS_MODE >= 3），请设为 1
#define IMU_MAG_ENABLE  0

// ==================== 定时器指针 ====================
static timer_fd *pit_timer = nullptr;

// ==================== 定时器回调（每 10ms 调用） ====================
static void timer_10ms_callback() {
    IMU_D_and_A_Enable = 1;               // 通知校准与解算模块
    IMU_Update_Data();                    // 读取原始传感器值
    IMU_Gyro_Calib_Check(&gyro_cal);      // 陀螺仪校准状态机
#if IMU_MAG_ENABLE
    IMU_Mag_Calib_Check(&mag_cal);        // 磁力计校准状态机
#endif
    IMU_Update_Analysis();                // 姿态解算（更新 Yaw_Result）
}

// ==================== IMU 初始化函数 ====================
static void imu_init() {
    printf("Starting IMU calibration...\n");

    // 启动陀螺仪校准（期间保持静止）
    IMU_Gyro_Calib_Start(&gyro_cal);

#if IMU_MAG_ENABLE
    // 若使用磁力计，启动磁力计校准
    IMU_Mag_Calib_Start(&mag_cal);
    printf("Please slowly rotate the device for 20 seconds...\n");
#endif

    // 创建 10ms 定时器
    pit_timer = new timer_fd(10, timer_10ms_callback);
    pit_timer->start();

    // 等待陀螺仪校准完成
    while (IMU_Gyro_Calib_Check(&gyro_cal) != 2) {
        system_delay_ms(10);
    }
    printf("Gyro calibration done.\n");

#if IMU_MAG_ENABLE
    // 等待磁力计校准完成
    while (IMU_Mag_Calib_Check(&mag_cal) != 2) {
        system_delay_ms(10);
    }
    printf("Mag calibration done.\n");
#endif

    // 校准完成后，将当前偏航角归零（车头方向作为 0 度）
    IMU_Reset_Data();
    printf("Yaw set to 0. IMU ready.\n");
}

// ==================== 总初始化函数 ====================
int all_init(void)
{
    // ---------- 初始化编码器 ----------
    if (encoder_left_init() != 0) {
        printf("Left encoder init failed!\n");
        return -1;
    }
    if (encoder_right_init() != 0) {
        printf("Right encoder init failed!\n");
        encoder_left_deinit();
        return -1;
    }

    // ---------- 初始化电机 ----------
    if (motor_control_init() != 0) {
        printf("Motor init failed! Please check hardware.\n");
        return -1;
    }
    printf("Motor init success.\n");

    // ---------- 初始化 IMU（包含校准和定时器启动） ----------
    imu_init();

    return 0;
}

// ==================== 总清理函数 ====================
void all_end(void)
{
    // 停止定时器
    if (pit_timer) {
        pit_timer->stop();
        delete pit_timer;
        pit_timer = nullptr;
    }

    // 关闭电机
    motor_control_stop();
    printf("Motors stopped.\n");

    // 释放编码器
    encoder_left_deinit();
    encoder_right_deinit();
    printf("Encoders deinitialized.\n");
}