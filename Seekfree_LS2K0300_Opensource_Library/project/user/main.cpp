#include "zf_common_headfile.h"
#include "IMU_Analysis.h"

// ---------- 定时器指针 ----------
timer_fd *pit_timer;

// ---------- 定时器回调（每 10ms 调用一次）----------
void timer_10ms_callback() {
    IMU_D_and_A_Enable = 1;               // 通知校准与解算模块
    IMU_Update_Data();                    // 读取原始传感器值
    IMU_Gyro_Calib_Check(&gyro_cal);      // 陀螺仪校准状态机
#if IMU_MAG_ENABLE   // 若使用磁力计模式则开启，下面会定义
    IMU_Mag_Calib_Check(&mag_cal);        // 磁力计校准状态机
#endif
    IMU_Update_Analysis();                // 姿态解算（更新 Yaw_Result）
}

// ---------- IMU 初始化（校准 + 定时器启动）----------
void imu_init() {
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
        system_delay_ms(10);   // 让定时器回调有机会执行
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

// ====================== 主函数 ======================
int main(int, char**) {
    // 初始化 IMU（包含校准和定时器启动）
    imu_init();

    float roll, yaw, pitch;

    printf("=== IMU Test: printing angles every 200ms ===\n");
    while (1) {
        imu_get_angle(&roll, &yaw, &pitch);
        printf("Roll: %7.2f  Yaw: %7.2f  Pitch: %7.2f\n", roll, yaw, pitch);
        system_delay_ms(200);
    }

    return 0;
}