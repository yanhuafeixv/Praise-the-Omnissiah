#include "zf_common_headfile.h"
#include <cmath>
#include <ctime>

#include "encoder.h"
#include "imu.h"

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

    // ---------- 初始化 IMU ----------
    imu_get_dev_info();
    if(DEV_IMU660RA == imu_type)      printf("IMU DEV IS IMU660RA\r\n");
    else if(DEV_IMU660RB == imu_type) printf("IMU DEV IS IMU660RB\r\n");
    else if(DEV_IMU963RA == imu_type) printf("IMU DEV IS IMU963RA\r\n");
    else { printf("NO FIND IMU DEV\r\n"); return -1; }

    // 陀螺仪零偏校准
    calibrate_gyro_bias(200);

    // 获取初始姿态
    if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
    } else if(DEV_IMU963RA == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_mag_x = imu_get_raw(imu_file_path[MAG_X_RAW]);
        imu_mag_y = imu_get_raw(imu_file_path[MAG_Y_RAW]);
        imu_mag_z = imu_get_raw(imu_file_path[MAG_Z_RAW]);
    }
    convert_to_physical();
    float init_roll, init_pitch;
    compute_acc_angles(init_roll, init_pitch);
    kf_roll.setAngle(init_roll);
    kf_pitch.setAngle(init_pitch);
    if(imu_type == DEV_IMU963RA) {
        float init_yaw;
        compute_mag_yaw(init_roll, init_pitch, init_yaw);
        kf_yaw.setAngle(0.0f);
    } else {
        kf_yaw.setAngle(0.0f);
    }

    // 启动定时器
    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    return 0;  // ← 必须有返回值！
}

void all_end(void)
{
    encoder_left_deinit();
    encoder_right_deinit();
    // 如有需要，停止定时器： pit_timer->stop(); delete pit_timer;
}