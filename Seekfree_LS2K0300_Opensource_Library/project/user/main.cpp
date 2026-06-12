#include <stdio.h>
#include "IMU_Analysis.h"
#include "init.h"
// 外部函数声明（init.cpp 提供）
int main(int, char**) {
    // 一次性初始化所有硬件（编码器、电机、IMU）
    if (all_init() != 0) {
        printf("Initialization failed!\n");
        return -1;
    }
    float roll, yaw, pitch;
    printf("=== IMU Test: printing angles every 200ms ===\n");
    while (1) {
        imu_get_angle(&roll, &yaw, &pitch);
        printf("Roll: %7.2f  Yaw: %7.2f  Pitch: %7.2f\n", roll, yaw, pitch);
        system_delay_ms(200);
    }
    // 实际上不会执行到这里
    all_end();
    return 0;
}