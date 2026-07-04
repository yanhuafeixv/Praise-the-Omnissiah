// #include <stdio.h>
// #include "IMU_Analysis.h"
// #include "init.h"
// #include "encoder.h"
// #include "odometry.h"
// #include "motor_control.h"
// // 外部函数声明（init.cpp 提供）
// int main(int, char**) {
//     // 一次性初始化所有硬件（编码器、电机、IMU）
//     if (all_init() != 0) {
//         printf("Initialization failed!\n");
//         return -1;
//     }
//     int l1,l2;
//     while (1) {

//         l1 = encoder_left_get_count();
//         l2 = encoder_right_get_count();
//         printf("left:%d, right:%d\n",l1,l2);
//         set_motor_duty(20,20);

//     }
//     // 实际上不会执行到这里
//     all_end();
//     return 0;
// }
#include "IMU_Analysis.h"
#include "init.h"
#include "encoder.h"
#include <stdio.h>
#include <unistd.h>        // 提供 usleep 延时
#include "navigator.h"     // 包含 car_goto, navigator_update, navigator_is_done
#include "odometry.h"      // 包含 odometry_init, odometry_update, odometry_get_position


int main(void) {
    // 一次性初始化所有硬件（编码器、电机、IMU）
    if (all_init() != 0) {
        printf("Initialization failed!\n");
        return -1;
    }

    // 2. 初始化里程计（会将当前位置归零，并记录编码器初值）
    odometry_init();

    // 3. 设定目标点（单位：米）
    float target_x = 0.6f;   // 可修改为任意值测试
    float target_y = 0.3f;
    printf("Start navigation to (%.2f, %.2f)\n", target_x, target_y);
    car_goto(target_x, target_y);

    // 4. 主循环：不断更新里程计和导航状态，直到到达目标
    while (!navigator_is_done()) {
        // 更新里程计（从编码器和IMU读取数据并计算当前位置）
        odometry_update();

        // 执行导航状态机（根据当前位置和目标点控制电机）
        navigator_update();

        // 打印实时位姿，方便观察
        float x, y, yaw;
        odometry_get_position(&x, &y, &yaw);
        printf("Current: (%.2f, %.2f)  Yaw=%.1f°\n", x, y, yaw);

        // 循环间隔约 20ms，可根据实际控制周期调整
        usleep(20000);   // 20000us = 20ms
    }

    printf("Arrived at target successfully!\n");
    return 0;
}

