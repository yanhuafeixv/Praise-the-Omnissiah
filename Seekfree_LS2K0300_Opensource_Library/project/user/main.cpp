#include <stdio.h>
#include "IMU_Analysis.h"
#include "init.h"
#include "encoder.h"
#include "odometry.h"
// 外部函数声明（init.cpp 提供）
int main(int, char**) {
    // 一次性初始化所有硬件（编码器、电机、IMU）
    if (all_init() != 0) {
        printf("Initialization failed!\n");
        return -1;
    }
    while (1) {

        odometry_update();  // 更新里程计数据（坐标和航向）

        odometry_show_position();

        system_delay_ms(200);
    }
    // 实际上不会执行到这里
    all_end();
    return 0;
}