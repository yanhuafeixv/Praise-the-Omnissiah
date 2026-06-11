#include "odometry.h"
#include "imu.h"
#include "encoder.h"
#include "motor_control.h"
#include "init.h"
int main(void) {
    // 1. 硬件初始化（IMU、编码器、电机等）
    // ... 你的 all_init() 或手动初始化代码 ...
    all_init();
    // 2. 初始化里程计，起点 (0,0)
    odometry_init(0.0f, 0.0f);

    // 3. 进入控制循环（假设 10ms 一次）
    while (1) {
        odometry_update();          // 更新坐标

        float x, y, yaw;
        get_odometry(&x, &y, &yaw); // 读取当前状态
        printf("Current position: x=%.2f m, y=%.2f m, yaw=%.2f deg\n", x, y, yaw * (180.0f / M_PI));
        // 在这里调用你的导航控制（如 car_goto 状态机）
        // 根据 x, y, yaw 和目标点计算电机速度
        // set_motor_duty(...)

        system_delay_ms(10);        // 保持约 10ms 周期
    }
    all_end(); // 结束前清理资源（如果需要）
}