// #ifndef NAVIGATOR_H
// #define NAVIGATOR_H

// #include <stdbool.h>

// /**
//  * @brief 设置目标坐标，启动导航
//  * @param target_x 目标 X 坐标（米）
//  * @param target_y 目标 Y 坐标（米）
//  */
// void car_goto(float target_x, float target_y);

// /**
//  * @brief 导航主循环函数（需周期性调用，例如每 10ms）
//  * @note  该函数会读取当前里程计数据并执行运动控制
//  */
// void navigator_update();

// /**
//  * @brief 查询导航是否已完成
//  * @return true 已到达目标，false 正在移动
//  */
// bool navigator_is_done();

// /**
//  * @brief 显示当前位置（封装里程计打印）
//  */
// void car_show_position(float x, float y);

// #endif // NAVIGATOR_H