#ifndef ODOMETRY_H
#define ODOMETRY_H

/**
 * @brief 初始化里程计（起点设为 0,0，航向 0°）
 */
void odometry_init();

/**
 * @brief 更新里程计（需周期性调用，推荐每 10ms 一次）
 * @note  内部读取 IMU 航向和编码器脉冲，计算当前坐标与航向
 */
void odometry_update();

/**
 * @brief 获取当前坐标和航向
 * @param x       输出：X 坐标（米）
 * @param y       输出：Y 坐标（米）
 * @param yaw_deg 输出：航向角（度，0° 为 X 轴正向，逆时针增加）
 */
void odometry_get_position(float *x, float *y, float *yaw_deg);

/**
 * @brief 在终端打印当前坐标（调试用）
 */
void odometry_show_position();

#endif // ODOMETRY_H