#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化里程计
 * @param init_x 初始 X 坐标（单位：米）
 * @param init_y 初始 Y 坐标（单位：米）
 */
void odometry_init(float init_x, float init_y);

/**
 * @brief 更新里程计（需周期性调用，例如每 10ms）
 * @note  内部会读取 IMU 航向和编码器累计脉冲，计算位置增量
 */
void odometry_update(void);

/**
 * @brief 获取当前里程计信息
 * @param x   输出：X 坐标（米）
 * @param y   输出：Y 坐标（米）
 * @param yaw 输出：航向角（弧度，范围 -π ~ π）
 */
void get_odometry(float *x, float *y, float *yaw);

#ifdef __cplusplus
}
#endif

#endif // ODOMETRY_H