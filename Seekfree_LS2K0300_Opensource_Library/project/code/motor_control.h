#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机驱动（获取 PWM 设备信息）
 * @return 0 成功，-1 失败
 */
int motor_control_init(void);

/**
 * @brief 直接设置左右电机的占空比
 * @param left_duty  左电机占空比（-100.0 ~ 100.0） 正=前进，负=后退
 * @param right_duty 右电机占空比（-100.0 ~ 100.0）
 */
void set_motor_duty(float left_duty, float right_duty);

/**
 * @brief 立即停止所有电机（占空比置 0）
 */
void motor_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H