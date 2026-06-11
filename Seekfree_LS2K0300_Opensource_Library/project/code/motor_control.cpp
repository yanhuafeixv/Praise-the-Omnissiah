#include "zf_common_headfile.h"
#include "motor_control.h"
#include <math.h>   // 提供 fabs 函数

// -------------------- 硬件设备路径 --------------------
#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"

// -------------------- 全局变量 --------------------
static struct pwm_info motor1_pwm_info;
static struct pwm_info motor2_pwm_info;
static bool initialized = false;

// -------------------- 内部工具函数 --------------------

/**
 * @brief 将占空比百分比（0~100）转换为硬件计数值并限幅
 */
static uint32_t duty_percent_to_count(float duty_percent, uint32_t duty_max) {
    if (duty_percent < 0.0f)   duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    return (uint32_t)(duty_percent * duty_max / 100.0f);
}

// -------------------- 对外接口实现 --------------------

int motor_control_init(void) {
    // 获取 PWM 设备信息（函数返回 void，不需要比较返回值）
    pwm_get_dev_info(MOTOR1_PWM, &motor1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor2_pwm_info);

    // 简单检查：如果 duty_max 为 0，说明可能获取失败
    if (motor1_pwm_info.duty_max == 0 || motor2_pwm_info.duty_max == 0) {
        printf("Failed to get motor PWM info (duty_max == 0)\n");
        return -1;
    }

    // 初始化为停止状态
    motor_control_stop();
    initialized = true;
    return 0;
}

void set_motor_duty(float left_duty, float right_duty) {
    if (!initialized) return;

    // ---------- 限幅到 -100 ~ 100 ----------
    if (left_duty > 100.0f)   left_duty = 100.0f;
    if (left_duty < -100.0f)  left_duty = -100.0f;
    if (right_duty > 100.0f)  right_duty = 100.0f;
    if (right_duty < -100.0f) right_duty = -100.0f;

    // ---------- 左电机控制 ----------
    uint32_t duty_left = duty_percent_to_count(fabs(left_duty), motor1_pwm_info.duty_max);
    if (left_duty >= 0) {
        gpio_set_level(MOTOR1_DIR, 1);   // DIR = 1 → 正转（前进）
    } else {
        gpio_set_level(MOTOR1_DIR, 0);   // DIR = 0 → 反转（后退）
    }
    pwm_set_duty(MOTOR1_PWM, duty_left);

    // ---------- 右电机控制 ----------
    uint32_t duty_right = duty_percent_to_count(fabs(right_duty), motor2_pwm_info.duty_max);
    if (right_duty >= 0) {
        gpio_set_level(MOTOR2_DIR, 1);
    } else {
        gpio_set_level(MOTOR2_DIR, 0);
    }
    pwm_set_duty(MOTOR2_PWM, duty_right);
}

void motor_control_stop(void) {
    if (!initialized) return;
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    // 方向引脚保持原状，不影响
}