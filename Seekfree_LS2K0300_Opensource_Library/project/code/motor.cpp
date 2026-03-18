#include "zf_common_headfile.h"

#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM "/dev/zf_driver_pwm_motor_1"

#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM "/dev/zf_driver_pwm_motor_2"

struct pwm_info motor1_pwm_info;
struct pwm_info motor2_pwm_info;

#define MOTOR1_PWM_DUTY_MAX (motor1_pwm_info.duty_max)
#define MOTOR2_PWM_DUTY_MAX (motor2_pwm_info.duty_max)

void motor_init()
{
    pwm_get_dev_info(MOTOR1_PWM, &motor1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor2_pwm_info);
}

void motor_set_speed(int16 speed_1, int16 speed_2)
{
    if (speed_1 >= 0)
    {
        gpio_set_level(MOTOR1_DIR, 1);
        pwm_set_duty(MOTOR1_PWM, speed_1 * (MOTOR1_PWM_DUTY_MAX / 100));
    }
    else if (speed_1 < 0)
    {
        gpio_set_level(MOTOR1_DIR, 0);
        pwm_set_duty(MOTOR1_PWM, -speed_1 * (MOTOR1_PWM_DUTY_MAX / 100));
    }
    if (speed_2 > 0)
    {
        gpio_set_level(MOTOR2_DIR, 1);
        pwm_set_duty(MOTOR2_PWM, speed_2 * (MOTOR2_PWM_DUTY_MAX / 100));
    }
    else if (speed_2 < 0)
    {
        gpio_set_level(MOTOR2_DIR, 0);
        pwm_set_duty(MOTOR2_PWM, -speed_2 * (MOTOR2_PWM_DUTY_MAX / 100));
    }
}