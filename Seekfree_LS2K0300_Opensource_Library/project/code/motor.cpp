#include "zf_common_headfile.hpp"

struct pwm_info pwm_1_info;
struct pwm_info pwm_2_info;//给结构体起个名字？


zf_driver_gpio  dir_1(ZF_GPIO_MOTOR_1, O_RDWR);
zf_driver_gpio  dir_2(ZF_GPIO_MOTOR_2, O_RDWR);
zf_driver_pwm   pwm_1(ZF_PWM_MOTOR_1);
zf_driver_pwm   pwm_2(ZF_PWM_MOTOR_2);

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (pwm_1_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (pwm_2_info.duty_max)        

#define MAX_DUTY        (30 )   // 最大 MAX_DUTY% 占空比

void motor_init(void)
{
    pwm_1.get_dev_info(&pwm_1_info);
    pwm_2.get_dev_info(&pwm_2_info);
}

void motor_speed(uint16 duty_1, uint16 duty_2)
{
    if(duty_1 >= 0)                                              // 正转
    {
        dir_1.set_level(1);                                      // DIR输出高电平
        pwm_1.set_duty(duty_1 * (MOTOR1_PWM_DUTY_MAX / 100));    // 计算占空比
    }
    else
    {
        dir_1.set_level(0);                                      // DIR输出低电平
        pwm_1.set_duty(-duty_1 * (MOTOR1_PWM_DUTY_MAX / 100));   // 计算占空比
    }
    if(duty_2 >= 0)                                              // 正转
    {
        dir_2.set_level(1);                                      // DIR输出高电平
        pwm_2.set_duty(duty_2 * (MOTOR2_PWM_DUTY_MAX / 100));    // 计算占空比
    }
    else
    {
        dir_2.set_level(0);                                      // DIR输出低电平
        pwm_2.set_duty(-duty_2 * (MOTOR2_PWM_DUTY_MAX / 100));   // 计算占空比
    }
}