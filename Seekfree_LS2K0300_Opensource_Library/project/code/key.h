#ifndef _key_h_
#define _key_h_

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"

#define KEY_LIST    { KEY_0, KEY_1, KEY_2, KEY_3 }

#define KEY_RELEASE_LEVEL           ( 1         )                               // 按键的默认状态 也就是按键释放状态的电平
#define KEY_MAX_SHOCK_PERIOD        ( 10        )                               // 按键消抖检测时长 单位毫秒 低于这个时长的信号会被认为是杂波抖动
#define KEY_LONG_PRESS_PERIOD       ( 1000      )                               // 最小长按时长 单位毫秒 高于这个时长的信号会被认为是长按动作

typedef enum
{
    KEY0,
    KEY1,
    KEY2,
    KEY3,
    KEY_NUMBER,
}key_index_enum;                                                                // 按键索引 对应上方定义的按键引脚个数 默认定义四个按键

typedef enum
{
    KEY_RELEASE,                                                                // 按键释放状态
    KEY_SHORT_PRESS,                                                            // 按键短按状态
    KEY_LONG_PRESS,                                                             // 按键长按状态
}key_state_enum;

void key_scanner (void);
key_state_enum key_get_state (key_index_enum key_n);

#endif