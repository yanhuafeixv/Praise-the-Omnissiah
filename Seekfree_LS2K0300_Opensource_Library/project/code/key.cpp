#include "zf_common_headfile.h"

static uint32               scanner_period = 1;                                 // 按键的扫描周期
static uint32               key_press_time[KEY_NUMBER];                         // 按键信号持续时长
static key_state_enum       key_state[KEY_NUMBER];                              // 按键状态

static const char *key_index[KEY_NUMBER] = KEY_LIST;

void key_scanner (void)
{
    uint8 i = 0;
    for(i = 0; KEY_NUMBER > i; i ++)
    {
        if(KEY_RELEASE_LEVEL != gpio_get_level(key_index[i]))                   // 按键按下
        {
            key_press_time[i] ++;
            if(KEY_LONG_PRESS_PERIOD / scanner_period <= key_press_time[i])
            {
                key_state[i] = KEY_LONG_PRESS;
            }
        }
        else                                                                    // 按键释放
        {
            if((KEY_LONG_PRESS != key_state[i]) && (KEY_MAX_SHOCK_PERIOD / scanner_period <= key_press_time[i]))
            {
                key_state[i] = KEY_SHORT_PRESS;
            }
            else
            {
                key_state[i] = KEY_RELEASE;
            }
            key_press_time[i] = 0;
        }
    }
}

key_state_enum key_get_state (key_index_enum key_n)
{
    return key_state[key_n];
}