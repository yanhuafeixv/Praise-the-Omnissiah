
#include "zf_common_headfile.h"

int main(int, char**) 
{

    while(1)
    {
        key_scanner();
        system_delay_ms(1);
        printf("key_0 = %d\r\n", key_get_state(KEY_0));
        printf("key_1 = %d\r\n", key_get_state(KEY_1));
        printf("key_2 = %d\r\n", key_get_state(KEY_2));
        printf("key_3 = %d\r\n", key_get_state(KEY_3));
    }
}
