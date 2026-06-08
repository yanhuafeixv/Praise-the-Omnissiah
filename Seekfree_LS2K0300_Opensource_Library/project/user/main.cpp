#include "zf_common_headfile.h"
#include "encoder.h"

int16 encoder_left;  // 原代码中的变量

void pit_callback() {
    encoder_left = encoder_get_count();
}

int main(int, char**) {
    // 初始化编码器：GPIO42 = A相, GPIO43 = B相
    if (encoder_init(22, 23) != 0) {
        printf("Encoder init failed!\n");
        return -1;
    }

    timer_fd *pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    while(1) {
        printf("encoder_left = %d.\r\n", encoder_left);
        system_delay_ms(100);
    }

    encoder_deinit();
    return 0;
}