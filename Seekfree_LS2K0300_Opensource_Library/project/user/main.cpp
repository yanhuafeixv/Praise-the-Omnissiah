#include <stdio.h>
#include <unistd.h>
#include "encoder.h"

int main(int, char**) {
    // 初始化左右编码器（GPIO 已在函数内指定）
    if (encoder_left_init() != 0) {
        printf("Left encoder init failed!\n");
        return -1;
    }
    if (encoder_right_init() != 0) {
        printf("Right encoder init failed!\n");
        encoder_left_deinit();
        return -1;
    }

    while (1) {
        int left = encoder_left_get_count();
        int right = encoder_right_get_count();
        printf("left = %d, right = %d\n", left, right);
        usleep(100000);  // 100ms
    }

    encoder_left_deinit();
    encoder_right_deinit();
    return 0;
}