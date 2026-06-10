#include "zf_common_headfile.h"
#include <stdio.h>
#include <unistd.h>
#include "encoder.h"
#include "imu.h"

// 声明 all_init 和 all_end
extern int all_init(void);
extern void all_end(void);

int main(int, char**) {
    if (all_init() != 0) {
        printf("Initialization failed!\n");
        return -1;
    }

    while (1) {
        // float p, y, r;
        // get_angle(&p, &y, &r);
        // printf("Pitch: %.2f, Yaw: %.2f, Roll: %.2f\n", p, y, r);
        int left;
     left = encoder_left_get_count();
     printf("Left Encoder Count: %d\n", left);


        system_delay_ms(200);
    }

    all_end();
    return 0;
}