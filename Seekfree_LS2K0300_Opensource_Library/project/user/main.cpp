#include <stdio.h>
#include <unistd.h>
#include "pca9685_servo.h"   // 仅包含头文件
#include "arm.h"

int main() {
    if (pca9685_init("/dev/i2c-1", 50.0) < 0) {
        fprintf(stderr, "Init failed.\n");
        return 1;
    }

    while(1)
    {
        // pca9685_set_servo(3, 45);     //底盘旋转角度
        // pca9685_set_servo(7, 135);     //底盘旋转角度
        // pca9685_set_servo(0, 0);     //底盘旋转角度
        // pca9685_set_servo(11, 180);     //底盘旋转角度

        arm_move(-10,12,18);
       // arm_stand();
    }

    // for(int i=35;i<55;i++)
    // {
    //         pca9685_set_servo(3, i);
    //         pca9685_set_servo(7, 180-i);
    //         printf("angle: %d\n",i);
            
    //         sleep(1);

    // }
    pca9685_close();
    return 0;
}
//550    80