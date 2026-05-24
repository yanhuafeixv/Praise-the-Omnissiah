#include <stdio.h>
#include <unistd.h>
#include "pca9685_servo.h"   // 仅包含头文件
#include "arm.h"

int main() {
    if (pca9685_init("/dev/i2c-1", 50.0) < 0) {
        fprintf(stderr, "Init failed.\n");
        return 1;
    }

    // printf("PCA9685 ready. Moving servo...\n");
    // for (int angle = 7; angle <= 220; angle += 1) {
    //     printf("Angle: %d\n", angle);
    //     pca9685_set_servo(0, angle);
    //     sleep(1);
    // }

    // arm_open();
    // sleep(1);
    // arm_close();
    // sleep(1);
    // arm_stand();


    while(1)
    {

        pca9685_set_servo(0, 0);     //底盘旋转角度
        pca9685_set_servo(3, 180);    //第一段机械臂角度
        pca9685_set_servo(7, 270);    //第二段机械
        pca9685_set_servo(11, 0);   //夹爪
    
    }

    pca9685_close();
    return 0;
}