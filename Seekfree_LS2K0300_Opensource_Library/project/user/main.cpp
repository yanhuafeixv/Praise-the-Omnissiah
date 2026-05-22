#include "zf_common_headfile.h"
#include <stdio.h>
#include <unistd.h>
#include "pca9685_servo.h"


int main(int, char**) {
    if (pca9685_init("/dev/i2c-1", 50.0) < 0) {
        fprintf(stderr, "Init failed.\n");
        return 1;
    }

    // printf("--- Moving servo to 0 deg ---\n");
    // pca9685_set_servo(0, 0);
    // sleep(2);
    // printf("--- Moving servo to 90 deg ---\n");
    // pca9685_set_servo(0, 90);
    // sleep(2);
    // printf("--- Moving servo to 270 deg ---\n");
    // pca9685_set_servo(0, 270);
    // sleep(2);

    int angle = 0;
    for(angle = 50; angle <= 220; angle += 1) {
        printf("--- Moving servo to %d deg ---\n", angle);
        pca9685_set_servo(0, angle);
        sleep(1);
        system_delay_ms(2000);
    }

    // while(1)
    // {
    //         pca9685_set_servo(0, 120);
    //         printf("Angle: %d\n", 120);
    //     sleep(1);
    //     // system_delay_ms(2000);
    // }
    pca9685_close();
    return 0;
}
