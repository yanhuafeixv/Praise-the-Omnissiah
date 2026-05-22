#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

int pca9685_init(const char *i2c_device, float freq);
void pca9685_set_servo(int channel, int angle);
void pca9685_close(void);

#ifdef __cplusplus
}
#endif

#endif


/*例程
int main() {
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


    while(1)
    {
            pca9685_set_servo(0, 120);
            printf("Angle: %d\n", 120);
        sleep(1);
        // system_delay_ms(2000);
    }
    pca9685_close();
    return 0;
}

*/