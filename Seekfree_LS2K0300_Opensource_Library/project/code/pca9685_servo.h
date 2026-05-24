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