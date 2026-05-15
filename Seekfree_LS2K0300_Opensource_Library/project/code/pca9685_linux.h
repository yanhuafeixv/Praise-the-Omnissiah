#ifndef _PCA9685_LINUX_H
#define _PCA9685_LINUX_H

#include <stdint.h>
#include "zf_common_headfile.h"

// I2C 总线设备路径，I2C1 对应 /dev/i2c-1
#define I2C_DEV_PATH "/dev/i2c-1"

// PCA9685 7位地址
#define PCA_ADDR      0x40

// 寄存器定义（保持一致）
#define MODE1         0x00
#define MODE2         0x01
#define LED0_ON_L     0x06
#define LED0_ON_H     0x07
#define LED0_OFF_L    0x08
#define LED0_OFF_H    0x09
#define ALL_LED_ON_L  0xFA
#define ALL_LED_ON_H  0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD
#define PRE_SCALE     0xFE

// 函数原型
int  pca9685_init(int fd, float freq);
int  pca9685_set_pwm(int fd, int channel, uint16_t on, uint16_t off);
int  pca9685_set_all_pwm(int fd, uint16_t on, uint16_t off);
void set_angle(int fd, int channel, uint16_t angle);

#endif