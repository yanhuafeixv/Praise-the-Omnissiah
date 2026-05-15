#include "pca9685_linux.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <errno.h>
#include "zf_common_headfile.h"

// 直接写寄存器
static int write_reg(int fd, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    if (write(fd, buf, 2) != 2) {
        perror("write_reg");
        return -1;
    }
    return 0;
}

// 读寄存器
static int read_reg(int fd, uint8_t reg, uint8_t *data) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data rdwr;

    msgs[0].addr = PCA_ADDR;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    msgs[1].addr = PCA_ADDR;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = data;

    rdwr.msgs = msgs;
    rdwr.nmsgs = 2;

    if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
        perror("read_reg");
        return -1;
    }
    return 0;
}

// 初始化 PCA9685，设置 PWM 频率
int pca9685_init(int fd, float freq) {
    uint8_t mode1;

    // 1. 进入睡眠模式
    if (read_reg(fd, MODE1, &mode1) < 0) return -1;
    mode1 = (mode1 & 0x7F) | 0x10;   // SLEEP=1
    if (write_reg(fd, MODE1, mode1) < 0) return -1;

    // 2. 设置预分频器
    double prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5);
    printf("Set prescaler to %d for freq=%.2f Hz\n", prescale, freq);
    if (write_reg(fd, PRE_SCALE, prescale) < 0) return -1;

    // 3. 唤醒（SLEEP=0）并重启
    mode1 = (mode1 & 0xEF) | 0x80;   // SLEEP=0, RESTART=1
    if (write_reg(fd, MODE1, mode1) < 0) return -1;

    // 可选：配置 MODE2 为默认（totem pole）
    if (write_reg(fd, MODE2, 0x04) < 0) return -1;

    return 0;
}

// 设置单通道 PWM
int pca9685_set_pwm(int fd, int channel, uint16_t on, uint16_t off) {
    if (channel < 0 || channel > 15) return -1;
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {reg, on & 0xFF, on >> 8, off & 0xFF, off >> 8};
    if (write(fd, buf, 5) != 5) {
        perror("pca9685_set_pwm");
        return -1;
    }
    return 0;
}

// 同时设置所有通道
int pca9685_set_all_pwm(int fd, uint16_t on, uint16_t off) {
    uint8_t reg = ALL_LED_ON_L;
    uint8_t buf[5] = {reg, on & 0xFF, on >> 8, off & 0xFF, off >> 8};
    if (write(fd, buf, 5) != 5) {
        perror("pca9685_set_all_pwm");
        return -1;
    }
    return 0;
}

// 舵机角度设置（修正了原始 STM32 代码中的错误，应将计算的 off 值传入）
void set_angle(int fd, int channel, uint16_t angle) {
    uint16_t off = (uint16_t)(103 + angle * 1.52);  // 270度舵机，每度1.52
    pca9685_set_pwm(fd, channel, 0, off);
}