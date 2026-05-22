/*
 * pca9685_servo.cpp - Robust PCA9685 servo-only driver with debug
 */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>

#define PCA9685_ADDR      0x40
#define MODE1             0x00
#define MODE2             0x01
#define PRESCALE          0xFE
#define LED0_ON_L         0x06

#define SERVO_ANGLE_MAX   270
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500

static int i2c_fd = -1;

/* 写单个寄存器（调试用） */
static int write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int ret = write(i2c_fd, buf, 2);
    if (ret != 2) {
        perror("write_reg");
        return -1;
    }
    // printf("  write_reg 0x%02X=0x%02X\n", reg, val);
    return 0;
}

/* 读单个寄存器 */
static int read_reg(uint8_t reg, uint8_t *val) {
    uint8_t addr = reg;
    if (write(i2c_fd, &addr, 1) != 1) return -1;
    if (read(i2c_fd, val, 1) != 1) return -1;
    return 0;
}

/* 初始化 */
int pca9685_init(const char *i2c_dev, float freq) {
    uint8_t prescale;
    float prescale_f;

    if (i2c_fd >= 0) return -1;

    i2c_fd = open(i2c_dev, O_RDWR);
    if (i2c_fd < 0) {
        perror("open I2C");
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("ioctl");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }

    printf("I2C device opened, starting init...\n");

    // 1. MODE2: 推挽输出 (OUTDRV=1)
    if (write_reg(MODE2, 0x04) < 0) goto fail;
    printf("MODE2 set.\n");

    // 2. 进入睡眠，开启自动增量 (AI=1) (SLEEP=1, AI=1, RESTART=0)
    if (write_reg(MODE1, 0x30) < 0) goto fail;
    printf("Entered sleep with AI.\n");

    // 3. 设置预分频器为 121 (50Hz)
    prescale_f = 25000000.0f / (4096.0f * freq) - 1.0f;
    if (prescale_f < 3.0f) prescale_f = 3.0f;
    if (prescale_f > 255.0f) prescale_f = 255.0f;
    prescale = (uint8_t)(prescale_f + 0.5f);
    printf("Prescale value = %u\n", prescale);
    if (write_reg(PRESCALE, prescale) < 0) goto fail;

    // 4. 退出睡眠：RESTART=1, AI=1, SLEEP=0, 其余位清零（不使用EXTCLK等）
    if (write_reg(MODE1, 0xA0) < 0) goto fail;   // 0xA0 = RESTART+AI
    printf("Exited sleep, RESTART+AI set.\n");
    usleep(5000);

    // 5. 验证 MODE1 是否正确
    uint8_t mode1_check;
    if (read_reg(MODE1, &mode1_check) == 0) {
        printf("MODE1 after init = 0x%02X\n", mode1_check);
        if ((mode1_check & 0xA0) != 0xA0) {
            printf("WARNING: MODE1 not as expected (0xA0)!\n");
        }
    }

    // 6. 清零所有通道（一次写入5字节：地址+4数据）
    printf("Clearing all channels...\n");
    for (int ch = 0; ch <= 15; ch++) {
        uint8_t reg = LED0_ON_L + 4 * ch;
        uint8_t buf[5] = {reg, 0x00, 0x00, 0x00, 0x00};
        if (write(i2c_fd, buf, 5) != 5) {
            perror("clear channel");
            goto fail;
        }
    }

    printf("PCA9685 init success.\n");
    return 0;

fail:
    printf("PCA9685 init FAILED.\n");
    close(i2c_fd);
    i2c_fd = -1;
    return -1;
}

/* 设置舵机角度 */
void pca9685_set_servo(int channel, int angle) {
    if (i2c_fd < 0) {
        printf("ERROR: I2C not initialized!\n");
        return;
    }
    if (channel < 0 || channel > 15) return;
    if (angle < 0) angle = 0;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    uint32_t pulse_us = SERVO_MIN_US + 
                        (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / SERVO_ANGLE_MAX;
    uint32_t off = (pulse_us * 4096UL) / 20000UL;
    if (off > 4095) off = 4095;

    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        0x00,                    // ON_L
        0x00,                    // ON_H
        (uint8_t)(off & 0xFF),   // OFF_L
        (uint8_t)((off >> 8) & 0x0F) // OFF_H
    };

    printf("Set servo ch%d: angle=%d, pulse=%u us, off=%u, reg=0x%02X, data=",
           channel, angle, pulse_us, off, reg);
    for (int i = 0; i < 5; i++) printf("%02X ", buf[i]);
    printf("\n");

    if (write(i2c_fd, buf, 5) != 5) {
        perror("write servo");
    }
}

/* 关闭 */
void pca9685_close(void) {
    if (i2c_fd >= 0) {
        write_reg(MODE1, 0x10);   // 进入睡眠
        close(i2c_fd);
        i2c_fd = -1;
        printf("PCA9685 closed.\n");
    }
}
