#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <math.h>

#define PCA9685_ADDR      0x40
#define MODE1             0x00
#define MODE2             0x01
#define PRESCALE          0xFE
#define LED0_ON_L         0x06

/* ----- 180° 舵机参数 ----- */
#define SERVO_ANGLE_MAX     180          /* 最大角度 */
#define FREQ_CALIB_FACTOR   0.98f        /* 频率校准因子，若实际频率不准可微调 */

/* 内部常数：0° 时 off 起始值 (对应 0.5ms 脉宽) */
#define SERVO_OFF_MIN       102

static int i2c_fd = -1;

static int write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(i2c_fd, buf, 2) != 2) {
        perror("write_reg");
        return -1;
    }
    return 0;
}

static int read_reg(uint8_t reg, uint8_t *val) {
    uint8_t addr = reg;
    if (write(i2c_fd, &addr, 1) != 1) return -1;
    if (read(i2c_fd, val, 1) != 1) return -1;
    return 0;
}

extern "C" {

int pca9685_init(const char *i2c_dev, float freq) {
    uint8_t mode1_ori, prescale;
    float prescale_f;

    if (i2c_fd >= 0) return -1;
    i2c_fd = open(i2c_dev, O_RDWR);
    if (i2c_fd < 0) { perror("open I2C"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("ioctl"); close(i2c_fd); i2c_fd = -1; return -1;
    }

    // 1. MODE2: 推挽输出
    if (write_reg(MODE2, 0x04) < 0) goto fail;

    // 2. 进入睡眠，开启自动增量 (AI=1)，清除 ALLCALL 位
    if (write_reg(MODE1, 0x30) < 0) goto fail;    // 0x30 = SLEEP + AI

    // 3. 计算预分频器（引入频率校准）
    freq *= FREQ_CALIB_FACTOR;                     // 修正内部振荡器偏差
    prescale_f = 25000000.0f / (4096.0f * freq) - 1.0f;
    if (prescale_f < 3.0f) prescale_f = 3.0f;
    if (prescale_f > 255.0f) prescale_f = 255.0f;
    prescale = (uint8_t)(prescale_f + 0.5f);
    if (write_reg(PRESCALE, prescale) < 0) goto fail;

    // 4. 退出睡眠，RESTART + AI，ALLCALL=0
    if (write_reg(MODE1, 0xA0) < 0) goto fail;    // 0xA0 = RESTART + AI
    usleep(5000);

    // 5. 清零所有通道
    for (int ch = 0; ch <= 15; ch++) {
        uint8_t reg = LED0_ON_L + 4 * ch;
        uint8_t buf[5] = {reg, 0x00, 0x00, 0x00, 0x00};
        if (write(i2c_fd, buf, 5) != 5) goto fail;
    }
    return 0;

fail:
    fprintf(stderr, "PCA9685 init failed\n");
    close(i2c_fd);
    i2c_fd = -1;
    return -1;
}

/**
 * @brief 设置 180° 舵机角度
 * @param channel  通道号 (0~15)
 * @param angle    角度 0 ~ 180
 * @note  0° -> off=102, 180° -> off=525，线性插值并四舍五入
 */
void pca9685_set_servo(int channel, int angle) {
    if (i2c_fd < 0) return;
    if (channel < 0 || channel > 15) return;
    if (angle < 0) angle = 0;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    // 线性映射：off = 102 + (angle * (525 - 102)) / 180   (四舍五入)
    uint32_t off = SERVO_OFF_MIN + (uint32_t)(angle * 423 + 90) / 180;
    if (off > 4095) off = 4095;   // 安全钳位

    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        0x00,                      // ON_L = 0
        0x00,                      // ON_H = 0
        (uint8_t)(off & 0xFF),     // OFF_L
        (uint8_t)((off >> 8) & 0x0F)  // OFF_H 仅低 4 位有效
    };
    if (write(i2c_fd, buf, 5) != 5)
        perror("write servo");
}

void pca9685_close(void) {
    if (i2c_fd >= 0) {
        write_reg(MODE1, 0x10);   // 进入睡眠模式
        close(i2c_fd);
        i2c_fd = -1;
    }
}

}  // extern "C"