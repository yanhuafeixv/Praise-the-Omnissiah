// #include "zf_common_headfile.h"
// #include "myi2c.h"
// #include "PCA9685.h"
// #include "key.h"
// #include "pca9685_linux.h"

// int main(int, char**) 
// {

//     //PCA9685_Init(50, 80);   //初始化 PCA9685，设置频率为 50Hz，初始角度为 130 度
//     //MyI2C_W_SCL(1);
//     //MyI2C_W_SDA(1);
//     while(1)
//     {    myi2c_test();     //测试代码，用来测试代码是否可以被正常引用    

//     //printf("124");
//     }
// }




// ///********************************************************************************************************************
// #include "pca9685_linux.h"
// #include <stdio.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <unistd.h>
// #include "zf_common_headfile.h"
// #include <linux/i2c-dev.h>

// int main(void) {
//     // 1. 打开 I2C 总线
//     int fd = open(I2C_DEV_PATH, O_RDWR);
//     if (fd < 0) {
//         perror("open i2c bus");
//         return -1;
//     }
//     // 2. 设置从设备地址
//     if (ioctl(fd, I2C_SLAVE, PCA_ADDR) < 0) {
//         perror("ioctl I2C_SLAVE");
//         close(fd);
//         return -1;
//     }

//     // 3. 初始化，设置 PWM 频率为 50Hz（舵机常用）
//     if (pca9685_init(fd, 50.0f) < 0) {
//         fprintf(stderr, "PCA9685 init failed\n");
//         close(fd);
//         return -1;
//     }

//     // 4. 设置通道0舵机到90度
//     printf("Set channel 0 to 90 degrees\n");
//     set_angle(fd, 0, 90);
//     sleep(1);

//     // 5. 设置通道1到0度
//     set_angle(fd, 1, 0);
//     sleep(1);

//     // 6. 关闭所有通道（可选）
//     pca9685_set_all_pwm(fd, 0, 0);

//     close(fd);
//     return 0;
// }
// //*/

// #include <stdio.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include "zf_common_headfile.h"

// int main() {
//     int fd = open("/dev/i2c-1", O_RDWR);
//     if (fd < 0) { perror("open"); return 1; }

//     if (ioctl(fd, I2C_SLAVE, 0x40) < 0) {
//         perror("ioctl");
//         close(fd);
//         return 1;
//     }

//     // 1. 退出休眠：MODE1 寄存器 (0x00) 写 0x01 -> 正常模式
//     uint8 buf[2] = {0x00, 0x01};
//     write(fd, buf, 2);
//     usleep(5000);

//     // 2. 将所有通道设置为完全导通 (LEDn_ON_H[4] = 1)
//     // 遍历 0~15 通道，设置 LEDn_ON_H 寄存器的高位 bit
//     for (int ch = 0; ch < 16; ch++) {
//         uint8 reg_on_h = 0x07 + (ch * 4); // LEDn_ON_H 地址
//         uint8 cmd[2] = {reg_on_h, 0x10};  // bit4 置1 表示完全导通
//         write(fd, cmd, 2);
//     }

//     printf("All channels set to FULL ON\n");
//     close(fd);
//     return 0;
// }

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

// 设置舵机角度（0-180度）
void pca9685_set_servo(int fd, int channel, int angle) {
    // 计算对应占空比（舵机一般是 0.5ms~2.5ms，对应 4096 计数值）
    int off = 102 + (angle * 204 / 180); // 0.5ms → 102, 2.5ms → 510
    unsigned char buf[5];
    buf[0] = LED0_ON_L + 4 * channel;
    buf[1] = 0x00; buf[2] = 0x00;
    buf[3] = off & 0xFF;
    buf[4] = off >> 8;
    write(fd, buf, 5);
}

int main()
{
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        perror("open /dev/i2c-1 failed");
        return -1;
    }

    // 设置I2C从机地址
    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("ioctl set address failed");
        close(fd);
        return -1;
    }

    // 复位PCA9685
    unsigned char buf[2];
    buf[0] = PCA9685_MODE1;
    buf[1] = 0x10; // 进入sleep模式
    write(fd, buf, 2);

    // 设置PWM频率为50Hz（舵机标准频率）
    buf[0] = PCA9685_PRESCALE;
    buf[1] = 121;  // 25MHz / 4096 / 50Hz - 1 = 121
    write(fd, buf, 2);

    // 退出sleep模式
    buf[0] = PCA9685_MODE1;
    buf[1] = 0x00;
    write(fd, buf, 2);
    usleep(5000);

    printf("PCA9685 init OK, moving servo on channel 0\n");
    // 舵机从0度转到180度再转回来
    for (int angle = 0; angle <= 180; angle += 10) {
        pca9685_set_servo(fd, 0, angle);
        printf("Set angle: %d\n", angle);
        sleep(1);
    }
    for (int angle = 180; angle >= 0; angle -= 10) {
        pca9685_set_servo(fd, 0, angle);
        printf("Set angle: %d\n", angle);
        sleep(1);
    }

    close(fd);
    return 0;
}