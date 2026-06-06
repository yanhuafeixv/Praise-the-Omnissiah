#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

// GPIO 编号（根据实际接线修改）
#define ENC_A  22
#define ENC_B  23

// 初始化 GPIO，导出并设置为双边沿触发中断
int gpio_init(int gpio) {
    char path[64];
    int fd;

    // 导出 GPIO
    fd = open("/sys/class/gpio/export", O_WRONLY);
    snprintf(path, sizeof(path), "%d", gpio);
    write(fd, path, strlen(path));
    close(fd);

    // 设置方向为输入
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(path, O_WRONLY);
    write(fd, "in", 2);
    close(fd);

    // 设置边沿触发方式（both 表示双边沿）
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpio);
    fd = open(path, O_WRONLY);
    write(fd, "both", 4);
    close(fd);

    // 打开 value 文件，用于 poll
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, O_RDONLY);
}

int main() {
    int fd_a, fd_b;
    struct pollfd fds[2];
    char buf_a[2], buf_b[2];
    int level_a, level_b;
    int prev_a = 0, prev_b = 0;
    int count = 0;

    fd_a = gpio_init(ENC_A);
    fd_b = gpio_init(ENC_B);

    while (1) {
        // 准备 poll
        memset(fds, 0, sizeof(fds));
        fds[0].fd = fd_a;
        fds[0].events = POLLPRI;     // 等待 GPIO 中断
        fds[1].fd = fd_b;
        fds[1].events = POLLPRI;

        // 等待事件（超时时间 100ms）
        int ret = poll(fds, 2, 100);
        if (ret < 0) {
            perror("poll");
            break;
        }

        // 读取当前电平
        lseek(fd_a, 0, SEEK_SET);
        read(fd_a, buf_a, 1);
        level_a = atoi(buf_a);

        lseek(fd_b, 0, SEEK_SET);
        read(fd_b, buf_b, 1);
        level_b = atoi(buf_b);

        // 简单正交解码状态机（4 倍频）
        // 根据 A、B 状态变化更新计数
        if (prev_a != level_a || prev_b != level_b) {
            if (prev_a == 0 && prev_b == 0) {
                if (level_a == 1 && level_b == 0) count++;
                if (level_a == 0 && level_b == 1) count--;
            } else if (prev_a == 1 && prev_b == 0) {
                if (level_a == 1 && level_b == 1) count++;
                if (level_a == 0 && level_b == 0) count--;
            } else if (prev_a == 1 && prev_b == 1) {
                if (level_a == 0 && level_b == 1) count++;
                if (level_a == 1 && level_b == 0) count--;
            } else if (prev_a == 0 && prev_b == 1) {
                if (level_a == 0 && level_b == 0) count++;
                if (level_a == 1 && level_b == 1) count--;
            }
            prev_a = level_a;
            prev_b = level_b;
            printf("Encoder count: %d\n", count);
        }
    }

    close(fd_a);
    close(fd_b);
    return 0;
}