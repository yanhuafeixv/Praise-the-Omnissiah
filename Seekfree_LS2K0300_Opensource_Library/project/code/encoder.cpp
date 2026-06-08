#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <atomic>

// ① 包含自己的头文件，让 encoder_t 类型可见
#include "encoder.h"

// ② C++ 原子类型
struct encoder_s {
    int gpio_a, gpio_b;
    int fd_a, fd_b;
    pthread_t thread;
    bool thread_running;
    int prev_a, prev_b;
    std::atomic<int> count;
};

// ---------- 内部辅助函数 ----------
static int gpio_init(int gpio) {
    char path[64];
    int fd;

    // 导出 GPIO
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) {
        snprintf(path, sizeof(path), "%d", gpio);
        write(fd, path, strlen(path));
        close(fd);
    }

    // 方向
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, "in", 2);
    close(fd);

    // 边沿：both
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, "both", 4);
    close(fd);

    // 打开 value 文件
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, O_RDONLY);
}

static void* encoder_thread(void* arg) {
    encoder_t* enc = (encoder_t*)arg;   // 现在 encoder_t 已知
    struct pollfd fds[2];
    char buf_a, buf_b;
    int level_a, level_b;

    while (enc->thread_running) {
        fds[0].fd = enc->fd_a;
        fds[0].events = POLLPRI | POLLERR;
        fds[1].fd = enc->fd_b;
        fds[1].events = POLLPRI | POLLERR;

        int ret = poll(fds, 2, 100);
        if (ret < 0) break;

        // 读取当前电平
        lseek(enc->fd_a, 0, SEEK_SET);
        if (read(enc->fd_a, &buf_a, 1) > 0)
            level_a = buf_a - '0';
        else continue;

        lseek(enc->fd_b, 0, SEEK_SET);
        if (read(enc->fd_b, &buf_b, 1) > 0)
            level_b = buf_b - '0';
        else continue;

        // 4倍频正交解码状态机（与你之前的逻辑完全一致）
        if (enc->prev_a != level_a || enc->prev_b != level_b) {
            int inc = 0;
            if (enc->prev_a == 0 && enc->prev_b == 0) {
                if (level_a == 1 && level_b == 0) inc = 1;
                else if (level_a == 0 && level_b == 1) inc = -1;
            } else if (enc->prev_a == 1 && enc->prev_b == 0) {
                if (level_a == 1 && level_b == 1) inc = 1;
                else if (level_a == 0 && level_b == 0) inc = -1;
            } else if (enc->prev_a == 1 && enc->prev_b == 1) {
                if (level_a == 0 && level_b == 1) inc = 1;
                else if (level_a == 1 && level_b == 0) inc = -1;
            } else if (enc->prev_a == 0 && enc->prev_b == 1) {
                if (level_a == 0 && level_b == 0) inc = 1;
                else if (level_a == 1 && level_b == 1) inc = -1;
            }
            enc->prev_a = level_a;
            enc->prev_b = level_b;
            if (inc) enc->count.fetch_add(inc, std::memory_order_relaxed);
        }
    }
    return nullptr;
}

// ---------- 对外接口 ----------
extern "C" {
    encoder_t* encoder_create(int gpio_a, int gpio_b) {
        encoder_t* enc = (encoder_t*)calloc(1, sizeof(encoder_t));
        if (!enc) return nullptr;

        enc->gpio_a = gpio_a;
        enc->gpio_b = gpio_b;
        enc->fd_a = gpio_init(gpio_a);
        enc->fd_b = gpio_init(gpio_b);
        if (enc->fd_a < 0 || enc->fd_b < 0) {
            close(enc->fd_a);
            close(enc->fd_b);
            free(enc);
            return nullptr;
        }

        enc->thread_running = true;
        if (pthread_create(&enc->thread, nullptr, encoder_thread, enc) != 0) {
            enc->thread_running = false;
            close(enc->fd_a);
            close(enc->fd_b);
            free(enc);
            return nullptr;
        }
        return enc;
    }

    int encoder_get_count(encoder_t* enc) {
        if (!enc) return 0;
        return enc->count.load(std::memory_order_relaxed);
    }

    void encoder_destroy(encoder_t* enc) {
        if (!enc) return;
        enc->thread_running = false;
        pthread_join(enc->thread, nullptr);
        close(enc->fd_a);
        close(enc->fd_b);
        free(enc);
    }
}