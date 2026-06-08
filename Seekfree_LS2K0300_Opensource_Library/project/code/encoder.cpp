#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <atomic>
#include <signal.h>
#include <sys/select.h>

static std::atomic<int> encoder_count{0};

static int fd_a = -1;
static int fd_b = -1;
static pthread_t thread_id;
static bool thread_running = false;

// 简单的 GPIO 初始化（导出、方向、边沿）
static int gpio_init(int gpio) {
    char path[64];
    int fd;

    // 导出
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    snprintf(path, sizeof(path), "%d", gpio);
    if (write(fd, path, strlen(path)) < 0) {
        close(fd);
        // 可能已经导出，忽略错误
    }
    close(fd);

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

// 解码线程
static void* encoder_thread(void* arg) {
    (void)arg;
    struct pollfd fds[2];
    char buf_a[2], buf_b[2];
    int level_a, level_b;
    int prev_a = 0, prev_b = 0;
    int local_count = 0;

    while (thread_running) {
        fds[0].fd = fd_a;
        fds[0].events = POLLPRI | POLLERR;
        fds[1].fd = fd_b;
        fds[1].events = POLLPRI | POLLERR;

        int ret = poll(fds, 2, 100); // 100ms 超时
        if (ret < 0) {
            perror("poll");
            break;
        }

        // 读取当前电平
        lseek(fd_a, 0, SEEK_SET);
        if (read(fd_a, buf_a, 1) > 0)
            level_a = buf_a[0] - '0';
        else continue;

        lseek(fd_b, 0, SEEK_SET);
        if (read(fd_b, buf_b, 1) > 0)
            level_b = buf_b[0] - '0';
        else continue;

        // 4倍频正交解码状态机
        if (prev_a != level_a || prev_b != level_b) {
            if (prev_a == 0 && prev_b == 0) {
                if (level_a == 1 && level_b == 0) local_count++;
                else if (level_a == 0 && level_b == 1) local_count--;
            } else if (prev_a == 1 && prev_b == 0) {
                if (level_a == 1 && level_b == 1) local_count++;
                else if (level_a == 0 && level_b == 0) local_count--;
            } else if (prev_a == 1 && prev_b == 1) {
                if (level_a == 0 && level_b == 1) local_count++;
                else if (level_a == 1 && level_b == 0) local_count--;
            } else if (prev_a == 0 && prev_b == 1) {
                if (level_a == 0 && level_b == 0) local_count++;
                else if (level_a == 1 && level_b == 1) local_count--;
            }
            prev_a = level_a;
            prev_b = level_b;
            encoder_count.store(local_count, std::memory_order_relaxed);
        }
    }
    return nullptr;
}
extern "C" {
int encoder_init(int gpio_a, int gpio_b) {
    if (thread_running) return -1; // 已初始化

    fd_a = gpio_init(gpio_a);
    fd_b = gpio_init(gpio_b);
    if (fd_a < 0 || fd_b < 0) {
        close(fd_a);
        close(fd_b);
        return -1;
    }

    thread_running = true;
    if (pthread_create(&thread_id, nullptr, encoder_thread, nullptr) != 0) {
        thread_running = false;
        close(fd_a);
        close(fd_b);
        return -1;
    }
    return 0;
}

int encoder_get_count(void) {
    return encoder_count.load(std::memory_order_relaxed);
}

void encoder_deinit(void) {
    thread_running = false;
    if (thread_id) {
        pthread_join(thread_id, nullptr);
    }
    close(fd_a);
    close(fd_b);
    // 可选：unexport GPIO
}
}
