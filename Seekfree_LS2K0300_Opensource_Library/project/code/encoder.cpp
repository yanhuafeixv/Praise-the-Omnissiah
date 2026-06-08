#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <atomic>

// ===================== 左编码器 =====================
static std::atomic<int> left_count{0};
static int left_fd_a = -1, left_fd_b = -1;
static pthread_t left_thread;
static bool left_running = false;

// ===================== 右编码器 =====================
static std::atomic<int> right_count{0};
static int right_fd_a = -1, right_fd_b = -1;
static pthread_t right_thread;
static bool right_running = false;

// ===================== 通用 GPIO 初始化 ==============
static int gpio_init(int gpio) {
    char path[64];
    int fd;

    // 导出
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

    // 边沿
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, "both", 4);
    close(fd);

    // 打开 value
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, O_RDONLY);
}

// ===================== 解码线程模板 ==================
struct encoder_ctx {
    int fd_a, fd_b;
    std::atomic<int>* count;
    bool* running;
};

static void* encoder_thread(void* arg) {
    encoder_ctx* ctx = (encoder_ctx*)arg;
    struct pollfd fds[2];
    char buf_a, buf_b;
    int level_a, level_b;
    int prev_a = 0, prev_b = 0;
    int local = 0;

    while (*(ctx->running)) {
        fds[0].fd = ctx->fd_a;
        fds[0].events = POLLPRI | POLLERR;
        fds[1].fd = ctx->fd_b;
        fds[1].events = POLLPRI | POLLERR;

        int ret = poll(fds, 2, 100);
        if (ret < 0) break;

        // 读 A 相
        lseek(ctx->fd_a, 0, SEEK_SET);
        if (read(ctx->fd_a, &buf_a, 1) > 0)
            level_a = buf_a - '0';
        else continue;

        // 读 B 相
        lseek(ctx->fd_b, 0, SEEK_SET);
        if (read(ctx->fd_b, &buf_b, 1) > 0)
            level_b = buf_b - '0';
        else continue;

        // 正交解码（4倍频）
        if (prev_a != level_a || prev_b != level_b) {
            int inc = 0;
            if (prev_a == 0 && prev_b == 0) {
                if (level_a == 1 && level_b == 0) inc = 1;
                else if (level_a == 0 && level_b == 1) inc = -1;
            } else if (prev_a == 1 && prev_b == 0) {
                if (level_a == 1 && level_b == 1) inc = 1;
                else if (level_a == 0 && level_b == 0) inc = -1;
            } else if (prev_a == 1 && prev_b == 1) {
                if (level_a == 0 && level_b == 1) inc = 1;
                else if (level_a == 1 && level_b == 0) inc = -1;
            } else if (prev_a == 0 && prev_b == 1) {
                if (level_a == 0 && level_b == 0) inc = 1;
                else if (level_a == 1 && level_b == 1) inc = -1;
            }
            prev_a = level_a;
            prev_b = level_b;
            if (inc) {
                local += inc;
                ctx->count->store(local, std::memory_order_relaxed);
            }
        }
    }
    return nullptr;
}

// ===================== 对外接口 =====================
extern "C" {

// ----------- 左编码器 -----------
int encoder_left_init(void) {
    if (left_running) return -1;
    left_fd_a = gpio_init(74);   // A
    left_fd_b = gpio_init(26);   // B
    if (left_fd_a < 0 || left_fd_b < 0) return -1;

    static encoder_ctx left_ctx;
    left_ctx.fd_a = left_fd_a;
    left_ctx.fd_b = left_fd_b;
    left_ctx.count = &left_count;
    left_ctx.running = &left_running;

    left_running = true;
    if (pthread_create(&left_thread, nullptr, encoder_thread, &left_ctx) != 0) {
        left_running = false;
        return -1;
    }
    return 0;
}

int encoder_left_get_count(void) {
    return left_count.load(std::memory_order_relaxed);
}

void encoder_left_deinit(void) {
    left_running = false;
    if (left_thread) pthread_join(left_thread, nullptr);
    close(left_fd_a);
    close(left_fd_b);
}

// ----------- 右编码器 -----------
int encoder_right_init(void) {
    if (right_running) return -1;
    right_fd_a = gpio_init(60);   // A
    right_fd_b = gpio_init(62);   // B
    if (right_fd_a < 0 || right_fd_b < 0) return -1;

    static encoder_ctx right_ctx;
    right_ctx.fd_a = right_fd_a;
    right_ctx.fd_b = right_fd_b;
    right_ctx.count = &right_count;
    right_ctx.running = &right_running;

    right_running = true;
    if (pthread_create(&right_thread, nullptr, encoder_thread, &right_ctx) != 0) {
        right_running = false;
        return -1;
    }
    return 0;
}

int encoder_right_get_count(void) {
    return right_count.load(std::memory_order_relaxed);
}

void encoder_right_deinit(void) {
    right_running = false;
    if (right_thread) pthread_join(right_thread, nullptr);
    close(right_fd_a);
    close(right_fd_b);
}

}