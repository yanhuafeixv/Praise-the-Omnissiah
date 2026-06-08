#include "encoder.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>

// 设置 GPIO 方向（0=输出, 1=输入）
// 参数：gpio 全局编号
static void gpio_set_direction_raw(int gpio, int is_input) {
    // GPIO 寄存器基地址（按位控制）
    volatile uint32_t *gpio_base = nullptr;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return;
    }

    // 0x16104000 是 GPIO 寄存器基地址
    // 按位控制方向寄存器的偏移是 0x00（GPIO_OEN）
    gpio_base = (volatile uint32_t*)mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                                         MAP_SHARED, fd, 0x16104000);
    if (gpio_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return;
    }

    close(fd);

    // 方向寄存器在 gpio_base[0]（因为每个寄存器 4 字节，0x00 偏移是第一个 32 位）
    // 每位对应一个 GPIO，1 = 输入，0 = 输出
    uint32_t *oen = (uint32_t*)gpio_base;  // 偏移 0x00

    if (is_input)
        *oen |= (1 << gpio);   // 写 1 为输入
    else
        *oen &= ~(1 << gpio);  // 写 0 为输出

    munmap((void*)gpio_base, 0x1000);
}
int main() {

    gpio_set_direction_raw(64, 1);  // 1 = 输入
    gpio_set_direction_raw(67, 1);
    // 然后正常初始化编码器
    // 编码器1: A=GPIO51, B=GPIO64
    encoder_t* enc1 = encoder_create(51, 64);
    // 编码器2: A=GPIO50, B=GPIO67
    encoder_t* enc2 = encoder_create(50, 67);

    if (!enc1 || !enc2) {
        printf("Encoder init failed!\n");
        return -1;
    }

    while (1) {
        printf("enc1 = %d, enc2 = %d\n",
               encoder_get_count(enc1),
               encoder_get_count(enc2));
        usleep(100000); // 100ms
    }

    encoder_destroy(enc1);
    encoder_destroy(enc2);
    return 0;
}