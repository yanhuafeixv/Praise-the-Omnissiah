#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>

#define UART1_BASE      0x16100000
#define UART1_MCR_OFF   0x404          // Modem Control Register
#define AFE_BIT         0x20           // bit5 = Auto Flow Enable

// 导出的函数用 extern "C" 包裹，避免 C++ 名称修饰
extern "C" {

int uart1_disable_auto_flow(void)
{
    int fd = open("/dev/mem", O_RDWR);
    if (fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    void *map = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, UART1_BASE);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return -1;
    }

    volatile uint8_t *mcr = reinterpret_cast<uint8_t *>(
                                static_cast<char *>(map) + UART1_MCR_OFF);
    uint8_t old_val = *mcr;
    *mcr = old_val & ~AFE_BIT;
    uint8_t new_val = *mcr;

    printf("[UART1] AFE disabled: MCR 0x%02x -> 0x%02x\n", old_val, new_val);

    munmap(map, 0x1000);
    close(fd);
    return 0;
}

} // extern "C"