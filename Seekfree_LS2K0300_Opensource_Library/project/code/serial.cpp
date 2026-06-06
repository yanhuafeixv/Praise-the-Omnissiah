#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>

// ========== 全局变量 ==========
char    Serial_RxPacket[100];
uint8_t Serial_RxFlag = 0;

static int      uart_fd   = -1;
static pthread_t rx_thread;
static int      rx_running = 0;

// ========== 内部辅助函数 ==========
static uint32_t Serial_Pow(uint32_t X, uint32_t Y) {
    uint32_t result = 1;
    while (Y--) result *= X;
    return result;
}

// ========== 接收线程（内部使用，不需要 C 链接） ==========
static void *rx_thread_func(void *) {
    uint8_t rx_state   = 0;
    uint8_t p_rx_packet = 0;
    uint8_t rx_data;
    rx_running = 1;

    while (rx_running) {
        ssize_t n = read(uart_fd, &rx_data, 1);
        if (n <= 0) {
            if (n == 0) continue;
            if (errno == EINTR) continue;
            break;
        }

        if (rx_state == 0) {
            if (rx_data == '[' && Serial_RxFlag == 0) {
                rx_state    = 1;
                p_rx_packet = 0;
            }
        }
        else if (rx_state == 1) {
            if (rx_data == ']') {
                rx_state = 0;
                Serial_RxPacket[p_rx_packet] = '\0';
                Serial_RxFlag = 1;
            }
            else {
                Serial_RxPacket[p_rx_packet] = rx_data;
                p_rx_packet++;
                if (p_rx_packet >= sizeof(Serial_RxPacket) - 1) {
                    p_rx_packet = 0;
                    rx_state    = 0;
                }
            }
        }
    }
    return NULL;
}

// ========== 导出的接口函数 —— 必须放在 extern "C" 块中 ==========
extern "C" {

int Serial_Init(const char *device, int baudrate)
{
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) {
        perror("Serial_Init: open failed");
        return -1;
    }

    int flags = fcntl(uart_fd, F_GETFL, 0);
    fcntl(uart_fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios opt;
    if (tcgetattr(uart_fd, &opt) != 0) {
        perror("tcgetattr");
        close(uart_fd);
        return -1;
    }

    speed_t speed;
    switch (baudrate) {
        case 2400:    speed = B2400;    break;
        case 4800:    speed = B4800;    break;
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        default:
            fprintf(stderr, "Unsupported baudrate: %d\n", baudrate);
            close(uart_fd);
            return -1;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);

    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |=  CS8;
    opt.c_cflag &= ~CRTSCTS;
    opt.c_cflag |= CREAD | CLOCAL;

    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;

    opt.c_cc[VMIN]  = 1;
    opt.c_cc[VTIME] = 1;

    if (tcsetattr(uart_fd, TCSANOW, &opt) != 0) {
        perror("tcsetattr");
        close(uart_fd);
        return -1;
    }

    tcflush(uart_fd, TCIOFLUSH);

    printf("Serial port %s opened, baudrate=%d, 8N1\n", device, baudrate);
    return 0;
}

void Serial_Close(void) {
    rx_running = 0;
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}

void Serial_SendByte(uint8_t byte) {
    if (uart_fd >= 0) {
        write(uart_fd, &byte, 1);
    }
}

void Serial_SendArray(const uint8_t *array, uint16_t length) {
    if (uart_fd >= 0) {
        write(uart_fd, array, length);
    }
}

void Serial_SendString(const char *string) {
    if (uart_fd >= 0 && string) {
        write(uart_fd, string, strlen(string));
    }
}

void Serial_SendNumber(uint32_t number, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        Serial_SendByte(number / Serial_Pow(10, length - i - 1) % 10 + '0');
    }
}

void Serial_Printf(const char *format, ...) {
    char string[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);
    Serial_SendString(string);
}

void Serial_StartRx(void) {
    pthread_create(&rx_thread, NULL, rx_thread_func, NULL);
    pthread_detach(rx_thread);
}

} // extern "C"