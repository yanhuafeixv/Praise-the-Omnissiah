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

/* ========== 全局变量 ========== */
char    Serial_RxPacket[100];
uint8_t Serial_RxFlag = 0;

static int      uart_fd   = -1;
static pthread_t rx_thread;
static int      rx_running = 0;

/* ========== 内部辅助 ========== */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t result = 1;
    while (Y--) result *= X;
    return result;
}

/* ========== 初始化串口 ========== */
int Serial_Init(const char *device, int baudrate)
{
    uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) {
        perror("Serial_Init: open failed");
        return -1;
    }

    /* 设置为阻塞模式 */
    int flags = fcntl(uart_fd, F_GETFL, 0);
    fcntl(uart_fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios opt;
    if (tcgetattr(uart_fd, &opt) != 0) {
        perror("tcgetattr");
        close(uart_fd);
        return -1;
    }

    /* 设置波特率 */
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

    /* 8位数据位，无校验，1位停止位 (8N1) */
    opt.c_cflag &= ~PARENB;        /* 无校验 */
    opt.c_cflag &= ~CSTOPB;        /* 1位停止位 */
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |=  CS8;           /* 8位数据位 */

    /* 无硬件流控 */
    opt.c_cflag &= ~CRTSCTS;

    /* 启用接收器，忽略调制解调器控制线 */
    opt.c_cflag |= CREAD | CLOCAL;

    /* 原始模式 */
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);   /* 无软件流控 */
    opt.c_oflag &= ~OPOST;

    /* 设置超时：0.1秒 */
    opt.c_cc[VMIN]  = 1;
    opt.c_cc[VTIME] = 1;

    if (tcsetattr(uart_fd, TCSANOW, &opt) != 0) {
        perror("tcsetattr");
        close(uart_fd);
        return -1;
    }

    /* 清空缓冲区 */
    tcflush(uart_fd, TCIOFLUSH);

    printf("Serial port %s opened, baudrate=%d, 8N1\n", device, baudrate);
    return 0;
}

/* ========== 关闭串口 ========== */
void Serial_Close(void)
{
    rx_running = 0;
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}

/* ========== 发送单字节 ========== */
void Serial_SendByte(uint8_t byte)
{
    if (uart_fd >= 0) {
        ssize_t ret = write(uart_fd, &byte, 1);
        if (ret < 0) perror("write");
    }
}

/* ========== 发送数组 ========== */
void Serial_SendArray(uint8_t *array, uint16_t length)
{
    if (uart_fd >= 0) {
        ssize_t ret = write(uart_fd, array, length);
        if (ret < 0) perror("write");
    }
}

/* ========== 发送字符串 ========== */
void Serial_SendString(char *string)
{
    if (uart_fd >= 0 && string) {
        ssize_t ret = write(uart_fd, string, strlen(string));
        if (ret < 0) perror("write");
    }
}

/* ========== 发送数字（指定长度，不足补空格的效果同原版） ========== */
void Serial_SendNumber(uint32_t number, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        Serial_SendByte(number / Serial_Pow(10, length - i - 1) % 10 + '0');
    }
}

/* ========== 格式化打印（等价于 STM32 的 Serial_Printf） ========== */
void Serial_Printf(char *format, ...)
{
    char string[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);
    Serial_SendString(string);
}

/* ========== 接收线程（模拟 STM32 的中断接收） ========== */
static void *rx_thread_func(void *arg)
{
    uint8_t rx_state   = 0;
    uint8_t p_rx_packet = 0;
    uint8_t rx_data;

    rx_running = 1;

    while (rx_running) {
        ssize_t n = read(uart_fd, &rx_data, 1);
        if (n <= 0) {
            if (n == 0) continue;  /* 超时 */
            if (errno == EINTR) continue;
            break;  /* 出错退出 */
        }

        /* ---------- 与 STM32 版完全一致的帧解析逻辑 ---------- */
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
                    /* 溢出保护 */
                    p_rx_packet = 0;
                    rx_state    = 0;
                }
            }
        }
    }
    return NULL;
}

/* ========== 启动接收线程 ========== */
void Serial_StartRx(void)
{
    pthread_create(&rx_thread, NULL, rx_thread_func, NULL);
    pthread_detach(rx_thread);
}