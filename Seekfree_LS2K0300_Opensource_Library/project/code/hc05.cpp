// #include <stdio.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <termios.h>
// #include <string.h>
// #include <stdarg.h>
// #include <stdlib.h>
// #include "hc05.h"

// static int serial_fd = -1;
// static struct termios old_tio;

// char HC05_RxPacket[HC05_PACKET_SIZE];
// uint8_t HC05_RxFlag = 0;

// static void wait_for_transmit(void) {
//     tcdrain(serial_fd);
// }

// int HC05_Init(const char *device, int baudrate) {
//     if (serial_fd >= 0) return -1;

//     serial_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (serial_fd < 0) {
//         perror("open serial");
//         return -1;
//     }

//     fcntl(serial_fd, F_SETFL, 0);   // 阻塞模式

//     struct termios options;
//     tcgetattr(serial_fd, &options);
//     old_tio = options;

//     speed_t baud;
//     switch (baudrate) {
//         case 2400:   baud = B2400;   break;
//         case 4800:   baud = B4800;   break;
//         case 9600:   baud = B9600;   break;
//         case 19200:  baud = B19200;  break;
//         case 38400:  baud = B38400;  break;
//         case 57600:  baud = B57600;  break;
//         case 115200: baud = B115200; break;
//         default:     baud = B9600;   break;
//     }
//     cfsetispeed(&options, baud);
//     cfsetospeed(&options, baud);

//     options.c_cflag |= (CLOCAL | CREAD);
//     options.c_cflag &= ~CSIZE;
//     options.c_cflag |= CS8;
//     options.c_cflag &= ~PARENB;
//     options.c_iflag &= ~INPCK;
//     options.c_cflag &= ~CSTOPB;
//     options.c_cflag &= ~CRTSCTS;
//     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//     options.c_oflag &= ~OPOST;

//     options.c_cc[VMIN] = 0;
//     options.c_cc[VTIME] = 0;

//     tcflush(serial_fd, TCIFLUSH);
//     if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
//         perror("tcsetattr");
//         close(serial_fd);
//         serial_fd = -1;
//         return -1;
//     }

//     return 0;
// }

// void HC05_SendByte(uint8_t byte) {
//     if (serial_fd < 0) return;
//     write(serial_fd, &byte, 1);
// }

// void HC05_SendArray(const uint8_t *data, uint16_t len) {
//     if (serial_fd < 0) return;
//     write(serial_fd, data, len);
// }

// void HC05_SendString(const char *str) {
//     if (serial_fd < 0) return;
//     write(serial_fd, str, strlen(str));
// }

// static uint32_t pow10(uint8_t n) {
//     uint32_t r = 1;
//     while (n--) r *= 10;
//     return r;
// }

// void HC05_SendNumber(uint32_t number, uint8_t length) {
//     for (int i = 0; i < length; i++) {
//         uint8_t digit = (number / pow10(length - i - 1)) % 10;
//         HC05_SendByte('0' + digit);
//     }
// }

// void HC05_Printf(const char *format, ...) {
//     char buf[256];
//     va_list args;
//     va_start(args, format);
//     vsnprintf(buf, sizeof(buf), format, args);
//     va_end(args);
//     HC05_SendString(buf);
// }

// int HC05_ReadByte(uint8_t *byte) {
//     if (serial_fd < 0) return -1;
//     int ret = read(serial_fd, byte, 1);
//     return (ret == 1) ? 0 : -1;
// }

// void HC05_ReceivePacket(void) {
//     static uint8_t rx_state = 0;
//     static uint8_t pkt_idx = 0;
//     uint8_t data;

//     while (HC05_ReadByte(&data) == 0) {
//         if (rx_state == 0) {
//             if (data == '[' && HC05_RxFlag == 0) {
//                 rx_state = 1;
//                 pkt_idx = 0;
//             }
//         } else if (rx_state == 1) {
//             if (data == ']') {
//                 rx_state = 0;
//                 HC05_RxPacket[pkt_idx] = '\0';
//                 HC05_RxFlag = 1;
//             } else {
//                 if (pkt_idx < HC05_PACKET_SIZE - 1) {
//                     HC05_RxPacket[pkt_idx++] = data;
//                 }
//             }
//         }
//     }
// }

// int HC05_ATCommand(const char *cmd, char *response, int timeout_ms) {
//     if (serial_fd < 0) return -1;

//     HC05_SendString(cmd);
//     HC05_SendString("\r\n");
//     wait_for_transmit();

//     char buf[128];
//     int total = 0;
//     int elapsed = 0;
//     const int step = 10;

//     while (elapsed < timeout_ms) {
//         int n = read(serial_fd, buf + total, sizeof(buf) - total - 1);
//         if (n > 0) {
//             total += n;
//             buf[total] = '\0';
//             if (strstr(buf, "OK") || strstr(buf, "ERROR")) break;
//         }
//         usleep(step * 1000);
//         elapsed += step;
//     }

//     if (response) {
//         strncpy(response, buf, 127);
//         response[127] = '\0';
//     }
//     return (strstr(buf, "OK") != NULL) ? 0 : -1;
// }

// void HC05_Close(void) {
//     if (serial_fd >= 0) {
//         tcsetattr(serial_fd, TCSANOW, &old_tio);
//         close(serial_fd);
//         serial_fd = -1;
//     }
// }

// /* ---------- 快捷 AT 命令 ---------- */
// int HC05_SetRole(int role) {
//     char resp[64], cmd[32];
//     snprintf(cmd, sizeof(cmd), "AT+ROLE=%d", role);
//     return HC05_ATCommand(cmd, resp, 1000);
// }

// int HC05_SetName(const char *name) {
//     char resp[64], cmd[64];
//     snprintf(cmd, sizeof(cmd), "AT+NAME=%s", name);
//     return HC05_ATCommand(cmd, resp, 1000);
// }

// int HC05_SetBaudrate(int baudrate) {
//     char resp[64], cmd[64];
//     snprintf(cmd, sizeof(cmd), "AT+UART=%d,0,0", baudrate);
//     return HC05_ATCommand(cmd, resp, 1000);
// }

// int HC05_Reset(void) {
//     char resp[64];
//     return HC05_ATCommand("AT+RESET", resp, 1000);
// }

// int HC05_Restore(void) {
//     char resp[64];
//     return HC05_ATCommand("AT+ORGL", resp, 1000);
// }