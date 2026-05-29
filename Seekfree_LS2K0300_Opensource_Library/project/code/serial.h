#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>

/* 全局变量，等价于 STM32 版本 */
extern char Serial_RxPacket[100];
extern uint8_t Serial_RxFlag;

/* 初始化和关闭 */
int  Serial_Init(const char *device, int baudrate);
void Serial_Close(void);

/* 发送函数 */
void Serial_SendByte(uint8_t byte);
void Serial_SendArray(uint8_t *array, uint16_t length);
void Serial_SendString(char *string);
void Serial_SendNumber(uint32_t number, uint8_t length);
void Serial_Printf(char *format, ...);

/* 启动接收线程（模拟中断接收） */
void Serial_StartRx(void);

#endif