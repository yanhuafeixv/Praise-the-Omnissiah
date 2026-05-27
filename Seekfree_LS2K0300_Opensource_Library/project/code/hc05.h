#ifndef HC05_H
#define HC05_H

#include <stdint.h>

// 接收数据包结构
#define HC05_PACKET_SIZE 100
extern char HC05_RxPacket[HC05_PACKET_SIZE];
extern uint8_t HC05_RxFlag;

// 初始化串口（设备路径、波特率）
int HC05_Init(const char *device, int baudrate);

// 发送单个字节
void HC05_SendByte(uint8_t byte);

// 发送字节数组
void HC05_SendArray(const uint8_t *data, uint16_t len);

// 发送字符串
void HC05_SendString(const char *str);

// 发送数字（指定长度，不足前面补0）
void HC05_SendNumber(uint32_t number, uint8_t length);

// 格式化发送（类似 printf）
void HC05_Printf(const char *format, ...);

// 读取一个字节（非阻塞，返回 0 成功，-1 无数据）
int HC05_ReadByte(uint8_t *byte);

// 接收数据包（自动检测帧头 '[' 和帧尾 ']'，非阻塞，数据存入 HC05_RxPacket）
void HC05_ReceivePacket(void);

// 发送 AT 命令并等待响应
int HC05_ATCommand(const char *cmd, char *response, int timeout_ms);

// 关闭串口
void HC05_Close(void);

// 以下为 HC-05 特有 AT 快捷函数（可选）
int HC05_SetRole(int role);            // 0:从机 1:主机 2:回环
int HC05_SetName(const char *name);
int HC05_SetBaudrate(int baudrate);    // 设置模块波特率（下次生效）
int HC05_Reset(void);
int HC05_Restore(void);

#endif