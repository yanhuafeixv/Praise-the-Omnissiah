// #ifndef HC05_H
// #define HC05_H

// #include <stdint.h>

// #define HC05_PACKET_SIZE 100

// extern char HC05_RxPacket[HC05_PACKET_SIZE];
// extern uint8_t HC05_RxFlag;

// int  HC05_Init(const char *device, int baudrate);
// void HC05_SendByte(uint8_t byte);
// void HC05_SendArray(const uint8_t *data, uint16_t len);
// void HC05_SendString(const char *str);
// void HC05_SendNumber(uint32_t number, uint8_t length);
// void HC05_Printf(const char *format, ...);
// int  HC05_ReadByte(uint8_t *byte);
// void HC05_ReceivePacket(void);
// int  HC05_ATCommand(const char *cmd, char *response, int timeout_ms);
// void HC05_Close(void);

// // 快捷 AT 命令
// int HC05_SetRole(int role);          // 0:从机, 1:主机, 2:回环
// int HC05_SetName(const char *name);
// int HC05_SetBaudrate(int baudrate);  // 设置后立即生效（下次通信需用新波特率）
// int HC05_Reset(void);
// int HC05_Restore(void);

// #endif