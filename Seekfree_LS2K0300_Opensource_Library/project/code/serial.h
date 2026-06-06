#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern char     Serial_RxPacket[100];
extern uint8_t  Serial_RxFlag;

int  Serial_Init(const char *device, int baudrate);
void Serial_Close(void);

void Serial_SendByte(uint8_t byte);
void Serial_SendArray(const uint8_t *array, uint16_t length);
void Serial_SendString(const char *string);
void Serial_SendNumber(uint32_t number, uint8_t length);
void Serial_Printf(const char *format, ...);

void Serial_StartRx(void);

#ifdef __cplusplus
}
#endif

#endif