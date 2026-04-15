#ifndef _myi2c_h_
#define _myi2c_h_

#include "zf_common_headfile.h"

void myi2c_test(void);

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8 Byte);
uint8 MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8 AckBit);
uint8 MyI2C_ReceiveAck(void);

#endif