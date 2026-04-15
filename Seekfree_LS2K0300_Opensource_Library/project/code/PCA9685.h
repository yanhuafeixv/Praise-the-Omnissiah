#ifndef __PCA9685_H
#define __PCA9685_H
#include "zf_common_headfile.h"

#define PCA_Addr 0x80
#define PCA_Model 0x00
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA_Pre 0xFE

void PCA9685_Init(float hz,uint16 angle);

void PCA9685_Write(uint8 addr,uint8 data);

uint8 PCA9685_Read(uint8 addr);
void PCA9685_setPWM(uint8 num,uint32 on,uint32 off);

void PCA9685_setFreq(float freq);

void setAngle(uint8 num,uint16 angle);
#endif

