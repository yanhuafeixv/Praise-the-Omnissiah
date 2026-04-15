#ifndef _arm_h
#define _arm_h

void arm_angle2pwm(float angle1,float angle2,float angle3,float angle4);
void arm_poistion2angle(uint16 x,uint16 y,uint16 z);     
void arm_move(uint16 x,uint16 y,uint16 z);

#endif