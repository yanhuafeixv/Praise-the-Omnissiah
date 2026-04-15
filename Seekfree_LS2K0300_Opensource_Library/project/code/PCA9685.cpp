#include "zf_common_headfile.h"
#include "PCA9685.h"
#include "myi2c.h"
//#define myi2c_test_gpio "/dev/zf_driver_gpio_motor_2"



void PCA9685_Write(uint8 addr,uint8 data)    //   addr 表示要写入数据的寄存器地址，data 表示要写入的数据
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(PCA_Addr);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(addr);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
	
}

uint8 PCA9685_Read(uint8 addr)                //   addr 表示要读取数据的寄存器地址
{
	uint8 Data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(PCA_Addr);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(addr);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(PCA_Addr | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
	
}

void PCA9685_setFreq(float freq)
{
	uint8 prescale,oldmode,newmode;               //定义了三个无符号 8 位整型变量 用于存储预分频器值、旧的模式寄存器值和新的模式寄存器值
	
	double prescaleval;                        //定义了一个双精度浮点型变量 prescaleval，用于计算预分频器的值。
	
	freq *= 0.98;                             //将传入的频率值乘以 0.98，这是为了微调频率值以适应 PCA9685 的实际需求
	prescaleval = 25000000;                   //这是 PCA9685 内部振荡器的频率
	prescaleval /= 4096;                      //每个周期从0计数到4095，除以 4096，得到每个计数器周期的时间，
	prescaleval /= freq;                      //除以所需的频率值，得到预分频器的值。
	prescaleval -= 1;                         //减去 1，得到最终的预分频器值
	prescale = floor(prescaleval+0.5f);       //将计算得到的预分频器值四舍五入取整，并将其赋值给 prescale 变量。
	oldmode = PCA9685_Read(PCA_Model);        //通过调用 PCA9685_Read 函数读取当前 PCA9685 寄存器中的模式值，并将其存储在 oldmode 变量中。
	
	newmode = (oldmode&0x7F)|0x10;            //根据旧的模式值计算出新的模式值，将最高位清零（bit 7）并将第 5 位设为1（bit 4），表示将 PCA9685 设置为睡眠模式。
	PCA9685_Write(PCA_Model,newmode);         //将新的模式值写入 PCA9685 的模式寄存器。
	PCA9685_Write(PCA_Pre,prescale);          //将计算得到的预分频器值写入 PCA9685 的预分频器寄存器。
	PCA9685_Write(PCA_Model,oldmode);         //恢复旧的模式值。
	system_delay_ms(5);                              // 延时 5 毫秒，等待 PCA9685 完全启动。
	PCA9685_Write(PCA_Model,oldmode|0xa1);    //将模式值的最高位和第 1 位设为1，表示将 PCA9685 设置为正常工作模式。
	
}

void PCA9685_setPWM(uint8 num,uint32 on,uint32 off)   //num 表示 PWM 通道号，on 表示 PWM 的起始位置，off 表示 PWM 的结束位置（即从高电平切换到低电平的时刻）
{
	MyI2C_Start();                              //发送 I2C 起始信号，开始 I2C 通信。
	
	MyI2C_SendByte(PCA_Addr);                  //发送 PCA9685 的地址，告诉设备我们要和 PCA9685 进行通信。
	MyI2C_ReceiveAck();                           //等待应答信号，确保设备准备好接收数据。

	MyI2C_SendByte(LED0_ON_L+4*num);           //发送 LED 寄存器的地址，根据 PWM 通道号计算出相应的寄存器地址。
	MyI2C_ReceiveAck();                           //
	
	MyI2C_SendByte(on&0xFF);                   //发送 PWM 的起始位置低 8 位。
	MyI2C_ReceiveAck();                           //等待应答信号。
	
	MyI2C_SendByte(on>>8);                     //发送 PWM 的起始位置高 8 位。
	MyI2C_ReceiveAck();                           //等待应答信号。
	
	MyI2C_SendByte(off&0xFF);                  //发送 PWM 的结束位置低 8 位。
	MyI2C_ReceiveAck();                           //等待应答信号。
	
	MyI2C_SendByte(off>>8);                    //发送 PWM 的结束位置高 8 位。
	MyI2C_ReceiveAck();                           //等待应答信号。
	
	MyI2C_Stop();                              //发送 I2C 停止信号，结束本次通信。
	
}

void setAngle(uint8 num,uint16 angle)    //这里的angle建议查看一下，应该不是直接传入角度吧
{
	uint32 off = 0;                
	off = (uint32)(103+angle*1.52);  //270度舵机，每转动一度=1.52   0.5ms -180度起始位置：103
	PCA9685_setPWM(num,0,angle);
}




void PCA9685_Init(float hz,uint16 angle)    //这里的angle建议试一下90
{
	uint32 off = 0;
	MyI2C_Init();
	PCA9685_Write(PCA_Model,0x00);
	PCA9685_setFreq(hz);
    off = (uint32)(103+angle*1.52);  //270度舵机，每转动一度=1.52   0.5ms -180度起始位置：103
	PCA9685_setPWM(0,0,off);
	PCA9685_setPWM(1,0,off);
	PCA9685_setPWM(2,0,off);
	PCA9685_setPWM(3,0,off);
	PCA9685_setPWM(4,0,off);
	PCA9685_setPWM(5,0,off);
	PCA9685_setPWM(6,0,off);
	PCA9685_setPWM(7,0,off);
	PCA9685_setPWM(8,0,off);
	PCA9685_setPWM(9,0,off);
	PCA9685_setPWM(10,0,off);
	PCA9685_setPWM(11,0,off);
	PCA9685_setPWM(12,0,off);
	PCA9685_setPWM(13,0,off);
	PCA9685_setPWM(14,0,off);
	PCA9685_setPWM(15,0,off);
	system_delay_ms(100);
	
}