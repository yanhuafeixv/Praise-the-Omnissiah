#include "zf_common_headfile.h"
#include "myi2c.h"


#define myi2c_test_gpio "/dev/zf_driver_gpio_motor_1"
#define myi2c_test_gpio2 "/dev/zf_driver_gpio_motor_2"

#define myi2c_scl "/dev/zf_device_pwm_esc_1"  //gpio88
#define myi2c_sda "/dev/zf_device_pwm_esc_2"  //gpio 89


int test_cnt=0;
void myi2c_test(void)
{
	if(test_cnt%2==0){
    gpio_set_level(myi2c_scl, 0);
    gpio_set_level(myi2c_sda, 1);
	test_cnt++;

	printf("sda 1, scl 0\n");
	}else{
	gpio_set_level(myi2c_scl, 1);
	gpio_set_level(myi2c_sda, 0);
	test_cnt++;
	printf("sda 0, scl 1\n");
	}


	system_delay_ms(2000);									 //延时10us，防止时序频率超过要求

}


void MyI2C_W_SCL(uint8 BitValue)
{
	gpio_set_level(myi2c_scl, BitValue);                       //根据BitValue，设置SCL引脚的电平
	system_delay_us(10);										 //延时10us，防止时序频率超过要求
    
    
}

/**
  * 函    数：I2C写SDA引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SDA的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue为1时，需要置SDA为高电平
  */
void MyI2C_W_SDA(uint8 BitValue)
{
	gpio_set_level(myi2c_sda, BitValue);		//根据BitValue，设置SDA引脚的电平，BitValue要实现非0即1的特性
	system_delay_us(10);												//延时10us，防止时序频率超过要求


	printf("myi2c_w_sda: %d\n", BitValue);
}

/**
  * 函    数：I2C读SDA引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前SDA的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
  */
uint8 MyI2C_R_SDA(void)
{
	uint8 BitValue;
	BitValue = gpio_get_level(myi2c_sda);		//读取SDA电平
	system_delay_us(10);												//延时10us，防止时序频率超过要求

	printf("myi2c_r_sda: %d\n", BitValue);

	return BitValue;											//返回SDA电平
}

/**
  * 函    数：I2C初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，实现SCL和SDA引脚的初始化
  */
void MyI2C_Init(void)             //设置scl和sda为开漏输出，并默认高电平,需要修改设备树
{
	// /*开启时钟*/
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟
	
	// /*GPIO初始化*/
	// GPIO_InitTypeDef GPIO_InitStructure;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB10和PB11引脚初始化为开漏输出
	
	// /*设置默认电平*/
	// GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);			//设置PB10和PB11引脚初始化后默认为高电平（释放总线状态）

    gpio_set_level(myi2c_scl, 1);						//设置SCL引脚为高电平
    gpio_set_level(myi2c_sda, 1);						//设置SDA引脚为高电平

	printf("myi2c_init()\n");

}

/*协议层*/

/**
  * 函    数：I2C起始
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平
	MyI2C_W_SCL(1);							//释放SCL，确保SCL为高电平
	MyI2C_W_SDA(0);							//在SCL高电平期间，拉低SDA，产生起始信号
	MyI2C_W_SCL(0);							//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接

	printf("myi2c_start()\n");
}

/**
  * 函    数：I2C终止
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号

	printf("myi2c_stop()\n");
}

/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MyI2C_SendByte(uint8 Byte)
{
	uint8 i;
	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
	{
		/*两个!可以对数据进行两次逻辑取反，作用是把非0值统一转换为1，即：!!(0) = 0，!!(非0) = 1*/
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	}

	printf("myi2c_send_byte: %02X\n", Byte);
}

/**
  * 函    数：I2C接收一个字节
  * 参    数：无
  * 返 回 值：接收到的一个字节数据，范围：0x00~0xFF
  */
uint8 MyI2C_ReceiveByte(void)
{
	uint8 i, Byte = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	for (i = 0; i < 8; i ++)				//循环8次，主机依次接收数据的每一位
	{
		MyI2C_W_SCL(1);						//释放SCL，主机机在SCL高电平期间读取SDA
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte变量
													//当SDA为1时，置变量指定位为1，当SDA为0时，不做处理，指定位为默认的初值0
		MyI2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
	}

	printf("myi2c_receive_byte: %02X\n", Byte);

	return Byte;							//返回接收到的一个字节数据
}

/**
  * 函    数：I2C发送应答位
  * 参    数：Byte 要发送的应答位，范围：0~1，0表示应答，1表示非应答
  * 返 回 值：无
  */
void MyI2C_SendAck(uint8 AckBit)
{
	MyI2C_W_SDA(AckBit);					//主机把应答位数据放到SDA线
	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块

	printf("myi2c_send_ack: %d\n", AckBit);

}

/**
  * 函    数：I2C接收应答位
  * 参    数：无
  * 返 回 值：接收到的应答位，范围：0~1，0表示应答，1表示非应答
  */
uint8 MyI2C_ReceiveAck(void)
{
	uint8 AckBit;							//定义应答位变量
	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	MyI2C_W_SCL(1);							//释放SCL，主机机在SCL高电平期间读取SDA
	AckBit = MyI2C_R_SDA();					//将应答位存储到变量里
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块

	printf("myi2c_receive_ack: %d\n", AckBit);

	return AckBit;							//返回定义应答位变量
}
