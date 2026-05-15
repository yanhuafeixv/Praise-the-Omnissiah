#include "zf_common_headfile.h"
#include "myi2c.h"


#define myi2c_test_gpio "/dev/zf_driver_gpio_motor_1"
#define myi2c_test_gpio2 "/dev/zf_driver_gpio_motor_2"

#define myi2c_scl "/dev/zf_driver_gpio_myi2c_scl"  //gpio88
#define myi2c_sda "/dev/zf_driver_gpio_myi2c_sda"  //gpio 89


int test_cnt=0;
void myi2c_test(void)
{
	if(test_cnt%2==0){
    gpio_set_level(myi2c_test_gpio, 0);
    gpio_set_level(myi2c_test_gpio2, 1);
	test_cnt++;

	printf("sda 1, scl 0\n");
	}else{
	gpio_set_level(myi2c_test_gpio2, 1);
	gpio_set_level(myi2c_test_gpio2, 0);
	test_cnt++;
	printf("sda 0, scl 1\n");
	}


	system_delay_ms(2000);									 //延时10us，防止时序频率超过要求

}



////////////////////////////////////////////////////////////////////////////////////////////////

int myi2c_scl_fd = -1;
int myi2c_sda_fd = -1;

// ===================== 底层驱动接口 =====================
int gpio_set_output(int fd)
{
    ioctl(fd, 0, 0); // 0 = 输出模式
    return 0;
}

int gpio_set_input(int fd)
{
    ioctl(fd, 1, 0); // 1 = 输入模式
    return 0;
}
// ===================== I2C 初始化 =====================
void MyI2C_Init(void)
{
    // 打开设备文件
    myi2c_scl_fd = open(myi2c_scl, O_RDWR);
    myi2c_sda_fd = open(myi2c_sda, O_RDWR);

    if (myi2c_scl_fd < 0 || myi2c_sda_fd < 0) {
        perror("open i2c gpio dev failed");
        return;
    }

    // 初始化：SCL/SDA 都设为输入（高阻，总线空闲）
    gpio_set_input(myi2c_scl_fd);
    gpio_set_input(myi2c_sda_fd);

    printf("myi2c_init()\n");
}

// ===================== I2C 写 SCL =====================
void MyI2C_W_SCL(uint8_t BitValue)
{
    if (BitValue == 1) {
        // 高电平：设为输入（高阻，靠外部上拉拉高）
        gpio_set_input(myi2c_scl_fd);
    } else {
        // 低电平：设为输出，拉低
        gpio_set_output(myi2c_scl_fd);
        gpio_set_level(myi2c_scl, 0);
    }
    system_delay_us(10);
}

// ===================== I2C 写 SDA =====================
void MyI2C_W_SDA(uint8_t BitValue)
{
    if (BitValue == 1) {
        // 高电平：设为输入（高阻）
        gpio_set_input(myi2c_sda_fd);
    } else {
        // 低电平：设为输出，拉低
        gpio_set_output(myi2c_sda_fd);
        gpio_set_level(myi2c_sda, 0);
    }
    system_delay_us(10);
    printf("myi2c_w_sda: %d\n", BitValue);
}

// ===================== I2C 读 SDA =====================
uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;
    // 读之前，先把SDA设为输入
    gpio_set_input(myi2c_sda_fd);
    BitValue = gpio_get_level(myi2c_sda);
    system_delay_us(10);
    printf("myi2c_r_sda: %d\n", BitValue);
    return BitValue;
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
