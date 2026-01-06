1、添加新的编码器解码，注册misc设备。 已写正交编码器
2、添加新的PWM功能，注册misc设备。  PWM已启用
3、添加新的TOF，注册IIO设备。 需要在开机的时候初始化，这里可以判断，是否初始化成功，如果没有。在获取数据的时候，再一次初始化。
4、修改IMU文件，都是用probe驱动，注册IIO设备。 需要在开机的时候初始化，这里可以判断，是否初始化成功，如果没有。在获取数据的时候，再一次初始化。
5、IMU驱动写得太烂，需要重写。可以参考bmi160_core.c文件。   static int imu_probe(struct spi_device *spi)，初始化的时候，可以获取*spi指针，然后使用指针里面的东西作为参数。 其次，这个函数有返回值，可以通过for循环判断。

// 名称为:encoder_atim、encoder_gtim、encoder_pwm
// 总的名称为:zf_driver_encoder_core
// 编码器分为:
//  1、ATIM
//  2、GTIM
//  3、PWM

// 名称为:pwm_atim、pwm_gtim、pwm_pwm
// 总的名称为:zf_driver_encoder_pwm_core
// PWM控制分为:
//  1、ATIM
//  2、GTIM
//  3、PWM

用户层（应用程序）
    ↓
Linux IIO 子系统（标准接口）
    ↓
stm32-adc-core.c（核心层：通用框架、IIO 对接、资源管理）
    ↓ 函数指针调用
stm32-adc.c（基础适配层：硬件寄存器操作、型号专属逻辑）
    ↓
STM32 硬件（ADC 外设）


dl1x文件，注册设备，被应用层调用
    ↓
dl1x_core 对接下面，对下面提供IIC相关接口
    ↓
dl1a、dl1b
    ↓
