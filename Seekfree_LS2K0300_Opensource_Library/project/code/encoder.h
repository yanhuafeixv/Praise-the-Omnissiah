#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

// 初始化正交编码器（绑定引脚，启动解码线程）
// gpio_a, gpio_b: BCM 编号（如 42, 43）
// 返回 0 成功，-1 失败
int encoder_init(int gpio_a, int gpio_b);

// 获取当前累计脉冲数（4倍频）
int encoder_get_count(void);

// 停止并释放资源
void encoder_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
