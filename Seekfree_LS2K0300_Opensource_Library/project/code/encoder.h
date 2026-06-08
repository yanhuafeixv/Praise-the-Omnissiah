#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

// 左编码器： A=GPIO22, B=GPIO23
int encoder_left_init(void);
int encoder_left_get_count(void);
void encoder_left_deinit(void);

// 右编码器： A=GPIO24, B=GPIO25
int encoder_right_init(void);
int encoder_right_get_count(void);
void encoder_right_deinit(void);

#ifdef __cplusplus
}
#endif

#endif