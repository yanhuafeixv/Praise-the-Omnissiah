#ifndef ENCODER_H
#define ENCODER_H

// 不透明句柄，C/C++ 均可使用
typedef struct encoder_s encoder_t;

#ifdef __cplusplus
extern "C" {
#endif

encoder_t* encoder_create(int gpio_a, int gpio_b);
int encoder_get_count(encoder_t* enc);
void encoder_destroy(encoder_t* enc);

#ifdef __cplusplus
}
#endif

#endif