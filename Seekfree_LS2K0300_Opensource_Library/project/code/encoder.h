#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

int encoder_left_init(void);
int encoder_left_get_count(void);
void encoder_left_deinit(void);

int encoder_right_init(void);
int encoder_right_get_count(void);
void encoder_right_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H