#include "zf_common_headfile.h"

#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

void encoder_get(int16 *encoder_left,int16 *encoder_right)
{
    *encoder_left  = encoder_get_count(ENCODER_1);
    *encoder_right = encoder_get_count(ENCODER_2);
}