#include "zf_common_headfile.h"
#include "myi2c.h"
#include "PCA9685.h"
#include "key.h"

int main(int, char**) 
{

    PCA9685_Init(50, 90);   //初始化 PCA9685，设置频率为 50Hz，初始角度为 130 度
    while(1)
    {   // myi2c_test();     //测试代码，用来测试代码是否可以被正常引用    

    //printf("124");
    }
}



