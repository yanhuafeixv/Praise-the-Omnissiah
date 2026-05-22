// //暂定坐标为int16，角度为float，后续根据实际情况修改

#include "zf_common_headfile.h"
#include "pca9685_servo.h"
#include <math.h>
#include <stdint.h>

// void arm_angle2pwm(float angle1,float angle2,float angle3,float angle4);//根据实际情况选择参数类型
// void arm_poistion2angle(uint16 x,uint16 y,uint16 z);     //参数按照坐标是int还是float类型进行修改


#define arm1_length  10.45   //宏定义机械臂的长度及圆盘半径
#define arm2_length 12.0
#define base_high  7.8



 void arm_move(double x,double y,double z)     //参数按照坐标是int还是float类型进行修改
{
    double h,l1,l2;               //h为底盘高度，a1为第一段机械臂长度，a2为第二段机械臂长度
    int16_t j1,j2,j3;               //三个姿态角 

 	h= base_high;     //底部圆盘半径
 	l1 = arm1_length; 	//底部圆盘高度	            
 	l2 = arm2_length;    //机械臂长度
	
// 1. 计算水平距离 r
    double r = sqrt(x * x + y * y);
    // 2. 求解 j1（注意 atan2(y, x) 返回弧度）
    double j1_rad = atan2(y, x);
    j1 = (int16_t) round(j1_rad * 180.0 / M_PI);
    // 3. 将空间坐标转化为平面坐标 (p, q)
    double p = r;                // 水平投影距离
    double q = z - h;            // 垂直高度差
    // 4. 计算中间值 d = cos(j3)
    double d = (p * p + q * q - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (d < -1.0 || d > 1.0) {
        return ;               // 目标点超出机械臂可达域
    }
    // 5. 求解 j3（取负值，即肘关节通常向下折叠）
    //    若需要另一组解，可将以下改为  j3_rad = acos(d);
    double j3_rad = -acos(d);   // 取肘部“下折”解
    // double j3_rad = acos(d); // 另一组解
    // 6. 求解 j2
    double phi = atan2(p, q);   // φ = atan2(r, z-h)
    double beta = atan2(l2 * sin(j3_rad), l1 + l2 * cos(j3_rad));
    double j2_rad = phi - beta;
    // 7. 转为度并四舍五入到 int16
    j2 = (int16_t) round(j2_rad * 180.0 / M_PI);
    j3 = (int16_t) round(j3_rad * 180.0 / M_PI);


    pca9685_set_servo(0, j1);
    printf("Angle: %d\n", j1);
    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(1, j2);
    printf("Angle: %d\n", j2);
    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(2, j3);
    printf("Angle: %d\n", j3);
    sleep(1);
    system_delay_ms(1000);


    return ;


}

