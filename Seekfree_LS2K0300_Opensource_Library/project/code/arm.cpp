// //暂定坐标为int16，角度为float，后续根据实际情况修改

#include "zf_common_headfile.h"
#include "pca9685_servo.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
// void arm_angle2pwm(float angle1,float angle2,float angle3,float angle4);//根据实际情况选择参数类型
// void arm_poistion2angle(uint16 x,uint16 y,uint16 z);     //参数按照坐标是int还是float类型进行修改


#define arm1_length  10.45   //宏定义机械臂的长度及圆盘半径
#define arm2_length 12.0
#define base_high  9.1



#define J1_MIN  -130
#define J1_MAX   47
#define J2_MIN  -30
#define J2_MAX   80
#define J3_MIN  -40
#define J3_MAX  100
/**
 * @brief 带关节限位的逆运动学求解
 * @param x,y,z  末端坐标
 * @param h      基座高度
 * @param l1,l2  连杆长度
 * @param j1,j2,j3  输出有效关节角度（度，四舍五入为 int16_t）
 * @return 0 成功，-1 不可达
 */
int solve_ik_limited(double x, double y, double z,
                    double h, double l1, double l2,
                     int16_t *j1_out, int16_t *j2_out, int16_t *j3_out) {
    double r = sqrt(x * x + y * y);
    // 1. 求解 j1（基座旋转），并映射到 [-130°, 47°]
    double j1_raw = atan2(x, y) * 180.0 / M_PI;   // 范围 (-180, 180]
    int16_t j1_candidate = (int16_t) round(j1_raw);
    // 调整 j1 到限定区间：尝试 j1 + k*360
    bool j1_ok = false;
    for (int k = -1; k <= 1; ++k) {
        int16_t j1_try = j1_candidate + k * 360;
        if (j1_try >= J1_MIN && j1_try <= J1_MAX) {
            j1_candidate = j1_try;
            j1_ok = true;
            break;
        }
    }
    if (!j1_ok) {
        return -1;   // 基座角度无法满足限位
    }
    // 2. 平面二连杆求解
    double p = r;
    double q = z - h;
    double cos_j3 = (p * p + q * q - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (cos_j3 < -1.0 || cos_j3 > 1.0) {
        return -1;   // 不可达
    }
    // 尝试两种肘关节构型：j3 正（上折）和 j3 负（下折），优先正
    double j3_rad_options[2] = { acos(cos_j3), -acos(cos_j3) };
    bool found = false;
    int16_t best_j2, best_j3;
    for (int i = 0; i < 2; ++i) {
        double j3_rad = j3_rad_options[i];
        double j3_deg = j3_rad * 180.0 / M_PI;
        int16_t j3_try = (int16_t) round(j3_deg);
        if (j3_try < J3_MIN || j3_try > J3_MAX) continue;
        // 计算 j2
        double phi = atan2(p, q);
        double beta = atan2(l2 * sin(j3_rad), l1 + l2 * cos(j3_rad));
        double j2_rad = phi - beta;   // 公式对正负 j3 均成立
        double j2_deg = j2_rad * 180.0 / M_PI;
        int16_t j2_try = (int16_t) round(j2_deg);
        // j2 可选加 360° 调整（若角度不在范围内但加 360 后可行）
        if (j2_try < J2_MIN || j2_try > J2_MAX) {
            j2_try += 360;
            if (j2_try < J2_MIN || j2_try > J2_MAX) continue;
        }
        found = true;
        best_j2 = j2_try;
        best_j3 = j3_try;
        break;
    }
    if (!found) {
        return -1;   // 关节限位无法满足
    }
    *j1_out = j1_candidate;
    *j2_out = best_j2;
    *j3_out = best_j3;
    return 0;
}


int arm_move(double x, double y, double z)
{
    double h = base_high, l1 = arm1_length, l2 = arm2_length;
    int16_t j1, j2, j3;
    if (solve_ik_limited(x, y, z, h, l1, l2, &j1, &j2, &j3) < 0) {
        printf("Target unreachable or joint limits exceeded.\n");
        return -1;
    }

    pca9685_set_servo(0, 48-j1);
    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(3, 132-j2);

    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(7, 137-j3);
    sleep(1);
    system_delay_ms(1000);


    printf("j1=%d, j2=%d, j3=%d\n", j1, j2, j3);
    return 0;
}

//  void arm_move(double x,double y,double z)     //参数按照坐标是int还是float类型进行修改
// {
//     double h,l1,l2;               //h为底盘高度，a1为第一段机械臂长度，a2为第二段机械臂长度
//     int16_t j1,j2,j3;               //三个姿态角 

//  	h= base_high;     //底部圆盘半径
//  	l1 = arm1_length; 	//底部圆盘高度	            
//  	l2 = arm2_length;    //机械臂长度
	
// // // 1. 计算水平距离 r
// //     double r = sqrt(x * x + y * y);
// //     // 2. 求解 j1（注意 atan2(y, x) 返回弧度）
// //     double j1_rad = atan2(y, x);
// //     j1 = (int16_t) round(j1_rad * 180.0 / M_PI);
// //     // 3. 将空间坐标转化为平面坐标 (p, q)
// //     double p = r;                // 水平投影距离
// //     double q = z - h;            // 垂直高度差
// //     // 4. 计算中间值 d = cos(j3)
// //     double d = (p * p + q * q - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
// //     if (d < -1.0 || d > 1.0) {
// //         return ;               // 目标点超出机械臂可达域
// //     }
// //     // 5. 求解 j3（取负值，即肘关节通常向下折叠）
// //     //    若需要另一组解，可将以下改为  j3_rad = acos(d);
// //     double j3_rad = -acos(d);   // 取肘部“下折”解
// //     // double j3_rad = acos(d); // 另一组解
// //     // 6. 求解 j2
// //     double phi = atan2(p, q);   // φ = atan2(r, z-h)
// //     double beta = atan2(l2 * sin(j3_rad), l1 + l2 * cos(j3_rad));
// //     double j2_rad = phi - beta;
// //     // 7. 转为度并四舍五入到 int16
// //     j2 = (int16_t) round(j2_rad * 180.0 / M_PI);
// //     j3 = (int16_t) round(j3_rad * 180.0 / M_PI);



//     double r = sqrt(x * x + y * y);
//     // 2. 关节1：tan(j1) = x/y
//     double j1_rad = atan2(x, y);               // 范围 (-π, π]
//     j1 = (int16_t) round(j1_rad * 180.0 / M_PI);
//     // 3. 平面坐标 (p, q) = (水平投影, 垂直高度差)
//     double p = r;                               // p = r
//     double q = z - h;                           // q = z - h
//     // 4. 余弦定理求 j3
//     double cos_j3 = (p * p + q * q - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
//     if (cos_j3 < -1.0 || cos_j3 > 1.0) {
//         return ;                              // 不可达
//     }
//     double j3_rad = -acos(cos_j3);              // 肘下折解，j3 < 0
//     // 5. 求解 j2
//     double phi = atan2(p, q);                   // 末端方向角
//     double beta = atan2(l2 * sin(j3_rad), l1 + l2 * cos(j3_rad));
//     double j2_rad = phi - beta;
//     j2 = (int16_t) round(j2_rad * 180.0 / M_PI);
//     j3 = (int16_t) round(j3_rad * 180.0 / M_PI);



//     pca9685_set_servo(0, 48-j1);
//     printf("Angle: %d\n", j1);
//     sleep(1);
//     system_delay_ms(1000);

//     pca9685_set_servo(3, 132-j2);
//         pca9685_set_servo(3, 0);

//     printf("Angle: %d\n", j2);
//     sleep(1);
//     system_delay_ms(1000);

//     pca9685_set_servo(7, 137-j3);
//     printf("Angle: %d\n", j3);
//     sleep(1);
//     system_delay_ms(1000);


//     return ;


// }

void arm_open(void)
{
    pca9685_set_servo(11, 28);   //假设舵机4控制夹爪
    printf("Gripper opened.\n");
    sleep(1);
    system_delay_ms(1000);
}

void arm_close(void)
{
    pca9685_set_servo(11, 85);  //假设舵机4控制夹爪
    printf("Gripper closed.\n");
    sleep(1);
    system_delay_ms(1000);
}

void arm_stand(void)
{
    pca9685_set_servo(0, 96);     //底盘旋转角度
    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(3, 32);    //第一段机械臂角度
    sleep(1);
    system_delay_ms(1000);

    pca9685_set_servo(7, 42);    //第二段机械臂角度
    sleep(1);
    system_delay_ms(1000);

    return ;
}