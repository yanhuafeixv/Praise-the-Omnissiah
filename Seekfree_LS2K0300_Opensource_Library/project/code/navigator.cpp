#include "navigator.h"
#include "odometry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>   // abs
#include "motor_control.h"
#include "zf_common_headfile.h"


// ==================== 控制参数（根据实际小车调试） ====================
#define WHEEL_TRACK          0.16f     // 两轮间距（米）
#define ANGLE_TOLERANCE      3.0f      // 角度到位容差（度）
#define DISTANCE_TOLERANCE   0.02f     // 距离到位容差（米）

#define MAX_DUTY             30         // 最大占空比（防止飞车）
#define SPEED_GAIN           300.0f     // 距离控制比例增益（占空比/米误差）
#define TURN_GAIN            1.3f     // 角度控制比例增益（占空比/度误差）
#define TURN_GAIN_zhuan       0.3f     // 角度控制比例增益（占空比/度误差）

// ===================================================================

// 导航状态枚举
typedef enum {
    STATE_IDLE,
    STATE_ROTATE_TO_X,   // 旋转到 X 方向（0° 或 180°）
    STATE_MOVE_X,        // 沿 X 方向直行
    STATE_ROTATE_TO_Y,   // 旋转到 Y 方向（90° 或 -90°）
    STATE_MOVE_Y,        // 沿 Y 方向直行
    STATE_DONE
} NavState;

static NavState state = STATE_IDLE;
static float target_x = 0.0f, target_y = 0.0f;

// ---------- 内部辅助函数 ----------

/**
 * @brief 将角度正规化到 [-180, 180] 范围
 */
static float normalize_angle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 简单的比例控制器，左右轮限幅后输出给电机
 */
static void set_motor_from_controls(float base_duty, float turn_duty) { 
    int left  = (int)(base_duty + turn_duty);
    int right = (int)(base_duty - turn_duty);
    // 限幅
    if (left > MAX_DUTY)   left = MAX_DUTY;
    if (left < -MAX_DUTY)  left = -MAX_DUTY;
    if (right > MAX_DUTY)  right = MAX_DUTY;
    if (right < -MAX_DUTY) right = -MAX_DUTY;
    set_motor_duty(left, right);
}

// =================== 对外接口 ===================

void car_goto(float x, float y) {
    target_x = x;
    target_y = y;
    state = STATE_ROTATE_TO_X;   // 启动导航：先旋转到 X 方向
    printf("Nav started: target (%.2f, %.2f)\n", target_x, target_y);
}

void navigator_update() {
    if (state == STATE_IDLE || state == STATE_DONE) return;

    float x, y, yaw;
    odometry_get_position(&x, &y, &yaw);

    float angle_error = 0.0f;
    float dist_error  = 0.0f;
    float base_duty   = 0.0f;  // 线速度占空比
    float turn_duty   = 0.0f;  // 旋转差速占空比

    switch (state) {
    case STATE_ROTATE_TO_X:
        // 决定应该朝向 +X (0°) 还是 -X (180°)
        {
            float target_angle = (target_x >= x) ? 0.0f : 180.0f;
            angle_error = normalize_angle(target_angle - yaw);
            if (fabs(angle_error) < ANGLE_TOLERANCE) {
                state = STATE_MOVE_X;
                break;
            }
            // 比例控制旋转
            turn_duty = TURN_GAIN * angle_error;
            if (turn_duty > MAX_DUTY)  turn_duty = MAX_DUTY;
            if (turn_duty < -MAX_DUTY) turn_duty = -MAX_DUTY;
            set_motor_from_controls(0, turn_duty);   // 原地旋转
        }
        break;

    case STATE_MOVE_X:
        dist_error = target_x - x;   // X 方向剩余距离
        // 保持目标航向不变（0 或 180，与上一步一致）
        {
            float target_angle = (target_x >= x) ? 0.0f : 180.0f;
            angle_error = normalize_angle(target_angle - yaw);
        }
        if (fabs(dist_error) < DISTANCE_TOLERANCE) {
            state = STATE_ROTATE_TO_Y;
            sleep(1);  // 停车后稍作延时
            break;
        }
        // 距离控制：线速度比例控制
        base_duty = SPEED_GAIN * dist_error;

        if(dist_error>0.2f){
        if (base_duty > MAX_DUTY)  base_duty = MAX_DUTY;
        if (base_duty < -MAX_DUTY) base_duty = -MAX_DUTY;
        }else{
            if (base_duty > MAX_DUTY/2)  base_duty = MAX_DUTY/2;
            if (base_duty < -MAX_DUTY/2) base_duty = -MAX_DUTY/2;
        }
        // 角度修正
        turn_duty = TURN_GAIN * angle_error;
        set_motor_from_controls(base_duty, turn_duty);
        break;

    case STATE_ROTATE_TO_Y:
        // 决定应该朝向 +Y (90°) 还是 -Y (-90°)
        {
            float target_angle = (target_y >= y) ? 90.0f : -90.0f;
            angle_error = normalize_angle(target_angle - yaw);
            if (fabs(angle_error) < ANGLE_TOLERANCE) {
                state = STATE_MOVE_Y;
                break;
            }
            turn_duty = TURN_GAIN_zhuan * angle_error;

            if(angle_error>30.0f){
            
                if (turn_duty > MAX_DUTY)  turn_duty = MAX_DUTY;
                if (turn_duty < -MAX_DUTY) turn_duty = -MAX_DUTY;
            }else{
                if (turn_duty > 9)  turn_duty = 9;
                if (turn_duty < -9) turn_duty = -9;
            }
            if(turn_duty>-9 && turn_duty<9){
                if(turn_duty>0){
                    turn_duty = 9;
                }else{
                    turn_duty = -9;
                }
            }
            set_motor_from_controls(0, turn_duty);
        }
        break;

    case STATE_MOVE_Y:
        dist_error = target_y - y;
        {
            float target_angle = (target_y >= y) ? 90.0f : -90.0f;
            angle_error = normalize_angle(target_angle - yaw);
        }
        if (fabs(dist_error) < DISTANCE_TOLERANCE) {
            state = STATE_DONE;
            set_motor_duty(0, 0);   // 停车
            printf("Arrived at (%.2f, %.2f)!\n", target_x, target_y);
            break;
        }
        base_duty = SPEED_GAIN * dist_error;
        if (base_duty > MAX_DUTY)  base_duty = MAX_DUTY;
        if (base_duty < -MAX_DUTY) base_duty = -MAX_DUTY;
        turn_duty = TURN_GAIN * angle_error;
        set_motor_from_controls(base_duty, turn_duty);
        break;

    default:
        break;
    }
}

bool navigator_is_done() {
    return (state == STATE_DONE);
}

void car_show_position(float x, float y) {
    odometry_show_position();   // 直接调用里程计打印
}