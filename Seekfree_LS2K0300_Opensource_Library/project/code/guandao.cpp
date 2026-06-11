#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "zf_common_headfile.h"
// ==================== 用户需根据实际硬件修改的常量 ====================
#define ENCODER_PULSES_PER_REV   409.0f   // 编码器每转脉冲数（4倍频后）
#define WHEEL_CIRCUMFERENCE      0.21834f    // 车轮周长，单位：米
// 说明：以上两个参数用于将脉冲数转换为实际行驶距离

// ==================== 里程计数据结构 ====================
typedef struct {
    float x;             // 当前位置 X 坐标（米）
    float y;             // 当前位置 Y 坐标（米）
    float yaw;           // 当前航向角（弧度），范围一般 [-π, π]
    
    // 内部变量，用于存储上一周期编码器读数
    uint32_t last_left;  
    uint32_t last_right;
    bool init_flag;      // 是否已初始化
} Odometry_t;

// 全局里程计实例
static Odometry_t odom;

// ==================== 外部函数声明 ====================
// 这两个函数由底层传感器驱动提供
extern void get_angle(float *pitch, float *yaw, float *roll);


 void get_encoder(uint32_t *left_num, uint32_t *right_num){

    *left_num=encoder_left_get_count();
    *right_num=encoder_right_get_count();
}

// ==================== 函数实现 ====================

/**
 * @brief 初始化里程计
 * @param init_x 初始 X 坐标（通常为 0）
 * @param init_y 初始 Y 坐标（通常为 0）
 */
void odometry_init(float init_x, float init_y)
{
    odom.x = init_x;
    odom.y = init_y;
    odom.yaw = 0.0f;
    odom.init_flag = false;  // 标记尚未读取过编码器初始值
    
    // 读取一次初始编码器值（防止第一次更新时出现巨大增量）
    uint32_t dummy_left, dummy_right;
    get_encoder(&dummy_left, &dummy_right);
    odom.last_left = dummy_left;
    odom.last_right = dummy_right;
}

/**
 * @brief 更新里程计（需周期性调用，例如每 10ms）
 * 
 * 核心步骤：
 * 1. 读取当前陀螺仪航向和编码器计数值
 * 2. 计算编码器增量并转换为行驶距离
 * 3. 结合航向角更新全局坐标
 */
void odometry_update(void)
{
    // ---------- 第1步：采集传感器数据 ----------
    float pitch, yaw_deg, roll;    // get_angle 返回的可能是度，我们统一转为弧度
    uint32_t cur_left, cur_right;
    
    get_angle(&pitch, &yaw_deg, &roll);   // 注意：yaw_deg 假设为度
    get_encoder(&cur_left, &cur_right);
    
    // 将角度从度转为弧度
    float yaw_rad = yaw_deg * (M_PI / 180.0f);
    odom.yaw = yaw_rad;   // 更新当前航向
    
    // ---------- 第2步：处理编码器溢出并计算脉冲增量 ----------
    uint32_t delta_left, delta_right;
    
    // 因为编码器计数是 uint32_t，相减会自动处理溢出（如 0xFFFFFFFF -> 0x00000000）
    // 但需保证两次采样间隔内增量不会超过 2^32 / 2，对小车来说不可能超过
    delta_left  = cur_left  - odom.last_left;
    delta_right = cur_right - odom.last_right;
    
    // 保存本次读数以备下次使用
    odom.last_left  = cur_left;
    odom.last_right = cur_right;
    
    // ---------- 第3步：将脉冲增量转换为轮子移动距离（米） ----------
    float dist_per_pulse = WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REV;
    float dist_left  = (float)delta_left  * dist_per_pulse;
    float dist_right = (float)delta_right * dist_per_pulse;
    
    // ---------- 第4步：计算小车中心前进的弧长 ----------
    float distance = (dist_left + dist_right) / 2.0f;
    
    // ---------- 第5步：根据航向角分解为 X、Y 方向位移 ----------
    // 坐标系设定：
    //   X 轴指向小车初始前进方向（yaw = 0 时车头朝向 X 轴正半轴）
    //   Y 轴指向小车左侧（左转时 yaw 增大，Y 坐标增大）
    float dx = distance * cosf(yaw_rad);
    float dy = distance * sinf(yaw_rad);
    
    // ---------- 第6步：累加更新全局坐标 ----------
    odom.x += dx;
    odom.y += dy;
}

/**
 * @brief 获取当前里程计信息
 * @param x     输出参数：X 坐标（米）
 * @param y     输出参数：Y 坐标（米）
 * @param yaw   输出参数：航向角（弧度）
 */
void get_odometry(float *x, float *y, float *yaw)
{
    *x   = odom.x;
    *y   = odom.y;
    *yaw = odom.yaw;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// PID结构体
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_limit;   // 积分限幅
    float output_limit;     // 输出限幅
} PID_t;

void pid_init(PID_t *pid, float Kp, float Ki, float Kd, float i_limit, float out_limit) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->integral = 0; pid->prev_error = 0;
    pid->integral_limit = i_limit;
    pid->output_limit = out_limit;
}

float pid_compute(PID_t *pid, float error, float dt) {
    pid->integral += error * dt;
    // 积分限幅
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 输出限幅
    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}

// 目标坐标
static float target_x = 0, target_y = 0;

// 状态定义
typedef enum {
    STATE_IDLE,
    STATE_MOVE_X,       // 直行调整 X
    STATE_TURN_Y,       // 原地旋转 90°
    STATE_MOVE_Y,       // 直行调整 Y
    STATE_DONE
} GotoState_t;

static GotoState_t goto_state = STATE_IDLE;

// 目标航向（用于直行保持）
static float target_yaw = 0;

// PID 控制器实例
static PID_t pid_angle, pid_distance;

// 参数配置
#define ANGLE_TOLERANCE    0.02f   // 角度到达容忍度 (rad) ≈ 1.15°
#define DISTANCE_TOLERANCE 0.02f   // 距离到达容忍度 (m) ≈ 2cm
#define WHEEL_TRACK        0.2f    // 两轮间距 (m)，需实际测量
#define MAX_ANGULAR_SPEED  2.0f    // 最大旋转角速度
#define MAX_LINEAR_SPEED   0.4f    // 最大线速度
#define CONTROL_DT          0.01f  // 控制周期 (s)，与 odometry_update 一致


void car_goto(int x, int y) {
    target_x = (float)x;
    target_y = (float)y;
    goto_state = STATE_MOVE_X;
    
    // 记录当前航向，作为直行 X 时的目标航向
    float p, r;
    get_angle(&p, &target_yaw, &r);   // 度
    target_yaw *= M_PI / 180.0f;      // 转为弧度
    
    // 重置 PID 积分
    pid_angle.integral = 0;
    pid_angle.prev_error = 0;
    pid_distance.integral = 0;
    pid_distance.prev_error = 0;
    
    printf("goto (%d, %d)\n", x, y);
}