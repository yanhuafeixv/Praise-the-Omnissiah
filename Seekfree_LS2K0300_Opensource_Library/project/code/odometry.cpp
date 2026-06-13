#include "odometry.h"
#include <math.h>
#include <stdio.h>
#include "encoder.h"
#include "IMU_Analysis.h"
// ==================== 硬件参数（根据实际小车修改） ====================
#define WHEEL_CIRCUMFERENCE       0.21834f    // 车轮周长（米）
#define ENCODER_PULSES_PER_REV    409.0f   // 编码器每转脉冲数（4 倍频后）
// ===================================================================


// 内部里程计数据
static float pos_x = 0.0f;
static float pos_y = 0.0f;
static float yaw_deg = 0.0f;      // 当前航向（度）

static int last_left_count = 0;   // 上一次左编码器值
static int last_right_count = 0;  // 上一次右编码器值
static bool first_read = true;    // 是否首次读取

void odometry_init() {
    pos_x = 0.0f;
    pos_y = 0.0f;
    yaw_deg = 0.0f;
    first_read = true;
    
    // 读取一次初始编码器值，防止第一次更新时出现巨大增量
    last_left_count = encoder_left_get_count();
    last_right_count = encoder_right_get_count();
}

void odometry_update() {
    // ---------- 1. 读取航向（度） ----------
    float roll, yaw, pitch;
    imu_get_angle(&roll, &yaw, &pitch);
    yaw_deg = yaw;   // 直接使用 IMU 给出的航向角（度）

    // ---------- 2. 读取编码器累计脉冲 ----------
    int cur_left  = encoder_left_get_count();
    int cur_right = encoder_right_get_count();

    // ---------- 3. 计算脉冲增量 ----------
    int delta_left  = cur_left - last_left_count;
    int delta_right = cur_right - last_right_count;
    last_left_count  = cur_left;
    last_right_count = cur_right;

    // 如果是第一次，增量清零（避免异常）
    if (first_read) {
        first_read = false;
        return;
    }

    // ---------- 4. 脉冲转换为行驶距离（米） ----------
    float dist_per_pulse = WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REV;
    float dist_left  = (float)delta_left  * dist_per_pulse;
    float dist_right = (float)delta_right * dist_per_pulse;

    // 小车中心前进距离（两轮平均）
    float distance = (dist_left + dist_right) / 2.0f;

    // ---------- 5. 根据航向角分解为全局坐标增量 ----------
    // 注意：yaw_deg = 0 时车头朝向 X 轴正方向，左转为正
    float yaw_rad = yaw_deg * (M_PI / 180.0f);
    float dx = distance * cosf(yaw_rad);
    float dy = distance * sinf(yaw_rad);

    pos_x += dx;
    pos_y += dy;
}

void odometry_get_position(float *x, float *y, float *yaw_out) {
    if (x)   *x   = pos_x;
    if (y)   *y   = pos_y;
    if (yaw_out) *yaw_out = yaw_deg;
}

void odometry_show_position() {
    printf("Position: X=%.2f m, Y=%.2f m, Yaw=%.1f deg\n", pos_x, pos_y, yaw_deg);
}