#include "odometry.h"
#include "imu.h"      // get_angle 接口
#include "encoder.h"  // 编码器脉冲读取接口
#include <math.h>     // sin, cos, fabs, M_PI
#include <stdbool.h>

// ==================== 硬件参数（根据实际小车修改） ====================
#define ENCODER_PULSES_PER_REV   409.0f    // 编码器每转脉冲数（4倍频后）
#define WHEEL_CIRCUMFERENCE      0.21834f  // 车轮周长（米）
// 说明： wheels_circumference / pulses_per_rev = 每个脉冲行走距离

// ==================== 内部数据结构 ====================
typedef struct {
    float x;               // 当前 X 坐标（米）
    float y;               // 当前 Y 坐标（米）
    float yaw;             // 当前航向角（弧度），以 x 轴正向为 0，逆时针为正
    
    // 保存上一轮编码器值，用于计算增量
    uint32_t last_left;
    uint32_t last_right;
    bool initialized;      // 是否已成功初始化（读取过初值）
} Odometry_t;

static Odometry_t odom;

// ==================== 对外接口实现 ====================

/**
 * @brief 初始化里程计
 */
void odometry_init(float init_x, float init_y) {
    odom.x = init_x;
    odom.y = init_y;
    odom.yaw = 0.0f;
    odom.initialized = false;   // 标记需要首次读取编码器初值

    // 读取一次初始编码器值，防止第一次更新时出现巨大增量
    odom.last_left  = (uint32_t)encoder_left_get_count();
    odom.last_right = (uint32_t)encoder_right_get_count();
}

/**
 * @brief 更新里程计（需周期性调用，例如每 10ms）
 * 
 * 核心步骤：
 * 1. 读取当前 IMU 航向角（度 → 弧度）
 * 2. 读取编码器累计脉冲，计算增量并转换为左右轮行程
 * 3. 计算小车中心的前进弧长
 * 4. 根据航向角分解为 x、y 方向位移并累加
 */
void odometry_update(void) {
    // ---------- 第1步：获取 IMU 航向 ----------
    float pitch, yaw_deg, roll;
    get_angle(&pitch, &yaw_deg, &roll);          // yaw_deg 单位：度
    float yaw_rad = yaw_deg * (M_PI / 180.0f);    // 转换为弧度
    odom.yaw = yaw_rad;

    // ---------- 第2步：获取编码器累计脉冲 ----------
    uint32_t cur_left  = (uint32_t)encoder_left_get_count();
    uint32_t cur_right = (uint32_t)encoder_right_get_count();

    // ---------- 第3步：计算脉冲增量（自动处理 uint32_t 回绕）----------
    uint32_t delta_left  = cur_left  - odom.last_left;
    uint32_t delta_right = cur_right - odom.last_right;

    // 保存本次值，供下一次使用
    odom.last_left  = cur_left;
    odom.last_right = cur_right;

    // ---------- 第4步：将脉冲增量转换为行驶距离（米）----------
    float dist_per_pulse = WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REV;
    float dist_left  = (float)delta_left  * dist_per_pulse;
    float dist_right = (float)delta_right * dist_per_pulse;

    // ---------- 第5步：计算小车中心前进的弧长 ----------
    float distance = (dist_left + dist_right) / 2.0f;

    // ---------- 第6步：根据航向角分解为全局 X、Y 位移 ----------
    // 坐标系设定：
    //   - 当 yaw = 0 时，车头朝向 X 轴正方向
    //   - 左转时 yaw 增大，Y 轴指向小车左侧
    float dx = distance * cosf(yaw_rad);
    float dy = distance * sinf(yaw_rad);

    // ---------- 第7步：累加更新全局坐标 ----------
    odom.x += dx;
    odom.y += dy;
}

/**
 * @brief 获取当前里程计信息
 * @param x   输出：X 坐标（米）
 * @param y   输出：Y 坐标（米）
 * @param yaw 输出：航向角（弧度）
 */
void get_odometry(float *x, float *y, float *yaw) {
    if (x)   *x   = odom.x;
    if (y)   *y   = odom.y;
    if (yaw) *yaw = odom.yaw;
}