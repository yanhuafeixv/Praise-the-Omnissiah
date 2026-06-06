// **************************** 网络配置 ****************************
#define SERVER_IP "192.168.23.117"   // 上位机 IP
#define PORT      8060             // 端口号
// **************************************************************

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "zf_common_headfile.h"      // 逐飞库通用头文件

// ---------------------------- 传感器标定 ----------------------------
// 加速度计：±16g → 2048 LSB/g
#define ACC_SCALE       (1.0f / 2048.0f)
// 陀螺仪：±2000dps → 16.4 LSB/(°/s)
#define GYRO_SCALE      (1.0f / 16.4f)
// 磁力计 QMC5883L，假设 ±8G 量程 → 3000 LSB/G
#define MAG_SCALE       (1.0f / 3000.0f)

#define RAD2DEG         (57.2957795f)

// ---------------------------- 互补滤波参数 ----------------------------
// 加速度/磁力计对姿态的修正权重（0~1，越大越相信观测，越小越平滑但响应慢）
#define ACC_WEIGHT      0.02f   // roll/pitch 修正权重
#define MAG_WEIGHT      0.02f   // yaw 修正权重

// ---------------------------- 低通滤波系数 ----------------------------
#define LPF_ALPHA_ACC   0.1f    // 加速度低通
#define LPF_ALPHA_MAG   0.1f    // 磁力计低通

// ---------------------------- 校准采样数 ----------------------------
#define GYRO_BIAS_SAMPLES  200
#define MAG_BIAS_SAMPLES   200

// ---------------------------- 全局变量 ----------------------------
timer_fd *pit_timer;
char send_buf[256];

// 当前姿态角（度）
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// 低通滤波状态
float acc_lpf[3] = {0.0f};
float mag_lpf[3] = {0.0f};

// 零偏
float gyro_bias[3] = {0.0f};
float mag_bias[3]  = {0.0f};

// ---------------------------- 函数声明 ----------------------------
void pit_callback(void);
void ComplementaryFilterEuler(float gx, float gy, float gz,   // rad/s
                              float ax, float ay, float az,   // g
                              float mx, float my, float mz,   // 高斯
                              float dt);
void calibrate_gyro_bias(void);
void calibrate_mag_bias(void);

// ---------------------------- 定时器回调（不变） ----------------------------
void pit_callback()
{
    if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type)
    {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
    }
    else if(DEV_IMU963RA == imu_type)
    {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
        imu_mag_x = imu_get_raw(imu_file_path[MAG_X_RAW]);
        imu_mag_y = imu_get_raw(imu_file_path[MAG_Y_RAW]);
        imu_mag_z = imu_get_raw(imu_file_path[MAG_Z_RAW]);
    }
}

// ---------------------------- 陀螺零偏校准 ----------------------------
void calibrate_gyro_bias(void)
{
    printf("Calibrating gyro bias... keep the IMU static!\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++)
    {
        system_delay_ms(10);
        sum_x += imu_gyro_x;
        sum_y += imu_gyro_y;
        sum_z += imu_gyro_z;
    }
    gyro_bias[0] = sum_x / GYRO_BIAS_SAMPLES;
    gyro_bias[1] = sum_y / GYRO_BIAS_SAMPLES;
    gyro_bias[2] = sum_z / GYRO_BIAS_SAMPLES;
    printf("Gyro bias: %.1f, %.1f, %.1f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

// ---------------------------- 磁力计硬铁零偏校准 ----------------------------
void calibrate_mag_bias(void)
{
    printf("Calibrating magnetometer bias... keep IMU static and away from magnets!\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < MAG_BIAS_SAMPLES; i++)
    {
        system_delay_ms(10);
        sum_x += imu_mag_x;
        sum_y += imu_mag_y;
        sum_z += imu_mag_z;
    }
    mag_bias[0] = sum_x / MAG_BIAS_SAMPLES;
    mag_bias[1] = sum_y / MAG_BIAS_SAMPLES;
    mag_bias[2] = sum_z / MAG_BIAS_SAMPLES;
    printf("Mag bias: %.1f, %.1f, %.1f\n", mag_bias[0], mag_bias[1], mag_bias[2]);
}

// ---------------------------- 欧拉角互补滤波（核心） ----------------------------
void ComplementaryFilterEuler(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz,
                              float dt)
{
    // 1. 加速度计计算 roll 和 pitch（度）
    float acc_roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD2DEG;   // 绕 X 轴
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG; // 绕 Y 轴（注意公式可能因安装方向不同需微调）

    // 2. 磁力计计算 yaw（倾斜补偿后）
    float cp = cosf(pitch * (M_PI / 180.0f));
    float sp = sinf(pitch * (M_PI / 180.0f));
    float cr = cosf(roll  * (M_PI / 180.0f));
    float sr = sinf(roll  * (M_PI / 180.0f));

    // 将磁力计数据旋转到水平面
    float mag_x = mx * cp + my * sp * sr + mz * sp * cr;
    float mag_y = my * cr - mz * sr;
    float yaw_mag = atan2f(-mag_y, mag_x) * RAD2DEG;  // 地磁北

    // 3. 陀螺仪积分（欧拉角直接累加，忽略旋转顺序误差）
    roll  += gx * dt * RAD2DEG;   // gx 绕 X 轴，影响 roll
    pitch += gy * dt * RAD2DEG;   // gy 绕 Y 轴，影响 pitch
    yaw   += gz * dt * RAD2DEG;   // gz 绕 Z 轴，影响 yaw（近似）

    // 4. 一阶互补滤波：用观测值修正积分结果
    roll  = roll  + ACC_WEIGHT * (acc_roll  - roll);
    pitch = pitch + ACC_WEIGHT * (acc_pitch - pitch);
    yaw   = yaw   + MAG_WEIGHT * (yaw_mag   - yaw);

    // 限制 yaw 在 0~360 度（可选）
    if (yaw < 0.0f)   yaw += 360.0f;
    if (yaw > 360.0f) yaw -= 360.0f;
}

// ---------------------------- 主函数 ----------------------------
int main(int, char**)
{
    // 1. 识别 IMU 型号
    imu_get_dev_info();
    if(DEV_IMU660RA == imu_type)      printf("IMU DEV IS IMU660RA\r\n");
    else if(DEV_IMU660RB == imu_type) printf("IMU DEV IS IMU660RB\r\n");
    else if(DEV_IMU963RA == imu_type) printf("IMU DEV IS IMU963RA\r\n");
    else { printf("NO FIND IMU DEV\r\n"); return -1; }

    // 2. 初始化 TCP
    if(tcp_client_init(SERVER_IP, PORT) == 0)
        printf("tcp_client ok\r\n");
    else {
        printf("tcp_client error\r\n");
        return -1;
    }

    // 3. 注册逐飞助手
    seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);

    // 4. 启动 10ms 定时器更新传感器数据
    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    // 5. 传感器零偏校准（保持静止）
    calibrate_gyro_bias();
    calibrate_mag_bias();

    // 6. 时间计量初始化
    struct timeval tv_prev, tv_now;
    gettimeofday(&tv_prev, NULL);
    printf("Start Euler complementary filter...\n");

    // 7. 主循环
    while(1)
    {
        // 计算真实时间间隔 dt
        gettimeofday(&tv_now, NULL);
        float dt = (tv_now.tv_sec - tv_prev.tv_sec) + 
                   (tv_now.tv_usec - tv_prev.tv_usec) * 1e-6f;
        tv_prev = tv_now;
        if (dt > 0.05f) dt = 0.02f;   // 防止异常大间隔
        if (dt <= 0.0f) dt = 0.02f;

        // 转换为物理单位，并减去零偏
        float gx = (imu_gyro_x - gyro_bias[0]) * GYRO_SCALE * (M_PI / 180.0f);  // rad/s
        float gy = (imu_gyro_y - gyro_bias[1]) * GYRO_SCALE * (M_PI / 180.0f);
        float gz = (imu_gyro_z - gyro_bias[2]) * GYRO_SCALE * (M_PI / 180.0f);

        float ax = imu_acc_x * ACC_SCALE;
        float ay = imu_acc_y * ACC_SCALE;
        float az = imu_acc_z * ACC_SCALE;

        float mx = (imu_mag_x - mag_bias[0]) * MAG_SCALE;
        float my = (imu_mag_y - mag_bias[1]) * MAG_SCALE;
        float mz = (imu_mag_z - mag_bias[2]) * MAG_SCALE;

        // 低通滤波
        acc_lpf[0] += LPF_ALPHA_ACC * (ax - acc_lpf[0]);
        acc_lpf[1] += LPF_ALPHA_ACC * (ay - acc_lpf[1]);
        acc_lpf[2] += LPF_ALPHA_ACC * (az - acc_lpf[2]);

        mag_lpf[0] += LPF_ALPHA_MAG * (mx - mag_lpf[0]);
        mag_lpf[1] += LPF_ALPHA_MAG * (my - mag_lpf[1]);
        mag_lpf[2] += LPF_ALPHA_MAG * (mz - mag_lpf[2]);

        // 欧拉角互补滤波
        ComplementaryFilterEuler(gx, gy, gz,
                                 acc_lpf[0], acc_lpf[1], acc_lpf[2],
                                 mag_lpf[0], mag_lpf[1], mag_lpf[2], dt);

        // 发送数据（12 通道）
        snprintf(send_buf, sizeof(send_buf),
                 "imu:%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f\n",
                 imu_acc_x, imu_acc_y, imu_acc_z,
                 imu_gyro_x, imu_gyro_y, imu_gyro_z,
                 imu_mag_x, imu_mag_y, imu_mag_z,
                 roll, pitch, yaw);

        tcp_client_send_data((uint8_t*)send_buf, strlen(send_buf));

        system_delay_ms(18);   // 维持约 50Hz
    }

    return 0;
}