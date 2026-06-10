/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
* 2025-03-24        AI             增加卡尔曼滤波姿态解算
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <cmath>
#include <ctime>

// *************************** 例程硬件连接说明 ***************************
// ...（与原来相同，省略）...

// **************************** 网络配置 ****************************
#define SERVER_IP "192.168.208.19"   // 上位机 IP
#define PORT      8086               // 端口号
// **************************************************************

#define M_PI 3.14159265358979323846

// 卡尔曼滤波器类（一维，用于 Roll / Pitch / Yaw 独立滤波）
class KalmanFilter {
public:
    float Q_angle;      // 过程噪声（角度）
    float Q_bias;       // 过程噪声（陀螺仪零偏）
    float R_measure;    // 测量噪声（加速度计/磁力计）
    float angle;        // 当前角度估计值
    float bias;         // 当前陀螺仪零偏估计值
    float P[2][2];      // 误差协方差矩阵

    KalmanFilter(float q_angle = 0.001f, float q_bias = 0.003f, float r_measure = 0.03f) {
        Q_angle = q_angle;
        Q_bias = q_bias;
        R_measure = r_measure;
        angle = 0.0f;
        bias = 0.0f;
        P[0][0] = 0.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 0.0f;
    }

    // 更新滤波器
    // gyro_rate: 陀螺仪角速度（rad/s）
    // acc_angle: 加速度计或磁力计计算的角度（rad）
    // dt: 时间间隔（秒）
    float update(float gyro_rate, float acc_angle, float dt) {
        float gyro_rate_unbias = gyro_rate - bias;
        // 状态预测
        angle += gyro_rate_unbias * dt;
        // 协方差预测
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 卡尔曼增益
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // 测量残差
        float y = acc_angle - angle;

        // 状态更新
        angle += K[0] * y;
        bias += K[1] * y;

        // 协方差更新
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

    void setAngle(float a) { angle = a; }
};

// 全局变量
timer_fd *pit_timer;
char send_buf[256];

// 物理单位转换后的数据（单位：m/s², rad/s, Gauss）
float acc_x_g, acc_y_g, acc_z_g;
float gyro_x_rad, gyro_y_rad, gyro_z_rad;
float mag_x_gauss, mag_y_gauss, mag_z_gauss;

// 姿态角（弧度）
float roll_rad = 0.0f, pitch_rad = 0.0f, yaw_rad = 0.0f;
// 卡尔曼滤波器实例
KalmanFilter kf_roll    (0.05f, 0.003f, 0.1f);
KalmanFilter kf_pitch   (0.05f, 0.003f, 0.1f);
KalmanFilter kf_yaw     (0.01f, 0.0001f, 0.1f);   // Yaw 单独调参

// 时间记录
struct timespec last_time;
bool first_time = true;

// 根据不同 IMU 型号定义物理量程转换系数
// 加速度：原始值 -> g（重力加速度）
// 陀螺仪：原始值 -> rad/s
// 磁力计：原始值 -> Gauss
void get_scale_factors(float &acc_scale, float &gyro_scale, float &mag_scale) {
    switch(imu_type) {
        case DEV_IMU660RA:
            // BMI088 典型值：加速度 ±24g -> 2048 LSB/g? 这里简化：假设原始值为16位有符号，±16g时 2048 LSB/g
            // 实际逐飞库可能已做了转换？为可靠，请根据实际传感器手册调整
            acc_scale = 1.0f / 2048.0f;   // 示例值，实际需要校准
            gyro_scale = (3.1415926f / 180.0f) / 16.4f; // deg/s -> rad/s，BMI088 2000dps -> 16.4 LSB/deg/s
            mag_scale = 1.0f;
            break;
        case DEV_IMU660RB:
            acc_scale = 1.0f / 2048.0f;
            gyro_scale = (3.1415926f / 180.0f) / 16.4f;
            mag_scale = 1.0f;
            break;
        case DEV_IMU963RA:
            // ICM42688P + AK09918 示例值，实际需根据数据手册调整
            acc_scale = 1.0f / 4098.0f;   // ±16g
            gyro_scale = (M_PI / 180.0f) / 14.29f; // 2000dps
            mag_scale = 0.15f;             // AK09918: 1 LSB = 0.15 uT? 实际需确认
            break;
        default:
            acc_scale = 1.0f; gyro_scale = 1.0f; mag_scale = 1.0f;
            break;
    }
}

// 从原始 int 值转换为物理值
void convert_to_physical() {
    float acc_scale, gyro_scale, mag_scale;
    get_scale_factors(acc_scale, gyro_scale, mag_scale);

    // 加速度：原始值 * 比例 -> g，再 * 9.8 得 m/s²（此处保留 g 以便计算角度）
    acc_x_g = (float)imu_acc_x * acc_scale;
    acc_y_g = (float)imu_acc_y * acc_scale;
    acc_z_g = (float)imu_acc_z * acc_scale;

    // 陀螺仪：原始值 * 比例 -> rad/s
    gyro_x_rad = (float)imu_gyro_x * gyro_scale;
    gyro_y_rad = (float)imu_gyro_y * gyro_scale;
    gyro_z_rad = (float)imu_gyro_z * gyro_scale;

    // 磁力计（仅 963RA）
    if(imu_type == DEV_IMU963RA) {
        mag_x_gauss = (float)imu_mag_x * mag_scale;
        mag_y_gauss = (float)imu_mag_y * mag_scale;
        mag_z_gauss = (float)imu_mag_z * mag_scale;
    }
}

// 用加速度计计算 Roll 和 Pitch（弧度）
// 公式：roll = atan2(acc_y, acc_z)
//       pitch = atan2(-acc_x, sqrt(acc_y^2 + acc_z^2))
void compute_acc_angles(float &roll_acc, float &pitch_acc) {
    roll_acc = atan2f(acc_y_g, acc_z_g);
    pitch_acc = atan2f(-acc_x_g, sqrtf(acc_y_g * acc_y_g + acc_z_g * acc_z_g));
}

// 用磁力计计算 Yaw（弧度），需要已知 Roll 和 Pitch 进行倾斜补偿
void compute_mag_yaw(float roll, float pitch, float &yaw_mag) {
    if(imu_type != DEV_IMU963RA) return;
    // 倾斜补偿：将磁力计读数从机体坐标系旋转到水平坐标系
    float bx = mag_x_gauss * cosf(pitch) + mag_y_gauss * sinf(pitch) * sinf(roll) + mag_z_gauss * sinf(pitch) * cosf(roll);
    float by = mag_y_gauss * cosf(roll) - mag_z_gauss * sinf(roll);
    yaw_mag = atan2f(by, bx);
}

// 陀螺仪零偏校准：静止 1 秒，累加求平均
void calibrate_gyro_bias(int samples = 200) {
    printf("Calibrating gyro bias, keep IMU still...\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for(int i = 0; i < samples; i++) {
        imu_gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
        imu_gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
        imu_gyro_z = imu_get_raw(imu_file_path[GYRO_Z_RAW]);
        sum_x += (float)imu_gyro_x;
        sum_y += (float)imu_gyro_y;
        sum_z += (float)imu_gyro_z;
        system_delay_ms(5);
    }
    float gyro_scale;
    float dummy;
    get_scale_factors(dummy, gyro_scale, dummy);
    // 将累计的原始零偏转换为 rad/s 并存储在滤波器的 bias 中（注意卡尔曼滤波器的 bias 也是 rad/s）
    // 此处直接设置卡尔曼滤波器的 bias，因为启动时无运动，认为 bias = 平均角速度
    float bias_x_rad = (sum_x / samples) * gyro_scale;
    float bias_y_rad = (sum_y / samples) * gyro_scale;
    float bias_z_rad = (sum_z / samples) * gyro_scale;
    kf_roll.bias = bias_x_rad;   // roll 对应 gyro_x
    kf_pitch.bias = bias_y_rad;  // pitch 对应 gyro_y
    kf_yaw.bias = bias_z_rad;    // yaw 对应 gyro_z
    printf("Gyro bias (rad/s): x=%.4f, y=%.4f, z=%.4f\n", bias_x_rad, bias_y_rad, bias_z_rad);
}

// 姿态解算主函数（在定时器回调中调用）
void update_attitude(float dt) {
    // 1. 原始值转换为物理单位
    convert_to_physical();

    // 2. 由加速度计计算 Roll, Pitch（含线性加速度噪声，但卡尔曼会处理）
    float roll_acc, pitch_acc;
    compute_acc_angles(roll_acc, pitch_acc);

    // 3. 卡尔曼滤波融合（Roll, Pitch）
    roll_rad = kf_roll.update(gyro_x_rad, roll_acc, dt);
    pitch_rad = kf_pitch.update(gyro_y_rad, pitch_acc, dt);

    // 4. Yaw 处理
    /*if(imu_type == DEV_IMU963RA) {
        // 有磁力计：先获得磁力计航向角（需要倾斜补偿），再与陀螺仪积分融合
        float yaw_mag;
        compute_mag_yaw(roll_rad, pitch_rad, yaw_mag);
        yaw_rad = kf_yaw.update(gyro_z_rad, yaw_mag, dt);
    } else {
        // 无磁力计：只对陀螺仪积分，不修正（没有测量值）
        // 此处仍调用 update，但将 acc_angle 参数设为当前 yaw（相当于无修正，只积分）*/
        yaw_rad = kf_yaw.update(gyro_z_rad, yaw_rad, dt);
    //}

    // 将角度限制到 [-pi, pi]
    if(yaw_rad > M_PI) yaw_rad -= 2.0f * M_PI;
    if(yaw_rad < -M_PI) yaw_rad += 2.0f * M_PI;
}

// 定时器回调：读取 IMU 原始数据并进行姿态解算
void pit_callback()
{
    // 读取原始数据
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

    // 计算时间差 dt
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    float dt = 0.01f; // 默认值
    if(!first_time) {
        dt = (now.tv_sec - last_time.tv_sec) + (now.tv_nsec - last_time.tv_nsec) * 1e-9f;
        if(dt < 0.001f) dt = 0.001f;   // 防止过小
        if(dt > 0.05f) dt = 0.05f;     // 限幅
    }
    last_time = now;
    first_time = false;

    // 姿态解算
    update_attitude(dt);
}

int main(int, char**)
{
    // 1. 识别 IMU 型号
    imu_get_dev_info();
    if(DEV_IMU660RA == imu_type)      printf("IMU DEV IS IMU660RA\r\n");
    else if(DEV_IMU660RB == imu_type) printf("IMU DEV IS IMU660RB\r\n");
    else if(DEV_IMU963RA == imu_type) printf("IMU DEV IS IMU963RA\r\n");
    else { printf("NO FIND IMU DEV\r\n"); return -1; }

    // 2. 初始化 TCP 客户端（必须在上位机打开 TCP 服务器后再运行）
    if(tcp_client_init(SERVER_IP, PORT) == 0)
        printf("tcp_client ok\r\n");
    else {
        printf("tcp_client error\r\n");
        return -1;
    }

    // 3. 注册逐飞助手接口（可选，保留原功能）
    seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);

    // 4. 陀螺仪零偏校准（静止1秒）
    calibrate_gyro_bias(200);

    // 5. 获取初始姿态（静止时用加速度计初始角度）
    // 先读一次数据以更新全局变量
    if(DEV_IMU660RA == imu_type || DEV_IMU660RB == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
    } else if(DEV_IMU963RA == imu_type) {
        imu_acc_x = imu_get_raw(imu_file_path[ACC_X_RAW]);
        imu_acc_y = imu_get_raw(imu_file_path[ACC_Y_RAW]);
        imu_acc_z = imu_get_raw(imu_file_path[ACC_Z_RAW]);
        imu_mag_x = imu_get_raw(imu_file_path[MAG_X_RAW]);
        imu_mag_y = imu_get_raw(imu_file_path[MAG_Y_RAW]);
        imu_mag_z = imu_get_raw(imu_file_path[MAG_Z_RAW]);
    }
    convert_to_physical();
    float init_roll, init_pitch;
    compute_acc_angles(init_roll, init_pitch);
    kf_roll.setAngle(init_roll);
    kf_pitch.setAngle(init_pitch);
    if(imu_type == DEV_IMU963RA) {
        float init_yaw;
        compute_mag_yaw(init_roll, init_pitch, init_yaw);
        kf_yaw.setAngle(/*init_yaw*/0.0f); // 初始航向角不确定，先设为0，后续由磁力计修正
    } else {
        kf_yaw.setAngle(0.0f);
    }

    // 6. 启动定时器（10ms 更新一次 IMU 数据并解算）
    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    // 7. 主循环：每 20ms 发送一次姿态角（度）
    while(1)
    {
        // 将弧度转换为度数
        float roll_deg = roll_rad * 180.0f / M_PI;
        float pitch_deg = pitch_rad * 180.0f / M_PI;
        float yaw_deg = yaw_rad * 180.0f / M_PI;

        snprintf(send_buf, sizeof(send_buf),
               "%.2f,%.2f,%.2f\n",
                (float)pitch_deg, (float)roll_deg, (float)yaw_deg);

        tcp_client_send_data((uint8_t*)send_buf, strlen(send_buf));


        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);
        system_delay_ms(200);   // 发送间隔
    }

    return 0;
}