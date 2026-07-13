#include "quaternion.h"
#include "math.h"
#include "SEEKFREE_IMU963RA.h"

/*============================================================================
 * 模块说明：四元数姿态解算模块
 * 功能：使用 Mahony 互补滤波算法进行姿态解算
 * 特点：通过宏定义自动计算灵敏度，节省内存空间
 *============================================================================*/

/*---------------------------------------------------------------------------
 * IMU 灵敏度计算宏（根据 IMU963RA_GYR_SAMPLE 和 IMU963RA_ACC_SAMPLE 配置）
 *---------------------------------------------------------------------------*/
// 陀螺仪灵敏度计算（dps/LSB）
#if (IMU963RA_GYR_SAMPLE == 0x52)
    #define GYRO_SENSITIVITY 0.004375f  // ±125dps
#elif (IMU963RA_GYR_SAMPLE == 0x50)
    #define GYRO_SENSITIVITY 0.00875f   // ±250dps
#elif (IMU963RA_GYR_SAMPLE == 0x54)
    #define GYRO_SENSITIVITY 0.0175f    // ±500dps
#elif (IMU963RA_GYR_SAMPLE == 0x58)
    #define GYRO_SENSITIVITY 0.035f     // ±1000dps
#elif (IMU963RA_GYR_SAMPLE == 0x5C)
    #define GYRO_SENSITIVITY 0.07f      // ±2000dps
#elif (IMU963RA_GYR_SAMPLE == 0x51)
    #define GYRO_SENSITIVITY 0.14f      // ±4000dps
#else
    #define GYRO_SENSITIVITY 0.07f      // 默认±2000dps
#endif

// 加速度计灵敏度计算（g/LSB）
#if (IMU963RA_ACC_SAMPLE == 0x30)
    #define ACC_SENSITIVITY (1.0f/16393.0f)  // ±2g
#elif (IMU963RA_ACC_SAMPLE == 0x38)
    #define ACC_SENSITIVITY (1.0f/8197.0f)   // ±4g
#elif (IMU963RA_ACC_SAMPLE == 0x3C)
    #define ACC_SENSITIVITY (1.0f/4098.0f)   // ±8g
#elif (IMU963RA_ACC_SAMPLE == 0x34)
    #define ACC_SENSITIVITY (1.0f/2049.0f)   // ±16g
#else
    #define ACC_SENSITIVITY (1.0f/4098.0f)   // 默认±8g
#endif

/*---------------------------------------------------------------------------
 * 全局变量
 *---------------------------------------------------------------------------*/
Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数初始化
EulerAngles euler = {0.0f, 0.0f, 0.0f};   // 欧拉角

float gyro_offset_x = 0.0f;  // 陀螺仪零偏
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
static float yaw_gyro_integral = 0.0f;

/*---------------------------------------------------------------------------
 * 快速平方根倒数算法
 *---------------------------------------------------------------------------*/
static float InvSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*---------------------------------------------------------------------------
 * 四元数初始化
 *---------------------------------------------------------------------------*/
void Quaternion_Init(void) {
    q.q0 = 1.0f;
    q.q1 = 0.0f;
    q.q2 = 0.0f;
    q.q3 = 0.0f;
    
    euler.roll = 0.0f;
    euler.pitch = 0.0f;
    euler.yaw = 0.0f;
    yaw_gyro_integral = 0.0f;
}

/*---------------------------------------------------------------------------
 * 陀螺仪零偏校准
 * 参数：samples - 采样次数
 *---------------------------------------------------------------------------*/
void Gyro_Calibration(uint16 samples) {
    uint16 i;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (i = 0; i < samples; i++) {
        imu963ra_get_gyro();
        sum_x += imu963ra_gyro_x;
        sum_y += imu963ra_gyro_y;
        sum_z += imu963ra_gyro_z;
        system_delay_ms(5);
    }
    
    // 计算平均值作为零偏
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
}

/*---------------------------------------------------------------------------
 * 四元数姿态更新（Mahony 互补滤波算法）
 * 参数：
 *   gx, gy, gz - 陀螺仪原始数据（度/秒）
 *   ax, ay, az - 加速度计原始数据（g）
 *   dt - 采样时间间隔（秒）
 *---------------------------------------------------------------------------*/
void Quaternion_Update(float gx, float gy, float gz, 
                       float ax, float ay, float az, 
                       float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    
    // 陀螺仪角速度转换为弧度
    gx = gx * 0.01745329f;  // degree to radian
    gy = gy * 0.01745329f;
    gz = gz * 0.01745329f;
    
    // Accelerometer correction is valid only when the vector is non-zero.
    recipNorm = ax * ax + ay * ay + az * az;
    if (recipNorm > 0.000001f)
    {
        recipNorm = InvSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
    }
    else
    {
        ax = 0.0f;
        ay = 0.0f;
        az = 1.0f;
    }
    
    // 辅助变量
    _2q0 = 2.0f * q.q0;
    _2q1 = 2.0f * q.q1;
    _2q2 = 2.0f * q.q2;
    _2q3 = 2.0f * q.q3;
    _4q0 = 4.0f * q.q0;
    _4q1 = 4.0f * q.q1;
    _4q2 = 4.0f * q.q2;
    _8q1 = 8.0f * q.q1;
    _8q2 = 8.0f * q.q2;
    q0q0 = q.q0 * q.q0;
    q1q1 = q.q1 * q.q1;
    q2q2 = q.q2 * q.q2;
    q3q3 = q.q3 * q.q3;
    
    // 梯度下降算法修正
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;
    
    recipNorm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (recipNorm > 0.000001f)
    {
        recipNorm = InvSqrt(recipNorm);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }
    else
    {
        s0 = 0.0f;
        s1 = 0.0f;
        s2 = 0.0f;
        s3 = 0.0f;
    }
    
    // 应用修正
    qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz) - Q_BETA * s0;
    qDot2 = 0.5f * (q.q0 * gx + q.q2 * gz - q.q3 * gy) - Q_BETA * s1;
    qDot3 = 0.5f * (q.q0 * gy - q.q1 * gz + q.q3 * gx) - Q_BETA * s2;
    qDot4 = 0.5f * (q.q0 * gz + q.q1 * gy - q.q2 * gx) - Q_BETA * s3;
    
    // 四元数积分
    q.q0 += qDot1 * dt;
    q.q1 += qDot2 * dt;
    q.q2 += qDot3 * dt;
    q.q3 += qDot4 * dt;
    
    // Normalize only when the quaternion is valid.
    recipNorm = q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3;
    if (recipNorm > 0.000001f)
    {
        recipNorm = InvSqrt(recipNorm);
        q.q0 *= recipNorm;
        q.q1 *= recipNorm;
        q.q2 *= recipNorm;
        q.q3 *= recipNorm;
    }
    else
    {
        q.q0 = 1.0f;
        q.q1 = 0.0f;
        q.q2 = 0.0f;
        q.q3 = 0.0f;
    }
}

/*---------------------------------------------------------------------------
 * 四元数转欧拉角
 *---------------------------------------------------------------------------*/
void Quaternion_ToEuler(void) {
    float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);

    float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);

    float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
    float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);

    // 横滚角（绕 X 轴）
    euler.roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // 俯仰角（绕 Y 轴）
    if (fabs(sinp) >= 1.0f)
        euler.pitch = (sinp >= 0) ? 90.0f : -90.0f;  // 万向锁情况，根据符号返回 ±90 度
    else
        euler.pitch = asin(sinp) * RAD_TO_DEG;
    
    // 航向角（绕 Z 轴）
    euler.yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/*---------------------------------------------------------------------------
 * IMU数据更新（从IMU963读取数据并更新姿态）
 * 应在定时中断中以固定周期调用，推荐周期为 5 ms
 *---------------------------------------------------------------------------*/
void IMU_Update_Dt(float dt) {
    float gx, gy, gz;
    float ax, ay, az;
    
    // 读取IMU数据
    imu963ra_get_acc();
    imu963ra_get_gyro();
    
    // 陀螺仪数据转换（度/秒），并去除零偏
    // 使用宏定义自动计算的灵敏度系数（根据 IMU963RA_GYR_SAMPLE 配置）
    gx = (float)(imu963ra_gyro_x - gyro_offset_x) * GYRO_SENSITIVITY;
    gy = (float)(imu963ra_gyro_y - gyro_offset_y) * GYRO_SENSITIVITY;
    gz = (float)(imu963ra_gyro_z - gyro_offset_z) * GYRO_SENSITIVITY;
    
    // 加速度计数据转换（g）
    // 使用宏定义自动计算的灵敏度系数（根据 IMU963RA_ACC_SAMPLE 配置）
    ax = (float)imu963ra_acc_x * ACC_SENSITIVITY;
    ay = (float)imu963ra_acc_y * ACC_SENSITIVITY;
    az = (float)imu963ra_acc_z * ACC_SENSITIVITY;
    
    // 死区处理
    if (gx > -0.2f && gx < 0.2f) gx = 0.0f;
    if (gy > -0.2f && gy < 0.2f) gy = 0.0f;
    if (gz > -0.2f && gz < 0.2f) gz = 0.0f;
    
    yaw_gyro_integral += gz * dt;
    if (yaw_gyro_integral > 180.0f) yaw_gyro_integral -= 360.0f;
    if (yaw_gyro_integral < -180.0f) yaw_gyro_integral += 360.0f;

    // Update roll and pitch with the quaternion path.
    Quaternion_Update(gx, gy, gz, ax, ay, az, dt);
    Quaternion_ToEuler();

    // A six-axis IMU cannot correct yaw from acceleration; use gyro integration.
    euler.yaw = yaw_gyro_integral;
}


void IMU_Update(void) {
    IMU_Update_Dt(0.005f);
}
/*---------------------------------------------------------------------------
 * 航向角复位
 *---------------------------------------------------------------------------*/
void Yaw_Reset(void) {
    // 保持当前roll和pitch，只复位yaw
    // 这需要从当前 roll 和 pitch 重新构建四元数
    float roll_rad = euler.roll * 0.01745329f;
    float pitch_rad = euler.pitch * 0.01745329f;
    
    float cr = cos(roll_rad * 0.5f);
    float sr = sin(roll_rad * 0.5f);
    float cp = cos(pitch_rad * 0.5f);
    float sp = sin(pitch_rad * 0.5f);

    euler.yaw = 0.0f;
    yaw_gyro_integral = 0.0f;
    
    q.q0 = cr * cp;
    q.q1 = sr * cp;
    q.q2 = cr * sp;
    q.q3 = -sr * sp;
}
