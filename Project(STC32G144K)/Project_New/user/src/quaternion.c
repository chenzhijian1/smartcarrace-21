#include "quaternion.h"
#include "math.h"
#include "zf_device_imu660rc.h"

/*============================================================================
 * 模块说明：四元数姿态解算模�?
 * 功能：使�?Mahony 互补滤波算法进行姿态解�?
 * 特点：通过宏定义自动计算灵敏度，节省内存空�?
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 全局变量
 *---------------------------------------------------------------------------*/
Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数初始化
EulerAngles euler = {0.0f, 0.0f, 0.0f};   // 欧拉�?

float gyro_offset_x = 0.0f;  // 陀螺仪零偏
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
static float yaw_gyro_integral = 0.0f;
static float mahony_integral_x = 0.0f;
static float mahony_integral_y = 0.0f;
static float mahony_integral_z = 0.0f;

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

static float Attitude_Atan2(float y, float x) {
    if (x > 0.0f)
    {
        return atan(y / x);
    }
    if (x < 0.0f)
    {
        if (y >= 0.0f)
        {
            return atan(y / x) + 3.1415926536f;
        }
        return atan(y / x) - 3.1415926536f;
    }
    if (y > 0.0f)
    {
        return 1.5707963268f;
    }
    if (y < 0.0f)
    {
        return -1.5707963268f;
    }
    return 0.0f;
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
    mahony_integral_x = 0.0f;
    mahony_integral_y = 0.0f;
    mahony_integral_z = 0.0f;
}

/*---------------------------------------------------------------------------
 * 陀螺仪零偏校准
 * 参数：samples - 采样次数
 *---------------------------------------------------------------------------*/
void Gyro_Calibration(uint16 samples) {
    uint16 i;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (i = 0; i < samples; i++) {
        imu660rc_get_gyro();
        sum_x += imu660rc_gyro_x;
        sum_y += imu660rc_gyro_y;
        sum_z += imu660rc_gyro_z;
        system_delay_ms(5);
    }
    
    // 计算平均值作为零�?
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
}

/*---------------------------------------------------------------------------
 * 四元数姿态更新（Mahony 互补滤波算法�?
 * 参数�?
 *   gx, gy, gz - 陀螺仪原始数据（度/秒）
 *   ax, ay, az - 加速度计原始数据（g�?
 *   dt - 采样时间间隔（秒�?
 *---------------------------------------------------------------------------*/
void Quaternion_Update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt) {
    float recipNorm;
    float accNormSq;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float q0, q1, q2, q3;
    float qa, qb, qc;

    if (dt <= 0.0f)
    {
        return;
    }

    gx *= 0.0174532925f;
    gy *= 0.0174532925f;
    gz *= 0.0174532925f;

    q0 = q.q0;
    q1 = q.q1;
    q2 = q.q2;
    q3 = q.q3;

    // Use gravity correction only when acceleration is close to 1 g.
    accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq > 0.5625f && accNormSq < 1.5625f)
    {
        recipNorm = InvSqrt(accNormSq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        if (MAHONY_KI > 0.0f)
        {
            mahony_integral_x += 2.0f * MAHONY_KI * halfex * dt;
            mahony_integral_y += 2.0f * MAHONY_KI * halfey * dt;
            mahony_integral_z += 2.0f * MAHONY_KI * halfez * dt;
            gx += mahony_integral_x;
            gy += mahony_integral_y;
            gz += mahony_integral_z;
        }

        gx += 2.0f * MAHONY_KP * halfex;
        gy += 2.0f * MAHONY_KP * halfey;
        gz += 2.0f * MAHONY_KP * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    recipNorm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (recipNorm > 0.000001f)
    {
        recipNorm = InvSqrt(recipNorm);
        q.q0 = q0 * recipNorm;
        q.q1 = q1 * recipNorm;
        q.q2 = q2 * recipNorm;
        q.q3 = q3 * recipNorm;
    }
    else
    {
        q.q0 = 1.0f;
        q.q1 = 0.0f;
        q.q2 = 0.0f;
        q.q3 = 0.0f;
    }
}

/* Standard Mahony six-axis update. */
void Quaternion_ToEuler(void) {
    float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
    float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);

    float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);

    float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
    float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);

    // 横滚角（�?X 轴）
    euler.roll = Attitude_Atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // 俯仰角（�?Y 轴）
    if (fabs(sinp) >= 1.0f)
        euler.pitch = (sinp >= 0) ? 90.0f : -90.0f;  // 万向锁情况，根据符号返回 ±90 �?
    else
        euler.pitch = asin(sinp) * RAD_TO_DEG;
    
    // 航向角（�?Z 轴）
    euler.yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/*---------------------------------------------------------------------------
 * IMU数据更新（从IMU963读取数据并更新姿态）
 * 应在定时中断中以固定周期调用，推荐周期为 5 ms
 *---------------------------------------------------------------------------*/
void IMU_Update_Dt(float dt) {
    float gx, gy, gz;
    float ax, ay, az;

    imu660rc_get_acc();
    imu660rc_get_gyro();

    // Sensor axes on the car: +X left, +Y rear, +Z up.
    // Quaternion body axes: +X forward, +Y left, +Z up.
    gx = -(float)(imu660rc_gyro_y - gyro_offset_y) / imu660rc_transition_factor[1];
    gy = (float)(imu660rc_gyro_x - gyro_offset_x) / imu660rc_transition_factor[1];
    gz = (float)(imu660rc_gyro_z - gyro_offset_z) / imu660rc_transition_factor[1];

    // The IMU660RC reports about +1 g on Z when the car is level.
    ax = -imu660rc_acc_transition(imu660rc_acc_y);
    ay = imu660rc_acc_transition(imu660rc_acc_x);
    az = imu660rc_acc_transition(imu660rc_acc_z);

    if (gx > -0.28f && gx < 0.28f) gx = 0.0f;
    if (gy > -0.28f && gy < 0.28f) gy = 0.0f;
    if (gz > -0.28f && gz < 0.28f) gz = 0.0f;

    yaw_gyro_integral += gz * dt;

    Quaternion_Update(gx, gy, gz, ax, ay, az, dt);
    Quaternion_ToEuler();

    // A six-axis IMU has no absolute yaw reference.
    euler.yaw = yaw_gyro_integral;
}

void IMU_Update(void) {
    IMU_Update_Dt(0.005f);
}
/*---------------------------------------------------------------------------
 * 航向角复�?
 *---------------------------------------------------------------------------*/
void Yaw_Reset(void) {
    // 保持当前roll和pitch，只复位yaw
    // 这需要从当前 roll �?pitch 重新构建四元�?
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
