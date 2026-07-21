#ifndef __QUATERNION_H
#define __QUATERNION_H

#include "headfile.h"

/*============================================================================
 * 模块说明：四元数姿态解算模�?
 * 功能�?
 *   1. 使用四元数进行姿态更�?
 *   2. 结合陀螺仪和加速度计数据进行互补滤�?
 *   3. 解算欧拉角（roll, pitch, yaw�?
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 常量定义
 *---------------------------------------------------------------------------*/
#define MAHONY_KP 1.2f
#define MAHONY_KI 0.0f
#define RAD_TO_DEG 57.2957795131f  // 弧度转角�?

/*---------------------------------------------------------------------------
 * 四元数结构体
 *---------------------------------------------------------------------------*/
typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} imu_sample_t;

typedef struct {
    float q0;  // w
    float q1;  // x
    float q2;  // y
    float q3;  // z
} Quaternion;

/*---------------------------------------------------------------------------
 * 欧拉角结构体
 *---------------------------------------------------------------------------*/
typedef struct {
    float roll;   // 横滚角（绕X轴旋转）
    float pitch;  // 俯仰角（绕Y轴旋转）
    float yaw;    // 航向角（绕Z轴旋转）
} EulerAngles;

/*---------------------------------------------------------------------------
 * 外部变量声明
 *---------------------------------------------------------------------------*/
extern Quaternion q;           // 四元�?
extern EulerAngles euler;      // 欧拉角（角度制）
extern float gyro_offset_x;    // 陀螺仪X轴零�?
extern float gyro_offset_y;    // 陀螺仪Y轴零�?
extern float gyro_offset_z;    // 陀螺仪Z轴零�?

/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
// 初始化四元数
void Quaternion_Init(void);

// 四元数姿态更新（使用陀螺仪和加速度计数据）
void Quaternion_Update(float gx, float gy, float gz, 
                       float ax, float ay, float az, 
                       float dt);

// 四元数转欧拉�?
void Quaternion_ToEuler(void);

// 陀螺仪零偏校准
void Gyro_Calibration(uint16 samples);

// IMU数据更新（从IMU963读取数据并更新姿态）
void IMU_Update(void);
void IMU_Update_Dt(float dt);

// 航向角复�?
void IMU_SampleCopy(imu_sample_t *sample);
float IMU_GetGyroZDps(void);

void Yaw_Reset(void);

#endif /* __QUATERNION_H */
