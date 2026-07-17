#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "headfile.h"

/*============================================================================
 * 模块说明：车辆控制模块
 * 功能：
 *   1. 车辆状态机管理
 *   2. 速度策略控制
 *   3. 方向PID控制执行
 *   4. 软启动/软停车
 * 
 * 参数设计：
 *   - PID参数在config.h中管理（需要频繁调试）
 *   - 本模块管理运行时状态变量和辅助参数
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 状态标志定义
 * flag值含义:
 *   0: 正常循迹模式
 *   1: 预环岛模式
 *   2: 入环模式
 *   3: 环内循迹
 *   4: 起步发车
 *   5: 慢速停车
 *   7: 出环直行
 *---------------------------------------------------------------------------*/
extern uint8 flag;              // 车辆状态
extern uint8 flag_stop;         // 停止标志
extern uint8 flag_key_control;  // 控制模式: 0调参 1跑车
extern uint8 flag_key_fast;     // 快速模式标志
extern uint8 nav_end_flag_sent; // 导航结束标志

/*---------------------------------------------------------------------------
 * 速度控制变量（运行时状态）
 *---------------------------------------------------------------------------*/
extern int16 normal_speed_pre;  // 上次目标速度
extern int16 normal_speed_cal;  // 计算后速度
extern int16 test_speed;        // 设定速度
extern int16 changed_speed;     // 差速调整量
extern int16 set_leftspeed;     // 左轮设定速度
extern int16 set_rightspeed;    // 右轮设定速度
extern int16 speed_huandao;     // 环岛速度

/*---------------------------------------------------------------------------
 * 编码器相关
 *---------------------------------------------------------------------------*/
extern float encoder_ave;       // 编码器平均积分值
extern float encoder_temp;      // 编码器临时值(用于距离计算)

/*---------------------------------------------------------------------------
 * 其他控制变量
 *---------------------------------------------------------------------------*/
extern float k;                 // 差速调整系数
extern float s;                 // 速度衰减系数
extern float gyro_z;            // 滤波后的角速度
extern float last_gyro_z;       // 上次角速度
extern float lpf_gyro;          // 陀螺仪低通滤波系数

extern uint8 cnt_stop;          // 软停车计数器
extern uint8 cnt_launch;        // 发车计数器

/*---------------------------------------------------------------------------
 * 风扇控制
 *---------------------------------------------------------------------------*/
extern uint8 flag_suction_fan_off;

/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
// 初始化和配置
void CarControl_Init(void);         // 车辆控制参数初始化
void CarControl_SaveConfig(void);   // 保存车辆控制参数

// 主控制函数
void CarControl_Update(void);       // 车辆状态更新(原speed_change)

// 模式处理函数
void CarControl_NormalMode(int16 c_speed, int16 s_speed);
void CarControl_LaunchMode(void);   // 起步发车模式(flag=4)
void CarControl_StopMode(void);     // 慢速停车模式(flag=5)

// 方向控制
void dir_pid(float error, float last_error, float gyro);    // 方向PID

// 速度调整
void speed_adjust(int16 c_speed, int16 s_speed);

// 安全控制
uint8 car_stop_judge(void);         // 脱线保护

void CarControl_RequestSoftStop(void);

#endif /* __CAR_CONTROL_H */
