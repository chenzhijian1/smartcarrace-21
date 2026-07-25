#ifndef __CONFIG_H
#define __CONFIG_H

#include "headfile.h"

/*============================================================================
 * 模块说明：全局配置模块
 * 功能：
 *   1. 管理全局共享参数（跨模块使用的参数）
 *   2. 管理需要频繁调试的参数（PID、速度策略等）
 *   3. 提供EEPROM读写功能
 *   4. 参数初始化和保存
 * 
 * 设计原则：
 *   - 全局参数：系统级配置、跨模块共享
 *   - 调试参数：需要频繁调整的算法参数（PID、速度阈值）
 *   - 模块专用参数：在各自模块内管理（电机硬件参数、环岛状态等）
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 全局系统配置
 *---------------------------------------------------------------------------*/
// 调试模式
extern uint8 debug_mode;            // 0:正常模式, 1:调试模式

/*---------------------------------------------------------------------------
 * 方向控制PID参数（频繁调试）
 *---------------------------------------------------------------------------*/
// 主方向环PID
extern float kpa;       // 方向环P系数a（三次项系数）
extern float kpb;       // 方向环P系数b（一次项系数）
extern float kd;        // 方向环D系数（误差变化率）
extern float kd_imu;    // 陀螺仪D系数（角速度反馈）

/*---------------------------------------------------------------------------
 * 电机速度环PID参数（频繁调试）
 *---------------------------------------------------------------------------*/
extern float kp_motor;  // 速度环P
extern float ki_motor;  // 速度环I
extern float kd_motor;  // 速度环D

/*---------------------------------------------------------------------------
 * 速度策略参数（频繁调试）
 *---------------------------------------------------------------------------*/
extern float speed_high;    // 直道高速
extern float speed_low;     // 弯道低速
extern float speed_90;      // 90度弯速度
extern float speed_S;       // S弯速度
extern int16 normal_speed;  // 当前目标速度

/*---------------------------------------------------------------------------
 * 全局辅助函数
 *---------------------------------------------------------------------------*/
float StrToDouble(const char *s);               // 字符串转浮点数
float Config_ReadFloat(uint8 len, uint16 addr);
void eeprom_write_float_ascii(double dat, uint8 num, uint8 pointnum, uint16 addr);

/*---------------------------------------------------------------------------
 * EEPROM默认值（首次烧录使用）
 *---------------------------------------------------------------------------*/
// PID默认值
extern float kpa_iap;
extern float kpb_iap;
extern float kd_iap;
extern float kd_imu_iap;
extern float kp_motor_iap;
extern float ki_motor_iap;
extern float kd_motor_iap;

// 速度默认值
extern float speed_high_iap;
extern float speed_low_iap;
extern float speed_90_iap;
extern float speed_S_iap;
extern float normal_speed_iap;

/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
void Config_Init(void);         // 初始化全局配置（从EEPROM读取）
uint8 Config_Save(void);
void Config_SaveAll(void);      // 保存全局配置到EEPROM
void Config_SetNormalSpeed(int16 speed);
void Config_ButtonInit(void);
void Config_ButtonPoll(void);

#endif /* __CONFIG_H */
