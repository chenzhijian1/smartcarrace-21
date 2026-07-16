#ifndef __HUANDAO_H
#define __HUANDAO_H

#include "headfile.h"

/*============================================================================
 * 模块说明：环岛控制模块
 * 功能：
 *   1. 环岛状态机管理
 *   2. 入环、环内、出环控制
 *   3. 环岛差速计算
 *   4. 环岛专用参数管理（路径相关）
 * 
 * 参数设计：
 *   - 本模块管理环岛路径参数（半径、距离、角度等）
 *   - 控制算法参数（PID）在config.h中统一管理
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 常量定义
 *---------------------------------------------------------------------------*/
#define CAR_WIDTH 16.0f  // 车宽(cm)

#define HUANDAO_DIR_SOURCE_ROUTE  (0)
#define HUANDAO_DIR_SOURCE_SENSOR (1)

/*---------------------------------------------------------------------------
 * 环岛参数配置
 *---------------------------------------------------------------------------*/
extern uint8 huandao_num;                   // 环岛个数
extern uint8 huandao_count;                 // 环岛计数
extern uint8 huandao_dir[6];                // 环岛方向 (0:左, 1:右)
extern uint8 huandao_dir_source[6];         // 方向来源 (0:预设, 1:横电感)
extern uint8 huandao_r[6];                  // 环岛半径
extern float distance_before_huandao[6];    // 环岛前距离
extern float distance_after_huandao;        // 环岛后距离
extern float g_angle_turn;                  // 入环转角
extern float g_angle_inside[6];             // 环内角度
extern float angle_in_threshold;            // 入环角度阈值
extern float angle_out_threshold;           // 出环角度阈值

/*---------------------------------------------------------------------------
 * 环岛状态变量
 *---------------------------------------------------------------------------*/
extern uint8 flag_huandao;      // 0:左环岛, 1:右环岛
extern uint8 flag_set_angle;    // 是否已设置入环角度
extern uint16 flag_circle_in;   // 入环标志
extern uint16 flag_circle_out;  // 出环标志

extern float ratio;             // 差速比例
extern float target_angle_in;       // 入环起始角度
extern float target_angle_in_end;   // 入环目标角度
extern float target_angle_inside;   // 环内目标角度
extern float target_angle_out;      // 出环目标角度

/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
// 初始化和配置
void Huandao_Init(void);            // 环岛参数初始化（从EEPROM读取）
void Huandao_SaveConfig(void);      // 保存环岛参数到EEPROM

// 环岛状态处理
void Huandao_PreCircle(void);       // 预环岛模式(flag=1)
void Huandao_EnterCircle(void);     // 入环模式(flag=2)
void Huandao_InsideCircle(void);    // 环内循迹(flag=3)
void Huandao_ExitStraight(void);    // 出环直行(flag=7)

// 辅助函数
void Huandao_Reset(void);           // 重置环岛状态

#endif /* __HUANDAO_H */
