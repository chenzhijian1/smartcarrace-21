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
#define HUANDAO_DIR_SOURCE_ROUTE  (0)
#define HUANDAO_DIR_SOURCE_SENSOR (1)
#define HUANDAO_MAX_COUNT          (5)

typedef enum
{
    HUANDAO_STATE_IDLE = 0,
    HUANDAO_STATE_PRE_CIRCLE,
    HUANDAO_STATE_ENTER_CIRCLE,
    HUANDAO_STATE_INSIDE_CIRCLE,
    HUANDAO_STATE_EXIT_STRAIGHT
} huandao_state_t;

/*---------------------------------------------------------------------------
 * 环岛参数配置
 *---------------------------------------------------------------------------*/
extern uint8 huandao_num;                   // 环岛个数
extern uint8 huandao_count;                 // 环岛计数
extern uint8 huandao_dir[HUANDAO_MAX_COUNT];
// 环岛方向：0 为左环（逆时针、航向角增加），1 为右环（顺时针、航向角减少）。
extern uint8 huandao_dir_source[HUANDAO_MAX_COUNT];
extern uint8 huandao_r[HUANDAO_MAX_COUNT];
extern float distance_before_huandao[HUANDAO_MAX_COUNT];

/*---------------------------------------------------------------------------
 * 环岛状态变量
 *---------------------------------------------------------------------------*/
extern uint8 flag_huandao;      // 0:左环岛, 1:右环岛
/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
// 初始化和配置
void Huandao_Init(void);            // 环岛参数初始化（从EEPROM读取）
void Huandao_SaveConfig(void);      // 保存环岛参数到EEPROM

// 环岛状态处理
uint8 Huandao_DetectUpdate(void);
uint8 Huandao_DetectIsStraightHold(void);
void Huandao_DetectReset(void);
uint8 Huandao_ConsumeExitEvent(void);
void Huandao_PrepareControl(int16 straight_speed,
                            int16 *target_speed,
                            int16 *direction_diff);
huandao_state_t Huandao_GetState(void);

// 辅助函数
void Huandao_Reset(void);           // 重置环岛状态

#endif /* __HUANDAO_H */
