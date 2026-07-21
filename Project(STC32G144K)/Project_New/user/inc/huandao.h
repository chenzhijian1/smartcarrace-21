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
#define CAR_WIDTH 13.0f  // 车宽(cm)

#define HUANDAO_DIR_SOURCE_ROUTE  (0)
#define HUANDAO_DIR_SOURCE_SENSOR (1)
#define HUANDAO_MAX_COUNT          (5)

typedef enum
{
    HUANDAO_DETECT_NORMAL = 0,
    HUANDAO_DETECT_SUSPECT,
    HUANDAO_DETECT_ACTIVE,
    HUANDAO_DETECT_REARM
} huandao_detect_state_t;

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
extern volatile uint8 huandao_detect_state;

extern float huandao_pre_h_threshold;
extern float huandao_confirm_h_threshold;
extern float huandao_suspect_max_distance;
extern float huandao_rearm_h_threshold;

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

uint8 Huandao_DetectUpdate(void);
uint8 Huandao_DetectIsStraightHold(void);
void Huandao_DetectReset(void);
void Huandao_DetectStartRearm(void);
uint8 Huandao_ConsumeExitEvent(void);

// 辅助函数
void Huandao_Reset(void);           // 重置环岛状态

#endif /* __HUANDAO_H */
