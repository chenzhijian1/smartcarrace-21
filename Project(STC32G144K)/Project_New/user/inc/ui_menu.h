#ifndef __UI_MENU_H
#define __UI_MENU_H

#include "headfile.h"

/*============================================================================
 * 模块说明：UI菜单模块
 * 功能：
 *   1. 菜单界面显示
 *   2. 按键交互处理
 *   3. 参数调整界面
 *============================================================================*/

/*---------------------------------------------------------------------------
 * UI结构体定义
 *---------------------------------------------------------------------------*/
typedef struct {
    void (*Disp)(void);     // 显示函数指针
    int16 cursor;           // 光标位置
    int16 page;             // 当前页面
    int16 last;             // 上一页面
} ui_struct;

extern ui_struct ui;

/*---------------------------------------------------------------------------
 * 按键状态变量
 *---------------------------------------------------------------------------*/
extern uint8 key1_status;
extern uint8 key2_status;
extern uint8 key3_status;
extern uint8 key4_status;

extern uint8 key1_last_status;
extern uint8 key2_last_status;
extern uint8 key3_last_status;
extern uint8 key4_last_status;

extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

/*---------------------------------------------------------------------------
 * 拨码开关状态变量
 *---------------------------------------------------------------------------*/
extern uint8 sw1_status;
extern uint8 sw2_status;
extern uint8 sw3_status;
extern uint8 sw4_status;

/*---------------------------------------------------------------------------
 * 函数声明
 *---------------------------------------------------------------------------*/
// 菜单显示
void UI_Display(void);              // 主菜单显示函数
void UI_DispStrings(uint8 strings[8][30]);  // 显示字符串数组

// 按键扫描
void UI_KeyScan(void);              // 菜单按键扫描

// 数据显示
void UI_ShowRuntime(void);          // 运行时数据显示(原ips114_show)

#endif /* __UI_MENU_H */
