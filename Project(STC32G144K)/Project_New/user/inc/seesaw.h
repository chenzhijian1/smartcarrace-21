#ifndef __SEESAW_H_
#define __SEESAW_H_

#include "zf_common_typedef.h"

/*
 * 跷跷板识别和控制模块。
 *
 * 主循环调用 Seesaw_ImuUpdate() 更新状态；CarControl_NormalMode() 在
 * TIM4 的 5 ms 控制中断内读取状态，并调用下面的速度/轮速辅助函数。
 * 这里的宏只影响识别阈值或“编码器目标速度”，不直接代表 PWM 占空比。
 * 规则中的长度、宽度和高度只作为识别背景，不在本文件中参与几何计算。
 * 识别只使用姿态解算得到的 euler.pitch，当前安装方向下负角表示车头上仰。
 * 所有 *_SAMPLES 都表示去重后的有效 IMU 帧数；“5 ms”只是当前配置。
 */

/* ---------- 姿态识别阈值 ---------- */

#define SEESAW_RISING_ENTER_DEG           (5.0f)  // pitch<=-8°确认进入RISING。
#define SEESAW_FALLING_ENTER_DEG          (5.0f)  // pitch>=+8°确认进入FALLING。
#define SEESAW_TILT_EXIT_DEG              (2.0f)  // |pitch|<2°认为已经回平。
#define SEESAW_TILT_MAX_DEG               (45.0f) // |pitch|超过45°时撤销候选。


/* ---------- 连续帧和超时 ---------- */
#define SEESAW_ENTER_CONFIRM_SAMPLES      (3U)//pitch<=-8°连续确认帧数。
#define SEESAW_FALL_CONFIRM_SAMPLES       (3U)//pitch>=+8°连续确认帧数。
#define SEESAW_EXIT_CONFIRM_SAMPLES       (3U)//回平连续确认帧数。
#define SEESAW_MAX_CANDIDATE_SAMPLES      (600U)//总看门狗，按5ms/帧约3秒。

/* ---------- 第二种速度策略 ---------- */
/* RISING/FALLING 阶段直接使用 normal_speed_cal 的百分比，不是 PWM。
调小更慢、更容易等待板子落下，但可能无法越过支点；调大更快。 */
#define SEESAW_SPEED_SLOW_PERCENT         (35U)


/* ---------- 状态机状态 ---------- */
#define SEESAW_STATE_IDLE                 (0U)//未看到有效上坡。
#define SEESAW_STATE_RISING               (1U)//已确认登板并正在上升，此时执行低速等待策略。
#define SEESAW_STATE_FALLING              (2U)//已确认姿态从上升转为下降，继续保持35%速度。
#define SEESAW_STATE_EXITED               (3U)//已连续回平，本次跷跷板元素结束。




void Seesaw_Reset(void);//重置本模块状态，恢复到 IDLE。
void Seesaw_ImuUpdate(float pitch_deg);//主循环每次姿态更新后调用；抬起pitch<0，下降pitch>0。
uint8 Seesaw_HasExited(void);//返回是否已经连续回到平面。
uint8 Seesaw_GetState(void);//返回当前状态。

int16 Seesaw_GetSpeedTarget(int16 current_speed);//供 TIM4 速度控制中断调用：RISING/FALLING 返回 normal_speed_cal 的35%，其他状态原样返回。

#endif /* __SEESAW_H_ */
