#ifndef __SEESAW_H_
#define __SEESAW_H_

#include "zf_common_typedef.h"
#include "quaternion.h"

/*
 * 跷跷板识别和控制模块。
 *
 * 主循环调用 Seesaw_ImuUpdate() 更新状态；CarControl_NormalMode() 在
 * TIM4 的 5 ms 控制中断内读取状态，并调用下面的速度/轮速辅助函数。
 * 这里的宏只影响识别阈值或“编码器目标速度”，不直接代表 PWM 占空比。
 * 规则中的长度、宽度和高度只作为识别背景，不在本文件中参与几何计算。
 * ax/ay/az 直接取当前 IMU 换算值，单位为 g；入口和转折使用
 * 姿态解算得到的 euler.pitch，当前安装方向下负角表示车头上仰。
 * 所有 *_SAMPLES 都表示去重后的有效 IMU 帧数；“5 ms”只是当前配置。
 */

/* ---------- 姿态识别阈值 ---------- */

#define SEESAW_ENTRY_PITCH_MAX_DEG        (-7.0f)  // 入口需 pitch<=-7°。
#define SEESAW_PEAK_PITCH_MAX_DEG         (-9.0f)  // 回平前必须至少达到 pitch<=-9°。
#define SEESAW_PITCH_MIN_DEG              (-90.0f) // pitch低于-90°时撤销候选。
#define SEESAW_PITCH_MAX_DEG              (90.0f)  // pitch高于+90°时撤销候选。

/* ---------- 连续帧和超时 ---------- */
#define SEESAW_BASELINE_CONFIRM_SAMPLES   (3U)//平面起始姿态连续确认帧数。没有基线时不接受倾角候选。
#define SEESAW_ENTER_CONFIRM_SAMPLES      (2U)//负 pitch 抬起连续确认帧数，决定何时进入 RISING。
#define SEESAW_MAX_CANDIDATE_SAMPLES      (500U)//候选最长：40帧，超时直接判定元素通过


/* ---------- 第二种速度策略 ---------- */
/* RISING 阶段的整车目标上限，占普通直线目标的百分比，不是 PWM。
调小更慢、更容易等待板子落下，但可能无法越过支点；调大更快。 */
#define SEESAW_SPEED_SLOW_PERCENT         (0U)



 //限制差速

/* 仅用于 RISING 阶段左右轮的低速下限，单位仍是编码器目标值。
它不会把整车中心目标从 n60 抬到 n100，只在中心目标已经不低于100时防止某一侧因差速被压得太低。需要根据电机实际起转值调整。 */
#define SEESAW_SPEED_CRAWL_MIN            (100)
/* RISING 阶段左右轮相对中心目标的允许范围，防止一侧反转或差速过大。 */
#define SEESAW_SLOW_WHEEL_LOW_PERCENT     (50U)
#define SEESAW_SLOW_WHEEL_HIGH_PERCENT    (150U)


/* ---------- 状态机状态 ---------- */
#define SEESAW_STATE_IDLE                 (0U)//未看到有效上坡。
#define SEESAW_STATE_RISING               (1U)//已确认登板并正在上升，此时执行低速等待策略。
#define SEESAW_STATE_EXITED               (4U)//pitch转正或候选超时，本次跷跷板元素结束。




void Seesaw_Init(void);//上电初始化入口，内部清空本模块全部状态。
void Seesaw_Reset(void);//重置本模块状态，恢复到 IDLE。
uint8 Seesaw_ImuUpdate(const imu_sample_t *sample, float pitch_deg);//主循环每收到一个新IMU样本调用一次；抬起pitch<0，下降pitch>0。
uint8 Seesaw_IsCandidate(void);//返回当前是否处于 RISING 候选阶段。
uint8 Seesaw_IsConfirmed(void);//返回是否已经判定本次跷跷板通过。
uint8 Seesaw_HasExited(void);//返回是否已通过跷跷板并释放路线。
uint8 Seesaw_GetState(void);//返回当前状态。

int16 Seesaw_GetSpeedTarget(int16 current_speed, int16 straight_speed);//供 TIM4 速度控制中断调用：RISING 时限制当前目标，其他状态原样返回。
void Seesaw_ClampWheelTargets(int16 center_speed,
                              int16 *left_speed,
                              int16 *right_speed);//供 TIM4 速度控制中断调用：RISING 时限制左右轮差速，避免一侧反转。

#endif /* __SEESAW_H_ */
