#ifndef __WALL_H_
#define __WALL_H_

#include "zf_common_typedef.h"
#include "quaternion.h"

/*
 * 墙面元素模块。
 *
 * 主循环把当前 IMU 样本交给 Wall_ImuUpdate()；该函数只负责识别阶段，
 * 不直接驱动电机。TIM4 控制中断通过下面的速度和差速函数读取阶段结果。
 * 轴约定采用当前实测结果：上坡 ay<0，横向时重力主要落在 ax，下坡 ay>0。
 * ax/ay/az直接使用当前换算值，单位为g；
 * 所有 *_SAMPLES 都是去重后的有效 IMU 帧数；“5 ms”只是当前配置。
 */

/* ===== 以下帧数按当前 IMU 约 1.5 cm/帧估算（3 m/s）。调大更稳但反应更晚，调小更快但更容易被振动触发。 ===== */

/* ---------- 0. 平面基线：pitch ∈ [-8°, 8°] 认为车在平路，避免误触发 ---------- */
#define WALL_BASELINE_CONFIRM_SAMPLES      (5U)   // 5帧约7.5cm

/* ---------- 1. 上坡入口：pitch 条件 + 连续帧 ---------- */
#define WALL_ENTRY_PITCH_MAX_DEG          (-13.0f)// 阈值：euler.pitch < -14° 才进入候选
#define WALL_CLIMB_CONFIRM_SAMPLES         (2U)   // 2帧约3cm

/* ---------- 2. 近竖直：pitch 条件 + 连续帧 ---------- */
#define WALL_VERTICAL_PITCH_MAX_DEG        (-40.0f)// 阈值：pitch < -70° 认为进入近竖直
#define WALL_VERTICAL_CONFIRM_SAMPLES      (3U)    // 5帧约7.5cm

/* ---------- 3. 横向：roll 条件 + 连续帧 ---------- */
#define WALL_LATERAL_ROLL_MIN_DEG          (50.0f) // 阈值：|roll| > 70° 认为进入横向
#define WALL_LATERAL_CONFIRM_SAMPLES       (3U)    // 3帧约4cm

/* ---------- 4. 下坡：pitch 条件 + 连续帧 ---------- */
#define WALL_DESCENT_PITCH_MIN_DEG         (40.0f) // 阈值：pitch > 70° 认为进入下坡
#define WALL_DESCENT_CONFIRM_SAMPLES       (3U)    // 5帧约7.5cm

/* ---------- 5. 退出（回到平面）：连续帧 ---------- */
#define WALL_EXIT_CONFIRM_SAMPLES          (50U)   // 5帧约7.5cm

/* ---------- 候选超时与容错 ---------- */
#define WALL_MAX_CANDIDATE_SAMPLES         (500U) // 候选最长约2250cm，超时撤销，防止长期占用路线

/* ---------- 墙面速度策略 ---------- */
/* 这些参数是普通直线目标的阶段目标百分比，不是 PWM；墙面活跃时可主动加速到该目标。 */
#define WALL_CLIMB_SPEED_PERCENT            (100U)//上坡阶段目标为普通直线目标的120%。
#define WALL_LATERAL_SPEED_PERCENT          (100U)//横向阶段目标为普通直线目标的120%。
#define WALL_DESCENT_SPEED_PERCENT          (130U)//下坡阶段目标为普通直线目标的120%。

#define WALL_GRAVITY_FF_PWM                (1710.0f)

/* ---------- 状态机状态 ---------- */
#define WALL_STATE_IDLE                    (0U)//尚未识别到有效墙面上坡。
#define WALL_STATE_CLIMB_CANDIDATE         (1U)//已确认上坡候选，开始使用上坡速度上限。
#define WALL_STATE_VERTICAL_PROVISIONAL    (2U)//已经近竖直，但圆筒也会经过该姿态，所以仍是临时状态。
#define WALL_STATE_LATERAL                 (3U)//已确认横向（|roll| > 70°），从此可以明确判定为墙面。
#define WALL_STATE_DESCENT                 (4U)//已确认进入下坡。
#define WALL_STATE_EXITED                  (5U)//已经连续回到平面，本次墙面元素结束。




void Wall_Init(void);//上电初始化入口，内部清空本模块全部状态。
void Wall_Reset(void);//重置本模块状态，恢复到 IDLE。
uint8 Wall_ImuUpdate(const imu_sample_t *sample,
                     float pitch_deg,
                     float roll_deg);//主循环每收到一个新IMU样本调用一次，使用与圆筒相同的pitch入口条件。
uint8 Wall_IsCandidate(void);//上坡、近竖直、横向或下坡进行中返回1，EXITED不再算候选。
uint8 Wall_IsConfirmed(void);//进入横向后返回1，用于元素管理器正式确认墙面。
uint8 Wall_HasExited(void);//连续回到平面并进入 EXITED 后返回1。
uint8 Wall_GetState(void);//返回当前墙面状态，供控制中断、元素管理器和调试打印读取。

int16 Wall_GetSpeedTarget(int16 current_speed, int16 straight_speed);//供TIM4控制链调用：按当前墙面阶段限制本周期整车目标速度。
void Wall_ClampWheelTargets(int16 center_speed,
                            int16 *left_speed,
                            int16 *right_speed);//供speed_adjust()之后调用：墙面进行中禁止某一侧轮速目标反向。
int16 Wall_GetDirectionBias(void);//当前横向阶段无差速前馈，接口保留，始终返回 0。
void Wall_UpdateGravityFeedforward(float pitch_sin);//主循环更新墙面重力前馈快照，输入为sin(pitch)。
int16 Wall_GetGravityFeedforwardPwm(void);//TIM4控制链读取墙面重力前馈快照。

#endif /* __WALL_H_ */
