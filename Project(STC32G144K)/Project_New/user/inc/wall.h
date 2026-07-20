#ifndef __WALL_H_
#define __WALL_H_

#include "zf_common_typedef.h"
#include "spatial_features.h"

/*
 * 墙面元素模块。
 *
 * 主循环把公共 IMU 特征交给 Wall_ImuUpdate()；该函数只负责识别阶段，
 * 不直接驱动电机。TIM4 控制中断通过下面的速度和差速函数读取阶段结果。
 * 轴约定采用当前实测结果：上坡 ay<0，横向时重力主要落在 ax，下坡 ay>0。
 * ax/ay/az单位为g；墙面入口使用与圆筒相同的euler.pitch<-5度，
 * climb_angle_deg只保留作诊断参考。
 * 所有 *_SAMPLES 都是去重后的有效 IMU 帧数；“5 ms”只是当前配置。
 */

/* ---------- 姿态识别阈值 ---------- */

#define WALL_ENTRY_PITCH_MAX_DEG          (-5.0f)//与圆筒入口相同：euler.pitch<-5度才进入墙面候选。
/* 旧版climb_angle_deg入口阈值保留作历史对照，不再参与识别。 */
/* #define WALL_CLIMB_ENTER_DEG             (8.0f) */
/* pitch连续满足WALL_CLIMB_CONFIRM_SAMPLES帧后进入上坡候选。 */

#define WALL_VERTICAL_AY_MAX_G             (-0.75f)//近竖直要求 ay<=-0.75g。改得更负会更接近真正竖直，但确认更晚。
#define WALL_VERTICAL_AZ_MAX_G             (0.45f)//近竖直要求 az<=0.45g，也用于排除仍接近平面的姿态。
#define WALL_INVERTED_AZ_MAX_G             (-0.35f)//若 az<=-0.35g 或公共特征判为倒置，则撤销墙面候选，避免把圆筒当成墙面。
#define WALL_EXIT_AY_MAX_G                 (0.15f)//退出要求 |ay|<=0.15g。调小更严格，调大更容易提前确认回平。
#define WALL_EXIT_AZ_MIN_G                 (0.85f)//退出还要求 az>=0.85g。调高更严格，调低更容易确认回平。

#define WALL_LATERAL_AX_MIN_G              (0.70f)//横向段要求 |ax|>=0.70g，证明重力主要落在轮轴方向；调高更严格。
#define WALL_VERTICAL_AX_MAX_G             (0.70f)//近竖直阶段要求 |ax|<=0.70g，防止把已经横向的姿态继续当成近竖直。
/* 两个 0.70g 不是重复参数：前者是进入横向段的下限，后者是近竖直段的上限。 */

#define WALL_LATERAL_AY_MAX_G              (0.45f)//横向段要求 |ay|<=0.45g。调大更宽松，但更容易混入上坡或下坡。
#define WALL_LATERAL_AZ_MAX_G              (0.45f)//横向段要求 |az|<=0.45g。调小更接近纯侧向，调大更能容忍过渡姿态。
#define WALL_DESCENT_AY_MIN_G              (0.45f)//横向后要求 ay>=0.45g 才确认下坡；调大更严格、更晚触发。
#define WALL_DESCENT_AX_MAX_G              (0.60f)//下坡还要求 |ax|<=0.60g，避免车辆仍处于墙面横向段。

/* ---------- 连续帧和超时 ---------- */
/* 以下帧数按当前 IMU 约 5 ms 一帧估算。调大更稳但反应更晚，调小更快但更容易被振动触发。 */
#define WALL_BASELINE_CONFIRM_SAMPLES      (5U)//平面基线连续确认帧数，5帧约25ms。
#define WALL_CLIMB_CONFIRM_SAMPLES         (5U)//上坡入口连续确认帧数，决定何时进入 CLIMB_CANDIDATE。
#define WALL_VERTICAL_CONFIRM_SAMPLES      (10U)//近竖直连续确认帧数，10帧约50ms。
#define WALL_LATERAL_CONFIRM_SAMPLES       (10U)//轮轴方向重力连续确认帧数，满足后才正式确认这是墙面。
#define WALL_DESCENT_CONFIRM_SAMPLES       (5U)//由横向转入下坡的连续确认帧数。
#define WALL_EXIT_CONFIRM_SAMPLES          (20U)//回到平面的连续确认帧数，20帧约100ms。
#define WALL_MAX_CANDIDATE_SAMPLES         (2400U)//一次墙面候选最长约12秒，超时后撤销，防止错误候选长期占用路线。
#define WALL_NORM_INVALID_GRACE_SAMPLES    (8U)//加速度模长暂时超出0.85～1.15g时允许保留候选的帧数。

/* ---------- 墙面速度策略 ---------- */
/* 这些参数是普通直线目标的百分比上限，不是 PWM；原方向误差已经降得更低时不会被重新抬高。 */
#define WALL_CLIMB_SPEED_PERCENT            (80U)//上坡阶段最高使用普通直线目标的80%。
#define WALL_VERTICAL_SPEED_PERCENT         (75U)//近竖直阶段最高使用普通直线目标的75%。
#define WALL_LATERAL_SPEED_PERCENT          (90U)//横向阶段最高使用普通直线目标的90%。

/* 当前赛道下坡后接直道，所以默认不主动减速。ENABLE 改为 1 后才启用下面的 75% 上限。 */
#define WALL_DESCENT_SPEED_ENABLE           (0U)//0关闭下坡减速，1开启下坡减速。
#define WALL_DESCENT_SPEED_PERCENT          (75U)//仅在下坡减速开关打开时生效。

#define WALL_GRAVITY_FF_PWM                (1600.0f)

/* ---------- 横向抗重力方向偏置 ---------- */
/*
 * 方向偏置的正负号必须用实车确认。默认关闭，避免符号错时把车推向墙外。
 * ENABLE=1 后，横向阶段会按 ax 符号加入固定差速 VALUE；VALUE 是方向环
 * 的差速目标单位，不是角度或 PWM。SIGN 只能取 +1 或 -1。
 */
#define WALL_DIRECTION_BIAS_ENABLE          (0U)//0关闭，1只在 LATERAL 横向阶段加入抗重力固定差速。
#define WALL_DIRECTION_BIAS_SIGN            (1)//只能取+1或-1，用实车确认哪个方向是朝墙内修正。
#define WALL_DIRECTION_BIAS_VALUE           (25)//加入 changed_speed 的差速目标值，不是角度、速度百分比或PWM。

/* ---------- 状态机状态 ---------- */
#define WALL_STATE_IDLE                    (0U)//尚未识别到有效墙面上坡。
#define WALL_STATE_CLIMB_CANDIDATE         (1U)//已确认上坡候选，开始使用上坡速度上限。
#define WALL_STATE_VERTICAL_PROVISIONAL    (2U)//已经近竖直，但圆筒也会经过该姿态，所以仍是临时状态。
#define WALL_STATE_LATERAL                 (3U)//已确认重力落在轮轴方向，从此可以明确判定为墙面。
#define WALL_STATE_DESCENT                 (4U)//横向段之后已经确认进入下坡。
#define WALL_STATE_EXITED                  (5U)//已经连续回到平面，本次墙面元素结束。




void Wall_Init(void);//上电初始化入口，内部清空本模块全部状态。
void Wall_Reset(void);//重置本模块状态，恢复到 IDLE。
uint8 Wall_ImuUpdate(const spatial_features_t *features,
                     float pitch_deg);//主循环每收到一个新IMU特征帧调用一次，使用与圆筒相同的pitch入口条件。
uint8 Wall_IsCandidate(void);//上坡、近竖直、横向或下坡进行中返回1，EXITED不再算候选。
uint8 Wall_IsConfirmed(void);//看到轮轴方向重力后返回1，用于元素管理器正式确认墙面。
uint8 Wall_HasExited(void);//连续回到平面并进入 EXITED 后返回1。
uint8 Wall_GetState(void);//返回当前墙面状态，供控制中断、元素管理器和调试打印读取。

int16 Wall_GetSpeedTarget(int16 current_speed, int16 straight_speed);//供TIM4控制链调用：按当前墙面阶段限制本周期整车目标速度。
void Wall_ClampWheelTargets(int16 center_speed,
                            int16 *left_speed,
                            int16 *right_speed);//供speed_adjust()之后调用：墙面进行中禁止某一侧轮速目标反向。
int16 Wall_GetDirectionBias(void);//供TIM4控制链调用：横向阶段返回抗重力差速偏置，未启用或非横向时返回0。
void Wall_UpdateGravityFeedforward(float pitch_sin);//主循环更新墙面重力前馈快照，输入为sin(pitch)。
int16 Wall_GetGravityFeedforwardPwm(void);//TIM4控制链读取墙面重力前馈快照。

#endif /* __WALL_H_ */
