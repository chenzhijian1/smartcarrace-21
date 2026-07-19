#ifndef __SEESAW_H_
#define __SEESAW_H_

#include "zf_common_typedef.h"
#include "spatial_features.h"

/*
 * 跷跷板识别和控制模块。
 *
 * 主循环调用 Seesaw_ImuUpdate() 更新状态；CarControl_NormalMode() 在
 * TIM4 的 5 ms 控制中断内读取状态，并调用下面的速度/轮速辅助函数。
 * 这里的宏只影响识别阈值或“编码器目标速度”，不直接代表 PWM 占空比。
 * 规则中的长度、宽度和高度只作为识别背景，不在本文件中参与几何计算。
 * ax/ay/az 单位为 g，gx 单位为度/秒；climb_angle_deg 由
 * atan2(-ay_lowpass, az_lowpass) 得到，当前安装方向下正角表示车头上仰。
 * 所有 *_SAMPLES 都表示去重后的有效 IMU 帧数；“5 ms”只是当前配置。
 */

/* ---------- 姿态识别阈值 ---------- */

#define SEESAW_TILT_ENTER_DEG             (8.0f)  // 连续达到该上仰角后进入候选。
#define SEESAW_TILT_MIN_PEAK_DEG          (15.0f) // 有效跷跷板轨迹必须达到的最小峰值角。
#define SEESAW_TILT_MAX_DEG               (60.0f) // 超过该上仰角时撤销跷跷板候选。
#define SEESAW_TILT_DROP_DEG              (6.0f)  // 当前角度比峰值下降该角度时确认下降趋势。

/* 旧版直接使用-ay重力分量的阈值，保留数值供历史调参对照，不再参与识别。 */
/* #define SEESAW_PITCH_ENTER_G           (0.1392f) */
/* #define SEESAW_PITCH_MIN_PEAK_G        (0.2588f) */
/* #define SEESAW_PITCH_MAX_G             (0.8660f) */
/* #define SEESAW_PITCH_DROP_G            (0.1045f) */

#define SEESAW_AZ_MIN_G                   (0.25f)//加速度 Z 轴最低分量，防止加速度模长可信但姿态已经倒置时误判。单位：g。
#define SEESAW_GYRO_MOTION_MIN_DPS        (2.0f)//忽略小于该值的 gx 角速度。单位：度/秒；调大可滤振，调小更灵敏。
#define SEESAW_GYRO_REVERSE_CONFIRM_SAMPLES (3U)// gx 反向连续确认帧数。当前 IMU 约 5 ms 一帧，3 帧约 15 ms。


/* ---------- 连续帧和超时 ---------- */
#define SEESAW_BASELINE_CONFIRM_SAMPLES   (5U)//平面起始姿态连续确认帧数。没有基线时不接受倾角候选。
#define SEESAW_ENTER_CONFIRM_SAMPLES      (5U)//正向上坡倾角连续确认帧数，决定何时进入 RISING。
#define SEESAW_TREND_CONFIRM_SAMPLES      (3U)//峰值之后持续低于峰值的确认帧数，决定何时允许进入 FALLING。
#define SEESAW_EXIT_CONFIRM_SAMPLES       (15U)//回到平面连续确认帧数，决定何时进入 EXITED。
#define SEESAW_MAX_CANDIDATE_SAMPLES      (1200U)//候选最长持续时间：1200 帧约等于 6 秒（按 5 ms/帧估算）。
#define SEESAW_NORM_INVALID_GRACE_SAMPLES (5U)//加速度模长短暂超出 0.85～1.15 g 时允许保留候选的帧数。

/* ---------- 第二种速度策略 ---------- */
/* RISING 阶段的整车目标上限，占普通直线目标的百分比，不是 PWM。
调小更慢、更容易等待板子落下，但可能无法越过支点；调大更快。 */
#define SEESAW_SPEED_SLOW_PERCENT         (60U)



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
#define SEESAW_STATE_FALLING              (2U)//已确认姿态从上升转为下降，此时释放低速限制。
#define SEESAW_STATE_ACTIVE               (3U)//已确认跷跷板轨迹，但还没有稳定回到平面。
#define SEESAW_STATE_EXITED               (4U)//已连续回平，本次跷跷板元素结束。




void Seesaw_Init(void);//上电初始化入口，内部清空本模块全部状态。
void Seesaw_Reset(void);//重置本模块状态，恢复到 IDLE。
uint8 Seesaw_ImuUpdate(const spatial_features_t *features);//主循环每收到一个新IMU特征帧调用一次，内部推进状态机；当前状态非IDLE时返回1。
uint8 Seesaw_IsCandidate(void);//返回当前是否处于 RISING/FALLING 候选阶段。
uint8 Seesaw_IsConfirmed(void);//返回是否已经完成峰值转折（ACTIVE 或 EXITED），可用于路线确认。
uint8 Seesaw_HasExited(void);//返回是否已经连续回到平面。
uint8 Seesaw_GetState(void);//返回当前状态。

int16 Seesaw_GetSpeedTarget(int16 current_speed, int16 straight_speed);//供 TIM4 速度控制中断调用：RISING 时限制当前目标，其他状态原样返回。
void Seesaw_ClampWheelTargets(int16 center_speed,
                              int16 *left_speed,
                              int16 *right_speed);//供 TIM4 速度控制中断调用：RISING 时限制左右轮差速，避免一侧反转。

#endif /* __SEESAW_H_ */
