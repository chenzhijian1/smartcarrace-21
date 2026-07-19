#ifndef __CYLINDER_H_
#define __CYLINDER_H_

#include "zf_common_typedef.h"

/*
 * 圆筒入口电感条件，输入均为12位ADC原始值。
 * outer指左右两只横向电感，total指四只电感之和。
 */
#define CYLINDER_DETECT_OUTER_SUM_MIN       (3800U) /* 两只横向电感之和下限 */
#define CYLINDER_DETECT_TOTAL_SUM_MIN       (4200U) /* 四只电感之和下限 */
#define CYLINDER_DETECT_OUTER_SINGLE_MIN    (1100U) /* 每只横向电感各自的下限 */
#define CYLINDER_DETECT_CONFIRM_SAMPLES     (8U)    /* 连续8个ADC帧，约80ms */

/*
 * 从圆筒右侧进入时的PRE_ENTRY差速限制。
 * changed_speed为正表示向左（桶内）转，为负表示向右（桶外）转。
 * 左右轮目标分别为base-diff和base+diff，所以实际轮速差是2*diff。
 */
#define CYLINDER_PRE_ENTRY_INWARD_DIFF_MAX   (8)   /* 允许向左转的最大差速 */
#define CYLINDER_PRE_ENTRY_OUTWARD_DIFF_MAX  (160) /* 允许向右修正的最大差速 */
#define CYLINDER_ENTRY_LEFT_GUARD_DEG         (2.0f) /* 入筒后前2度继续禁止向左转 */

/* 上筒与最终出口使用模长门控；顶部、后半圈兜底保持原始轴阈值判断。 */
#define CYLINDER_IMU_NORM_MIN_G              (0.85f)
#define CYLINDER_IMU_NORM_MAX_G              (1.15f)

/* 开始登筒：车辆pitch连续小于-8度。 */
#define CYLINDER_ENTRY_PITCH_MAX_DEG          (-8.0f)
/* 旧加速度入口条件保留作参考，当前不再参与判定。 */
/* #define CYLINDER_CLIMB_AY_MAX_G           (-0.12f) */
/* #define CYLINDER_CLIMB_TAN_8_DEG          (0.1405f) */
#define CYLINDER_CLIMB_CONFIRM_SAMPLES       (3U)     /* 约15ms */

/* 不依赖陀螺积分的后备位置特征：顶部az为负，后半圈ay为正。 */
#define CYLINDER_TOP_AZ_MAX_G                (-0.70f)
#define CYLINDER_TOP_CONFIRM_SAMPLES         (5U)
#define CYLINDER_RETURN_HALF_AY_MIN_G        (0.50f)
#define CYLINDER_RETURN_HALF_CONFIRM_SAMPLES (5U)

/* 绕圆筒运动主要体现在gyro_x；积分只作粗略进度参考，不是精确角度。 */
#define CYLINDER_GYRO_X_DEADBAND_DPS         (1.0f)   /* 小于此值按零漂处理 */
#define CYLINDER_GYRO_SAMPLE_DT_S            (0.005f) /* IMU标称周期5ms */
#define CYLINDER_INSIDE_PROGRESS_DEG         (90.0f)  /* 进度参考，不是公开状态 */
#define CYLINDER_RETURN_HALF_PROGRESS_DEG    (210.0f) /* 后半圈保护参考 */
#define CYLINDER_EXIT_PROGRESS_DEG           (270.0f) /* 允许检查最终出口姿态 */
#define CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG (300.0f) /* 进入出口直道阶段 */

/* 出口最终姿态：pitch小于30度且az大于0.60g。 */
#define CYLINDER_EXIT_PITCH_MAX_DEG          (45.0f)
#define CYLINDER_EXIT_AZ_MIN_G               (0.60f)
#define CYLINDER_EXIT_IMU_CONFIRM_SAMPLES    (10U) /* 无电感辅助，约100ms */
#define CYLINDER_EXIT_IMU_WITH_EM_SAMPLES    (5U) /* 有电感辅助，约50ms */
#define CYLINDER_EXIT_CONFIRM_LATCH          (255U) /* 出口确认后的计数锁存值 */

/*
 * 短赛道稳定性测试用自动复位。
 * 退出后正立100个IMU帧即清零本轮状态；随后暂时锁住入口检测，
 * 入口特征消失8个ADC帧可提前解锁，否则200个ADC帧（约2s）后解锁。
 */
#define CYLINDER_TEST_AUTO_REARM_ENABLE       (1U)
#define CYLINDER_REARM_FLAT_CONFIRM_SAMPLES  (100U)
#define CYLINDER_REARM_FLAT_AY_ABS_MAX_G     (0.25f)
#define CYLINDER_REARM_FLAT_AZ_MIN_G         (0.80f)
#define CYLINDER_REARM_CLEAR_CONFIRM_SAMPLES  (8U)
#define CYLINDER_REARM_LOCKOUT_MAX_SAMPLES   (200U)

/* 出口电感只提供辅助证据，只能缩短IMU确认时间，不能单独判定退出。 */
#define CYLINDER_EXIT_OUTER_SUM_MAX          (5600U) /* 两只横向电感之和上限 */
#define CYLINDER_EXIT_LONGITUDINAL_SUM_MAX   (1000U) /* 两只纵向电感之和上限 */
#define CYLINDER_EXIT_EM_CONFIRM_SAMPLES     (5U)

/* 圆筒行为状态机，与car_control.c中的全局flag完全独立。 */
#define CYLINDER_STATE_IDLE                 (0U) /* 未识别圆筒 */
#define CYLINDER_STATE_PRE_ENTRY            (1U) /* 已识别入口，尚未登筒 */
#define CYLINDER_STATE_ON_CYLINDER          (2U) /* 已上筒，正在累计运动证据 */
#define CYLINDER_STATE_EXIT_STRAIGHT        (3U) /* 接近/离开出口的保护直道 */

void Cylinder_Init(void);

/* ---------- 入口ADC与IMU状态更新 ---------- */
/* 消费direction_adc_get()刚刚更新的ad_ave[]。 */
uint8 Cylinder_AdcUpdate(void);
/* activate=0时只累计入口证据，不进入PRE_ENTRY。 */
uint8 Cylinder_AdcCandidateUpdate(uint8 activate);
/* 激活已经确认的入口候选，不读取新ADC帧。 */
uint8 Cylinder_ActivateCandidate(void);
uint8 Cylinder_ImuUpdate(float ax_g, float ay_g, float az_g,
                         float gyro_x_dps, float pitch_deg);
uint8 Cylinder_ClimbSignalIsPresent(float pitch_deg);

/* ---------- 状态查询与入口转向保护 ---------- */
uint8 Cylinder_IsDetected(void);       /* 已识别入口且尚未退出 */
uint8 Cylinder_EntryIsDetected(void);  /* 本轮曾识别入口 */
uint8 Cylinder_IsOnSurface(void);      /* 当前是否仍在桶面 */
uint8 Cylinder_ExitInductanceIsReady(void);
uint8 Cylinder_HasExited(void);        /* 本轮是否已确认退出 */
uint8 Cylinder_ConsumeExitEvent(void);  /* 读取并清除一次性退出事件 */
uint8 Cylinder_IsEntryLockedOut(void); /* 自动重布防期间禁止新入口 */
uint8 Cylinder_GetState(void);         /* CYLINDER_STATE_xxx */
float Cylinder_GetRotationProgress(void); /* gyro_x绝对值积分，单位deg */
uint8 Cylinder_IsEntryLeftTurnGuardActive(void); /* PRE_ENTRY及入筒后的前90度禁止向左转 */
int16 Cylinder_LimitPreEntryDiff(int16 direction_diff);

void Cylinder_Reset(void); /* 清除状态、进度和确认计数 */

#endif /* __CYLINDER_H_ */
