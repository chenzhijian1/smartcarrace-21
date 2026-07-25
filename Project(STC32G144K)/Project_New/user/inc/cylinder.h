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
#define CYLINDER_DETECT_CONFIRM_SAMPLES     (4U)    /* 4帧约6cm */

/*
 * 从圆筒右侧进入时的PRE_ENTRY差速限制。
 * changed_speed为正表示向左（桶内）转，为负表示向右（桶外）转。
 * 左右轮目标分别为base-diff和base+diff，所以实际轮速差是2*diff。
 */
#define CYLINDER_PRE_ENTRY_INWARD_DIFF_MAX   (3)   /* 入口左转上限 */
#define CYLINDER_PRE_ENTRY_OUTWARD_DIFF_MAX  (160) /* 允许向右修正的最大差速 */
#define CYLINDER_ENTRY_LEFT_GUARD_DEG         (0.0f) /* 入口左转保护解除角度，进度到达后允许左转 */

#define CYLINDER_GRAVITY_FF_PWM                (1680.0f) /* 圆筒重力前馈PWM基值 */
#define CYLINDER_TOP_SPEED_PERCENT             (100U)    /* 圆筒顶部目标为普通设定速度的120% */
#define CYLINDER_SATURATION_PWM_THRESHOLD      (9500)    /* 电机饱和检测：PWM占空比阈值 */
#define CYLINDER_SATURATION_ERROR_THRESHOLD    (80)      /* 电机饱和检测：误差阈值 */
#define CYLINDER_SATURATION_CONFIRM_TICKS      (4U)      /* 电机饱和检测：连续确认拍数 */
#define CYLINDER_SATURATION_DIFF_PERCENT       (90L)     /* 电机饱和后保留的差速百分比（90%） */

/* 开始登筒：车辆pitch连续小于-5度。 */
#define CYLINDER_ENTRY_PITCH_MAX_DEG          (-15.0f)
#define CYLINDER_CLIMB_CONFIRM_SAMPLES       (3U)     /* 5帧约7.5cm */

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

/* 出口最终姿态：车身接近水平（pitch小、az接近1g）、角速度小。 */
#define CYLINDER_EXIT_GYRO_X_ABS_MAX_DPS     (80.0f)  /* gyro_x绝对值上限，角速度需足够小 */
#define CYLINDER_EXIT_PITCH_MAX_DEG          (60.0f)  /* 俯仰角上限，车头需接近水平 */
#define CYLINDER_EXIT_AZ_MIN_G               (0.60f)   /* Z轴加速度下限，车身需接近正立 */
#define CYLINDER_EXIT_IMU_CONFIRM_SAMPLES    (4U)      /* 出口姿态连续确认帧数，4帧约6cm */
#define CYLINDER_LEAVE_ENCODER_DELTA           (400.0f) /* 进入状态3后，编码器前进差值达到该值才完成 */
#define CYLINDER_EXIT_CONFIRM_LATCH          (30U)     /* 出口确认锁存值，用于HasExited()判断 */

/*
 * 短赛道稳定性测试用自动复位。
 * 退出后正立100个IMU帧即清零本轮状态；随后暂时锁住入口检测，
 * 入口特征消失8个ADC帧可提前解锁，否则200个ADC帧（约2s）后解锁。
 */
#define CYLINDER_TEST_AUTO_REARM_ENABLE       (1U)
#define CYLINDER_REARM_FLAT_CONFIRM_SAMPLES  (5U)  /* 自动复位：正立连续确认帧数 */
#define CYLINDER_REARM_CLEAR_CONFIRM_SAMPLES  (2U)  /* 自动复位：入口信号消失解除锁定的帧数 */
#define CYLINDER_REARM_LOCKOUT_MAX_SAMPLES   (15U)  /* 自动复位：入口锁定超时帧数 */

/* 圆筒行为状态机，与car_control.c中的全局flag完全独立。 */
#define CYLINDER_STATE_IDLE                 (0U) /* 未识别圆筒 */
#define CYLINDER_STATE_PRE_ENTRY            (1U) /* 已识别入口，尚未登筒 */
#define CYLINDER_STATE_ON_CYLINDER          (2U) /* 已上筒，正在累计运动证据 */
#define CYLINDER_STATE_EXIT_STRAIGHT        (3U) /* 接近/离开出口的保护直道 */

void Cylinder_Init(void);

/* ---------- 入口ADC与IMU状态更新 ---------- */
/* 消费direction_adc_get()刚刚更新的ad_ave[]，不引入额外电感帧结构。 */
uint8 Cylinder_AdcUpdate(void);
uint8 Cylinder_ImuUpdate(float ay_g, float az_g,
                         float gyro_x_dps, float pitch_deg);

/* ---------- 状态查询与入口转向保护 ---------- */
uint8 Cylinder_EntryIsDetected(void);  /* 本轮曾识别入口 */
uint8 Cylinder_IsOnSurface(void);      /* 当前是否仍在桶面 */
uint8 Cylinder_HasExited(void);        /* 本轮是否已确认退出 */
uint8 Cylinder_GetState(void);         /* CYLINDER_STATE_xxx */
uint8 Cylinder_IsEntryLeftTurnGuardActive(void);
int16 Cylinder_LimitPreEntryDiff(int16 direction_diff);
int16 Cylinder_GetSpeedTarget(int16 current_speed, int16 straight_speed);
void Cylinder_UpdateGravityFeedforward(float pitch_sin);
int16 Cylinder_GetGravityFeedforwardPwm(void);

void Cylinder_Reset(void); /* 清除状态、进度和确认计数 */

#endif /* __CYLINDER_H_ */
