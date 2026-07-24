#ifndef __CYLINDER_H_
#define __CYLINDER_H_

#include "zf_common_typedef.h"

#define CYLINDER_DETECT_OUTER_SUM_MIN       (3800U)
#define CYLINDER_DETECT_TOTAL_SUM_MIN       (4200U)
#define CYLINDER_DETECT_OUTER_SINGLE_MIN    (1100U)
#define CYLINDER_DETECT_CONFIRM_SAMPLES     (2U)
#define CYLINDER_PRE_ENTRY_INWARD_DIFF_MAX  (3)
#define CYLINDER_PRE_ENTRY_OUTWARD_DIFF_MAX (160)
#define CYLINDER_ENTRY_LEFT_GUARD_DEG        (0.0f)

#define CYLINDER_GRAVITY_FF_PWM              (1700.0f)
#define CYLINDER_TOP_SPEED_PERCENT           (100U)
#define CYLINDER_SATURATION_PWM_THRESHOLD    (9500)
#define CYLINDER_SATURATION_ERROR_THRESHOLD  (80)
#define CYLINDER_SATURATION_CONFIRM_TICKS    (4U)
#define CYLINDER_SATURATION_DIFF_PERCENT     (90L)

#define CYLINDER_ENTRY_PITCH_MAX_DEG         (-5.0f)
#define CYLINDER_CLIMB_CONFIRM_SAMPLES       (3U)
#define CYLINDER_TOP_AZ_MAX_G                (-0.70f)
#define CYLINDER_TOP_CONFIRM_SAMPLES         (5U)
#define CYLINDER_RETURN_HALF_AY_MIN_G        (0.50f)
#define CYLINDER_RETURN_HALF_CONFIRM_SAMPLES (5U)
#define CYLINDER_GYRO_X_DEADBAND_DPS         (1.0f)
#define CYLINDER_GYRO_SAMPLE_DT_S            (0.005f)
#define CYLINDER_INSIDE_PROGRESS_DEG         (90.0f)
#define CYLINDER_RETURN_HALF_PROGRESS_DEG    (210.0f)
#define CYLINDER_EXIT_PROGRESS_DEG           (270.0f)
#define CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG  (300.0f)

#define CYLINDER_EXIT_GYRO_X_ABS_MAX_DPS     (30.0f)
#define CYLINDER_EXIT_PITCH_ABS_MAX_DEG      (10.0f)
#define CYLINDER_EXIT_AZ_MIN_G               (0.75f)
#define CYLINDER_EXIT_IMU_CONFIRM_SAMPLES    (5U)
#define CYLINDER_EXIT_DISTANCE               (400.0f)
#define CYLINDER_EXIT_CONFIRM_LATCH          (30U)

#define CYLINDER_TEST_AUTO_REARM_ENABLE      (1U)
#define CYLINDER_REARM_FLAT_CONFIRM_SAMPLES  (5U)
#define CYLINDER_REARM_CLEAR_CONFIRM_SAMPLES (2U)
#define CYLINDER_REARM_LOCKOUT_MAX_SAMPLES   (15U)

#define CYLINDER_STATE_IDLE                  (0U)
#define CYLINDER_STATE_PRE_ENTRY             (1U)
#define CYLINDER_STATE_ON_CYLINDER           (2U)
#define CYLINDER_STATE_EXIT_STRAIGHT         (3U)

void Cylinder_AdcUpdate(void);
uint8 Cylinder_ImuUpdate(float ay_g, float az_g,
                         float gyro_x_dps, float pitch_deg, float encoder);
uint8 Cylinder_EntryIsDetected(void);
uint8 Cylinder_IsOnSurface(void);
uint8 Cylinder_HasExited(void);
uint8 Cylinder_GetState(void);
uint8 Cylinder_IsEntryLeftTurnGuardActive(void);
int16 Cylinder_LimitPreEntryDiff(int16 direction_diff);
int16 Cylinder_GetSpeedTarget(int16 current_speed, int16 straight_speed);
int16 Cylinder_CalcGravityFeedforward(float pitch_sin);
void Cylinder_Reset(void);

#endif
