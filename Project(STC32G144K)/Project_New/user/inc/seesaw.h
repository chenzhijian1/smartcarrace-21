#ifndef __SEESAW_H_
#define __SEESAW_H_

#include "zf_common_typedef.h"
#include "quaternion.h"

#define SEESAW_TILT_ENTER_DEG             (8.0f)
#define SEESAW_TILT_MIN_PEAK_DEG          (15.0f)
#define SEESAW_TILT_MAX_DEG               (90.0f)
#define SEESAW_BASELINE_PITCH_MIN_DEG     (-8.0f)
#define SEESAW_BASELINE_PITCH_MAX_DEG     (8.0f)
#define SEESAW_BASELINE_CONFIRM_SAMPLES   (3U)
#define SEESAW_ENTER_CONFIRM_SAMPLES      (2U)
#define SEESAW_MAX_CANDIDATE_SAMPLES      (20U)

#define SEESAW_SPEED_SLOW_PERCENT         (50U)
#define SEESAW_SPEED_CRAWL_MIN            (100)
#define SEESAW_SLOW_WHEEL_LOW_PERCENT     (50U)
#define SEESAW_SLOW_WHEEL_HIGH_PERCENT    (150U)

#define SEESAW_STATE_IDLE                 (0U)
#define SEESAW_STATE_RISING               (1U)
#define SEESAW_STATE_EXITED               (4U)

void Seesaw_Reset(void);
void Seesaw_ImuUpdate(const imu_sample_t *sample, float pitch_deg);
uint8 Seesaw_HasExited(void);
uint8 Seesaw_GetState(void);
int16 Seesaw_GetSpeedTarget(int16 current_speed, int16 straight_speed);
void Seesaw_ClampWheelTargets(int16 center_speed,
                              int16 *left_speed,
                              int16 *right_speed);

#endif
