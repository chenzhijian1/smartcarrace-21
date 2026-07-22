#ifndef __WALL_H_
#define __WALL_H_

#include "zf_common_typedef.h"
#include "quaternion.h"

#define WALL_BASELINE_PITCH_MIN_DEG       (-8.0f)
#define WALL_BASELINE_PITCH_MAX_DEG       (8.0f)
#define WALL_BASELINE_CONFIRM_SAMPLES     (5U)
#define WALL_ENTRY_PITCH_MAX_DEG          (-13.0f)
#define WALL_CLIMB_CONFIRM_SAMPLES        (2U)
#define WALL_VERTICAL_PITCH_MAX_DEG       (-40.0f)
#define WALL_VERTICAL_CONFIRM_SAMPLES     (3U)
#define WALL_LATERAL_ROLL_MIN_DEG         (50.0f)
#define WALL_LATERAL_CONFIRM_SAMPLES      (3U)
#define WALL_DESCENT_PITCH_MIN_DEG        (40.0f)
#define WALL_DESCENT_CONFIRM_SAMPLES      (3U)
#define WALL_EXIT_CONFIRM_SAMPLES         (50U)
#define WALL_MAX_CANDIDATE_SAMPLES        (500U)

#define WALL_CLIMB_SPEED_PERCENT          (100U)
#define WALL_LATERAL_SPEED_PERCENT        (125U)
#define WALL_DESCENT_SPEED_PERCENT        (150U)
#define WALL_GRAVITY_FF_PWM               (1600.0f)

#define WALL_STATE_IDLE                   (0U)
#define WALL_STATE_CLIMB_CANDIDATE        (1U)
#define WALL_STATE_VERTICAL_PROVISIONAL   (2U)
#define WALL_STATE_LATERAL                (3U)
#define WALL_STATE_DESCENT                (4U)
#define WALL_STATE_EXITED                 (5U)

void Wall_Reset(void);
void Wall_ImuUpdate(const imu_sample_t *sample,
                    float pitch_deg,
                    float roll_deg);
uint8 Wall_HasExited(void);
uint8 Wall_GetState(void);
int16 Wall_GetSpeedTarget(int16 current_speed, int16 straight_speed);
void Wall_ClampWheelTargets(int16 center_speed,
                            int16 *left_speed,
                            int16 *right_speed);
int16 Wall_CalcGravityFeedforward(float pitch_sin);

#endif
