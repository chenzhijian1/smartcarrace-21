#include "wall.h"
#include "spatial_features.h"

static volatile uint8 wall_state = WALL_STATE_IDLE;
static uint16 wall_baseline_count = 0;
static uint8 wall_baseline_seen = 0;
static uint16 wall_climb_count = 0;
static uint16 wall_vertical_count = 0;
static uint16 wall_lateral_count = 0;
static uint16 wall_descent_count = 0;
static uint16 wall_exit_count = 0;
static uint16 wall_candidate_age = 0;

static void wall_clear_candidate(void)
{
    wall_state = WALL_STATE_IDLE;
    wall_climb_count = 0;
    wall_vertical_count = 0;
    wall_lateral_count = 0;
    wall_descent_count = 0;
    wall_exit_count = 0;
    wall_candidate_age = 0;
}

void Wall_Reset(void)
{
    wall_baseline_count = 0;
    wall_baseline_seen = 0;
    wall_clear_candidate();
}

int16 Wall_CalcGravityFeedforward(float pitch_sin)
{
    if (wall_state == WALL_STATE_IDLE || wall_state == WALL_STATE_EXITED)
        return 0;

    if (pitch_sin > 1.0f)
        pitch_sin = 1.0f;
    else if (pitch_sin < -1.0f)
        pitch_sin = -1.0f;
    return (int16)(-WALL_GRAVITY_FF_PWM * pitch_sin);
}

void Wall_ImuUpdate(const imu_sample_t *sample,
                    float pitch_deg,
                    float roll_deg)
{
    uint8 climb_valid;
    uint8 vertical_valid;
    uint8 lateral_valid;
    uint8 descent_valid;
    uint8 flat;

    if (sample == (const imu_sample_t *)0)
        return;

    flat = (uint8)(pitch_deg >= WALL_BASELINE_PITCH_MIN_DEG &&
                   pitch_deg <= WALL_BASELINE_PITCH_MAX_DEG);

    if (wall_state == WALL_STATE_IDLE)
    {
        if (flat)
        {
            if (spatial_confirm_update(1,
                                       WALL_BASELINE_CONFIRM_SAMPLES,
                                       &wall_baseline_count))
            {
                wall_baseline_count = WALL_BASELINE_CONFIRM_SAMPLES;
                wall_baseline_seen = 1;
            }
        }
        else if (!wall_baseline_seen)
        {
            (void)spatial_confirm_update(0,
                                         WALL_BASELINE_CONFIRM_SAMPLES,
                                         &wall_baseline_count);
        }

        climb_valid = (uint8)(wall_baseline_seen &&
                              pitch_deg < WALL_ENTRY_PITCH_MAX_DEG);
        if (spatial_confirm_update(climb_valid,
                                   WALL_CLIMB_CONFIRM_SAMPLES,
                                   &wall_climb_count))
        {
            wall_state = WALL_STATE_CLIMB_CANDIDATE;
            wall_candidate_age = 0;
            wall_vertical_count = 0;
            wall_lateral_count = 0;
            wall_descent_count = 0;
            wall_exit_count = 0;
            wall_climb_count = 0;
        }
        return;
    }

    if (wall_state == WALL_STATE_EXITED)
        return;

    if (wall_candidate_age < 65535U)
        wall_candidate_age++;
    if (wall_candidate_age > WALL_MAX_CANDIDATE_SAMPLES)
    {
        wall_clear_candidate();
        return;
    }

    vertical_valid = (uint8)(pitch_deg < WALL_VERTICAL_PITCH_MAX_DEG);
    lateral_valid = (uint8)(spatial_absf(roll_deg) >
                            WALL_LATERAL_ROLL_MIN_DEG);
    descent_valid = (uint8)(pitch_deg > WALL_DESCENT_PITCH_MIN_DEG);

    if (wall_state == WALL_STATE_CLIMB_CANDIDATE)
    {
        if (flat)
        {
            wall_clear_candidate();
            return;
        }
        if (spatial_confirm_update(vertical_valid,
                                   WALL_VERTICAL_CONFIRM_SAMPLES,
                                   &wall_vertical_count))
        {
            wall_state = WALL_STATE_VERTICAL_PROVISIONAL;
            wall_vertical_count = 0;
            wall_lateral_count = 0;
            wall_descent_count = 0;
        }
        return;
    }

    if (wall_state == WALL_STATE_VERTICAL_PROVISIONAL)
    {
        if (spatial_confirm_update(lateral_valid,
                                   WALL_LATERAL_CONFIRM_SAMPLES,
                                   &wall_lateral_count))
        {
            wall_state = WALL_STATE_LATERAL;
            wall_lateral_count = 0;
            wall_descent_count = 0;
        }
        return;
    }

    if (wall_state == WALL_STATE_LATERAL)
    {
        if (spatial_confirm_update(descent_valid,
                                   WALL_DESCENT_CONFIRM_SAMPLES,
                                   &wall_descent_count))
        {
            wall_state = WALL_STATE_DESCENT;
            wall_descent_count = 0;
            wall_exit_count = 0;
        }
        return;
    }

    if (spatial_confirm_update(flat,
                               WALL_EXIT_CONFIRM_SAMPLES,
                               &wall_exit_count))
    {
        wall_state = WALL_STATE_EXITED;
        wall_exit_count = 0;
    }
    else if (!flat)
    {
        wall_exit_count = 0;
    }
}

uint8 Wall_HasExited(void)
{
    return (uint8)(wall_state == WALL_STATE_EXITED);
}

uint8 Wall_GetState(void)
{
    return wall_state;
}

static int16 wall_stage_target_from_percent(int16 current_speed,
                                            int16 straight_speed,
                                            uint8 percent)
{
    int32 straight_abs;
    int32 target_abs;

    if (straight_speed == 0)
        return current_speed;
    straight_abs = straight_speed < 0 ? -(int32)straight_speed : straight_speed;
    target_abs = straight_abs * (int32)percent / 100L;
    if (current_speed < 0 || (current_speed == 0 && straight_speed < 0))
        return (int16)-target_abs;
    return (int16)target_abs;
}

int16 Wall_GetSpeedTarget(int16 current_speed, int16 straight_speed)
{
    switch (wall_state)
    {
        case WALL_STATE_CLIMB_CANDIDATE:
        case WALL_STATE_VERTICAL_PROVISIONAL:
            return wall_stage_target_from_percent(current_speed,
                                                  straight_speed,
                                                  WALL_CLIMB_SPEED_PERCENT);
        case WALL_STATE_LATERAL:
            return wall_stage_target_from_percent(current_speed,
                                                  straight_speed,
                                                  WALL_LATERAL_SPEED_PERCENT);
        case WALL_STATE_DESCENT:
            return wall_stage_target_from_percent(current_speed,
                                                  straight_speed,
                                                  WALL_DESCENT_SPEED_PERCENT);
        default:
            return current_speed;
    }
}

void Wall_ClampWheelTargets(int16 center_speed,
                            int16 *left_speed,
                            int16 *right_speed)
{
    int32 center_abs;
    int32 high_abs;
    int16 low;
    int16 high;

    if (left_speed == (int16 *)0 || right_speed == (int16 *)0 ||
        center_speed == 0 || wall_state == WALL_STATE_IDLE ||
        wall_state == WALL_STATE_EXITED)
        return;

    center_abs = center_speed < 0 ? -(int32)center_speed : center_speed;
    high_abs = center_abs * 2L;
    if (center_speed > 0)
    {
        low = 0;
        high = (int16)high_abs;
    }
    else
    {
        low = (int16)-high_abs;
        high = 0;
    }

    *left_speed = spatial_clamp_i16(*left_speed, low, high);
    *right_speed = spatial_clamp_i16(*right_speed, low, high);
}
