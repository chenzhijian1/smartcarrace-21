#include "seesaw.h"
#include "spatial_features.h"

static volatile uint8 seesaw_state = SEESAW_STATE_IDLE;
static uint16 seesaw_baseline_count = 0;
static uint8 seesaw_baseline_seen = 0;
static uint16 seesaw_enter_count = 0;
static uint16 seesaw_candidate_age = 0;
static float seesaw_peak_tilt_deg = 0.0f;

static float seesaw_rising_tilt_deg(float pitch_deg)
{
    return pitch_deg < 0.0f ? -pitch_deg : 0.0f;
}

static void seesaw_clear_candidate(void)
{
    seesaw_state = SEESAW_STATE_IDLE;
    seesaw_enter_count = 0;
    seesaw_candidate_age = 0;
    seesaw_peak_tilt_deg = 0.0f;
}

static void seesaw_complete_candidate(void)
{
    seesaw_state = SEESAW_STATE_EXITED;
    seesaw_enter_count = 0;
    seesaw_candidate_age = 0;
    seesaw_peak_tilt_deg = 0.0f;
}

void Seesaw_Reset(void)
{
    seesaw_baseline_count = 0;
    seesaw_baseline_seen = 0;
    seesaw_clear_candidate();
}

void Seesaw_ImuUpdate(const imu_sample_t *sample, float pitch_deg)
{
    float tilt_deg;
    uint8 tilt_valid;
    uint8 norm_valid;
    uint8 flat;

    if (sample == (const imu_sample_t *)0)
        return;

    norm_valid = spatial_accel_vector_norm_in_range(
        sample->ax_g, sample->ay_g, sample->az_g,
        SPATIAL_NORM_MIN_G, SPATIAL_NORM_MAX_G);
    flat = (uint8)(norm_valid &&
                   pitch_deg >= SEESAW_BASELINE_PITCH_MIN_DEG &&
                   pitch_deg <= SEESAW_BASELINE_PITCH_MAX_DEG);

    if (seesaw_state == SEESAW_STATE_IDLE)
    {
        if (flat)
        {
            if (spatial_confirm_update(1,
                                       SEESAW_BASELINE_CONFIRM_SAMPLES,
                                       &seesaw_baseline_count))
            {
                seesaw_baseline_count = SEESAW_BASELINE_CONFIRM_SAMPLES;
                seesaw_baseline_seen = 1;
            }
        }
        else if (!seesaw_baseline_seen)
        {
            (void)spatial_confirm_update(0,
                                         SEESAW_BASELINE_CONFIRM_SAMPLES,
                                         &seesaw_baseline_count);
        }

        tilt_deg = seesaw_rising_tilt_deg(pitch_deg);
        tilt_valid = (uint8)(seesaw_baseline_seen && norm_valid &&
                             tilt_deg >= SEESAW_TILT_ENTER_DEG &&
                             tilt_deg <= SEESAW_TILT_MAX_DEG);

        if (spatial_confirm_update(tilt_valid,
                                   SEESAW_ENTER_CONFIRM_SAMPLES,
                                   &seesaw_enter_count))
        {
            seesaw_state = SEESAW_STATE_RISING;
            seesaw_candidate_age = 0;
            seesaw_peak_tilt_deg = tilt_deg;
            seesaw_enter_count = 0;
        }
        return;
    }

    if (seesaw_state == SEESAW_STATE_EXITED)
        return;

    if (seesaw_candidate_age < 65535U)
        seesaw_candidate_age++;

    if (seesaw_candidate_age > SEESAW_MAX_CANDIDATE_SAMPLES)
    {
        seesaw_complete_candidate();
        return;
    }

    if (pitch_deg > 0.0f)
    {
        seesaw_complete_candidate();
        return;
    }

    if (!norm_valid)
        return;

    if (pitch_deg > SEESAW_TILT_MAX_DEG ||
        pitch_deg < -SEESAW_TILT_MAX_DEG)
    {
        seesaw_clear_candidate();
        return;
    }

    tilt_deg = seesaw_rising_tilt_deg(pitch_deg);
    if (tilt_deg > seesaw_peak_tilt_deg)
        seesaw_peak_tilt_deg = tilt_deg;

    if (flat && seesaw_peak_tilt_deg < SEESAW_TILT_MIN_PEAK_DEG)
        seesaw_clear_candidate();
}

uint8 Seesaw_HasExited(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_EXITED);
}

uint8 Seesaw_GetState(void)
{
    return seesaw_state;
}

uint8 Seesaw_DebugBrakeRequested(void)
{
#if SEESAW_RISING_BRAKE_DEBUG
    return (uint8)(seesaw_state == SEESAW_STATE_RISING);
#else
    return 0;
#endif
}

static int16 seesaw_crawl_target(int16 current_speed, int16 straight_speed)
{
    int32 current_abs;
    int32 straight_abs;
    int32 cap_abs;

    if (straight_speed == 0)
        return current_speed;

    straight_abs = straight_speed < 0 ? -(int32)straight_speed : straight_speed;
    current_abs = current_speed < 0 ? -(int32)current_speed : current_speed;
    cap_abs = straight_abs * SEESAW_SPEED_SLOW_PERCENT / 100L;

    if (current_abs <= cap_abs)
        return current_speed;
    return current_speed < 0 ? (int16)-cap_abs : (int16)cap_abs;
}

int16 Seesaw_GetSpeedTarget(int16 current_speed, int16 straight_speed)
{
    if (seesaw_state != SEESAW_STATE_RISING)
        return current_speed;
    return seesaw_crawl_target(current_speed, straight_speed);
}

void Seesaw_ClampWheelTargets(int16 center_speed,
                              int16 *left_speed,
                              int16 *right_speed)
{
    int32 center_abs;
    int32 low_abs;
    int32 high_abs;
    int16 low;
    int16 high;

    if (left_speed == (int16 *)0 || right_speed == (int16 *)0 ||
        seesaw_state != SEESAW_STATE_RISING || center_speed == 0)
        return;

    center_abs = center_speed < 0 ? -(int32)center_speed : center_speed;
    low_abs = center_abs * SEESAW_SLOW_WHEEL_LOW_PERCENT / 100L;
    high_abs = center_abs * SEESAW_SLOW_WHEEL_HIGH_PERCENT / 100L;
    if (center_abs >= SEESAW_SPEED_CRAWL_MIN &&
        low_abs < SEESAW_SPEED_CRAWL_MIN)
        low_abs = SEESAW_SPEED_CRAWL_MIN;

    if (center_speed > 0)
    {
        low = (int16)low_abs;
        high = (int16)high_abs;
    }
    else
    {
        low = (int16)-high_abs;
        high = (int16)-low_abs;
    }

    *left_speed = spatial_clamp_i16(*left_speed, low, high);
    *right_speed = spatial_clamp_i16(*right_speed, low, high);
}
