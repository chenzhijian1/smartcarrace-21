#include "wall.h"
#include "spatial_features.h"

static volatile uint8 wall_state = WALL_STATE_IDLE;            // 墙壁状态机当前状态（IDLE→CLIMB_CANDIDATE→VERTICAL→LATERAL→DESCENT→EXITED）
static uint16 wall_baseline_count = 0;                          // 基线采集帧数计数器，用于统计进入墙壁前的平路姿态参考
static uint8 wall_baseline_seen = 0;                            // 基线是否已采集完成的标志
static uint16 wall_climb_count = 0;                             // 上坡阶段连续帧计数
static uint16 wall_vertical_count = 0;                          // 近竖直阶段连续帧计数
static uint16 wall_lateral_count = 0;                           // 横向阶段连续帧计数
static uint16 wall_descent_count = 0;                           // 下坡阶段连续帧计数
static uint16 wall_exit_count = 0;                              // 退出后回到平面阶段连续帧计数
static uint16 wall_candidate_age = 0;                           // 候选状态存活帧数，用于超时退回到IDLE

/* 主循环写非活动槽，读取方只访问活动槽。 */
static volatile int16 wall_gravity_ff_pwm[2] = {0, 0};
static volatile uint8 wall_gravity_ff_active_index = 0;

/* 清除本次候选的计数，保留基线由 Wall_Reset() 负责。 */
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

/* 完整清空墙面状态机和连续帧计数。 */
void Wall_Reset(void)
{
    wall_baseline_count = 0;
    wall_baseline_seen = 0;
    wall_clear_candidate();
    Wall_UpdateGravityFeedforward(0.0f);
}

/* 上电初始化包装函数；实际复位工作由 Wall_Reset() 完成。 */
void Wall_Init(void)
{
    Wall_Reset();
}

void Wall_UpdateGravityFeedforward(float pitch_sin)
{
    uint8 next_index;
    float gravity_pwm;

    if (!Wall_IsCandidate())
    {
        gravity_pwm = 0.0f;
    }
    else
    {
        if (pitch_sin > 1.0f)
            pitch_sin = 1.0f;
        else if (pitch_sin < -1.0f)
            pitch_sin = -1.0f;

        gravity_pwm = -WALL_GRAVITY_FF_PWM * pitch_sin;
    }

    next_index = (uint8)(wall_gravity_ff_active_index ^ 1U);
    wall_gravity_ff_pwm[next_index] = (int16)gravity_pwm;
    wall_gravity_ff_active_index = next_index;
}

int16 Wall_GetGravityFeedforwardPwm(void)
{
    uint8 index;

    index = wall_gravity_ff_active_index;
    return wall_gravity_ff_pwm[index];
}

/*
 * 主循环每个去重后的 IMU 样本调用一次，直接使用当前 ax/ay/az。
 * 依次识别：平面基线 -> 上坡 -> 近竖直 -> 横向 -> 下坡 -> 回平。
 * 只更新状态和控制快照，不直接写电机或风机；返回非零表示候选仍存在。
 */
uint8 Wall_ImuUpdate(const imu_sample_t *sample, float pitch_deg, float roll_deg)
{
    uint8 climb_valid;
    uint8 vertical_valid;
    uint8 lateral_valid;
    uint8 descent_valid;
    uint8 exit_valid;
    uint8 flat;

    if (sample == (const imu_sample_t *)0)
        return 0;

    flat = (uint8)(pitch_deg >= SPATIAL_BASELINE_PITCH_MIN_DEG &&
                   pitch_deg <= SPATIAL_BASELINE_PITCH_MAX_DEG);

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
            (void)spatial_confirm_update(
                0,
                WALL_BASELINE_CONFIRM_SAMPLES,
                &wall_baseline_count);
        }

        /* 与圆筒一致，使用euler.pitch<-5度确认上坡入口。 */
        climb_valid = (uint8)(
            wall_baseline_seen &&
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

        return (uint8)(wall_state != WALL_STATE_IDLE);
    }

    if (wall_state == WALL_STATE_EXITED)
        return 1;

    if (wall_candidate_age < 65535U)
        wall_candidate_age++;

    if (wall_candidate_age > WALL_MAX_CANDIDATE_SAMPLES)
    {
        wall_clear_candidate();
        return 0;
    }

    vertical_valid = (uint8)(pitch_deg < WALL_VERTICAL_PITCH_MAX_DEG);

    lateral_valid = (uint8)(spatial_absf(roll_deg) > WALL_LATERAL_ROLL_MIN_DEG);

    descent_valid = (uint8)(pitch_deg > WALL_DESCENT_PITCH_MIN_DEG);

    if (wall_state == WALL_STATE_CLIMB_CANDIDATE)
    {
        if (flat)
        {
            wall_clear_candidate();
            return 0;
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
        return 1;
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
        return 1;
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
        return 1;
    }

    exit_valid = (uint8)(flat);

    if (spatial_confirm_update(exit_valid,
                               WALL_EXIT_CONFIRM_SAMPLES,
                               &wall_exit_count))
    {
        wall_state = WALL_STATE_EXITED;
        wall_exit_count = 0;
    }
    else if (!exit_valid)
    {
        wall_exit_count = 0;
    }

    return 1;
}

/* 返回墙面进行中的阶段。 */
uint8 Wall_IsCandidate(void)
{
    return (uint8)(wall_state == WALL_STATE_CLIMB_CANDIDATE ||
                   wall_state == WALL_STATE_VERTICAL_PROVISIONAL ||
                   wall_state == WALL_STATE_LATERAL ||
                   wall_state == WALL_STATE_DESCENT);
}

/* 进入横向后确认墙面，供元素管理器提交该路线元素。 */
uint8 Wall_IsConfirmed(void)
{
    return (uint8)(wall_state == WALL_STATE_LATERAL ||
                   wall_state == WALL_STATE_DESCENT ||
                   wall_state == WALL_STATE_EXITED);
}

/* 仅在 EXITED 时返回 1，供元素管理器释放墙面控制并推进路线。 */
uint8 Wall_HasExited(void)
{
    return (uint8)(wall_state == WALL_STATE_EXITED);
}

/* 返回当前状态字节，供控制中断和调试打印读取。 */
uint8 Wall_GetState(void)
{
    return wall_state;
}

/* Build an active wall-stage target from the configured straight speed. */
static int16 wall_stage_target_from_percent(int16 current_speed,
                                            int16 straight_speed,
                                            uint8 percent)
{
    int32 straight_abs;
    int32 target_abs;

    if (straight_speed == 0)
        return current_speed;

    straight_abs = straight_speed < 0 ?
        -(int32)straight_speed : straight_speed;
    target_abs = straight_abs * (int32)percent / 100L;

    if (current_speed < 0 || (current_speed == 0 && straight_speed < 0))
        return (int16)-target_abs;
    return (int16)target_abs;
}

/* TIM4 control target: active wall stages may accelerate to their stage target. */
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

/* 将单个轮速目标限制在闭区间内。 */
/*
 * TIM4 在 speed_adjust() 后调用，禁止墙面进行中单轮目标反向。
 * 正向范围是 [0, 2*center]，倒车范围是 [-2*abs(center), 0]。
 */
void Wall_ClampWheelTargets(int16 center_speed,
                            int16 *left_speed,
                            int16 *right_speed)
{
    int32 center_abs;
    int32 high_abs;
    int16 low;
    int16 high;

    if (left_speed == 0 || right_speed == 0 ||
        center_speed == 0 ||
        wall_state == WALL_STATE_IDLE ||
        wall_state == WALL_STATE_EXITED)
        return;

    center_abs = center_speed < 0 ? -(int32)center_speed : center_speed;
    high_abs = center_abs * 2L;

    if (center_speed > 0)
    {
        low = 0;
        high = (int16)high_abs;
        *left_speed = spatial_clamp_i16(*left_speed, low, high);
        *right_speed = spatial_clamp_i16(*right_speed, low, high);
    }
    else
    {
        low = (int16)-high_abs;
        high = 0;
        *left_speed = spatial_clamp_i16(*left_speed, low, high);
        *right_speed = spatial_clamp_i16(*right_speed, low, high);
    }
}

/* 当前横向阶段无差速前馈，接口保留，始终返回 0。 */
int16 Wall_GetDirectionBias(void)
{
    return 0;
}
