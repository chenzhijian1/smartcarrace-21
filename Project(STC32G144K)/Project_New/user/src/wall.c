#include "wall.h"

static volatile uint8 wall_state = WALL_STATE_IDLE;
static uint16 wall_baseline_count = 0;
static uint8 wall_baseline_seen = 0;
static uint16 wall_climb_count = 0;
static uint16 wall_vertical_count = 0;
static uint16 wall_lateral_count = 0;
static uint16 wall_descent_count = 0;
static uint16 wall_exit_count = 0;
static uint16 wall_candidate_age = 0;
static uint8 wall_norm_invalid_count = 0;
static uint8 wall_lateral_seen = 0;
static volatile int8 wall_lateral_side = 0;

/* 浮点绝对值工具，供横向和下坡的分量阈值比较使用。 */
static float wall_absf(float value)
{
    return value >= 0.0f ? value : -value;
}

/* 清除本次候选的计数和横向侧别证据，保留基线由 Wall_Reset() 负责。 */
static void wall_clear_candidate(void)
{
    wall_state = WALL_STATE_IDLE;
    wall_climb_count = 0;
    wall_vertical_count = 0;
    wall_lateral_count = 0;
    wall_descent_count = 0;
    wall_exit_count = 0;
    wall_candidate_age = 0;
    wall_norm_invalid_count = 0;
    wall_lateral_seen = 0;
    wall_lateral_side = 0;
}

/* 完整清空墙面状态机、连续帧计数和已锁定的横向方向。 */
void Wall_Reset(void)
{
    wall_baseline_count = 0;
    wall_baseline_seen = 0;
    wall_clear_candidate();
}

/* 上电初始化包装函数；实际复位工作由 Wall_Reset() 完成。 */
void Wall_Init(void)
{
    Wall_Reset();
}

/*
 * 主循环每个去重后的 IMU 特征帧调用一次。
 * 依次识别：平面基线 -> 上坡 -> 近竖直 -> 轮轴方向横向 -> 下坡 -> 回平。
 * 只更新状态和控制快照，不直接写电机或风机；返回非零表示候选仍存在。
 */
uint8 Wall_ImuUpdate(const spatial_features_t *features)
{
    uint8 climb_valid;
    uint8 vertical_valid;
    uint8 lateral_valid;
    uint8 descent_valid;
    uint8 exit_valid;

    if (features == (const spatial_features_t *)0)
        return 0;

    if (wall_state == WALL_STATE_IDLE)
    {
        if (features->flat)
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

        /* 使用低通ay、az经atan2得到的上仰角确认墙面上坡入口。 */
        climb_valid = (uint8)(
            wall_baseline_seen &&
            features->norm_valid &&
            features->climb_angle_deg >= WALL_CLIMB_ENTER_DEG &&
            features->az_lowpass_g > WALL_VERTICAL_AZ_MAX_G);

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

    if (!features->norm_valid)
    {
        if (wall_norm_invalid_count < WALL_NORM_INVALID_GRACE_SAMPLES)
            wall_norm_invalid_count++;
        if (wall_norm_invalid_count >= WALL_NORM_INVALID_GRACE_SAMPLES)
        {
            wall_clear_candidate();
            return 0;
        }
        return 1;
    }
    wall_norm_invalid_count = 0;

    if (features->az_lowpass_g <= WALL_INVERTED_AZ_MAX_G ||
        features->inverted)
    {
        /* 在回平前出现真正倒置更像圆筒翻转轨迹，不符合墙面模型。 */
        wall_clear_candidate();
        return 0;
    }

    vertical_valid = (uint8)(
        features->ay_lowpass_g <= WALL_VERTICAL_AY_MAX_G &&
        features->az_lowpass_g <= WALL_VERTICAL_AZ_MAX_G &&
        features->az_lowpass_g > WALL_INVERTED_AZ_MAX_G &&
        wall_absf(features->ax_lowpass_g) <= WALL_VERTICAL_AX_MAX_G);

    lateral_valid = (uint8)(
        features->norm_valid &&
        wall_absf(features->ax_lowpass_g) >= WALL_LATERAL_AX_MIN_G &&
        wall_absf(features->ay_lowpass_g) <= WALL_LATERAL_AY_MAX_G &&
        wall_absf(features->az_lowpass_g) <= WALL_LATERAL_AZ_MAX_G);

    if (wall_state == WALL_STATE_CLIMB_CANDIDATE)
    {
        if (features->flat)
        {
            /* 车辆在确认墙面前已经回到地面，撤销本次墙面候选。 */
            wall_clear_candidate();
            return 0;
        }

        if (spatial_confirm_update(vertical_valid,
                                   WALL_VERTICAL_CONFIRM_SAMPLES,
                                   &wall_vertical_count))
        {
            /* 近竖直仍只是临时状态，因为圆筒也会经过这里；还要等待轮轴方向重力。 */
            wall_state = WALL_STATE_VERTICAL_PROVISIONAL;
            wall_vertical_count = 0;
            wall_lateral_count = 0;
        }
        return 1;
    }

    if (wall_state == WALL_STATE_VERTICAL_PROVISIONAL)
    {
        if (features->flat)
        {
            wall_clear_candidate();
            return 0;
        }

        if (spatial_confirm_update(lateral_valid,
                                   WALL_LATERAL_CONFIRM_SAMPLES,
                                   &wall_lateral_count))
        {
            wall_state = WALL_STATE_LATERAL;
            wall_lateral_seen = 1;
            wall_lateral_side = features->ax_lowpass_g >= 0.0f ? 1 : -1;
            wall_lateral_count = 0;
            wall_descent_count = 0;
        }
        return 1;
    }

    if (wall_state == WALL_STATE_LATERAL)
    {
        descent_valid = (uint8)(
            wall_lateral_seen &&
            features->ay_lowpass_g >= WALL_DESCENT_AY_MIN_G &&
            wall_absf(features->ax_lowpass_g) <= WALL_DESCENT_AX_MAX_G &&
            features->az_lowpass_g > WALL_INVERTED_AZ_MAX_G);

        if (spatial_confirm_update(descent_valid,
                                   WALL_DESCENT_CONFIRM_SAMPLES,
                                   &wall_descent_count))
        {
            wall_state = WALL_STATE_DESCENT;
            wall_descent_count = 0;
            wall_exit_count = 0;
        }
        else if (features->flat && wall_lateral_seen)
        {
            /* 对很短或有噪声的下坡过渡提供后备判断：一旦轮轴重力确认墙面，
             * 稳定回到地面就足以释放墙面控制。 */
            if (spatial_confirm_update(1,
                                       WALL_EXIT_CONFIRM_SAMPLES,
                                       &wall_exit_count))
            {
                wall_state = WALL_STATE_EXITED;
                wall_exit_count = 0;
            }
        }
        else
        {
            wall_exit_count = 0;
        }
        return 1;
    }

    /* 只有经过横向阶段后的下坡流程才能声明墙面完成。 */
    exit_valid = (uint8)(
        wall_lateral_seen &&
        features->flat &&
        features->ay_lowpass_g <= WALL_EXIT_AY_MAX_G &&
        features->ay_lowpass_g >= -WALL_EXIT_AY_MAX_G &&
        features->az_lowpass_g >= WALL_EXIT_AZ_MIN_G &&
        !features->inverted);

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

/* 返回墙面进行中的四个阶段；LATERAL/DESCENT 已有较强身份依据但未退出。 */
uint8 Wall_IsCandidate(void)
{
    return (uint8)(wall_state == WALL_STATE_CLIMB_CANDIDATE ||
                   wall_state == WALL_STATE_VERTICAL_PROVISIONAL ||
                   wall_state == WALL_STATE_LATERAL ||
                   wall_state == WALL_STATE_DESCENT);
}

/* 看到轮轴方向重力后确认墙面，供元素管理器提交该路线元素。 */
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

/* 按普通直线参考目标计算阶段速度上限，保留当前目标的正负号。 */
static int16 wall_target_from_percent(int16 current_speed,
                                      int16 straight_speed,
                                      uint8 percent)
{
    int32 current_abs;
    int32 straight_abs;
    int32 cap_abs;

    if (straight_speed == 0)
        return current_speed;

    straight_abs = straight_speed < 0 ?
        -(int32)straight_speed : straight_speed;
    current_abs = current_speed < 0 ? -(int32)current_speed : current_speed;
    cap_abs = straight_abs * (int32)percent / 100L;

    if (current_abs <= cap_abs)
        return current_speed;

    return current_speed < 0 ? (int16)-cap_abs : (int16)cap_abs;
}

/* TIM4 控制中断调用：按上坡/竖直/横向阶段选择速度上限。 */
int16 Wall_GetSpeedTarget(int16 current_speed, int16 straight_speed)
{
    switch (wall_state)
    {
    case WALL_STATE_CLIMB_CANDIDATE:
        return wall_target_from_percent(current_speed,
                                        straight_speed,
                                        WALL_CLIMB_SPEED_PERCENT);

    case WALL_STATE_VERTICAL_PROVISIONAL:
        return wall_target_from_percent(current_speed,
                                        straight_speed,
                                        WALL_VERTICAL_SPEED_PERCENT);

    case WALL_STATE_LATERAL:
        return wall_target_from_percent(current_speed,
                                        straight_speed,
                                        WALL_LATERAL_SPEED_PERCENT);

    case WALL_STATE_DESCENT:
#if WALL_DESCENT_SPEED_ENABLE
        return wall_target_from_percent(current_speed,
                                        straight_speed,
                                        WALL_DESCENT_SPEED_PERCENT);
#else
        return current_speed;
#endif

    default:
        return current_speed;
    }
}

/* 将单个轮速目标限制在闭区间内。 */
static int16 wall_clamp_i16(int16 value, int16 low, int16 high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

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
        *left_speed = wall_clamp_i16(*left_speed, low, high);
        *right_speed = wall_clamp_i16(*right_speed, low, high);
    }
    else
    {
        low = (int16)-high_abs;
        high = 0;
        *left_speed = wall_clamp_i16(*left_speed, low, high);
        *right_speed = wall_clamp_i16(*right_speed, low, high);
    }
}

/*
 * 返回横向阶段的抗重力固定差速前馈；默认关闭，未标定时始终返回 0。
 */
int16 Wall_GetDirectionBias(void)
{
#if WALL_DIRECTION_BIAS_ENABLE
    if (wall_state == WALL_STATE_LATERAL && wall_lateral_side != 0)
    {
        return (int16)(wall_lateral_side *
                       WALL_DIRECTION_BIAS_SIGN *
                       WALL_DIRECTION_BIAS_VALUE);
    }
#endif
    return 0;
}
