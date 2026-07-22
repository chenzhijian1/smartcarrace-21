#include "cylinder.h"
#include "inductance.h"
#include "spatial_features.h"

/* 入口由新ADC帧累计确认，达到阈值后进入PRE_ENTRY。 */
static uint16 cylinder_detect_count = 0;

/* 对外阶段只由cylinder_state表示，查询函数均从状态推导。 */
static volatile uint8 cylinder_state = CYLINDER_STATE_IDLE;

/* 由主循环更新、控制中断只读，避免中断直接读取多字节float进度。 */
static volatile uint8 cylinder_left_guard_active = 0;
static volatile uint8 cylinder_speed_percent = 100U;

/* 各阶段独立连续帧计数，条件中断时清零。 */
static uint16 cylinder_climb_count = 0;
static uint16 cylinder_top_count = 0;
static uint16 cylinder_return_half_count = 0;
static uint16 cylinder_exit_imu_count = 0;

/* 一旦见过顶部或后半圈便锁存，确保入口处正立姿态不会被当成出口。 */
static uint8 cylinder_top_seen = 0;
static uint8 cylinder_return_half_seen = 0;

/* 短赛道测试用自动重布防状态。 */
static uint16 cylinder_rearm_flat_count = 0;
static uint16 cylinder_rearm_clear_count = 0;
static uint8 cylinder_entry_lockout = 0;
static uint16 cylinder_rearm_lockout_count = 0;

/* 圆筒位置判定使用的粗略运动进度，不属于桶面转向控制器输出。 */
static float cylinder_rotation_progress_deg = 0.0f; /* |gyro_x|积分，0到360度 */
static float cylinder_yaw_leave_deg = 0.0f;         /* 首次进入状态3时的连续航向角 */

static volatile int16 cylinder_gravity_ff_pwm[2] = {0, 0};
static volatile uint8 cylinder_gravity_ff_active_index = 0;

static uint8 cylinder_climb_signal_is_present(float pitch_deg);

/* 入口0度、顶部180度、出口直道300度；速度曲线只暴露顶部百分比。 */
static void cylinder_update_speed_percent(void)
{
    float top_percent;
    float speed_percent;

    if (cylinder_state != CYLINDER_STATE_ON_CYLINDER)
    {
        cylinder_speed_percent = 100U;
        return;
    }

    top_percent = (float)CYLINDER_TOP_SPEED_PERCENT;
    if (top_percent < 100.0f)
        top_percent = 100.0f;
    else if (top_percent > 255.0f)
        top_percent = 255.0f;

    if (cylinder_rotation_progress_deg <= 180.0f)
    {
        speed_percent = 100.0f + (top_percent - 100.0f) *
            cylinder_rotation_progress_deg / 180.0f;
    }
    else if (cylinder_rotation_progress_deg <
             CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG)
    {
        speed_percent = 100.0f + (top_percent - 100.0f) *
            (CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG -
             cylinder_rotation_progress_deg) /
            (CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG - 180.0f);
    }
    else
    {
        speed_percent = 100.0f;
    }

    cylinder_speed_percent = (uint8)(speed_percent + 0.5f);
}

/* 状态3的入口统一在这里处理，避免不同进入路径重复覆盖yaw_leave。 */
static void cylinder_enter_exit_straight(float yaw_deg)
{
    if (cylinder_state == CYLINDER_STATE_EXIT_STRAIGHT)
        return;

    cylinder_yaw_leave_deg = yaw_deg;
    cylinder_state = CYLINDER_STATE_EXIT_STRAIGHT;
    cylinder_left_guard_active = 0;
    cylinder_speed_percent = 100U;
}

/* 当前电感帧是否满足入口候选条件，只判断单帧。 */
static uint8 cylinder_entry_signal_is_present(void)
{
    uint16 horizontal_sum = ad_ave[0] + ad_ave[4];
    uint16 total_sum = horizontal_sum + ad_ave[1] + ad_ave[3];

    return (uint8)(horizontal_sum >= CYLINDER_DETECT_OUTER_SUM_MIN &&
                   total_sum >= CYLINDER_DETECT_TOTAL_SUM_MIN &&
                   ad_ave[0] >= CYLINDER_DETECT_OUTER_SINGLE_MIN &&
                   ad_ave[4] >= CYLINDER_DETECT_OUTER_SINGLE_MIN);
}

#if CYLINDER_TEST_AUTO_REARM_ENABLE
/* 自动重布防后，入口信号消失8帧或等待100帧即可解锁。 */
static void cylinder_update_entry_lockout(uint8 entry_signal_present)
{
    (void)spatial_confirm_update((uint8)!entry_signal_present,
                                  CYLINDER_REARM_CLEAR_CONFIRM_SAMPLES,
                                  &cylinder_rearm_clear_count);

    if (spatial_confirm_update(1,
                               CYLINDER_REARM_LOCKOUT_MAX_SAMPLES,
                               &cylinder_rearm_lockout_count) ||
        cylinder_rearm_clear_count >= CYLINDER_REARM_CLEAR_CONFIRM_SAMPLES)
    {
        cylinder_entry_lockout = 0;
        cylinder_rearm_clear_count = 0;
        cylinder_rearm_lockout_count = 0;
    }
}
#endif

/* 更新陀螺积分、顶部证据、后半圈证据和出口直道阶段。 */
static void cylinder_update_motion_evidence(float ay_g, float az_g,
                                             float gyro_x_dps,
                                             float yaw_deg)
{
    float gyro_x_abs;

    gyro_x_abs = spatial_absf(gyro_x_dps);
    if (gyro_x_abs > CYLINDER_GYRO_X_DEADBAND_DPS &&
        cylinder_rotation_progress_deg < 360.0f)
    {
        cylinder_rotation_progress_deg +=
            gyro_x_abs * CYLINDER_GYRO_SAMPLE_DT_S;
        if (cylinder_rotation_progress_deg > 360.0f)
            cylinder_rotation_progress_deg = 360.0f;
    }

    if (cylinder_rotation_progress_deg >= CYLINDER_ENTRY_LEFT_GUARD_DEG)
        cylinder_left_guard_active = 0;

    if (cylinder_rotation_progress_deg >= CYLINDER_INSIDE_PROGRESS_DEG)
        cylinder_top_seen = 1;

    if (cylinder_rotation_progress_deg >=
        CYLINDER_RETURN_HALF_PROGRESS_DEG)
    {
        cylinder_return_half_seen = 1;
    }

    if (!cylinder_top_seen &&
        spatial_confirm_update(
            (uint8)(az_g <= CYLINDER_TOP_AZ_MAX_G),
            CYLINDER_TOP_CONFIRM_SAMPLES,
            &cylinder_top_count))
    {
        cylinder_top_seen = 1;
        cylinder_top_count = 0;
    }

    if (!cylinder_return_half_seen &&
        spatial_confirm_update(
            (uint8)(cylinder_top_seen &&
                    ay_g >= CYLINDER_RETURN_HALF_AY_MIN_G),
            CYLINDER_RETURN_HALF_CONFIRM_SAMPLES,
            &cylinder_return_half_count))
    {
        cylinder_return_half_seen = 1;
        cylinder_return_half_count = 0;
    }

    if (cylinder_state == CYLINDER_STATE_ON_CYLINDER &&
        cylinder_return_half_seen &&
        cylinder_rotation_progress_deg >=
            CYLINDER_EXIT_STRAIGHT_PROGRESS_DEG)
    {
        cylinder_enter_exit_straight(yaw_deg);
    }
}

/* 出口姿态连续确认后进入状态3，最终完成由状态3中的航向变化决定。 */
static void cylinder_update_exit_pose(float gyro_x_dps,
                                      float pitch_deg, float az_g,
                                      float yaw_deg)
{
    uint8 exit_pose_present;

    exit_pose_present = (uint8)(
        (cylinder_rotation_progress_deg >= CYLINDER_EXIT_PROGRESS_DEG ||
         cylinder_return_half_seen) &&
        spatial_absf(gyro_x_dps) <=
            CYLINDER_EXIT_GYRO_X_ABS_MAX_DPS &&
        pitch_deg <= CYLINDER_EXIT_PITCH_MAX_DEG &&
        az_g >= CYLINDER_EXIT_AZ_MIN_G);

    if (!exit_pose_present)
    {
        if (!Cylinder_HasExited())
            cylinder_exit_imu_count = 0;
        return;
    }

    if (cylinder_state == CYLINDER_STATE_EXIT_STRAIGHT)
        return;

    if (!spatial_confirm_update(1,
                                CYLINDER_EXIT_IMU_CONFIRM_SAMPLES,
                                &cylinder_exit_imu_count))
        return;

    cylinder_exit_imu_count = 0;
    cylinder_enter_exit_straight(yaw_deg);
}

/* 离开圆筒后继续保持状态3；正向转过70度才解除元素锁存。 */
static void cylinder_update_leave_yaw(float yaw_deg)
{
    if (cylinder_state != CYLINDER_STATE_EXIT_STRAIGHT ||
        Cylinder_HasExited())
        return;

    if (yaw_deg - cylinder_yaw_leave_deg >
        CYLINDER_LEAVE_YAW_DELTA_DEG)
    {
        cylinder_exit_imu_count = CYLINDER_EXIT_CONFIRM_LATCH;
    }
}

/* 退出后稳定正立足够久时，清空本轮状态并临时锁住入口识别。 */
static void cylinder_try_auto_rearm(void)
{
#if CYLINDER_TEST_AUTO_REARM_ENABLE
    if (cylinder_exit_imu_count == CYLINDER_EXIT_CONFIRM_LATCH &&
        cylinder_rearm_flat_count >= CYLINDER_REARM_FLAT_CONFIRM_SAMPLES)
    {
        Cylinder_Reset();
        cylinder_entry_lockout = 1;
    }
#endif
}

void Cylinder_Init(void)
{
    Cylinder_Reset();
}

uint8 Cylinder_AdcUpdate(void)
{
    uint8 entry_signal_present;

    entry_signal_present = cylinder_entry_signal_is_present();

#if CYLINDER_TEST_AUTO_REARM_ENABLE
    if (cylinder_entry_lockout)
    {
        cylinder_update_entry_lockout(entry_signal_present);
        return 0;
    }
#endif

    if (cylinder_state != CYLINDER_STATE_IDLE)
        return 1;

    if (!spatial_confirm_update(entry_signal_present,
                                CYLINDER_DETECT_CONFIRM_SAMPLES,
                                &cylinder_detect_count))
        return 0;

    cylinder_state = CYLINDER_STATE_PRE_ENTRY;
    cylinder_left_guard_active = 1;
    return 1;
}

uint8 Cylinder_EntryIsDetected(void)
{
    return (uint8)(cylinder_state != CYLINDER_STATE_IDLE);
}

uint8 Cylinder_IsOnSurface(void)
{
    return (uint8)(cylinder_state == CYLINDER_STATE_ON_CYLINDER ||
                   (cylinder_state == CYLINDER_STATE_EXIT_STRAIGHT &&
                    !Cylinder_HasExited()));
}

/*
 * IMU流程：PRE_ENTRY确认上筒，ON_CYLINDER累计运动证据，
 * EXIT_STRAIGHT继续确认正立出口。返回1表示当前需要筒面负压。
 */
uint8 Cylinder_ImuUpdate(float ay_g, float az_g,
                         float gyro_x_dps, float pitch_deg,
                         float yaw_deg)
{
    if (cylinder_state == CYLINDER_STATE_IDLE)
        return 0;

    if (Cylinder_HasExited())
    {
#if CYLINDER_TEST_AUTO_REARM_ENABLE
        (void)spatial_confirm_update(
            cylinder_climb_signal_is_present(pitch_deg),
            CYLINDER_REARM_FLAT_CONFIRM_SAMPLES,
            &cylinder_rearm_flat_count);

        cylinder_try_auto_rearm();
#endif
        return Cylinder_IsOnSurface();
    }

    if (cylinder_state == CYLINDER_STATE_PRE_ENTRY)
    {
        if (spatial_confirm_update(
                cylinder_climb_signal_is_present(pitch_deg),
                CYLINDER_CLIMB_CONFIRM_SAMPLES,
                &cylinder_climb_count))
        {
            cylinder_state = CYLINDER_STATE_ON_CYLINDER;
            cylinder_climb_count = 0;
        }

        return Cylinder_IsOnSurface();
    }

    cylinder_update_motion_evidence(ay_g, az_g, gyro_x_dps, yaw_deg);
    cylinder_update_exit_pose(gyro_x_dps, pitch_deg, az_g, yaw_deg);
    cylinder_update_leave_yaw(yaw_deg);
    cylinder_update_speed_percent();

    return Cylinder_IsOnSurface();
}

static uint8 cylinder_climb_signal_is_present(float pitch_deg)
{
    return (uint8)(pitch_deg < CYLINDER_ENTRY_PITCH_MAX_DEG);
}

uint8 Cylinder_HasExited(void)
{
    return (uint8)(cylinder_exit_imu_count ==
                   CYLINDER_EXIT_CONFIRM_LATCH);
}

uint8 Cylinder_GetState(void)
{
    return cylinder_state;
}

uint8 Cylinder_IsEntryLeftTurnGuardActive(void)
{
    return cylinder_left_guard_active;
}

int16 Cylinder_LimitPreEntryDiff(int16 direction_diff)
{
    /* 正数向左/桶内，负数向右/桶外；两侧采用不同限幅。 */
    if (direction_diff > CYLINDER_PRE_ENTRY_INWARD_DIFF_MAX)
        return CYLINDER_PRE_ENTRY_INWARD_DIFF_MAX;

    if (direction_diff < -CYLINDER_PRE_ENTRY_OUTWARD_DIFF_MAX)
        return -CYLINDER_PRE_ENTRY_OUTWARD_DIFF_MAX;

    return direction_diff;
}

/* 状态1保持100%；状态2按运动进度升至顶部速度，再于状态3前回到100%。 */
int16 Cylinder_GetSpeedTarget(int16 current_speed, int16 straight_speed)
{
    int32 straight_abs;
    int32 target_abs;
    uint8 speed_percent;

    if (cylinder_state == CYLINDER_STATE_IDLE || straight_speed == 0)
        return current_speed;

    speed_percent = cylinder_speed_percent;
    straight_abs = straight_speed < 0 ?
        -(int32)straight_speed : straight_speed;
    target_abs = straight_abs * (int32)speed_percent / 100L;

    if (current_speed < 0 || (current_speed == 0 && straight_speed < 0))
        return (int16)-target_abs;
    return (int16)target_abs;
}

void Cylinder_UpdateGravityFeedforward(float pitch_sin)
{
    uint8 next_index;
    float gravity_pwm;

    if (!Cylinder_IsOnSurface())
    {
        gravity_pwm = 0.0f;
    }
    else
    {
        if (pitch_sin > 1.0f)
            pitch_sin = 1.0f;
        else if (pitch_sin < -1.0f)
            pitch_sin = -1.0f;

        gravity_pwm = -CYLINDER_GRAVITY_FF_PWM * pitch_sin;
    }

    next_index = (uint8)(cylinder_gravity_ff_active_index ^ 1U);
    cylinder_gravity_ff_pwm[next_index] = (int16)gravity_pwm;
    cylinder_gravity_ff_active_index = next_index;
}

int16 Cylinder_GetGravityFeedforwardPwm(void)
{
    uint8 index;

    index = cylinder_gravity_ff_active_index;
    return cylinder_gravity_ff_pwm[index];
}

void Cylinder_Reset(void)
{
    /* 完整复位本轮检测历史；公开调用时不会保留自动复位锁定。 */
    cylinder_detect_count = 0;
    cylinder_climb_count = 0;
    cylinder_top_count = 0;
    cylinder_top_seen = 0;
    cylinder_return_half_count = 0;
    cylinder_return_half_seen = 0;
    cylinder_exit_imu_count = 0;
    cylinder_rearm_flat_count = 0;
    cylinder_rearm_clear_count = 0;
    cylinder_entry_lockout = 0;
    cylinder_rearm_lockout_count = 0;
    cylinder_rotation_progress_deg = 0.0f;
    cylinder_yaw_leave_deg = 0.0f;
    cylinder_left_guard_active = 0;
    cylinder_speed_percent = 100U;
    cylinder_state = CYLINDER_STATE_IDLE;
    Cylinder_UpdateGravityFeedforward(0.0f);
}
