#include "cylinder.h"
#include "inductance.h"
#include "spatial_common.h"

/* 入口由新ADC帧累计确认，达到阈值后进入PRE_ENTRY。 */
static uint16 cylinder_detect_count = 0;

/* 对外阶段只由cylinder_state表示，查询函数均从状态推导。 */
static volatile uint8 cylinder_state = CYLINDER_STATE_IDLE;

/* 由主循环更新、控制中断只读，避免中断直接读取多字节float进度。 */
static volatile uint8 cylinder_left_guard_active = 0;

/* 各阶段独立连续帧计数，条件中断时清零。 */
static uint16 cylinder_climb_count = 0;
static uint16 cylinder_top_count = 0;
static uint16 cylinder_return_half_count = 0;
static uint16 cylinder_exit_imu_count = 0;

/* 一旦见过顶部或后半圈便锁存，确保入口处正立姿态不会被当成出口。 */
static uint8 cylinder_top_seen = 0;
static uint8 cylinder_return_half_seen = 0;

/* 出口电感只作为辅助：ready=1时把IMU确认帧数由20缩短为10。 */
static uint16 cylinder_exit_em_count = 0;
static uint8 cylinder_exit_em_ready = 0;

/* 短赛道测试用自动重布防状态。 */
static uint16 cylinder_rearm_flat_count = 0;
static uint16 cylinder_rearm_clear_count = 0;
static uint8 cylinder_entry_lockout = 0;
static uint16 cylinder_rearm_lockout_count = 0;

/* 圆筒位置判定使用的粗略运动进度，不属于桶面转向控制器输出。 */
static float cylinder_rotation_progress_deg = 0.0f; /* |gyro_x|积分，0到360度 */

/* 保留的一次性退出事件接口，供外部按需消费。 */
static uint8 cylinder_exit_event = 0;

static float cylinder_absf(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint8 cylinder_accel_is_valid(float ax_g, float ay_g, float az_g)
{
    return spatial_accel_vector_norm_in_range(
        ax_g, ay_g, az_g,
        CYLINDER_IMU_NORM_MIN_G,
        CYLINDER_IMU_NORM_MAX_G);
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
/* 自动重布防后，入口信号消失8帧或等待200帧即可解锁。 */
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

/* 进入后继续读取电感；后半圈的出口信号只能缩短IMU确认时间。 */
static void cylinder_update_exit_inductance(void)
{
    uint8 exit_signal_present;

    if (Cylinder_HasExited())
        return;

    if (!cylinder_return_half_seen ||
        (cylinder_state != CYLINDER_STATE_ON_CYLINDER &&
         cylinder_state != CYLINDER_STATE_EXIT_STRAIGHT))
    {
        cylinder_exit_em_count = 0;
        cylinder_exit_em_ready = 0;
        return;
    }

    exit_signal_present = (uint8)(
        ad_ave[0] + ad_ave[4] <= CYLINDER_EXIT_OUTER_SUM_MAX &&
        ad_ave[1] + ad_ave[3] <= CYLINDER_EXIT_LONGITUDINAL_SUM_MAX);

    if (spatial_confirm_update(exit_signal_present,
                               CYLINDER_EXIT_EM_CONFIRM_SAMPLES,
                               &cylinder_exit_em_count))
    {
        cylinder_exit_em_ready = 1;
    }
    else if (!exit_signal_present)
    {
        cylinder_exit_em_ready = 0;
    }
}

/* 更新陀螺积分、顶部证据、后半圈证据和出口直道阶段。 */
static void cylinder_update_motion_evidence(float ay_g, float az_g,
                                             float gyro_x_dps)
{
    float gyro_x_abs;

    gyro_x_abs = cylinder_absf(gyro_x_dps);
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
        cylinder_state = CYLINDER_STATE_EXIT_STRAIGHT;
    }
}

/* 出口最终必须由pitch和az连续确认。 */
static void cylinder_update_exit_pose(float pitch_deg, float az_g)
{
    uint8 exit_samples_required;
    uint8 exit_pose_present;

    exit_pose_present = (uint8)(
        (cylinder_rotation_progress_deg >= CYLINDER_EXIT_PROGRESS_DEG ||
         cylinder_return_half_seen) &&
        pitch_deg < CYLINDER_EXIT_PITCH_MAX_DEG &&
        az_g > CYLINDER_EXIT_AZ_MIN_G);

    if (!exit_pose_present)
    {
        if (!Cylinder_HasExited())
            cylinder_exit_imu_count = 0;
        return;
    }

    if (Cylinder_HasExited())
        return;

    exit_samples_required = cylinder_exit_em_ready ?
        CYLINDER_EXIT_IMU_WITH_EM_SAMPLES :
        CYLINDER_EXIT_IMU_CONFIRM_SAMPLES;

    if (!spatial_confirm_update(1,
                                exit_samples_required,
                                &cylinder_exit_imu_count))
        return;

    cylinder_exit_imu_count = CYLINDER_EXIT_CONFIRM_LATCH;
    cylinder_exit_event = 1;
    cylinder_state = CYLINDER_STATE_EXIT_STRAIGHT;
    cylinder_left_guard_active = 0;
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
    return Cylinder_AdcCandidateUpdate(1);
}

uint8 Cylinder_AdcCandidateUpdate(uint8 activate)
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
    {
        cylinder_update_exit_inductance();
        return 1;
    }

    if (!spatial_confirm_update(entry_signal_present,
                                CYLINDER_DETECT_CONFIRM_SAMPLES,
                                &cylinder_detect_count))
        return 0;

    if (activate)
    {
        return Cylinder_ActivateCandidate();
    }

    return 1;
}

uint8 Cylinder_ActivateCandidate(void)
{
    if (cylinder_state != CYLINDER_STATE_IDLE ||
        cylinder_entry_lockout ||
        cylinder_detect_count < CYLINDER_DETECT_CONFIRM_SAMPLES)
        return 0;

    cylinder_state = CYLINDER_STATE_PRE_ENTRY;
    cylinder_left_guard_active = 1;
    return 1;
}

uint8 Cylinder_IsDetected(void)
{
    return (uint8)(cylinder_state != CYLINDER_STATE_IDLE &&
                   !Cylinder_HasExited());
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
uint8 Cylinder_ImuUpdate(float ax_g, float ay_g, float az_g,
                         float gyro_x_dps, float pitch_deg)
{
    uint8 norm_valid;

    if (cylinder_state == CYLINDER_STATE_IDLE)
        return 0;

    if (Cylinder_HasExited())
    {
#if CYLINDER_TEST_AUTO_REARM_ENABLE
        norm_valid = cylinder_accel_is_valid(ax_g, ay_g, az_g);
        (void)spatial_confirm_update(
            (uint8)(norm_valid &&
                    cylinder_absf(ay_g) <=
                        CYLINDER_REARM_FLAT_AY_ABS_MAX_G &&
                    az_g >= CYLINDER_REARM_FLAT_AZ_MIN_G),
            CYLINDER_REARM_FLAT_CONFIRM_SAMPLES,
            &cylinder_rearm_flat_count);

        cylinder_try_auto_rearm();
#endif
        return Cylinder_IsOnSurface();
    }

    if (cylinder_state == CYLINDER_STATE_PRE_ENTRY)
    {
        if (spatial_confirm_update(
                Cylinder_ClimbSignalIsPresent(pitch_deg),
                CYLINDER_CLIMB_CONFIRM_SAMPLES,
                &cylinder_climb_count))
        {
            cylinder_state = CYLINDER_STATE_ON_CYLINDER;
            cylinder_climb_count = 0;
        }

        return Cylinder_IsOnSurface();
    }

    cylinder_update_motion_evidence(ay_g, az_g, gyro_x_dps);
    cylinder_update_exit_pose(pitch_deg, az_g);

    return Cylinder_IsOnSurface();
}

uint8 Cylinder_ClimbSignalIsPresent(float pitch_deg)
{
    return (uint8)(pitch_deg < CYLINDER_ENTRY_PITCH_MAX_DEG);
}

uint8 Cylinder_ExitInductanceIsReady(void)
{
    return cylinder_exit_em_ready;
}

uint8 Cylinder_HasExited(void)
{
    return (uint8)(cylinder_exit_imu_count ==
                   CYLINDER_EXIT_CONFIRM_LATCH);
}

uint8 Cylinder_ConsumeExitEvent(void)
{
    uint8 event;

    event = cylinder_exit_event;
    cylinder_exit_event = 0;
    return event;
}

uint8 Cylinder_IsEntryLockedOut(void)
{
    return cylinder_entry_lockout;
}

uint8 Cylinder_GetState(void)
{
    return cylinder_state;
}

float Cylinder_GetRotationProgress(void)
{
    return cylinder_rotation_progress_deg;
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
    cylinder_exit_em_count = 0;
    cylinder_exit_em_ready = 0;
    cylinder_rearm_flat_count = 0;
    cylinder_rearm_clear_count = 0;
    cylinder_entry_lockout = 0;
    cylinder_rearm_lockout_count = 0;
    cylinder_rotation_progress_deg = 0.0f;
    cylinder_exit_event = 0;
    cylinder_left_guard_active = 0;
    cylinder_state = CYLINDER_STATE_IDLE;
}
