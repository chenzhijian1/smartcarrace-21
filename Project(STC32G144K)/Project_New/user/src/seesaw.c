/**
 * @file    seesaw.c
 * @brief   跷跷板元素识别与控制模块。
 *
 * 本模块负责识别赛道中的跷跷板（平衡木/跷跷板）元素，并在车辆通过时调整
 * 速度策略以确保稳定通过。
 *
 * ## 模块架构
 * - **状态机**：IDLE → RISING → FALLING → EXITED，共四个状态，
 *   通过 Seesaw_ImuUpdate() 在每个 IMU 帧中推进。
 * - **速度控制**：在 RISING/FALLING 阶段将车速设为 normal_speed_cal 的35%，
 *   左右轮差速仍完全使用 flag=0 的正常循迹结果，EXITED 后恢复正常速度。
 * - **外部接口**：本模块不直接操作电机，只接收当前 pitch，
 *   并发布状态供元素管理器和控制链调用。
 *
 * ## 数据流
 * ```
 * pitch → Seesaw_ImuUpdate() → 状态机
 *                                  ↓
 * TIM4 5ms中断 → Seesaw_GetSpeedTarget() → 正常循迹差速 → 电机
 * ```
 *
 * ## 关键设计决策
 * 1. **连续确认**：每个角度条件连续满足3次后才切换状态。
 * 2. **方向翻转**：负pitch确认上板，正pitch确认下板。
 * 3. **宽松看门狗**：候选持续约3秒仍未完成时自动撤销，防止状态机永久卡死。
 *
 * ## 坐标系约定
 * - pitch直接由姿态解算传入；实车抬起时为负，下降时为正
 * - 跷跷板入口、抬起峰值和下降判断统一使用pitch，单位为度
 * - 所有 *_SAMPLES 表示去重后的有效 IMU 帧数（约 5ms/帧）
 */

#include "seesaw.h"
#include "spatial_features.h"

/* ==========================================================================
 * 模块静态变量
 * ========================================================================== */

static volatile uint8 seesaw_state = SEESAW_STATE_IDLE; // 当前状态机状态，初始为 IDLE
static uint16 seesaw_enter_count = 0; // 上坡倾角连续确认帧计数（进入 RISING 用）
static uint16 seesaw_fall_count = 0; // 正pitch连续确认帧计数
static uint16 seesaw_exit_count = 0; // 回到平面连续确认帧计数（进入 EXITED 用）
static uint16 seesaw_candidate_age = 0; // 候选已存活帧数，用于超时检测

/**
 * @brief  清除本次候选的所有动态数据，回到 IDLE 状态。
 * @note   清除状态、连续确认计数和候选年龄。
 */
static void seesaw_clear_candidate(void)
{
    seesaw_state = SEESAW_STATE_IDLE;
    seesaw_enter_count = 0;
    seesaw_fall_count = 0;
    seesaw_exit_count = 0;
    seesaw_candidate_age = 0;
}

/**
 * @brief  完整重置跷跷板候选数据。
 */
void Seesaw_Reset(void)
{
    seesaw_clear_candidate();
}

/**
 * @brief  主循环跷跷板状态机入口，每次姿态更新后调用一次。
 * @param  pitch_deg 姿态解算俯仰角，单位度。
 *
 * ## 状态机说明
 *
 * ### IDLE（空闲）
 * pitch连续3次小于等于-8度后进入RISING。
 *
 * RISING 阶段：
 * pitch连续3次大于等于+8度后进入FALLING。
 *
 * FALLING 阶段：
 * |pitch|连续3次小于2度后进入EXITED。
 *
 * ### EXITED
 * 直接返回 1，等待外部调用 Reset 或下一轮候选。
 *
 * @note   本函数只发布状态，不直接操作电机。
 *          速度缩放由 Seesaw_GetSpeedTarget() 执行，不干预正常循迹差速。
 */
void Seesaw_ImuUpdate(float pitch_deg)
{
    uint8 tilt_valid;    /* 上坡条件是否满足 */

    /* IDLE：负pitch连续确认后进入RISING。 */
    if (seesaw_state == SEESAW_STATE_IDLE)
    {
        tilt_valid = (uint8)(
            pitch_deg <= -SEESAW_RISING_ENTER_DEG &&
            pitch_deg >= -SEESAW_TILT_MAX_DEG);

        if (spatial_confirm_update(tilt_valid,
                                   SEESAW_ENTER_CONFIRM_SAMPLES,
                                   &seesaw_enter_count))
        {
            seesaw_state = SEESAW_STATE_RISING;
            seesaw_candidate_age = 0;
            seesaw_fall_count = 0;
            seesaw_exit_count = 0;
            seesaw_enter_count = 0;
        }

        return;
    }

    /* EXITED 状态等待元素管理器推进路线。 */
    if (seesaw_state == SEESAW_STATE_EXITED)
        return;

    /* RISING/FALLING共享一个宽松看门狗。 */
    if (seesaw_candidate_age < 65535U)
        seesaw_candidate_age++;

    if (seesaw_candidate_age > SEESAW_MAX_CANDIDATE_SAMPLES)
    {
        seesaw_clear_candidate();
        return;
    }

    /* 只保留明显异常角度保护。 */
    if (pitch_deg > SEESAW_TILT_MAX_DEG ||
        pitch_deg < -SEESAW_TILT_MAX_DEG)
    {
        seesaw_clear_candidate();
        return;
    }

    /* RISING：正pitch连续3次进入FALLING。 */
    if (seesaw_state == SEESAW_STATE_RISING)
    {
        if (spatial_confirm_update(
                (uint8)(pitch_deg >= SEESAW_FALLING_ENTER_DEG),
                SEESAW_FALL_CONFIRM_SAMPLES,
                &seesaw_fall_count))
        {
            seesaw_state = SEESAW_STATE_FALLING;
            seesaw_fall_count = 0;
            seesaw_exit_count = 0;
        }
    }
    /* FALLING：回平连续3次后结束元素。 */
    else if (seesaw_state == SEESAW_STATE_FALLING)
    {
        if (spatial_confirm_update(
                (uint8)(spatial_absf(pitch_deg) < SEESAW_TILT_EXIT_DEG),
                SEESAW_EXIT_CONFIRM_SAMPLES,
                &seesaw_exit_count))
            seesaw_state = SEESAW_STATE_EXITED;
    }

}

/**
 * @brief  判断是否已退出跷跷板元素（回到平面）。
 * @return 1 表示已退出（EXITED），0 表示未退出。
 * @note   供元素管理器推进到下一个路线元素。
 */
uint8 Seesaw_HasExited(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_EXITED);
}

/**
 * @brief  获取当前状态机状态字节。
 * @return 当前状态值（SEESAW_STATE_IDLE / RISING / FALLING / EXITED）。
 * @note   供 TIM4 控制中断和调试打印读取。
 */
uint8 Seesaw_GetState(void)
{
    return seesaw_state;
}

/**
 * @brief  获取跷跷板场景下的速度目标值（供 TIM4 速度控制中断调用）。
 * @param  current_speed 当前 normal_speed_cal。
 * @return 调整后的速度目标值。
 *
 * 行为：
 * - RISING / FALLING 状态：直接返回 normal_speed_cal * 35%，防止车辆在
 *   跷跷板上因速度过快导致姿态失控。
 * - EXITED / IDLE 状态：原样返回 current_speed。
 *   调用处的 current_speed 是已经计算完成的 normal_speed_cal。
 *
 * @note   EXITED 后释放低速限制，恢复 normal_speed_cal。
 */
int16 Seesaw_GetSpeedTarget(int16 current_speed)
{
    if (seesaw_state != SEESAW_STATE_RISING &&
        seesaw_state != SEESAW_STATE_FALLING)
        return current_speed;

    return (int16)((int32)current_speed * SEESAW_SPEED_SLOW_PERCENT / 100L);
}
