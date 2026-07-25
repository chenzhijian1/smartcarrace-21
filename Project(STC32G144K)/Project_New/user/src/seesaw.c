/**
 * @file    seesaw.c
 * @brief   跷跷板元素识别与控制模块。
 *
 * 本模块负责识别赛道中的跷跷板（平衡木/跷跷板）元素，并在车辆通过时调整
 * 速度策略以确保稳定通过。
 *
 * ## 模块架构
 * - **状态机**：IDLE → RISING → EXITED，共三个有效状态，
 *   通过 Seesaw_ImuUpdate() 在每个 IMU 帧中推进。
 * - **速度控制**：在 RISING 阶段限制车速和左右轮差速，防止车辆在跷跷板上
 *   因姿态变化导致控制失稳；EXITED 后恢复正常速度。
 * - **外部接口**：本模块不直接操作电机，只接收当前 IMU 样本和 pitch，
 *   并发布状态供元素管理器和控制链调用。
 *
 * ## 数据流
 * ```
 * IMU采样 → Seesaw_ImuUpdate() → 状态机
 *                                  ↓
 * TIM4 5ms中断 → Seesaw_GetSpeedTarget() / Seesaw_ClampWheelTargets() → 电机
 * ```
 *
 * ## 关键设计决策
 * 1. **平面基线**：必须先建立平面基线（连续 flat 帧），才能接受上坡候选，
 *    避免车辆在非平面路段误判。
 * 2. **正值完成**：确认登板后，只要 pitch 转为正值就直接判定元素通过。
 * 3. **候选超时**：超过 SEESAW_MAX_CANDIDATE_SAMPLES 帧时
 *    直接判定本次跷跷板通过，防止状态机长期占用路线。
 * 4. **模长容错**：候选期间模长无效只暂停其他姿态判断，不撤销候选。
 *
 * ## 坐标系约定
 * - ax/ay/az 单位为 g
 * - pitch直接由姿态解算传入；实车抬起时为负，下降时为正
 * - 跷跷板入口、抬起峰值和下降判断统一使用pitch，单位为度
 * - 所有 *_SAMPLES 表示去重后的有效 IMU 帧数（约 1.5cm/帧，按 3 m/s）。
 */

#include "seesaw.h"
#include "spatial_features.h"

/* ==========================================================================
 * 模块静态变量
 * ========================================================================== */

static volatile uint8 seesaw_state = SEESAW_STATE_IDLE; // 当前状态机状态，初始为 IDLE
static uint16 seesaw_baseline_count = 0; // 平面基线连续确认帧计数
static uint8 seesaw_baseline_seen = 0; // 是否已建立平面基线（1 已建立，0 未建立）
static uint16 seesaw_enter_count = 0; // 上坡倾角连续确认帧计数（进入 RISING 用）
static uint16 seesaw_candidate_age = 0; // 候选已存活帧数，用于超时检测
static float seesaw_peak_pitch_deg = 0.0f; // 本次候选期间记录的最小pitch，单位度

/**
 * @brief  清除本次候选的所有动态数据，回到 IDLE 状态。
 * @note   不清除平面基线（seesaw_baseline_seen 和 seesaw_baseline_count），
 *          因为基线是相对稳定的环境信息，无需每次候选结束都重新建立。
 *          清除的包括：状态、进入计数、候选年龄和峰值倾角。
 */
static void seesaw_clear_candidate(void)
{
    seesaw_state = SEESAW_STATE_IDLE;
    seesaw_enter_count = 0;
    seesaw_candidate_age = 0;
    seesaw_peak_pitch_deg = 0.0f;
}

/* 标记本次跷跷板已通过，保留 EXITED 状态供元素管理器消费。 */
static void seesaw_complete_candidate(void)
{
    seesaw_state = SEESAW_STATE_EXITED;
    seesaw_enter_count = 0;
    seesaw_candidate_age = 0;
    seesaw_peak_pitch_deg = 0.0f;
}

/**
 * @brief  完整重置模块，包括平面基线和候选数据。
 * @note   供启动初始化、元素切换和管理器放弃候选时调用。
 *          调用后模块回到上电初始状态，下次需要重新建立平面基线。
 */
void Seesaw_Reset(void)
{
    seesaw_baseline_count = 0;
    seesaw_baseline_seen = 0;
    seesaw_clear_candidate();
}

/**
 * @brief  上电初始化入口，内部调用 Seesaw_Reset() 完成全部复位。
 * @note   应在系统启动时调用一次，确保模块从干净状态开始运行。
 */
void Seesaw_Init(void)
{
    Seesaw_Reset();
}

/**
 * @brief  主循环跷跷板状态机入口，每个去重后的 IMU 样本调用一次。
 * @param  sample    当前 IMU 样本指针（不可为 NULL）。
 * @param  pitch_deg 姿态解算俯仰角，单位度。
 * @return 当前状态非 IDLE 时返回 1，IDLE 时返回 0。
 *
 * ## 状态机说明
 *
 * ### IDLE（空闲）
 * 等待平面基线建立，然后检测负pitch抬起候选：
 * 1. 连续 flat 帧建立平面基线（seesaw_baseline_seen = 1）。
 * 2. 基线建立后，检测pitch/norm条件是否满足抬起候选。
 * 3. 连续满足 SEESAW_ENTER_CONFIRM_SAMPLES 帧后进入 RISING。
 *
 * ### RISING（登板后）
 * - 候选年龄超时 → 直接判定元素通过
 * - pitch转为正值 → 直接判定元素通过
 * - 加速度模长无效 → 暂停其他姿态判断，但候选超时继续计时
 * - 倾角过大 → 撤销候选（可能进入其他立体元素）
 *
 * ### EXITED
 * 直接返回 1，等待外部调用 Reset 或下一轮候选。
 *
 * @note   本函数只发布状态，不直接操作电机。
 *          速度限制由 Seesaw_GetSpeedTarget() 和 Seesaw_ClampWheelTargets()
 *          在 TIM4 中断中根据状态机状态独立执行。
 */
uint8 Seesaw_ImuUpdate(const imu_sample_t *sample, float pitch_deg)
{
    uint8 entry_valid;   /* 上坡入口条件是否满足 */
    uint8 norm_valid;
    uint8 flat;

    /* 空指针保护 */
    if (sample == (const imu_sample_t *)0)
        return 0;

    norm_valid = spatial_accel_vector_norm_in_range(
        sample->ax_g, sample->ay_g, sample->az_g,
        SPATIAL_NORM_MIN_G, SPATIAL_NORM_MAX_G);
    flat = (uint8)(norm_valid &&
                   pitch_deg >= SPATIAL_BASELINE_PITCH_MIN_DEG &&
                   pitch_deg <= SPATIAL_BASELINE_PITCH_MAX_DEG);

    /* ==================================================================
     * IDLE 状态：建立平面基线 + 检测上坡候选
     * ================================================================== */
    if (seesaw_state == SEESAW_STATE_IDLE)
    {
        /*
         * 步骤 1：建立平面基线。
         * 只有在平面（flat）上连续确认足够帧数后，才认为基线已建立。
         * 基线建立之前，任何上坡倾角都不会被接受为候选。
         */
        if (flat)
        {
            if (spatial_confirm_update(1,
                                       SEESAW_BASELINE_CONFIRM_SAMPLES,
                                       &seesaw_baseline_count))
            {
                /* 饱和保持，避免后续溢出 */
                seesaw_baseline_count = SEESAW_BASELINE_CONFIRM_SAMPLES;
                seesaw_baseline_seen = 1;
            }
        }
        else if (!seesaw_baseline_seen)
        {
            /* 不在平面且基线尚未建立，重置计数 */
            (void)spatial_confirm_update(
                0,
                SEESAW_BASELINE_CONFIRM_SAMPLES,
                &seesaw_baseline_count);
        }

        /*
         * 步骤 2：检测上坡候选条件。
         * - 平面基线已建立
         * - 加速度模长有效（norm_valid）
         * - pitch在[SEESAW_PITCH_MIN_DEG, SEESAW_ENTRY_PITCH_MAX_DEG]范围内
         */
        entry_valid = (uint8)(
            seesaw_baseline_seen &&
            norm_valid &&
            pitch_deg >= SEESAW_PITCH_MIN_DEG &&
            pitch_deg <= SEESAW_ENTRY_PITCH_MAX_DEG);

        /*
         * 步骤 3：连续确认后进入 RISING 状态。
         * 同时初始化峰值倾角等候选数据。
         */
        if (spatial_confirm_update(entry_valid,
                                   SEESAW_ENTER_CONFIRM_SAMPLES,
                                   &seesaw_enter_count))
        {
            seesaw_state = SEESAW_STATE_RISING;
            seesaw_candidate_age = 0;
            seesaw_peak_pitch_deg = pitch_deg;
            seesaw_enter_count = 0;
        }

        return (uint8)(seesaw_state != SEESAW_STATE_IDLE);
    }

    /* ==================================================================
     * EXITED 状态：已结束，维持返回 1
     * ================================================================== */
    if (seesaw_state == SEESAW_STATE_EXITED)
        return 1;

    /* ==================================================================
     * RISING 候选存活期检查
     * ================================================================== */

    /* 候选年龄递增，防止溢出 */
    if (seesaw_candidate_age < 65535U)
        seesaw_candidate_age++;

    /* 超时 → 直接完成本次跷跷板元素 */
    if (seesaw_candidate_age > SEESAW_MAX_CANDIDATE_SAMPLES)
    {
        seesaw_complete_candidate();
        return 1;
    }

    /* 小车开始下俯即视为已飞离跷跷板控制窗口。 */
    if (pitch_deg > 0.0f)
    {
        seesaw_complete_candidate();
        return 1;
    }

    /* 动态飞离时模长可能长期偏离1g；暂停姿态推进，但不撤销候选。 */
    if (!norm_valid)
        return 1;

    /* pitch绝对值过大时撤销候选。 */
    if (pitch_deg > SEESAW_PITCH_MAX_DEG ||
        pitch_deg < SEESAW_PITCH_MIN_DEG)
    {
        seesaw_clear_candidate();
        return 0;
    }

    /* 更新峰值倾角 */
    if (pitch_deg < seesaw_peak_pitch_deg)
        seesaw_peak_pitch_deg = pitch_deg;

    /* 短暂倾斜后回平且峰值不足时，仍按入口误触处理。 */
    if (flat &&
        seesaw_peak_pitch_deg > SEESAW_PEAK_PITCH_MAX_DEG)
    {
        seesaw_clear_candidate();
        return 0;
    }

    return (uint8)(seesaw_state != SEESAW_STATE_IDLE);
}

/**
 * @brief  判断当前是否处于登板候选阶段。
 * @return 1 表示处于候选阶段，0 表示不是。
 * @note   供元素管理器保存候选并触发低速保护。
 *          候选阶段表示已检测到上坡但尚未确认完整的跷跷板轨迹。
 */
uint8 Seesaw_IsCandidate(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_RISING);
}

/**
 * @brief  判断是否已判定本次跷跷板通过。
 * @return 1 表示已进入 EXITED，0 表示尚未通过。
 */
uint8 Seesaw_IsConfirmed(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_EXITED);
}

/**
 * @brief  判断是否已通过跷跷板并释放当前路线元素。
 * @return 1 表示已退出（EXITED），0 表示未退出。
 * @note   供元素管理器推进到下一个路线元素。
 */
uint8 Seesaw_HasExited(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_EXITED);
}

/**
 * @brief  获取当前状态机状态字节。
 * @return 当前状态值（SEESAW_STATE_IDLE / RISING / EXITED）。
 * @note   供 TIM4 控制中断和调试打印读取。
 */
uint8 Seesaw_GetState(void)
{
    return seesaw_state;
}

/**
 * @brief  RISING 阶段的速度上限计算（内部辅助函数）。
 * @param  current_speed   当前速度目标值（已包含循迹修正）。
 * @param  straight_speed  普通直线目标速度。
 * @return 限制后的速度值。
 *
 * 工作机制：
 * - 将 straight_speed 乘以 SEESAW_SPEED_SLOW_PERCENT%（默认 60%）作为上限。
 * - 如果 current_speed 绝对值不超过上限，原样返回。
 * - 如果超过上限，则压低到上限值，保持原有符号方向。
 *
 * @note   只在 current_speed 超过上限时压低，不会抬高原有弯道降速。
 *          全程使用整数运算，适合在 5 ms 电机中断中调用。
 *          straight_speed 为 0 时直接返回 current_speed。
 */
static int16 seesaw_crawl_target(int16 current_speed,
                                 int16 straight_speed)
{
    int32 current_abs;    /* 当前速度绝对值（32 位防溢出） */
    int32 straight_abs;   /* 直线速度绝对值 */
    int32 cap_abs;        /* 速度上限绝对值 */

    if (straight_speed == 0)
        return current_speed;

    straight_abs = straight_speed < 0 ?
        -(int32)straight_speed : straight_speed;
    current_abs = current_speed < 0 ? -(int32)current_speed : current_speed;
    cap_abs = straight_abs * SEESAW_SPEED_SLOW_PERCENT / 100L;

    if (current_abs <= cap_abs)
        return current_speed;

    /* 超过上限，压低但保留方向 */
    return current_speed < 0 ? (int16)-cap_abs : (int16)cap_abs;
}

/**
 * @brief  获取跷跷板场景下的速度目标值（供 TIM4 速度控制中断调用）。
 * @param  current_speed   当前速度目标值（已包含循迹修正）。
 * @param  straight_speed  普通直线目标速度。
 * @return 调整后的速度目标值。
 *
 * 行为：
 * - RISING 状态：返回低速上限值（straight_speed * 60%），防止车辆在
 *   跷跷板上因速度过快导致姿态失控。
 * - 其他状态（EXITED / IDLE）：原样返回 current_speed，
 *   车辆恢复普通巡线速度。
 *
 * @note   一旦 pitch 转正或候选超时进入 EXITED，立即释放低速限制。
 */
int16 Seesaw_GetSpeedTarget(int16 current_speed, int16 straight_speed)
{
    if (seesaw_state != SEESAW_STATE_RISING)
        return current_speed;

    return seesaw_crawl_target(current_speed, straight_speed);
}

/**
 * @brief  RISING 阶段的左右轮速保护（供 TIM4 速度控制中断调用）。
 * @param  center_speed  中心速度目标值。
 * @param  left_speed    左轮速度目标值指针（输入输出）。
 * @param  right_speed   右轮速度目标值指针（输入输出）。
 *
 * 工作机制：
 * - 仅在 RISING 状态且参数有效时生效。
 * - 以 center_speed 为基准，计算左右轮允许范围：
 *   - 下限 = center_speed * SEESAW_SLOW_WHEEL_LOW_PERCENT%（默认 50%）
 *   - 上限 = center_speed * SEESAW_SLOW_WHEEL_HIGH_PERCENT%（默认 150%）
 *   - 下限不低于 SEESAW_SPEED_CRAWL_MIN（防止一侧被压得太低）
 * - 将 left_speed 和 right_speed 限制在上述范围内。
 *
 * @note   在 speed_adjust() 之后调用。
 *         目的是防止 RISING 阶段低速等待时，一侧轮子被方向环算成反转
 *         或差速过大导致车辆失控。EXITED 不干预原有差速。
 *         支持正反转（center_speed 为负时会交换上下限）。
 */
void Seesaw_ClampWheelTargets(int16 center_speed,
                              int16 *left_speed,
                              int16 *right_speed)
{
    int32 center_abs;  /* 中心速度绝对值 */
    int32 low_abs;     /* 下限绝对值 */
    int32 high_abs;    /* 上限绝对值 */
    int16 low;         /* 实际下限 */
    int16 high;        /* 实际上限 */

    /* 参数有效性检查 */
    if (left_speed == 0 || right_speed == 0 ||
        seesaw_state != SEESAW_STATE_RISING ||
        center_speed == 0)
        return;

    /* 计算允许范围（绝对值） */
    center_abs = center_speed < 0 ? -(int32)center_speed : center_speed;
    low_abs = center_abs * SEESAW_SLOW_WHEEL_LOW_PERCENT / 100L;
    high_abs = center_abs * SEESAW_SLOW_WHEEL_HIGH_PERCENT / 100L;

    /* 确保下限不低于爬行最小值 */
    if (center_abs >= SEESAW_SPEED_CRAWL_MIN &&
        low_abs < SEESAW_SPEED_CRAWL_MIN)
        low_abs = SEESAW_SPEED_CRAWL_MIN;
    if (low_abs > high_abs)
        low_abs = high_abs;

    /* 根据中心速度方向确定实际上下限 */
    if (center_speed > 0)
    {
        low = (int16)low_abs;
        high = (int16)high_abs;
        *left_speed = spatial_clamp_i16(*left_speed, low, high);
        *right_speed = spatial_clamp_i16(*right_speed, low, high);
    }
    else
    {
        /* 反向时交换上下限 */
        low = (int16)-high_abs;
        high = (int16)-low_abs;
        *left_speed = spatial_clamp_i16(*left_speed, low, high);
        *right_speed = spatial_clamp_i16(*right_speed, low, high);
    }
}
