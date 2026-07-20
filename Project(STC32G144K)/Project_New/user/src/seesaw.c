/**
 * @file    seesaw.c
 * @brief   跷跷板元素识别与控制模块。
 *
 * 本模块负责识别赛道中的跷跷板（平衡木/跷跷板）元素，并在车辆通过时调整
 * 速度策略以确保稳定通过。
 *
 * ## 模块架构
 * - **状态机**：IDLE → RISING → FALLING → ACTIVE → EXITED，共五个状态，
 *   通过 Seesaw_ImuUpdate() 在每个 IMU 帧中推进。
 * - **速度控制**：在 RISING 阶段限制车速和左右轮差速，防止车辆在跷跷板上
 *   因姿态变化导致控制失稳；FALLING 之后恢复正常速度。
 * - **外部接口**：本模块不直接操作电机、不读取传感器，只接收
 *   spatial_features_t 特征快照并发布状态，供元素管理器和控制链调用。
 *
 * ## 数据流
 * ```
 * IMU采样 → SpatialFeatures → Seesaw_ImuUpdate() → 状态机
 *                                                    ↓
 * TIM4 5ms中断 → Seesaw_GetSpeedTarget() / Seesaw_ClampWheelTargets() → 电机
 * ```
 *
 * ## 关键设计决策
 * 1. **平面基线**：必须先建立平面基线（连续 flat 帧），才能接受上坡候选，
 *    避免车辆在非平面路段误判。
 * 2. **倾角峰值**：跟踪上升过程中的最大倾角，只有峰值超过最小有效峰值
 *    且出现持续下降趋势后才确认进入 FALLING。
 * 3. **陀螺辅助证据**：记录登板时 gx 的初始方向，检测反向旋转作为转折
 *    辅助证据，但不单独决定状态切换。
 * 4. **候选超时**：超过 SEESAW_MAX_CANDIDATE_SAMPLES 帧或姿态倒置时
 *    自动撤销候选，防止状态机卡死。
 * 5. **模长容错**：加速度模长短时间超出有效范围时给予宽限期，避免因瞬时
 *    振动丢失候选。
 *
 * ## 坐标系约定
 * - ax/ay/az 单位为 g，gx 单位为度/秒
 * - pitch直接由姿态解算传入；实车抬起时为负，下降时为正
 * - 跷跷板入口、抬起峰值和下降判断统一使用pitch，单位为度
 * - 所有 *_SAMPLES 表示去重后的有效 IMU 帧数（约 5ms/帧）
 */

#include "seesaw.h"

/* ==========================================================================
 * 模块静态变量
 * ========================================================================== */

static volatile uint8 seesaw_state = SEESAW_STATE_IDLE; // 当前状态机状态，初始为 IDLE
static uint16 seesaw_baseline_count = 0; // 平面基线连续确认帧计数
static uint8 seesaw_baseline_seen = 0; // 是否已建立平面基线（1 已建立，0 未建立）
static uint16 seesaw_enter_count = 0; // 上坡倾角连续确认帧计数（进入 RISING 用）
static uint16 seesaw_trend_count = 0; // 峰值后下降趋势连续确认帧计数
static uint16 seesaw_exit_count = 0; // 回到平面连续确认帧计数（进入 EXITED 用）
static uint16 seesaw_candidate_age = 0; // 候选已存活帧数，用于超时检测
static uint8 seesaw_norm_invalid_count = 0; // 加速度模长连续无效帧计数
static float seesaw_peak_tilt_deg = 0.0f; // 本次候选期间记录的最大上仰角，单位度
static int8 seesaw_motion_sign = 0; // 登板时 gx 角速度的初始方向：1 正转，-1 反转，0 未记录
static uint8 seesaw_reverse_seen = 0; // 是否已检测到 gx 反向（1 已反向，0 未反向）
static uint16 seesaw_reverse_count = 0; // gx 反向连续确认帧计数

/* ==========================================================================
 * 静态辅助函数
 * ========================================================================== */

/**
 * @brief  从实车pitch提取抬起角度幅值。
 * @param  pitch_deg  姿态解算俯仰角；抬起为负，下降为正。
 * @return 抬起角度幅值，单位度；下降或平坦时返回0。
 */
static float seesaw_rising_tilt_deg(float pitch_deg)
{
    return pitch_deg < 0.0f ? -pitch_deg : 0.0f;
}

/**
 * @brief  清除本次候选的所有动态数据，回到 IDLE 状态。
 * @note   不清除平面基线（seesaw_baseline_seen 和 seesaw_baseline_count），
 *          因为基线是相对稳定的环境信息，无需每次候选结束都重新建立。
 *          清除的包括：状态、进入计数、趋势计数、退出计数、候选年龄、
 *          模长无效计数、峰值倾角、运动方向标志、反向检测标志。
 */
static void seesaw_clear_candidate(void)
{
    seesaw_state = SEESAW_STATE_IDLE;
    seesaw_enter_count = 0;
    seesaw_trend_count = 0;
    seesaw_exit_count = 0;
    seesaw_candidate_age = 0;
    seesaw_norm_invalid_count = 0;
    seesaw_peak_tilt_deg = 0.0f;
    seesaw_motion_sign = 0;
    seesaw_reverse_seen = 0;
    seesaw_reverse_count = 0;
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
 * @brief  更新 gx 角速度运动方向检测。
 * @param  features  IMU 特征快照指针。
 *
 * 工作机制：
 * 1. 当 gx 角速度绝对值低于 SEESAW_GYRO_MOTION_MIN_DPS 时，视为无有效
 *    运动，重置反向计数但不改变已记录的方向。
 * 2. 首次检测到有效角速度时，记录其方向为 seesaw_motion_sign（+1 或 -1）。
 * 3. 之后持续检测 gx 是否反向（越过阈值），使用 spatial_confirm_update()
 *    连续确认 SEESAW_GYRO_REVERSE_CONFIRM_SAMPLES 帧后设置
 *    seesaw_reverse_seen = 1。
 *
 * @note   该函数只产生"转折辅助证据"，不单独决定状态切换；
 *          主判断仍以倾角趋势（seesaw_trend_count）为准。
 *          陀螺反向证据用于加速 FALLING 状态的确认。
 */
static void seesaw_update_motion_sign(const spatial_features_t *features)
{
    uint8 reverse_condition;  /* 是否检测到 gx 反向 */

    /* 角速度太小，忽略 */
    if (features->gx_abs_dps < SEESAW_GYRO_MOTION_MIN_DPS)
    {
        seesaw_reverse_count = 0;
        return;
    }

    /* 首次记录运动方向 */
    if (seesaw_motion_sign == 0)
    {
        seesaw_motion_sign = features->gx_dps >= 0.0f ? 1 : -1;
        seesaw_reverse_count = 0;
    }
    else
    {
        /* 检查是否与初始方向相反 */
        reverse_condition = (uint8)(
            (seesaw_motion_sign > 0 &&
             features->gx_dps <= -SEESAW_GYRO_MOTION_MIN_DPS) ||
            (seesaw_motion_sign < 0 &&
             features->gx_dps >= SEESAW_GYRO_MOTION_MIN_DPS));

        /* 连续确认反向 */
        if (spatial_confirm_update(
                reverse_condition,
                SEESAW_GYRO_REVERSE_CONFIRM_SAMPLES,
                &seesaw_reverse_count))
        {
            seesaw_reverse_seen = 1;
            /* 饱和保持，避免计数器溢出 */
            seesaw_reverse_count =
                SEESAW_GYRO_REVERSE_CONFIRM_SAMPLES;
        }
    }
}

/**
 * @brief  主循环跷跷板状态机入口，每个去重后的 IMU 特征帧调用一次。
 * @param  features  IMU 特征快照指针（不可为 NULL）。
 * @return 当前状态非 IDLE 时返回 1，IDLE 时返回 0。
 *
 * ## 状态机说明
 *
 * ### IDLE（空闲）
 * 等待平面基线建立，然后检测负pitch抬起候选：
 * 1. 连续 flat 帧建立平面基线（seesaw_baseline_seen = 1）。
 * 2. 基线建立后，检测pitch/az/norm条件是否满足抬起候选。
 * 3. 连续满足 SEESAW_ENTER_CONFIRM_SAMPLES 帧后进入 RISING。
 *
 * ### RISING / FALLING / ACTIVE（登板后）
 * 共享前置检查：
 * - 候选年龄超时 → 撤销候选
 * - 姿态倒置 → 撤销候选
 * - 加速度模长连续无效过多 → 撤销候选
 * - 倾角过大或 az 过低 → 撤销候选（可能进入其他立体元素）
 *
 * RISING 阶段：
 * - 跟踪峰值倾角seesaw_peak_tilt_deg
 * - 检测峰值是否达到最小有效值SEESAW_TILT_MIN_PEAK_DEG
 * - 检测pitch是否由负转正并连续达到+3度
 * - 满足负峰值和正pitch趋势后进入FALLING
 *
 * FALLING 阶段：
 * - 正pitch继续连续确认后进入ACTIVE
 *
 * ACTIVE 阶段：
 * - 连续 flat 帧确认后进入 EXITED
 *
 * ### EXITED
 * 直接返回 1，等待外部调用 Reset 或下一轮候选。
 *
 * @note   本函数只发布状态，不直接操作电机。
 *          速度限制由 Seesaw_GetSpeedTarget() 和 Seesaw_ClampWheelTargets()
 *          在 TIM4 中断中根据状态机状态独立执行。
 */
uint8 Seesaw_ImuUpdate(const spatial_features_t *features, float pitch_deg)
{
    float tilt_deg;      /* 负pitch对应的抬起角度幅值，单位度 */
    uint8 tilt_valid;    /* 上坡条件是否满足 */
    uint8 trend_valid;   /* 下降趋势是否满足 */

    /* 空指针保护 */
    if (features == (const spatial_features_t *)0)
        return 0;

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
        if (features->flat)
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
         * - Z 轴加速度不低于最低阈值（排除倒置姿态）
         * - pitch在[-SEESAW_TILT_MAX_DEG, -SEESAW_TILT_ENTER_DEG]范围内
         */
        tilt_deg = seesaw_rising_tilt_deg(pitch_deg);
        tilt_valid = (uint8)(
            seesaw_baseline_seen &&
            features->norm_valid &&
            features->az_lowpass_g >= SEESAW_AZ_MIN_G &&
            tilt_deg >= SEESAW_TILT_ENTER_DEG &&
            tilt_deg <= SEESAW_TILT_MAX_DEG);

        /*
         * 步骤 3：连续确认后进入 RISING 状态。
         * 同时初始化峰值倾角、运动方向等候选数据。
         */
        if (spatial_confirm_update(tilt_valid,
                                   SEESAW_ENTER_CONFIRM_SAMPLES,
                                   &seesaw_enter_count))
        {
            seesaw_state = SEESAW_STATE_RISING;
            seesaw_candidate_age = 0;
            seesaw_peak_tilt_deg = tilt_deg;
            seesaw_trend_count = 0;
            seesaw_exit_count = 0;
            seesaw_motion_sign = 0;
            seesaw_reverse_seen = 0;
            seesaw_reverse_count = 0;
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
     * 候选存活期通用检查（RISING / FALLING / ACTIVE）
     * ================================================================== */

    /* 候选年龄递增，防止溢出 */
    if (seesaw_candidate_age < 65535U)
        seesaw_candidate_age++;

    /* 超时或姿态倒置 → 撤销候选 */
    if (seesaw_candidate_age > SEESAW_MAX_CANDIDATE_SAMPLES ||
        features->inverted)
    {
        seesaw_clear_candidate();
        return 0;
    }

    /* 加速度模长无效容错 */
    if (!features->norm_valid)
    {
        if (seesaw_norm_invalid_count <
            SEESAW_NORM_INVALID_GRACE_SAMPLES)
            seesaw_norm_invalid_count++;
        if (seesaw_norm_invalid_count >=
            SEESAW_NORM_INVALID_GRACE_SAMPLES)
        {
            /* 连续无效帧数超过宽限期，撤销候选 */
            seesaw_clear_candidate();
            return 0;
        }
        /* 宽限期内保持候选，但不更新状态 */
        return 1;
    }
    seesaw_norm_invalid_count = 0;  /* 模长恢复正常，清零计数器 */

    /* pitch绝对值过大或Z轴过低时撤销候选。 */
    tilt_deg = seesaw_rising_tilt_deg(pitch_deg);
    if (pitch_deg > SEESAW_TILT_MAX_DEG ||
        pitch_deg < -SEESAW_TILT_MAX_DEG ||
        features->az_lowpass_g < SEESAW_AZ_MIN_G)
    {
        seesaw_clear_candidate();
        return 0;
    }

    /* 更新陀螺运动方向检测 */
    seesaw_update_motion_sign(features);

    /* 更新峰值倾角 */
    if (tilt_deg > seesaw_peak_tilt_deg)
        seesaw_peak_tilt_deg = tilt_deg;

    /* 实车下降时pitch为正；连续达到+3度后确认下降趋势。 */
    trend_valid = (uint8)(pitch_deg >= SEESAW_TILT_ENTER_DEG);
    if (trend_valid)
    {
        if (spatial_confirm_update(1,
                                   SEESAW_TREND_CONFIRM_SAMPLES,
                                   &seesaw_trend_count))
            /* 饱和保持 */
            seesaw_trend_count = SEESAW_TREND_CONFIRM_SAMPLES;
    }
    else
    {
        seesaw_trend_count = 0;
    }

    /* ==================================================================
     * RISING → FALLING 状态转换
     * ================================================================== */
    if (seesaw_state == SEESAW_STATE_RISING)
    {
        /*
         * 短暂倾斜后立即回平不足以识别跷跷板。
         * 如果在平地上峰值仍小于最小有效峰值，放弃候选。
         */
        if (features->flat &&
            seesaw_peak_tilt_deg < SEESAW_TILT_MIN_PEAK_DEG)
        {
            seesaw_clear_candidate();
            return 0;
        }

        /*
         * 进入 FALLING 的条件（同时满足）：
         * 1. 峰值倾角 >= 最小有效峰值
         * 2. 正pitch下降趋势连续确认
         */
        if (seesaw_peak_tilt_deg >= SEESAW_TILT_MIN_PEAK_DEG &&
            seesaw_trend_count >= SEESAW_TREND_CONFIRM_SAMPLES)
        {
            seesaw_state = SEESAW_STATE_FALLING;
            seesaw_trend_count = 0;
        }
    }
    /* ==================================================================
     * FALLING → ACTIVE 状态转换
     * ================================================================== */
    else if (seesaw_state == SEESAW_STATE_FALLING)
    {
        if (seesaw_trend_count >= SEESAW_TREND_CONFIRM_SAMPLES)
        {
            /* 进入FALLING后正pitch继续成立，确认有效跷跷板轨迹。 */
            seesaw_state = SEESAW_STATE_ACTIVE;
            seesaw_exit_count = 0;  /* 初始化退出计数 */
        }
    }

    /* ==================================================================
     * ACTIVE → EXITED 状态转换
     * ================================================================== */
    if (seesaw_state == SEESAW_STATE_ACTIVE)
    {
        if (features->flat)
        {
            /* 连续回到平面后退出 */
            if (spatial_confirm_update(1,
                                       SEESAW_EXIT_CONFIRM_SAMPLES,
                                       &seesaw_exit_count))
                seesaw_state = SEESAW_STATE_EXITED;
        }
        else
        {
            /* 不满足平面条件，重置退出计数 */
            seesaw_exit_count = 0;
        }
    }

    return (uint8)(seesaw_state != SEESAW_STATE_IDLE);
}

/**
 * @brief  判断当前是否处于候选阶段（RISING 或 FALLING）。
 * @return 1 表示处于候选阶段，0 表示不是。
 * @note   供元素管理器保存候选并触发低速保护。
 *          候选阶段表示已检测到上坡但尚未确认完整的跷跷板轨迹。
 */
uint8 Seesaw_IsCandidate(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_RISING ||
                   seesaw_state == SEESAW_STATE_FALLING);
}

/**
 * @brief  判断是否已确认完成有效的跷跷板转折轨迹。
 * @return 1 表示已确认（ACTIVE 或 EXITED），0 表示未确认。
 * @note   返回 1 表示检测到完整的"上坡→峰值→下坡"过程，
 *          可用于路线确认和元素计数。
 */
uint8 Seesaw_IsConfirmed(void)
{
    return (uint8)(seesaw_state == SEESAW_STATE_ACTIVE ||
                   seesaw_state == SEESAW_STATE_EXITED);
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
 * @return 当前状态值（SEESAW_STATE_IDLE / RISING / FALLING / ACTIVE / EXITED）。
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
 * - 其他状态（FALLING / ACTIVE / EXITED / IDLE）：原样返回 current_speed，
 *   车辆恢复普通巡线速度。
 *
 * @note   一旦 FALLING 确认，立即释放低速限制，使车辆恢复全速。
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
 *         或差速过大导致车辆失控。FALLING/ACTIVE/EXITED 不干预原有差速。
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
