#include "huandao.h"
#include "config.h"
#include "car_control.h"
#include "my_motor.h"
#include "inductance.h"
#include "quaternion.h"
#include "navigation.h"

#define HUANDAO_ENTER_ANGLE 45.0f
#define HUANDAO_INSIDE_ANGLE 260.0f
#define HUANDAO_EXIT_DISTANCE 200.0f
#define HUANDAO_EXIT_OUTWARD_DISTANCE 70.0f
#define HUANDAO_EXIT_OUTWARD_BIAS_RATIO 0.2f
#define HUANDAO_EXIT_OUTWARD_BIAS_MAX 300
#define HUANDAO_ENTRY_BIAS_RATIO 0.7f
#define HUANDAO_ENTRY_DIFF_LIMIT 500
#define HUANDAO_DETECT_CONFIRM_COUNT (3U)

enum
{
    HUANDAO_DETECT_NORMAL = 0,
    HUANDAO_DETECT_ACTIVE,
    HUANDAO_DETECT_REARM
};

/*============================================================================
 * 模块说明：环岛控制模块
 * 功能：环岛状态机管理、参数存储
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 环岛参数配置
 *---------------------------------------------------------------------------*/
uint8 huandao_num = 1; // 环岛数量
uint8 huandao_count = 0;
uint8 huandao_dir[HUANDAO_MAX_COUNT] = {0, 0, 0, 0, 0};
// 环岛方向数组：0 为左环（逆时针、航向角增加），1 为右环（顺时针、航向角减少）。
uint8 huandao_dir_source[HUANDAO_MAX_COUNT] = {0, 0, 0, 0, 0};
uint8 huandao_r[HUANDAO_MAX_COUNT] = {18, 30, 30, 30, 30};   // 环岛半径数组（单位：cm）
float distance_before_huandao[HUANDAO_MAX_COUNT] = {170, 200, 200, 200, 200};  // 环岛前距离数组（单位：编码器）

/*---------------------------------------------------------------------------
 * 环岛状态变量
 *---------------------------------------------------------------------------*/
uint8 flag_huandao = 0;         // 0:左环岛, 1:右环岛
static volatile huandao_state_t huandao_state = HUANDAO_STATE_IDLE;
static uint8 huandao_angle_set = 0;
static float huandao_enter_start_yaw = 0.0f;
static float huandao_inside_start_yaw = 0.0f;

static volatile uint8 huandao_detect_state = HUANDAO_DETECT_NORMAL;
static float huandao_confirm_h_threshold = 70.0f;
static float huandao_rearm_h_threshold = 30.0f;

static uint8 huandao_left_count = 0;
static uint8 huandao_right_count = 0;
static uint8 huandao_rearm_count = 0;
static uint8 huandao_exit_event = 0;
static uint8 huandao_route_reverse = 0;

static void huandao_detect_reset_counters(void)
{
    huandao_left_count = 0;
    huandao_right_count = 0;
    huandao_rearm_count = 0;
}

void Huandao_DetectReset(void)
{
    huandao_detect_state = HUANDAO_DETECT_NORMAL;
    huandao_state = HUANDAO_STATE_IDLE;
    huandao_exit_event = 0;
    huandao_detect_reset_counters();
}

static void huandao_detect_start_rearm(void)
{
    huandao_detect_state = HUANDAO_DETECT_REARM;
    huandao_detect_reset_counters();
}

static void huandao_set_detected_direction(uint8 detected_dir)
{
    if (huandao_dir_source[huandao_count] == HUANDAO_DIR_SOURCE_SENSOR)
        flag_huandao = detected_dir;
    else
        flag_huandao =
            (uint8)(huandao_dir[huandao_count] ^ huandao_route_reverse);
}

void Huandao_SetRouteReverse(uint8 reverse_run)
{
    huandao_route_reverse = (uint8)(reverse_run != 0);
}

uint8 Huandao_DetectUpdate(void)
{
    uint8 circle_left;
    uint8 circle_right;

    if (huandao_detect_state == HUANDAO_DETECT_ACTIVE)
        return 0;

    if (huandao_detect_state == HUANDAO_DETECT_REARM)
    {
        if (flag == CAR_STATE_NORMAL && AD_ONE[0] < huandao_rearm_h_threshold &&
            AD_ONE[4] < huandao_rearm_h_threshold)
        {
            if (++huandao_rearm_count >= HUANDAO_DETECT_CONFIRM_COUNT)
                Huandao_DetectReset();
        }
        else
        {
            huandao_rearm_count = 0;
        }
        return 0;
    }

    if (huandao_detect_state != HUANDAO_DETECT_NORMAL)
        return 0;

    if (flag != CAR_STATE_NORMAL)
    {
        huandao_left_count = 0;
        huandao_right_count = 0;
        return 0;
    }

    circle_left = (uint8)(AD_ONE[0] > huandao_confirm_h_threshold);
    circle_right = (uint8)(AD_ONE[4] > huandao_confirm_h_threshold);

    huandao_left_count = circle_left ? (uint8)(huandao_left_count + 1U) : 0;
    huandao_right_count = circle_right ? (uint8)(huandao_right_count + 1U) : 0;

    if (huandao_left_count >= HUANDAO_DETECT_CONFIRM_COUNT ||
        huandao_right_count >= HUANDAO_DETECT_CONFIRM_COUNT)
    {
        huandao_set_detected_direction(
            (uint8)(huandao_left_count >= HUANDAO_DETECT_CONFIRM_COUNT ? 0 : 1));
        encoder_temp = encoder_ave;
        huandao_detect_state = HUANDAO_DETECT_ACTIVE;
        huandao_state = HUANDAO_STATE_PRE_CIRCLE;
        return 1;
    }

    return 0;
}

uint8 Huandao_ConsumeExitEvent(void)
{
    uint8 event = huandao_exit_event;
    huandao_exit_event = 0;
    return event;
}

/*---------------------------------------------------------------------------
 * 预环岛阶段
 * 功能：直行到环岛入口
 *---------------------------------------------------------------------------*/
static void huandao_prepare_pre_circle(int16 straight_speed,
                                       int16 *target_speed,
                                       int16 *direction_diff) {
    *target_speed = straight_speed;
    *direction_diff = 0;

    if (encoder_ave - encoder_temp < distance_before_huandao[huandao_count]) {
        return;
    }

    huandao_angle_set = 0;
    huandao_state = HUANDAO_STATE_ENTER_CIRCLE;
}

/*---------------------------------------------------------------------------
 * 入环阶段
 * 功能：差速入环直到达到目标角度
 *---------------------------------------------------------------------------*/
static void huandao_prepare_enter_circle(int16 straight_speed,
                                         int16 *direction_diff) {
    int16 entry_bias;

    if (huandao_angle_set == 0) {
        huandao_enter_start_yaw = euler.yaw;
        huandao_angle_set = 1;
    }

    entry_bias = (int16)((float)straight_speed * HUANDAO_ENTRY_BIAS_RATIO);
    if (entry_bias < 0) {
        entry_bias = 0;
    }

    // direction_diff > 0 turns left; direction_diff < 0 turns right.
    if (flag_huandao == 0) {
        if (*direction_diff < entry_bias) {
            *direction_diff = entry_bias;
        }
    }
    else if (*direction_diff > -entry_bias) {
        *direction_diff = -entry_bias;
    }

    if (flag_huandao == 0) {
        if (euler.yaw >= huandao_enter_start_yaw + HUANDAO_ENTER_ANGLE) {
            huandao_inside_start_yaw = euler.yaw;
            huandao_state = HUANDAO_STATE_INSIDE_CIRCLE;
        }
    }
    else if (euler.yaw <= huandao_enter_start_yaw - HUANDAO_ENTER_ANGLE) {
        huandao_inside_start_yaw = euler.yaw;
        huandao_state = HUANDAO_STATE_INSIDE_CIRCLE;
    }
}

/*---------------------------------------------------------------------------
 * 环内循迹阶段
 * 功能：环内电感循迹直到出环角度
 *---------------------------------------------------------------------------*/
static void huandao_prepare_inside_circle(void) {
    if (flag_huandao == 0) {
        if (euler.yaw >= huandao_inside_start_yaw + HUANDAO_INSIDE_ANGLE) {
            huandao_state = HUANDAO_STATE_EXIT_STRAIGHT;
            encoder_temp = encoder_ave;
        }
    }
    else {
        if (euler.yaw <= huandao_inside_start_yaw - HUANDAO_INSIDE_ANGLE) {
            huandao_state = HUANDAO_STATE_EXIT_STRAIGHT;
            encoder_temp = encoder_ave;
        }
    }
}

/*---------------------------------------------------------------------------
 * 出环直行阶段
 * 功能：出环后直行一段距离
 *---------------------------------------------------------------------------*/
static void huandao_prepare_exit_straight(int16 straight_speed,
                                          int16 *target_speed,
                                          int16 *direction_diff) {
    float exit_distance;
    int16 exit_bias;

    exit_distance = encoder_ave - encoder_temp;
    if (exit_distance < HUANDAO_EXIT_OUTWARD_DISTANCE) {
        *target_speed = straight_speed;
        exit_bias = (int16)((float)straight_speed *
                            HUANDAO_EXIT_OUTWARD_BIAS_RATIO);
        if (exit_bias < 0) {
            exit_bias = -exit_bias;
        }
        if (exit_bias > HUANDAO_EXIT_OUTWARD_BIAS_MAX) {
            exit_bias = HUANDAO_EXIT_OUTWARD_BIAS_MAX;
        }

        // Left circle moves outward to the right; right circle is mirrored.
        *direction_diff = (flag_huandao == 0) ? -exit_bias : exit_bias;
    }

    if (exit_distance < HUANDAO_EXIT_DISTANCE) {
        return;
    }

    huandao_detect_start_rearm();
    huandao_exit_event = 1;
    Huandao_Reset();
    if (huandao_num > 0) {
        huandao_count = (huandao_count + 1) % huandao_num;
    }
}

void Huandao_PrepareControl(int16 straight_speed,
                            int16 *target_speed,
                            int16 *direction_diff,
                            int16 *direction_diff_limit)
{
    if (target_speed == (int16 *)0 || direction_diff == (int16 *)0 ||
        direction_diff_limit == (int16 *)0)
        return;

    switch (huandao_state)
    {
        case HUANDAO_STATE_PRE_CIRCLE:
            huandao_prepare_pre_circle(straight_speed,
                                       target_speed,
                                       direction_diff);
            break;
        case HUANDAO_STATE_ENTER_CIRCLE:
            *direction_diff_limit = HUANDAO_ENTRY_DIFF_LIMIT;
            huandao_prepare_enter_circle(straight_speed, direction_diff);
            break;
        case HUANDAO_STATE_INSIDE_CIRCLE:
            huandao_prepare_inside_circle();
            break;
        case HUANDAO_STATE_EXIT_STRAIGHT:
            huandao_prepare_exit_straight(straight_speed,
                                          target_speed,
                                          direction_diff);
            break;
        default:
            break;
    }
}

huandao_state_t Huandao_GetState(void)
{
    return huandao_state;
}

/*---------------------------------------------------------------------------
 * 重置环岛状态
 *---------------------------------------------------------------------------*/
void Huandao_Reset(void) {
    huandao_state = HUANDAO_STATE_IDLE;
    flag_huandao = 0;
    huandao_angle_set = 0;
    huandao_enter_start_yaw = 0.0f;
    huandao_inside_start_yaw = 0.0f;
}

/*---------------------------------------------------------------------------
 * 环岛参数初始化（从EEPROM读取）
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * 保存环岛参数到EEPROM
 *---------------------------------------------------------------------------*/
