#include "element.h"
#include "inductance.h"
#include "car_control.h"
#include "huandao.h"

#define ELEMENT_CONFIRM_COUNT (3)  // 条件连续成立次数。

volatile uint8 element_state = ELEMENT_STATE_NORMAL; // 当前元素状态。
float element_pre_h_threshold = 33.0f;               // 两个横电感同时超过此值，进入元素预处理。
float element_circle_h_threshold = 55.0f;            // 单侧横电感确认环岛的强信号阈值。
float element_circle_side_delta = 15.0f;             // 确认左右环岛所需的横电感差值。
float element_cross_v_exit_threshold = 10.0f;        // 两个竖电感低于此值，作为十字结束条件。
float element_suspect_min_distance = 200.0f;         // 进入预处理后，允许判定十字结束的最小距离。
float element_suspect_max_distance = 350.0f;         // 未完成分类时，预处理状态允许的最大距离。
float element_rearm_h_threshold = 30.0f;             // 两个横电感低于此值后，允许识别下一个元素。

static uint8 element_pre_count = 0;                  // 连续满足进入预处理条件的次数。
static uint8 element_circle_left_count = 0;          // 连续满足左环岛条件的次数。
static uint8 element_circle_right_count = 0;         // 连续满足右环岛条件的次数。
static uint8 element_cross_exit_count = 0;           // 连续满足十字结束条件的次数。
static uint8 element_rearm_count = 0;                // 连续满足重新武装条件的次数。
static uint8 element_vertical_falling_seen = 0;      // 本次预处理内是否观察到两个竖电感同时减小。
static float element_entry_encoder = 0.0f;           // 进入预处理时的编码器位置。
static float element_last_v_left = 0.0f;             // 上一次左竖电感读数。
static float element_last_v_right = 0.0f;            // 上一次右竖电感读数。

static void element_reset_counters(void)
{
    element_pre_count = 0;
    element_circle_left_count = 0;
    element_circle_right_count = 0;
    element_cross_exit_count = 0;
    element_rearm_count = 0;
    element_vertical_falling_seen = 0;
}

static void element_start_suspect(void)
{
    element_state = ELEMENT_STATE_SUSPECT;
    element_entry_encoder = encoder_ave;
    element_reset_counters();
    element_last_v_left = AD_ONE[1];
    element_last_v_right = AD_ONE[3];
    aaddcc.err_dir = 0.0f;
    aaddcc.last_err_dir = 0.0f;
}

void element_handler_start_rearm(void)
{
    element_state = ELEMENT_STATE_REARM;
    element_reset_counters();
}

uint8 element_handler_is_straight(void)
{
    return element_state == ELEMENT_STATE_SUSPECT;
}

static void element_set_circle_direction(uint8 detected_dir)
{
    if (huandao_dir_source[huandao_count] == HUANDAO_DIR_SOURCE_SENSOR)
    {
        flag_huandao = detected_dir;
    }
    else
    {
        flag_huandao = huandao_dir[huandao_count];
    }
}

uint8 element_process(void)
{
    float element_distance;
    uint8 circle_left;
    uint8 circle_right;
    uint8 cross_exit;

    if (element_state == ELEMENT_STATE_CIRCLE_ACTIVE)
    {
        return 0;
    }

    if (element_state == ELEMENT_STATE_REARM)
    {
        if (flag == 0 && AD_ONE[0] < element_rearm_h_threshold && AD_ONE[4] < element_rearm_h_threshold)
        {
            if (++element_rearm_count >= ELEMENT_CONFIRM_COUNT)
            {
                element_state = ELEMENT_STATE_NORMAL;
                element_reset_counters();
            }
        }
        else
        {
            element_rearm_count = 0;
        }
        return 0;
    }

    if (element_state == ELEMENT_STATE_NORMAL)
    {
        if (flag == 0 && AD_ONE[0] > element_pre_h_threshold && AD_ONE[4] > element_pre_h_threshold)
        {
            if (++element_pre_count >= ELEMENT_CONFIRM_COUNT)
            {
                element_start_suspect();
                return 1;
            }
        }
        else
        {
            element_pre_count = 0;
        }
        return 0;
    }

    element_distance = encoder_ave - element_entry_encoder;
    if (AD_ONE[1] < element_last_v_left && AD_ONE[3] < element_last_v_right)
    {
        element_vertical_falling_seen = 1;
    }
    element_last_v_left = AD_ONE[1];
    element_last_v_right = AD_ONE[3];

    circle_left = AD_ONE[0] > element_circle_h_threshold &&
                  (AD_ONE[0] - AD_ONE[4]) > element_circle_side_delta;
    circle_right = AD_ONE[4] > element_circle_h_threshold &&
                   (AD_ONE[4] - AD_ONE[0]) > element_circle_side_delta;

    if (circle_left)
    {
        ++element_circle_left_count;
    }
    else
    {
        element_circle_left_count = 0;
    }

    if (circle_right)
    {
        ++element_circle_right_count;
    }
    else
    {
        element_circle_right_count = 0;
    }

    if (element_circle_left_count >= ELEMENT_CONFIRM_COUNT ||
        element_circle_right_count >= ELEMENT_CONFIRM_COUNT)
    {
        element_set_circle_direction(element_circle_left_count >= ELEMENT_CONFIRM_COUNT ? 0 : 1);
        encoder_temp = encoder_ave;
        element_state = ELEMENT_STATE_CIRCLE_ACTIVE;
        flag = 1;
        return 1;
    }

    cross_exit = element_distance >= element_suspect_min_distance &&
                 element_vertical_falling_seen &&
                 AD_ONE[1] < element_cross_v_exit_threshold &&
                 AD_ONE[3] < element_cross_v_exit_threshold;
    if (cross_exit)
    {
        if (++element_cross_exit_count >= ELEMENT_CONFIRM_COUNT)
        {
            element_handler_start_rearm();
            return 0;
        }
    }
    else
    {
        element_cross_exit_count = 0;
    }

    if (element_distance >= element_suspect_max_distance)
    {
        element_handler_start_rearm();
        return 0;
    }

    return 1;
}
