#include "huandao.h"
#include "config.h"
#include "car_control.h"
#include "my_motor.h"
#include "inductance.h"
#include "quaternion.h"
#include "navigation.h"

#define HUANDAO_ENTER_ANGLE 60.0f
#define HUANDAO_INSIDE_ANGLE 280.0f
#define HUANDAO_EXIT_DISTANCE 250.0f
#define HUANDAO_ENTRY_BIAS_RATIO 0.65f
#define HUANDAO_DETECT_CONFIRM_COUNT (3U)

enum
{
    HUANDAO_DETECT_NORMAL = 0,
    HUANDAO_DETECT_SUSPECT,
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
uint8 huandao_dir[HUANDAO_MAX_COUNT] = {1, 0, 0, 0, 0};
// 环岛方向数组：0 为左环（逆时针、航向角增加），1 为右环（顺时针、航向角减少）。
uint8 huandao_dir_source[HUANDAO_MAX_COUNT] = {0, 0, 0, 0, 0};
uint8 huandao_r[HUANDAO_MAX_COUNT] = {20, 35, 30, 30, 30};   // 环岛半径数组（单位：cm）
float distance_before_huandao[HUANDAO_MAX_COUNT] = {200, 200, 200, 200, 200};  // 环岛前距离数组（单位：编码器）

// EEPROM默认值
static float distance_before_huandao_iap[HUANDAO_MAX_COUNT] = {200, 200, 200, 200, 200};
static float huandao_num_iap = 1;
static uint8 huandao_dir_iap[HUANDAO_MAX_COUNT] = {1, 0, 0, 0, 0};
static uint8 huandao_r_iap[HUANDAO_MAX_COUNT] = {30, 35, 30, 30, 30};

/*---------------------------------------------------------------------------
 * 环岛状态变量
 *---------------------------------------------------------------------------*/
uint8 flag_huandao = 0;         // 0:左环岛, 1:右环岛
static volatile huandao_state_t huandao_state = HUANDAO_STATE_IDLE;
static uint8 huandao_angle_set = 0;
static float huandao_enter_start_yaw = 0.0f;
static float huandao_inside_start_yaw = 0.0f;

static volatile uint8 huandao_detect_state = HUANDAO_DETECT_NORMAL;
static float huandao_pre_h_threshold = 33.0f;
static float huandao_confirm_h_threshold = 55.0f;
static float huandao_suspect_max_distance = 350.0f;
static float huandao_rearm_h_threshold = 30.0f;

static uint8 huandao_pre_count = 0;
static uint8 huandao_left_count = 0;
static uint8 huandao_right_count = 0;
static uint8 huandao_rearm_count = 0;
static float huandao_detect_entry_encoder = 0.0f;
static uint8 huandao_exit_event = 0;

static void huandao_detect_reset_counters(void)
{
    huandao_pre_count = 0;
    huandao_left_count = 0;
    huandao_right_count = 0;
    huandao_rearm_count = 0;
}

void Huandao_DetectReset(void)
{
    huandao_detect_state = HUANDAO_DETECT_NORMAL;
    huandao_state = HUANDAO_STATE_IDLE;
    huandao_detect_entry_encoder = 0.0f;
    huandao_exit_event = 0;
    huandao_detect_reset_counters();
}

static void huandao_detect_start_rearm(void)
{
    huandao_detect_state = HUANDAO_DETECT_REARM;
    huandao_detect_reset_counters();
}

uint8 Huandao_DetectIsStraightHold(void)
{
    return (uint8)(huandao_detect_state == HUANDAO_DETECT_SUSPECT);
}

static void huandao_set_detected_direction(uint8 detected_dir)
{
    if (huandao_dir_source[huandao_count] == HUANDAO_DIR_SOURCE_SENSOR)
        flag_huandao = detected_dir;
    else
        flag_huandao = huandao_dir[huandao_count];
}

uint8 Huandao_DetectUpdate(void)
{
    float element_distance;
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

    if (huandao_detect_state == HUANDAO_DETECT_NORMAL)
    {
        if (flag == CAR_STATE_NORMAL && AD_ONE[0] > huandao_pre_h_threshold &&
            AD_ONE[4] > huandao_pre_h_threshold)
        {
            if (++huandao_pre_count >= HUANDAO_DETECT_CONFIRM_COUNT)
            {
                huandao_detect_state = HUANDAO_DETECT_SUSPECT;
                huandao_detect_entry_encoder = encoder_ave;
                huandao_detect_reset_counters();
                aaddcc.err_dir = 0.0f;
                aaddcc.last_err_dir = 0.0f;
                return 1;
            }
        }
        else
        {
            huandao_pre_count = 0;
        }
        return 0;
    }

    element_distance = encoder_ave - huandao_detect_entry_encoder;
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

    if (element_distance >= huandao_suspect_max_distance)
        huandao_detect_start_rearm();

    return (uint8)(huandao_detect_state == HUANDAO_DETECT_SUSPECT);
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
    *target_speed = straight_speed;
    *direction_diff = 0;

    if (encoder_ave - encoder_temp < HUANDAO_EXIT_DISTANCE) {
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
                            int16 *direction_diff)
{
    if (target_speed == (int16 *)0 || direction_diff == (int16 *)0)
        return;

    switch (huandao_state)
    {
        case HUANDAO_STATE_PRE_CIRCLE:
            huandao_prepare_pre_circle(straight_speed,
                                       target_speed,
                                       direction_diff);
            break;
        case HUANDAO_STATE_ENTER_CIRCLE:
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
void Huandao_Init(void) {
    // 读取环岛参数
    distance_before_huandao[0] = Config_ReadFloat(7, 0x100) > 0 ? Config_ReadFloat(7, 0x100) : distance_before_huandao_iap[0];
    distance_before_huandao[1] = Config_ReadFloat(7, 0x107) > 0 ? Config_ReadFloat(7, 0x107) : distance_before_huandao_iap[1];
    distance_before_huandao[2] = Config_ReadFloat(7, 0x10e) > 0 ? Config_ReadFloat(7, 0x10e) : distance_before_huandao_iap[2];
    distance_before_huandao[3] = Config_ReadFloat(7, 0x115) > 0 ? Config_ReadFloat(7, 0x115) : distance_before_huandao_iap[3];
    distance_before_huandao[4] = Config_ReadFloat(7, 0x11c) > 0 ? Config_ReadFloat(7, 0x11c) : distance_before_huandao_iap[4];
    
    huandao_num = (uint8)(Config_ReadFloat(7, 0x13f) > 0 ? Config_ReadFloat(7, 0x13f) : huandao_num_iap);
    if (huandao_num == 0)
    {
        huandao_num = 1;
    }
    else if (huandao_num > HUANDAO_MAX_COUNT)
    {
        huandao_num = HUANDAO_MAX_COUNT;
    }
    if (huandao_count >= huandao_num)
    {
        huandao_count = 0;
    }
    
    // 环岛方向
    huandao_dir[0] = (uint8)(Config_ReadFloat(7, 0x146) > 0 ? Config_ReadFloat(7, 0x146) + 0.5f : huandao_dir_iap[0]);
    huandao_dir[1] = (uint8)(Config_ReadFloat(7, 0x14a) > 0 ? Config_ReadFloat(7, 0x14a) + 0.5f : huandao_dir_iap[1]);
    huandao_dir[2] = (uint8)(Config_ReadFloat(7, 0x14e) > 0 ? Config_ReadFloat(7, 0x14e) + 0.5f : huandao_dir_iap[2]);
    huandao_dir[3] = (uint8)(Config_ReadFloat(7, 0x152) > 0 ? Config_ReadFloat(7, 0x152) + 0.5f : huandao_dir_iap[3]);
    huandao_dir[4] = (uint8)(Config_ReadFloat(7, 0x156) > 0 ? Config_ReadFloat(7, 0x156) + 0.5f : huandao_dir_iap[4]);
    
    // 环岛半径
    huandao_r[0] = (uint8)(Config_ReadFloat(7, 0x15e) > 0 ? Config_ReadFloat(7, 0x15e) + 0.5f : huandao_r_iap[0]);
    huandao_r[1] = (uint8)(Config_ReadFloat(7, 0x162) > 0 ? Config_ReadFloat(7, 0x162) + 0.5f : huandao_r_iap[1]);
    huandao_r[2] = (uint8)(Config_ReadFloat(7, 0x166) > 0 ? Config_ReadFloat(7, 0x166) + 0.5f : huandao_r_iap[2]);
    huandao_r[3] = (uint8)(Config_ReadFloat(7, 0x16a) > 0 ? Config_ReadFloat(7, 0x16a) + 0.5f : huandao_r_iap[3]);
    huandao_r[4] = (uint8)(Config_ReadFloat(7, 0x16e) > 0 ? Config_ReadFloat(7, 0x16e) + 0.5f : huandao_r_iap[4]);

}

/*---------------------------------------------------------------------------
 * 保存环岛参数到EEPROM
 *---------------------------------------------------------------------------*/
void Huandao_SaveConfig(void) {
    // 更新默认值数组
    int i;
    for (i = 0; i < HUANDAO_MAX_COUNT; i++) {
        distance_before_huandao_iap[i] = distance_before_huandao[i];
        huandao_dir_iap[i] = huandao_dir[i];
        huandao_r_iap[i] = huandao_r[i];
    }
    huandao_num_iap = huandao_num;
    
    // 写入EEPROM
    eeprom_write_float_ascii(distance_before_huandao[0], 3, 1, 0x100);
    eeprom_write_float_ascii(distance_before_huandao[1], 3, 1, 0x107);
    eeprom_write_float_ascii(distance_before_huandao[2], 3, 1, 0x10e);
    eeprom_write_float_ascii(distance_before_huandao[3], 3, 1, 0x115);
    eeprom_write_float_ascii(distance_before_huandao[4], 3, 1, 0x11c);
    eeprom_write_float_ascii((float)huandao_num, 3, 1, 0x13f);
    
    eeprom_write_float_ascii((float)huandao_dir[0], 3, 1, 0x146);
    eeprom_write_float_ascii((float)huandao_dir[1], 3, 1, 0x14a);
    eeprom_write_float_ascii((float)huandao_dir[2], 3, 1, 0x14e);
    eeprom_write_float_ascii((float)huandao_dir[3], 3, 1, 0x152);
    eeprom_write_float_ascii((float)huandao_dir[4], 3, 1, 0x156);
    
    eeprom_write_float_ascii((float)huandao_r[0], 3, 1, 0x15e);
    eeprom_write_float_ascii((float)huandao_r[1], 3, 1, 0x162);
    eeprom_write_float_ascii((float)huandao_r[2], 3, 1, 0x166);
    eeprom_write_float_ascii((float)huandao_r[3], 3, 1, 0x16a);
    eeprom_write_float_ascii((float)huandao_r[4], 3, 1, 0x16e);
    
}
