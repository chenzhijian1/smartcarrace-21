#include "huandao.h"
#include "config.h"
#include "car_control.h"
#include "my_motor.h"
#include "inductance.h"
#include "element.h"
#include "quaternion.h"
#include "navigation.h"

/*============================================================================
 * 模块说明：环岛控制模块
 * 功能：环岛状态机管理、参数存储
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 环岛参数配置
 *---------------------------------------------------------------------------*/
uint8 huandao_num = 2;
uint8 huandao_count = 0;
uint8 huandao_dir[6] = {1, 0, 0, 0, 0, 0};
uint8 huandao_dir_source[6] = {0, 0, 0, 0, 0, 0};
uint8 huandao_r[6] = {30, 35, 30, 30, 30, 30};
float distance_before_huandao[6] = {210, 260, 210, 210, 210, 210};
float distance_after_huandao = 0;
float g_angle_turn = 30;
float g_angle_inside[6] = {280, 280, 280, 280, 280, 280};
float angle_in_threshold = 20;
float angle_out_threshold = 30;

// EEPROM默认值
static float distance_before_huandao_iap[6] = {210, 240, 210, 210, 210, 210};
static float distance_after_huandao_iap = 150;
static float angle_in_threshold_iap = 30;
static float angle_out_threshold_iap = 30;
static float huandao_num_iap = 2;
static uint8 huandao_dir_iap[6] = {1, 0, 0, 0, 0, 0};
static uint8 huandao_r_iap[6] = {30, 35, 30, 30, 30, 30};
static float g_angle_inside_iap[6] = {280, 270, 280, 280, 280, 280};

/*---------------------------------------------------------------------------
 * 环岛状态变量
 *---------------------------------------------------------------------------*/
uint8 flag_huandao = 0;         // 0:左环岛, 1:右环岛
uint8 flag_set_angle = 0;       // 是否已设置入环角度
uint16 flag_circle_in = 0;      // 入环标志
uint16 flag_circle_out = 0;     // 出环标志

float ratio = 0;                // 差速比例
float target_angle_in = 0;          // 入环起始角度
float target_angle_in_end = 0;      // 入环目标角度
float target_angle_inside = 0;      // 环内目标角度
float target_angle_out = 0;         // 出环目标角度

/*---------------------------------------------------------------------------
 * 预环岛模式 (flag=1)
 * 功能：直行到环岛入口
 *---------------------------------------------------------------------------*/
void Huandao_PreCircle(void) {
    if (encoder_ave - encoder_temp < distance_before_huandao[huandao_count]) {
        // 还没到环岛交点，直行
        set_leftspeed = normal_speed;
        set_rightspeed = normal_speed;
    }
    else {
        flag = 2;  // 进入入环模式
    }
}

/*---------------------------------------------------------------------------
 * 入环模式 (flag=2)
 * 功能：差速入环直到达到目标角度
 *---------------------------------------------------------------------------*/
void Huandao_EnterCircle(void) {
    if (flag_set_angle == 0) {
        // 第一次进入，记录当前角度
        target_angle_in = euler.yaw;
        flag_set_angle = 1;
    }

    // 计算差速比例
    ratio = (float)(huandao_r[huandao_count] - (CAR_WIDTH/2)) / 
            (float)(huandao_r[huandao_count] + (CAR_WIDTH/2));

    if (flag_huandao == 0) {
        // 左环岛
        target_angle_in_end = target_angle_in - g_angle_turn;
        
        set_leftspeed = normal_speed * ratio;
        set_rightspeed = normal_speed;

        if (euler.yaw < target_angle_in_end) {
            flag = 3;  // 进入环内循迹
            encoder_temp = encoder_ave;
            target_angle_inside = euler.yaw;
        }
    }
    else {
        // 右环岛
        target_angle_in_end = target_angle_in + g_angle_turn;
        
        set_leftspeed = normal_speed;
        set_rightspeed = normal_speed * ratio;

        if (euler.yaw > target_angle_in_end) {
            flag = 3;  // 进入环内循迹
            encoder_temp = encoder_ave;
            target_angle_inside = euler.yaw;
        }
    }
}

/*---------------------------------------------------------------------------
 * 环内循迹 (flag=3)
 * 功能：环内电感循迹直到出环角度
 *---------------------------------------------------------------------------*/
void Huandao_InsideCircle(void) {
    if (flag_huandao == 0) {
        // 左环岛
        target_angle_out = target_angle_inside - g_angle_inside[huandao_count];

        if (euler.yaw > target_angle_out) {
            dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, gyro_z);
            normal_speed_cal = (int16)-s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;
            normal_speed_pre = normal_speed;
            test_speed = normal_speed_cal;
            speed_adjust(110, 600);
        }
        else {
            flag = 7;
            encoder_temp = encoder_ave;
        }
    }
    else {
        // 右环岛
        target_angle_out = target_angle_inside + g_angle_inside[huandao_count];

        if (euler.yaw < target_angle_out) {
            dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, gyro_z);
            normal_speed_cal = (int16)-s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;
            normal_speed_pre = normal_speed;
            test_speed = normal_speed_cal;
            speed_adjust(110, 600);
        }
        else {
            flag = 7;
            encoder_temp = encoder_ave;
        }
    }
}

/*---------------------------------------------------------------------------
 * 出环直行 (flag=7)
 * 功能：出环后直行一段距离
 *---------------------------------------------------------------------------*/
void Huandao_ExitStraight(void) {
    distance_after_huandao = huandao_r[huandao_count] * DISTANCE_RATIO * 1.3;

    if (encoder_ave - encoder_temp <= distance_after_huandao) {
        set_leftspeed = normal_speed;
        set_rightspeed = normal_speed;
    }
    else {
        // 恢复到正常循迹
        flag = 0;
        element_handler_start_rearm();
        Huandao_Reset();
        huandao_count = (huandao_count + 1) % huandao_num;
    }
}

/*---------------------------------------------------------------------------
 * 重置环岛状态
 *---------------------------------------------------------------------------*/
void Huandao_Reset(void) {
    flag_huandao = 0;
    flag_set_angle = 0;
    flag_circle_in = 0;
    flag_circle_out = 0;
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
    distance_before_huandao[5] = Config_ReadFloat(7, 0x123) > 0 ? Config_ReadFloat(7, 0x123) : distance_before_huandao_iap[5];
    
    distance_after_huandao = Config_ReadFloat(7, 0x12a) > 0 ? Config_ReadFloat(7, 0x12a) : distance_after_huandao_iap;
    angle_in_threshold = Config_ReadFloat(7, 0x131) > 0 ? Config_ReadFloat(7, 0x131) : angle_in_threshold_iap;
    angle_out_threshold = Config_ReadFloat(7, 0x138) > 0 ? Config_ReadFloat(7, 0x138) : angle_out_threshold_iap;
    huandao_num = (uint8)(Config_ReadFloat(7, 0x13f) > 0 ? Config_ReadFloat(7, 0x13f) : huandao_num_iap);
    
    // 环岛方向
    huandao_dir[0] = (uint8)(Config_ReadFloat(7, 0x146) > 0 ? Config_ReadFloat(7, 0x146) + 0.5f : huandao_dir_iap[0]);
    huandao_dir[1] = (uint8)(Config_ReadFloat(7, 0x14a) > 0 ? Config_ReadFloat(7, 0x14a) + 0.5f : huandao_dir_iap[1]);
    huandao_dir[2] = (uint8)(Config_ReadFloat(7, 0x14e) > 0 ? Config_ReadFloat(7, 0x14e) + 0.5f : huandao_dir_iap[2]);
    huandao_dir[3] = (uint8)(Config_ReadFloat(7, 0x152) > 0 ? Config_ReadFloat(7, 0x152) + 0.5f : huandao_dir_iap[3]);
    huandao_dir[4] = (uint8)(Config_ReadFloat(7, 0x156) > 0 ? Config_ReadFloat(7, 0x156) + 0.5f : huandao_dir_iap[4]);
    huandao_dir[5] = (uint8)(Config_ReadFloat(7, 0x15a) > 0 ? Config_ReadFloat(7, 0x15a) + 0.5f : huandao_dir_iap[5]);
    
    // 环岛半径
    huandao_r[0] = (uint8)(Config_ReadFloat(7, 0x15e) > 0 ? Config_ReadFloat(7, 0x15e) + 0.5f : huandao_r_iap[0]);
    huandao_r[1] = (uint8)(Config_ReadFloat(7, 0x162) > 0 ? Config_ReadFloat(7, 0x162) + 0.5f : huandao_r_iap[1]);
    huandao_r[2] = (uint8)(Config_ReadFloat(7, 0x166) > 0 ? Config_ReadFloat(7, 0x166) + 0.5f : huandao_r_iap[2]);
    huandao_r[3] = (uint8)(Config_ReadFloat(7, 0x16a) > 0 ? Config_ReadFloat(7, 0x16a) + 0.5f : huandao_r_iap[3]);
    huandao_r[4] = (uint8)(Config_ReadFloat(7, 0x16e) > 0 ? Config_ReadFloat(7, 0x16e) + 0.5f : huandao_r_iap[4]);
    huandao_r[5] = (uint8)(Config_ReadFloat(7, 0x172) > 0 ? Config_ReadFloat(7, 0x172) + 0.5f : huandao_r_iap[5]);

    // 环内角度
    g_angle_inside[0] = Config_ReadFloat(7, 0x17d) > 0 ? Config_ReadFloat(7, 0x17d) : g_angle_inside_iap[0];
    g_angle_inside[1] = Config_ReadFloat(7, 0x184) > 0 ? Config_ReadFloat(7, 0x184) : g_angle_inside_iap[1];
    g_angle_inside[2] = Config_ReadFloat(7, 0x18b) > 0 ? Config_ReadFloat(7, 0x18b) : g_angle_inside_iap[2];
    g_angle_inside[3] = Config_ReadFloat(7, 0x192) > 0 ? Config_ReadFloat(7, 0x192) : g_angle_inside_iap[3];
    g_angle_inside[4] = Config_ReadFloat(7, 0x199) > 0 ? Config_ReadFloat(7, 0x199) : g_angle_inside_iap[4];
    g_angle_inside[5] = Config_ReadFloat(7, 0x1a0) > 0 ? Config_ReadFloat(7, 0x1a0) : g_angle_inside_iap[5];
}

/*---------------------------------------------------------------------------
 * 保存环岛参数到EEPROM
 *---------------------------------------------------------------------------*/
void Huandao_SaveConfig(void) {
    // 更新默认值数组
    int i;
    for (i = 0; i < 6; i++) {
        distance_before_huandao_iap[i] = distance_before_huandao[i];
        huandao_dir_iap[i] = huandao_dir[i];
        huandao_r_iap[i] = huandao_r[i];
        g_angle_inside_iap[i] = g_angle_inside[i];
    }
    distance_after_huandao_iap = distance_after_huandao;
    angle_in_threshold_iap = angle_in_threshold;
    angle_out_threshold_iap = angle_out_threshold;
    huandao_num_iap = huandao_num;
    
    // 写入EEPROM
    eeprom_write_float_ascii(distance_before_huandao[0], 3, 1, 0x100);
    eeprom_write_float_ascii(distance_before_huandao[1], 3, 1, 0x107);
    eeprom_write_float_ascii(distance_before_huandao[2], 3, 1, 0x10e);
    eeprom_write_float_ascii(distance_before_huandao[3], 3, 1, 0x115);
    eeprom_write_float_ascii(distance_before_huandao[4], 3, 1, 0x11c);
    eeprom_write_float_ascii(distance_before_huandao[5], 3, 1, 0x123);
    eeprom_write_float_ascii(distance_after_huandao, 3, 1, 0x12a);
    eeprom_write_float_ascii(angle_in_threshold, 3, 1, 0x131);
    eeprom_write_float_ascii(angle_out_threshold, 3, 1, 0x138);
    eeprom_write_float_ascii((float)huandao_num, 3, 1, 0x13f);
    
    eeprom_write_float_ascii((float)huandao_dir[0], 3, 1, 0x146);
    eeprom_write_float_ascii((float)huandao_dir[1], 3, 1, 0x14a);
    eeprom_write_float_ascii((float)huandao_dir[2], 3, 1, 0x14e);
    eeprom_write_float_ascii((float)huandao_dir[3], 3, 1, 0x152);
    eeprom_write_float_ascii((float)huandao_dir[4], 3, 1, 0x156);
    eeprom_write_float_ascii((float)huandao_dir[5], 3, 1, 0x15a);
    
    eeprom_write_float_ascii((float)huandao_r[0], 3, 1, 0x15e);
    eeprom_write_float_ascii((float)huandao_r[1], 3, 1, 0x162);
    eeprom_write_float_ascii((float)huandao_r[2], 3, 1, 0x166);
    eeprom_write_float_ascii((float)huandao_r[3], 3, 1, 0x16a);
    eeprom_write_float_ascii((float)huandao_r[4], 3, 1, 0x16e);
    eeprom_write_float_ascii((float)huandao_r[5], 3, 1, 0x172);
    
    eeprom_write_float_ascii(g_angle_inside[0], 3, 1, 0x17d);
    eeprom_write_float_ascii(g_angle_inside[1], 3, 1, 0x184);
    eeprom_write_float_ascii(g_angle_inside[2], 3, 1, 0x18b);
    eeprom_write_float_ascii(g_angle_inside[3], 3, 1, 0x192);
    eeprom_write_float_ascii(g_angle_inside[4], 3, 1, 0x199);
    eeprom_write_float_ascii(g_angle_inside[5], 3, 1, 0x1a0);
}
