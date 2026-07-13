#include "ui_menu.h"
#include "quaternion.h"
#include "config.h"
#include "car_control.h"
#include "my_motor.h"
#include "inductance.h"
#include "navigation.h"

/*============================================================================
 * 模块说明：UI菜单模块
 * 功能：菜单显示和按键交互
 *============================================================================*/

/*---------------------------------------------------------------------------
 * UI结构体实例
 *---------------------------------------------------------------------------*/
ui_struct ui = {&UI_Display, 1, 0, 0};

/*---------------------------------------------------------------------------
 * 按键状态变量
 *---------------------------------------------------------------------------*/
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

uint8 key1_last_status = 0;
uint8 key2_last_status = 0;
uint8 key3_last_status = 0;
uint8 key4_last_status = 0;

uint8 key1_flag = 0;
uint8 key2_flag = 0;
uint8 key3_flag = 0;
uint8 key4_flag = 0;

/*---------------------------------------------------------------------------
 * 拨码开关状态变量
 *---------------------------------------------------------------------------*/
uint8 sw1_status = 0;
uint8 sw2_status = 0;
uint8 sw3_status = 0;
uint8 sw4_status = 0;

/*---------------------------------------------------------------------------
 * 页面标题数组
 *---------------------------------------------------------------------------*/
unsigned char xdata ui_page0[8][30] = {
    "  <INCREDIBLE_KING>   <page0>",
    "  pid",
    "  speed",
    "  circle",
    "  fan_off",
    "",
    "",
    "  <LOAD_IN_DATA>"
};

unsigned char xdata ui_page1[8][30] = {
    "  <pid>               <page1>",
    "  kpa",
    "  kpb",
    "  kd",
    "  kd_imu",
    "  kp_motor",
    "  ki_motor",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page2[8][30] = {
    "  <speed>             <page2>",
    "  normal_speed",
    "  speed_high",
    "  speed_low",
    "  speed_90",
    "  speed_S",
    "",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page3[8][30] = {
    "  <circle>            <page3>",
    "  huandao_num",
    "  huandao_dir",
    "  huandao_r",
    "  dis_before_each",
    "  g_angle_inside",
    "",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page4[8][30] = {
    "  <LOAD_IN_DATA>      <page4>",
    "   ****   ****  *    * *****",
    "   *   * *    * **   * *",
    "   *   * *    * * *  * *****",
    "   *   * *    * *  * * *",
    "   *   * *    * *   ** *",
    "   ****   ****  *    * *****",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page5[8][30] = {
    "  <circle_each>       <page5>",
    "  dis_before_0",
    "  dis_before_1", 
    "  dis_before_2",
    "  dis_before_3",
    "  dis_before_4",
    "  dis_before_5",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page6[8][30] = {
    "  <huandao_dir>       <page6>",
    "  dir_0 (0L/1R)",
    "  dir_1 (0L/1R)", 
    "  dir_2 (0L/1R)",
    "  dir_3 (0L/1R)",
    "  dir_4 (0L/1R)",
    "  dir_5 (0L/1R)",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page7[8][30] = {
    "  <huandao_r>         <page7>",
    "  r_0",
    "  r_1", 
    "  r_2",
    "  r_3",
    "  r_4",
    "  r_5",
    "  <EXIT>---------------------"
};

unsigned char xdata ui_page8[8][30] = {
    "  <g_angle_inside>     <page8>",
    "  angle_inside_0",
    "  angle_inside_1", 
    "  angle_inside_2",
    "  angle_inside_3",
    "  angle_inside_4",
    "  angle_inside_5",
    "  <EXIT>---------------------"
};

/*---------------------------------------------------------------------------
 * 显示字符串数组(带光标)
 *---------------------------------------------------------------------------*/
void UI_DispStrings(uint8 strings[8][30]) {
    uint8 i;
    for (i = 0; i < 8; i++) {
        if (i == ui.cursor) {
            strings[i][0] = '=';
            strings[i][1] = '>';
        }
        else if (i != 0) {
            strings[i][0] = strings[i][1] = ' ';
        }
        ips114_show_string(0, i, strings[i]);
    }
}

/*---------------------------------------------------------------------------
 * 主菜单显示函数
 *---------------------------------------------------------------------------*/
void UI_Display(void) {
    switch (ui.page) {
    case 0:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page0);
            ips114_show_uint8(155, 4, flag_suction_fan_off);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page0);
            ips114_show_uint8(155, 4, flag_suction_fan_off);
        }
        break;

    case 1:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page1);
            ips114_show_float(150, 1, kpa, 2, 2);
            ips114_show_float(150, 2, kpb, 2, 2);
            ips114_show_float(150, 3, kd, 2, 2);
            ips114_show_float(150, 4, kd_imu, 2, 2);
            ips114_show_float(150, 5, kp_motor, 2, 2);
            ips114_show_float(150, 6, ki_motor, 2, 2);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page1);
            ips114_show_float(150, 1, kpa, 2, 2);
            ips114_show_float(150, 2, kpb, 2, 2);
            ips114_show_float(150, 3, kd, 2, 2);
            ips114_show_float(150, 4, kd_imu, 2, 2);
            ips114_show_float(150, 5, kp_motor, 2, 2);
            ips114_show_float(150, 6, ki_motor, 2, 2);
        }
        break;

    case 2:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page2);
            ips114_show_float(150, 1, normal_speed, 3, 2);
            ips114_show_float(150, 2, speed_high, 3, 2);
            ips114_show_float(150, 3, speed_low, 3, 2);
            ips114_show_float(150, 4, speed_90, 3, 2);
            ips114_show_float(150, 5, speed_S, 3, 2);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page2);
            ips114_show_float(150, 1, normal_speed, 3, 2);
            ips114_show_float(150, 2, speed_high, 3, 2);
            ips114_show_float(150, 3, speed_low, 3, 2);
            ips114_show_float(150, 4, speed_90, 3, 2);
            ips114_show_float(150, 5, speed_S, 3, 2);
        }
        break;

    case 3:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page3);
            ips114_show_uint8(155, 1, huandao_num);
            ips114_show_string(155, 2, "-> page6");
            ips114_show_string(155, 3, "-> page7");
            ips114_show_string(155, 4, "-> page5");
            ips114_show_string(155, 5, "-> page8");
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page3);
            ips114_show_uint8(155, 1, huandao_num);
            ips114_show_string(155, 2, "-> page6");
            ips114_show_string(155, 3, "-> page7");
            ips114_show_string(155, 4, "-> page5");
            ips114_show_string(155, 5, "-> page8");
        }
        break;

    case 4:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page4);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page4);
        }
        break;

    case 5:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page5);
            ips114_show_float(155, 1, distance_before_huandao[0], 3, 2);
            ips114_show_float(155, 2, distance_before_huandao[1], 3, 2);
            ips114_show_float(155, 3, distance_before_huandao[2], 3, 2);
            ips114_show_float(155, 4, distance_before_huandao[3], 3, 2);
            ips114_show_float(155, 5, distance_before_huandao[4], 3, 2);
            ips114_show_float(155, 6, distance_before_huandao[5], 3, 2);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page5);
            ips114_show_float(155, 1, distance_before_huandao[0], 3, 2);
            ips114_show_float(155, 2, distance_before_huandao[1], 3, 2);
            ips114_show_float(155, 3, distance_before_huandao[2], 3, 2);
            ips114_show_float(155, 4, distance_before_huandao[3], 3, 2);
            ips114_show_float(155, 5, distance_before_huandao[4], 3, 2);
            ips114_show_float(155, 6, distance_before_huandao[5], 3, 2);
        }
        break;

    case 6:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page6);
            ips114_show_uint8(155, 1, huandao_dir[0]);
            ips114_show_uint8(155, 2, huandao_dir[1]);
            ips114_show_uint8(155, 3, huandao_dir[2]);
            ips114_show_uint8(155, 4, huandao_dir[3]);
            ips114_show_uint8(155, 5, huandao_dir[4]);
            ips114_show_uint8(155, 6, huandao_dir[5]);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page6);
            ips114_show_uint8(155, 1, huandao_dir[0]);
            ips114_show_uint8(155, 2, huandao_dir[1]);
            ips114_show_uint8(155, 3, huandao_dir[2]);
            ips114_show_uint8(155, 4, huandao_dir[3]);
            ips114_show_uint8(155, 5, huandao_dir[4]);
            ips114_show_uint8(155, 6, huandao_dir[5]);
        }
        break;

    case 7:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page7);
            ips114_show_uint8(155, 1, huandao_r[0]);
            ips114_show_uint8(155, 2, huandao_r[1]);
            ips114_show_uint8(155, 3, huandao_r[2]);
            ips114_show_uint8(155, 4, huandao_r[3]);
            ips114_show_uint8(155, 5, huandao_r[4]);
            ips114_show_uint8(155, 6, huandao_r[5]);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page7);
            ips114_show_uint8(155, 1, huandao_r[0]);
            ips114_show_uint8(155, 2, huandao_r[1]);
            ips114_show_uint8(155, 3, huandao_r[2]);
            ips114_show_uint8(155, 4, huandao_r[3]);
            ips114_show_uint8(155, 5, huandao_r[4]);
            ips114_show_uint8(155, 6, huandao_r[5]);
        }
        break;

    case 8:
        if (ui.last == ui.page) {
            UI_DispStrings(ui_page8);
            ips114_show_float(155, 1, g_angle_inside[0], 3, 2);
            ips114_show_float(155, 2, g_angle_inside[1], 3, 2);
            ips114_show_float(155, 3, g_angle_inside[2], 3, 2);
            ips114_show_float(155, 4, g_angle_inside[3], 3, 2);
            ips114_show_float(155, 5, g_angle_inside[4], 3, 2);
            ips114_show_float(155, 6, g_angle_inside[5], 3, 2);
        }
        else if (ui.last != ui.page) {
            ui.cursor = 1;
            ui.last = ui.page;
            ips114_clear(RGB565_BLACK);
            UI_DispStrings(ui_page8);
            ips114_show_float(155, 1, g_angle_inside[0], 3, 2);
            ips114_show_float(155, 2, g_angle_inside[1], 3, 2);
            ips114_show_float(155, 3, g_angle_inside[2], 3, 2);
            ips114_show_float(155, 4, g_angle_inside[3], 3, 2);
            ips114_show_float(155, 5, g_angle_inside[4], 3, 2);
            ips114_show_float(155, 6, g_angle_inside[5], 3, 2);
        }
        break;

    default:
        break;
    }
}

/*---------------------------------------------------------------------------
 * 菜单按键扫描
 *---------------------------------------------------------------------------*/
void UI_KeyScan(void) {
    // KEY2 - DOWN
    if (KEY2_PIN == 0) {
        ui.cursor++;
        if (ui.cursor > 7) {
            ui.cursor = 1;
            ui.page = ui.last;
        }
        while (KEY2_PIN == 0);
    }

    // KEY1 - UP
    if (KEY1_PIN == 0) {
        ui.cursor--;
        if (ui.cursor < 1) {
            ui.cursor = 7;
            ui.page = ui.last;
        }
        while (KEY1_PIN == 0);
    }

    // KEY4 - ENTER (增加)
    if (KEY4_PIN == 0) {
        switch (ui.page) {
        case 0:
            if (ui.cursor == 1) ui.page = 1;
            else if (ui.cursor == 2) ui.page = 2;
            else if (ui.cursor == 3) ui.page = 3;
            else if (ui.cursor == 4) flag_suction_fan_off = !flag_suction_fan_off;
            else if (ui.cursor == 7) {
                ui.page = 4;
                Config_SaveAll();
                Huandao_SaveConfig();
                CarControl_SaveConfig();
            }
            break;
        case 1:
            if (ui.cursor == 1) kpa += 0.5f;
            else if (ui.cursor == 2) kpb += 1.0f;
            else if (ui.cursor == 3) kd += 1.0f;
            else if (ui.cursor == 4) kd_imu += 0.5f;
            else if (ui.cursor == 5) kp_motor += 1.0f;
            else if (ui.cursor == 6) ki_motor += 0.1f;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 2:
            if (ui.cursor == 1) normal_speed += 10;
            else if (ui.cursor == 2) speed_high += 5;
            else if (ui.cursor == 3) speed_low += 5;
            else if (ui.cursor == 4) speed_90 += 5;
            else if (ui.cursor == 5) speed_S += 5;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 3:
            if (ui.cursor == 1) huandao_num += 1;
            else if (ui.cursor == 2) ui.page = 6;
            else if (ui.cursor == 3) ui.page = 7;
            else if (ui.cursor == 4) ui.page = 5;
            else if (ui.cursor == 5) ui.page = 8;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 4:
            if (ui.cursor == 7) ui.page = 0;
            break;
        case 5:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                distance_before_huandao[ui.cursor - 1] += 10;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 6:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                huandao_dir[ui.cursor - 1] = !huandao_dir[ui.cursor - 1];
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 7:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                huandao_r[ui.cursor - 1] += 5;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 8:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                g_angle_inside[ui.cursor - 1] += 5;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        default:
            break;
        }
        while (KEY4_PIN == 0);
    }

    // KEY3 - OUT (减少)
    if (KEY3_PIN == 0) {
        switch (ui.page) {
        case 0:
            ui.page = 0;
            break;
        case 1:
            if (ui.cursor == 1) kpa -= 0.5f;
            else if (ui.cursor == 2) kpb -= 1.0f;
            else if (ui.cursor == 3) kd -= 1.0f;
            else if (ui.cursor == 4) kd_imu -= 0.5f;
            else if (ui.cursor == 5) kp_motor -= 1.0f;
            else if (ui.cursor == 6) ki_motor -= 0.1f;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 2:
            if (ui.cursor == 1) normal_speed -= 10;
            else if (ui.cursor == 2) speed_high -= 5;
            else if (ui.cursor == 3) speed_low -= 5;
            else if (ui.cursor == 4) speed_90 -= 5;
            else if (ui.cursor == 5) speed_S -= 5;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 3:
            if (ui.cursor == 1) huandao_num -= 1;
            else if (ui.cursor == 2) ui.page = 6;
            else if (ui.cursor == 3) ui.page = 7;
            else if (ui.cursor == 4) ui.page = 5;
            else if (ui.cursor == 5) ui.page = 8;
            else if (ui.cursor == 7) ui.page = 0;
            break;
        case 4:
            if (ui.cursor == 7) ui.page = 0;
            break;
        case 5:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                distance_before_huandao[ui.cursor - 1] -= 10;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 6:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                huandao_dir[ui.cursor - 1] = !huandao_dir[ui.cursor - 1];
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 7:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                huandao_r[ui.cursor - 1] -= 5;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        case 8:
            if (ui.cursor >= 1 && ui.cursor <= 6)
                g_angle_inside[ui.cursor - 1] -= 5;
            else if (ui.cursor == 7) ui.page = 3;
            break;
        default:
            break;
        }
        while (KEY3_PIN == 0);
    }
}

/*---------------------------------------------------------------------------
 * 运行时数据显示
 *---------------------------------------------------------------------------*/
void UI_ShowRuntime(void) {
    ips114_show_string(0, 0, "n");
    ips114_show_float(30, 0, normal_speed, 3, 2);

    ips114_show_string(100, 0, "encoder");
    ips114_show_float(160, 0, encoder_ave, 5, 1);

    ips114_show_string(0, 1, "l");
    ips114_show_float(30, 1, AD_ONE[0], 3, 2);
    ips114_show_float(80, 1, AD_ONE[1], 3, 2);

    ips114_show_string(140, 1, "m");
    ips114_show_float(170, 1, AD_ONE[2], 3, 2);

    ips114_show_string(0, 2, "r");
    ips114_show_float(30, 2, AD_ONE[3], 3, 2);
    ips114_show_float(80, 2, AD_ONE[4], 3, 2);

    ips114_show_string(140, 2, "flag");
    ips114_show_uint8(170, 2, flag);

    ips114_show_string(0, 3, "fast");
    ips114_show_uint8(50, 3, flag_key_fast);

    ips114_show_string(0, 4, "err");
    ips114_show_float(50, 4, aaddcc.err_dir, 4, 2);

    ips114_show_string(110, 4, "hd_cnt");
    ips114_show_uint8(170, 4, huandao_count);

    ips114_show_string(0, 5, "target");
    ips114_show_int16(60, 5, motor_left.setspeed);
    ips114_show_int16(120, 5, motor_right.setspeed);

    ips114_show_string(0, 6, "actual");
    ips114_show_int16(60, 6, motor_left.encoder_data);
    ips114_show_int16(120, 6, motor_right.encoder_data);

    if (flag_key_fast == 1) {
        ips114_show_string(0, 7, "j");
        ips114_show_uint16(50, 7, j);
    }
    else {
        ips114_show_string(0, 7, "j");
        ips114_show_uint16(50, 7, path_point_count);
    }

    ips114_show_string(110, 7, "angle");
    ips114_show_float(150, 7, euler.yaw, 4, 2);
}
