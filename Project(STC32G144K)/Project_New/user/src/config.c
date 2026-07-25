#include "config.h"
#include "navigation.h"
#include "huandao.h"

/*============================================================================
 * 模块说明：全局配置模块
 * 功能：
 *   1. 管理需要频繁调试的参数（PID、速度策略）
 *   2. EEPROM读写功能
 *   3. 全局配置初始化
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 全局系统配置
 *---------------------------------------------------------------------------*/
uint8 debug_mode = 0;

/*---------------------------------------------------------------------------
 * 方向控制PID参数
 *---------------------------------------------------------------------------*/
float kpa = 50.0f;
float kpb = 70.0f;
float kd = 80.0f;
float kd_imu = 30.0f;

/*---------------------------------------------------------------------------
 * 电机速度环PID参数
 *---------------------------------------------------------------------------*/

float kp_motor = 10.0f;
float ki_motor = 2.0f;
float kd_motor = 0.0f;

/*---------------------------------------------------------------------------
 * 速度策略参数
 *---------------------------------------------------------------------------*/
float speed_high = 500.0f;
float speed_low = 400.0f;
float speed_90 = 300.0f;
float speed_S = 200.0f;
int16 normal_speed = 0;

/*---------------------------------------------------------------------------
 * EEPROM默认值
 *---------------------------------------------------------------------------*/
// PID默认值
float kpa_iap = 50;
float kpb_iap = 80;
float kd_iap = 70;
float kd_imu_iap = 10;
float kp_motor_iap = 10;
float ki_motor_iap = 2;
float kd_motor_iap = 0;

// 速度默认值
float speed_high_iap = 500;
float speed_low_iap = 400;
float speed_90_iap = 300;
float speed_S_iap = 200;
float normal_speed_iap = 350;

/*---------------------------------------------------------------------------
 * 辅助函数：字符串转浮点数
 *---------------------------------------------------------------------------*/
float StrToDouble(const char *s) {
    int i = 0;
    int k = 0;
    float j;
    int symbol = 1;
    float result = 0.0;
    
    if (s[i] == '+') {
        i++;
    }
    if (s[i] == '-') {
        i++;
        symbol = -1;
    }
    while (s[i] != '\0' && s[i] != '.') {
        j = (s[i] - '0') * 1.0;
        result = result * 10 + j;
        i++;
    }
    if (s[i] == '.') {
        i++;
        while (s[i] != '\0' && s[i] != ' ') {
            k++;
            j = s[i] - '0';
            result = result + (1.0 * j) / pow(10.0, k);
            i++;
        }
    }
    result = symbol * result;
    return result;
}

/*---------------------------------------------------------------------------
 * EEPROM读取浮点数
 *---------------------------------------------------------------------------*/
float Config_ReadFloat(uint8 len, uint16 addr) {
    uint8 buf[34];

    if (len >= sizeof(buf)) {
        len = (uint8)(sizeof(buf) - 1);
    }

    memset(buf, 0, sizeof(buf));
    iap_read_buff(addr, buf, len);
    buf[len] = '\0';
    return StrToDouble((const char *)buf);
}

void eeprom_write_float_ascii(double dat, uint8 num, uint8 pointnum, uint16 addr) {
    uint32 length;
    int8 buff[34];
    int8 start;
    int8 end;
    int8 point;

    memset(buff, 0, sizeof(buff));

    if (dat < 0) {
        length = sprintf((char *)buff, "%f", dat);
    }
    else {
        length = sprintf((char *)&buff[1], "%f", dat);
        length++;
    }

    point = (int8)(length - 7);
    start = (int8)(point - num - 1);
    end = (int8)(point + pointnum + 1);

    while (start < 0) {
        buff[end] = ' ';
        end++;
        start++;
    }

    buff[start] = (dat < 0) ? '-' : '+';
    buff[end - 1] = '\0';
    buff[end] = '\n';

    extern_iap_write_buff(addr, (uint8 *)buff, (uint16)(num + pointnum + 3));
}
/*---------------------------------------------------------------------------
 * 参数初始化（从EEPROM读取）
 *---------------------------------------------------------------------------*/
void Config_Init(void) {
    iap_init();
    
    // PID参数
    kpa = Config_ReadFloat(7, 0x07) > 0 ? Config_ReadFloat(7, 0x07) : kpa_iap;
    kpb = Config_ReadFloat(7, 0x10) > 0 ? Config_ReadFloat(7, 0x10) : kpb_iap;
    kd = Config_ReadFloat(7, 0x90) > 0 ? Config_ReadFloat(7, 0x90) : kd_iap;
    kd_imu = Config_ReadFloat(7, 0xa0) > 0 ? Config_ReadFloat(7, 0xa0) : kd_imu_iap;
    kp_motor = Config_ReadFloat(7, 0xa6) > 0 ? Config_ReadFloat(7, 0xa6) : kp_motor_iap;
    ki_motor = Config_ReadFloat(7, 0xb0) > 0 ? Config_ReadFloat(7, 0xb0) : ki_motor_iap;
    
    // 速度参数
    speed_high = Config_ReadFloat(7, 0x17) > 0 ? Config_ReadFloat(7, 0x17) : speed_high_iap;
    speed_low = Config_ReadFloat(7, 0x20) > 0 ? Config_ReadFloat(7, 0x20) : speed_low_iap;
    speed_90 = Config_ReadFloat(7, 0x27) > 0 ? Config_ReadFloat(7, 0x27) : speed_90_iap;
    speed_S = Config_ReadFloat(7, 0x30) > 0 ? Config_ReadFloat(7, 0x30) : speed_S_iap;
    normal_speed = Config_ReadFloat(7, 0x37) > 0 ? Config_ReadFloat(7, 0x37) : normal_speed_iap;
}

/*---------------------------------------------------------------------------
 * 保存全局配置到EEPROM
 *---------------------------------------------------------------------------*/
void Config_SaveAll(void) {
    // 更新IAP变量
    kpa_iap = kpa;
    kpb_iap = kpb;
    kd_iap = kd;
    kd_imu_iap = kd_imu;
    kp_motor_iap = kp_motor;
    ki_motor_iap = ki_motor;
    kd_motor_iap = kd_motor;
    
    speed_high_iap = speed_high;
    speed_low_iap = speed_low;
    speed_90_iap = speed_90;
    speed_S_iap = speed_S;
    normal_speed_iap = normal_speed;

    // 写入EEPROM - PID参数
    eeprom_write_float_ascii(kpa_iap, 3, 1, 0x07);
    eeprom_write_float_ascii(kpb_iap, 3, 1, 0x10);
    eeprom_write_float_ascii(kd_iap, 3, 1, 0x90);
    eeprom_write_float_ascii(kd_imu_iap, 3, 1, 0xa0);
    eeprom_write_float_ascii(kp_motor_iap, 3, 1, 0xa6);
    eeprom_write_float_ascii(ki_motor_iap, 3, 1, 0xb0);

    // 写入EEPROM - 速度参数
    eeprom_write_float_ascii(speed_high_iap, 3, 1, 0x17);
    eeprom_write_float_ascii(speed_low_iap, 3, 1, 0x20);
    eeprom_write_float_ascii(speed_90_iap, 3, 1, 0x27);
    eeprom_write_float_ascii(speed_S_iap, 3, 1, 0x30);
    eeprom_write_float_ascii(normal_speed_iap, 3, 1, 0x37);
}
