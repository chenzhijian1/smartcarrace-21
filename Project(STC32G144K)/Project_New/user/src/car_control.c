#include "car_control.h"
#include "config.h"
#include "huandao.h"
#include "my_motor.h"
#include "inductance.h"
#include "navigation.h"
#include "quaternion.h"
#include "zf_device_imu963ra.h"

/*============================================================================
 * 模块说明：车辆控制模块
 * 功能：状态机管理、速度策略、方向控制
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 状态标志变量
 *---------------------------------------------------------------------------*/
uint8 flag = 0;
uint8 flag_stop = 0;
uint8 flag_key_control = 0;
uint8 flag_key_fast = 0;
uint8 flag_start = 0;
uint8 nav_end_flag_sent = 0;

/*---------------------------------------------------------------------------
 * 速度控制变量（运行时状态）
 *---------------------------------------------------------------------------*/
int16 normal_speed_pre = 0;
int16 normal_speed_cal = 0;
int16 test_speed = 0;
int16 changed_speed = 0;
int16 set_leftspeed = 0;
int16 set_rightspeed = 0;
int16 speed_huandao = 0;

/*---------------------------------------------------------------------------
 * 编码器相关
 *---------------------------------------------------------------------------*/
float encoder_ave = 0.0;
float encoder_temp = 0.0;

/*---------------------------------------------------------------------------
 * 其他控制变量
 *---------------------------------------------------------------------------*/
float k = 0;
float s = 0;
float gyro_z = 0;
float last_gyro_z = 0;
float lpf_gyro = 0.2;

uint8 cnt_start = 0;
uint8 cnt_stop = 0;
uint8 cnt_launch = 0;
static volatile int16 soft_stop_start_speed = 0;

/*---------------------------------------------------------------------------
 * 风扇控制
 *---------------------------------------------------------------------------*/
uint8 flag_suction_fan_off = 0;

// EEPROM默认值
static uint8 flag_suction_fan_off_iap = 0;

#define error_turn 17.0f

/*---------------------------------------------------------------------------
 * 正常循迹模式 (flag=0)
 *---------------------------------------------------------------------------*/
void CarControl_NormalMode(void) {
    dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, gyro_z);

    // 速度策略
    if (flag_start && cnt_start < 100) {
        cnt_start++;
        normal_speed_cal = (int16)((float)normal_speed * cnt_start / 100.0f);
    }
    else {
        flag_start = 0;
        cnt_start = 0;
        normal_speed_cal = (int16)-s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;
    }

    normal_speed_pre = normal_speed;
    test_speed = (int16)normal_speed_cal;

    // if (flag_key_fast == 1) {
    //     speed_adjust(120, 600);
    // }
    // else {
        speed_adjust(160, 800);  // 差速和最高速度限幅
    // }
}

/*---------------------------------------------------------------------------
 * 起步发车模式 (flag=4)
 *---------------------------------------------------------------------------*/
void CarControl_LaunchMode(void) {
    // 发车时先打开风扇
    if (cnt_launch == 0) {
        if (flag_suction_fan_off == 0)
            suction_fan_on(2000);
    }
    
    cnt_launch++;
    
    // 直接切换到正常循迹模式
    flag = 0;
    flag_start = 1;
    cnt_launch = 0;
}

/*---------------------------------------------------------------------------
 * 慢速停车模式 (flag=5)
 *---------------------------------------------------------------------------*/
void CarControl_StopMode(void) {
    if (cnt_stop < 200) {
        cnt_stop++;
        normal_speed_cal = (int16)((int32)soft_stop_start_speed * (200 - cnt_stop) / 200);
        set_leftspeed = normal_speed_cal;
        set_rightspeed = normal_speed_cal;
    }
    else {
        cnt_stop = 0;
        normal_speed = 0;
        set_leftspeed = 0;
        set_rightspeed = 0;
        flag_key_control = 0;
        suction_fan_off();
        send_flag_nav = 1;
        flag = 0;
    }
}

void CarControl_RequestSoftStop(void) {
    if (flag == 0 && normal_speed > 0) {
        soft_stop_start_speed = normal_speed;
        cnt_stop = 0;
        flag = 5;
        normal_speed = 0;
    }
}

/*---------------------------------------------------------------------------
 * 车辆状态更新主函数 (原speed_change)
 *---------------------------------------------------------------------------*/
void CarControl_Update(void) {
    if (flag_stop == 0) {
        // 使用四元数解算的角速度数据（已在IMU_Update中处理）
        // 从IMU963读取的陀螺仪原始数据并转换
        // IMU963RA陀螺仪±2000dps量程，灵敏度70 mdps/LSB = 0.07 dps/LSB
        if (imu963ra_gyro_z <= 4 && imu963ra_gyro_z >= -4)
            imu963ra_gyro_z = 0;
        gyro_z = (float)(imu963ra_gyro_z - gyro_offset_z) * 0.07f;

        switch (flag) {
            case 0:  // 正常模式
                CarControl_NormalMode();
                break;
            
#if 0
            /* Circle state dispatch. Temporarily disabled. */
            case 1:
                Huandao_PreCircle();
                break;

            case 2:
                Huandao_EnterCircle();
                break;

            case 3:
                Huandao_InsideCircle();
                break;

            case 7:
                Huandao_ExitStraight();
                break;
#endif

            case 4:  // 起步发车
                CarControl_LaunchMode();
                break;

            case 5:  // 慢速停车
                CarControl_StopMode();
                break;

            default:
                break;
        }
    }
}

/*---------------------------------------------------------------------------
 * 脱线保护
 *---------------------------------------------------------------------------*/
uint8 car_stop_judge(void) {
    if (AD_ONE[0] < 0.5 && AD_ONE[1] < 0.5 && AD_ONE[3] < 0.5 && AD_ONE[4] < 0.5) {
        set_leftspeed = 0;
        set_rightspeed = 0;
        normal_speed = 0;
        flag_key_control = 0;
        suction_fan_off();
        return 1;
    }

    return 0;
}

/*---------------------------------------------------------------------------
 * 方向PID控制
 *---------------------------------------------------------------------------*/
void dir_pid(float error, float last_error, float gyro) {
    int16 p_out, d_out, output;
    
    p_out = (int16)((kpa / 10) * error + kpb * (error / error_turn) * (error / error_turn) * (error / error_turn));
    d_out = (int16)(kd * (error - last_error)) + (int16)(kd_imu / 100.0 * gyro);
    output = p_out + d_out;

    changed_speed = output;
}

/*---------------------------------------------------------------------------
 * 速度调整(差速计算)
 *---------------------------------------------------------------------------*/
void speed_adjust(int16 c_speed, int16 s_speed) {
    changed_speed = MINMAX(changed_speed, -c_speed, c_speed);

    k = fabs(aaddcc.err_dir / 60.0f);
    if (changed_speed > 0) {
        set_leftspeed = test_speed - changed_speed * (1 + k);
        set_rightspeed = test_speed + changed_speed;
    }
    else {
        set_leftspeed = test_speed - changed_speed;
        set_rightspeed = test_speed + changed_speed * (1 + k);
    }

    set_leftspeed = MINMAX(set_leftspeed, -100, s_speed);
    set_rightspeed = MINMAX(set_rightspeed, -100, s_speed);
}

/*---------------------------------------------------------------------------
 * 车辆控制参数初始化（从EEPROM读取）
 *---------------------------------------------------------------------------*/
void CarControl_Init(void) {
    // 读取风扇控制标志
    flag_suction_fan_off = (uint8)(Config_ReadFloat(7, 0x176) > 0 ? Config_ReadFloat(7, 0x176) + 0.5f : flag_suction_fan_off_iap);
}

/*---------------------------------------------------------------------------
 * 保存车辆控制参数到EEPROM
 *---------------------------------------------------------------------------*/
void CarControl_SaveConfig(void) {
    flag_suction_fan_off_iap = flag_suction_fan_off;
    eeprom_write_float_ascii((float)flag_suction_fan_off, 3, 1, 0x176);
}
