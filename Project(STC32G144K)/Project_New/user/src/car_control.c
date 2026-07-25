#include "car_control.h"
#include "config.h"
#include "huandao.h"
#include "my_motor.h"
#include "inductance.h"
#include "element.h"
#include "cylinder.h"
#include "navigation.h"
#include "quaternion.h"
#include "zf_device_imu660rc.h"

/*============================================================================
 * 模块说明：车辆控制模�?
 * 功能：状态机管理、速度策略、方向控�?
 *============================================================================*/

/*---------------------------------------------------------------------------
 * 状态标志变�?
 *---------------------------------------------------------------------------*/
uint8 flag = 0;
uint8 flag_stop = 0;
uint8 flag_key_control = 0;
uint8 flag_key_fast = 0;
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
 * 编码器相�?
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

uint8 cnt_stop = 0;
uint16 cnt_launch = 0;
static volatile int16 soft_stop_start_speed = 0;
static uint8 launch_ready = 1;
static uint8 cylinder_left_saturation_count = 0;
static uint8 cylinder_right_saturation_count = 0;
static uint8 cylinder_left_diff_protected = 0;
static uint8 cylinder_right_diff_protected = 0;

/*---------------------------------------------------------------------------
 * 风扇控制
 *---------------------------------------------------------------------------*/
uint8 flag_suction_fan_off = 0;

// EEPROM默认�?
static uint8 flag_suction_fan_off_iap = 0;

#define error_turn 17.0f
uint16 suction_fan_pwm_start = 00;
#define LAUNCH_FAN_DELAY_TICKS   400

static int16 car_control_protect_cylinder_diff(int16 direction_diff)
{
    if (Element_GetCurrent() != ELEMENT_CYLINDER ||
        !Cylinder_IsOnSurface())
    {
        cylinder_left_saturation_count = 0;
        cylinder_right_saturation_count = 0;
        cylinder_left_diff_protected = 0;
        cylinder_right_diff_protected = 0;
        return direction_diff;
    }

    if (!cylinder_left_diff_protected && direction_diff < 0 &&
        motor_left.duty1 >= CYLINDER_SATURATION_PWM_THRESHOLD &&
        motor_left.err > CYLINDER_SATURATION_ERROR_THRESHOLD)
    {
        if (cylinder_left_saturation_count <
            CYLINDER_SATURATION_CONFIRM_TICKS)
            cylinder_left_saturation_count++;
        if (cylinder_left_saturation_count >=
            CYLINDER_SATURATION_CONFIRM_TICKS)
            cylinder_left_diff_protected = 1;
    }
    else if (!cylinder_left_diff_protected)
    {
        cylinder_left_saturation_count = 0;
    }

    if (!cylinder_right_diff_protected && direction_diff > 0 &&
        motor_right.duty1 >= CYLINDER_SATURATION_PWM_THRESHOLD &&
        motor_right.err > CYLINDER_SATURATION_ERROR_THRESHOLD)
    {
        if (cylinder_right_saturation_count <
            CYLINDER_SATURATION_CONFIRM_TICKS)
            cylinder_right_saturation_count++;
        if (cylinder_right_saturation_count >=
            CYLINDER_SATURATION_CONFIRM_TICKS)
            cylinder_right_diff_protected = 1;
    }
    else if (!cylinder_right_diff_protected)
    {
        cylinder_right_saturation_count = 0;
    }

    if ((cylinder_left_diff_protected && direction_diff < 0) ||
        (cylinder_right_diff_protected && direction_diff > 0))
    {
        direction_diff = (int16)((int32)direction_diff *
            CYLINDER_SATURATION_DIFF_PERCENT / 100L);
    }

    return direction_diff;
}

/*---------------------------------------------------------------------------
 * 正常循迹模式 (flag=0)
 *---------------------------------------------------------------------------*/
void CarControl_NormalMode(int16 c_speed, int16 s_speed) {
    if (Element_IsStraightHold()) {
        changed_speed = 0;
        normal_speed_cal = normal_speed;
        normal_speed_pre = normal_speed;
        test_speed = normal_speed;
        set_leftspeed = normal_speed;
        set_rightspeed = normal_speed;
        return;
    }

    dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, gyro_z);

    // 速度策略
    normal_speed_cal = (int16)-s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;

    normal_speed_pre = normal_speed;
    test_speed = (int16)normal_speed_cal;
    Element_PrepareControl(normal_speed, &test_speed, &changed_speed);

    // if (flag_key_fast == 1) {
    //     speed_adjust(120, 600);
    // }
    // else {
        speed_adjust(c_speed, s_speed);
        Element_ClampWheelTargets(test_speed, &set_leftspeed, &set_rightspeed);
    // }
}

/*---------------------------------------------------------------------------
 * 起步发车模式 (flag=4)
 *---------------------------------------------------------------------------*/
void CarControl_LaunchMode(void) {
    uint16 fan_pwm;

    if (cnt_launch < LAUNCH_FAN_DELAY_TICKS) {
        cnt_launch++;

        if (flag_suction_fan_off == 0) {
            fan_pwm = (uint16)((uint32)suction_fan_pwm_start * cnt_launch / LAUNCH_FAN_DELAY_TICKS);
            suction_fan_on(fan_pwm);
        }
    }

    if (cnt_launch >= LAUNCH_FAN_DELAY_TICKS) {
        cnt_launch = 0;
        flag = 0;
    }
}

/*---------------------------------------------------------------------------
 * 慢速停车模�?(flag=5)
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
    if (normal_speed == 0) {
        launch_ready = 1;
        cnt_launch = 0;
        suction_fan_off();
    }
    else if (launch_ready) {
        launch_ready = 0;
        cnt_launch = 0;
        flag = 4;
    }

    if (flag_stop == 0) {
        // 使用四元数解算的角速度数据（已在IMU_Update中处理）
        // 从IMU963读取的陀螺仪原始数据并转�?
        // IMU963RA陀螺仪±2000dps量程，灵敏度70 mdps/LSB = 0.07 dps/LSB
        gyro_z = IMU_GetGyroZDps();
        if (gyro_z > -0.2f && gyro_z < 0.2f)
            gyro_z = 0.0f;

        switch (flag) {
            case 0:  // 正常模式
                CarControl_NormalMode(250, 1000);
                break;
            
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

            case 4:  // 起步发车
                CarControl_LaunchMode();
                break;

            case 5:  // 慢速停�?
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
    changed_speed = car_control_protect_cylinder_diff(changed_speed);
}

/*---------------------------------------------------------------------------
 * 速度调整(差速计�?
 *---------------------------------------------------------------------------*/
void speed_adjust(int16 c_speed, int16 s_speed) {
    changed_speed = MINMAX(changed_speed, -c_speed, c_speed);

    k = fabs(aaddcc.err_dir / 40.0f);
    // k = 0;
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
 * 车辆控制参数初始化（从EEPROM读取�?
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
