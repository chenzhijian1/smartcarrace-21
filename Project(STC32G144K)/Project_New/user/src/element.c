#include "element.h"
#include "seesaw.h"
#include "cylinder.h"
#include "huandao.h"
#include "wall.h"
#include "car_control.h"
#include "my_motor.h"
#include "voltage.h"

/* 赛道顺序只在这里配置，允许重复同一种元素。 */
static const element_type_t element_route[] =
{

    ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,

 /*  ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,
   ELEMENT_SEESAW,*/
  //  ELEMENT_CYLINDER,
/*
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,

ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
*/


  /*
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
ELEMENT_SEESAW,
ELEMENT_CYLINDER,
ELEMENT_HUANDAO,
ELEMENT_WALL,
    */

 //   ELEMENT_WALL,
};

#define ELEMENT_ROUTE_COUNT \
    ((uint8)(sizeof(element_route) / sizeof(element_route[0])))

static volatile uint8 element_route_index = 0;               // 当前赛道路线索引，指向 element_route[] 中的位置
static volatile uint8 element_current_type = ELEMENT_DONE;   // 当前活跃的立体元素类型
static uint8 element_cylinder_fan_boosted = 0;               // 圆柱体风扇是否已提升功率的标志
static uint16 element_cylinder_fan_applied_pwm = 0;          // 圆柱体风扇当前实际应用的PWM值

uint16 suction_fan_pwm_cylinder = 7800;                     // 圆柱体吸风风扇目标PWM（默认满功率10000）

/* 判断是否允许重力前馈补偿：电池电压低或调试模式(flag==4)时禁止，停车时也禁止（紧急停止flag==5除外）。 */
static uint8 element_wall_fan_boosted = 0;
static uint16 element_wall_fan_applied_pwm = 0;
uint16 suction_fan_pwm_wall = 8000;

static uint8 element_feedforward_is_allowed(void)
{
    if (voltage_battery_is_low() || flag == 4)
        return 0;

    if (normal_speed == 0 && flag != 5)
        return 0;

    return 1;
}

/* 根据四元数姿态计算俯仰角正弦值，按当前元素类型调用对应模块的重力前馈补偿，最终将补偿PWM注入电机控制。 */
static void element_update_gravity_feedforward(void)
{
    float pitch_sin;
    int16 feedforward_pwm;

    if (!element_feedforward_is_allowed())
    {
        motor_set_feedforward_pwm(0);
        return;
    }

    pitch_sin = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
    feedforward_pwm = 0;

    switch ((element_type_t)element_current_type)
    {
        case ELEMENT_CYLINDER:
            Cylinder_UpdateGravityFeedforward(pitch_sin);
            feedforward_pwm = Cylinder_GetGravityFeedforwardPwm();
            break;
        case ELEMENT_WALL:
            Wall_UpdateGravityFeedforward(pitch_sin);
            feedforward_pwm = Wall_GetGravityFeedforwardPwm();
            break;
        default:
            break;
    }

    motor_set_feedforward_pwm(feedforward_pwm);
}

/* 根据圆柱体上表面检测状态切换吸风风扇：进入上表面时提升风扇功率增强吸附力，离开时恢复初始功率。 */
static void element_update_cylinder_fan(uint8 on_surface)
{
    if (on_surface && !element_cylinder_fan_boosted)
    {
        if (pwm_fan == 0 || flag_suction_fan_off)
            return;

        element_cylinder_fan_boosted = 1;
        element_cylinder_fan_applied_pwm = suction_fan_pwm_cylinder;
        if (suction_fan_pwm_cylinder == 0)
            suction_fan_off();
        else
            suction_fan_on(suction_fan_pwm_cylinder);
    }
    else if (!on_surface && element_cylinder_fan_boosted)
    {
        element_cylinder_fan_boosted = 0;

        /* 安全逻辑已关闭风机时，不在主循环中重新启动。 */
        if (pwm_fan != element_cylinder_fan_applied_pwm ||
            flag_suction_fan_off || voltage_battery_is_low() ||
            (normal_speed == 0 && flag != 5))
            return;

        suction_fan_on(suction_fan_pwm_start);
        element_cylinder_fan_applied_pwm = 0;
    }
}

/* 重置指定立体元素模块的状态并清零前馈PWM，在进入新元素或完成元素时调用。 */
/* Mirror the cylinder fan boost policy after the wall is confirmed. */
static void element_update_wall_fan(uint8 on_surface)
{
    if (on_surface && !element_wall_fan_boosted)
    {
        if (pwm_fan == 0 || flag_suction_fan_off)
            return;

        element_wall_fan_boosted = 1;
        element_wall_fan_applied_pwm = suction_fan_pwm_wall;
        if (suction_fan_pwm_wall == 0)
            suction_fan_off();
        else
            suction_fan_on(suction_fan_pwm_wall);
    }
    else if (!on_surface && element_wall_fan_boosted)
    {
        element_wall_fan_boosted = 0;

        if (pwm_fan != element_wall_fan_applied_pwm ||
            flag_suction_fan_off || voltage_battery_is_low() ||
            (normal_speed == 0 && flag != 5))
            return;

        suction_fan_on(suction_fan_pwm_start);
        element_wall_fan_applied_pwm = 0;
    }
}

static void element_reset_type(element_type_t type)
{
    motor_set_feedforward_pwm(0);

    switch (type)
    {
        case ELEMENT_SEESAW:
            Seesaw_Reset();
            break;
        case ELEMENT_CYLINDER:
            Cylinder_Reset();
            break;
        case ELEMENT_HUANDAO:
            Huandao_DetectReset();
            Huandao_Reset();
            break;
        case ELEMENT_WALL:
            Wall_Reset();
            break;
        default:
            break;
    }
}

/* 处理立体元素完成事件：验证当前元素类型一致后，切换到路线中的下一个元素；若已是最后一个元素则标记为完成。 */
static void element_complete(element_type_t completed_type)
{
    if ((element_type_t)element_current_type != completed_type)
        return;

    motor_set_feedforward_pwm(0);

    if ((uint8)(element_route_index + 1U) >= ELEMENT_ROUTE_COUNT)
    {
        element_route_index = ELEMENT_ROUTE_COUNT;
        element_current_type = ELEMENT_DONE;
        return;
    }

    element_route_index++;
    element_current_type = (uint8)element_route[element_route_index];
    element_reset_type((element_type_t)element_current_type);
}

/* 初始化所有立体元素模块，设置路线索引为第一个元素，并重置该元素状态。 */
void Element_Init(void)
{
    Seesaw_Init();
    Cylinder_Init();
    Huandao_DetectReset();
    Wall_Init();

    element_route_index = 0;
    element_current_type = (uint8)element_route[0];
    element_cylinder_fan_boosted = 0;
    element_cylinder_fan_applied_pwm = 0;
    element_wall_fan_boosted = 0;
    element_wall_fan_applied_pwm = 0;
    motor_set_feedforward_pwm(0);
    element_reset_type((element_type_t)element_current_type);
}

/* 将IMU采样数据分发给当前活跃的立体元素模块处理，同时更新圆柱体风扇和重力前馈补偿。 */
void Element_ImuUpdate(const imu_sample_t *sample)
{
    uint8 cylinder_on_surface;
    uint8 wall_on_surface;

    if (sample == (const imu_sample_t *)0)
        return;

    switch ((element_type_t)element_current_type)
    {
        case ELEMENT_SEESAW:
            (void)Seesaw_ImuUpdate(sample, euler.pitch);
            if (Seesaw_HasExited())
                element_complete(ELEMENT_SEESAW);
            break;

        case ELEMENT_CYLINDER:
            cylinder_on_surface = Cylinder_ImuUpdate(sample->ay_g,
                                                     sample->az_g,
                                                     sample->gx_dps,
                                                     euler.pitch,
                                                     euler.yaw);
            element_update_cylinder_fan(cylinder_on_surface);
            if (Cylinder_HasExited())
                element_complete(ELEMENT_CYLINDER);
            break;

        case ELEMENT_HUANDAO:
            if (Huandao_ConsumeExitEvent())
                element_complete(ELEMENT_HUANDAO);
            break;

        case ELEMENT_WALL:
            (void)Wall_ImuUpdate(sample, euler.pitch, euler.roll);
            wall_on_surface = (uint8)(Wall_IsConfirmed() &&
                                      !Wall_HasExited());
            element_update_wall_fan(wall_on_surface);
            if (Wall_HasExited())
                element_complete(ELEMENT_WALL);
            break;

        default:
            break;
    }

    element_update_gravity_feedforward();
}

/* 将ADC数据转发给当前元素模块：圆柱体用于电感检测，环岛用于入环/出环判断。 */
uint8 Element_AdcUpdate(void)
{
    if ((element_type_t)element_current_type == ELEMENT_CYLINDER)
    {
        (void)Cylinder_AdcUpdate();
        return 0;
    }

    if ((element_type_t)element_current_type == ELEMENT_HUANDAO)
        return Huandao_DetectUpdate();

    return 0;
}

/* 判断当前是否处于环岛直行保持阶段，用于环岛入环后的直行锁定。 */
uint8 Element_IsStraightHold(void)
{
    return (uint8)(
        (element_type_t)element_current_type == ELEMENT_HUANDAO &&
        Huandao_DetectIsStraightHold());
}

/* 在控制循环前，根据当前元素类型调整目标速度和方向偏差，供各元素模块施加特定控制策略。 */
void Element_PrepareControl(int16 straight_speed,
                            int16 *target_speed,
                            int16 *direction_diff)
{
    if (target_speed == (int16 *)0 || direction_diff == (int16 *)0)
        return;

    switch ((element_type_t)element_current_type)
    {
        case ELEMENT_SEESAW:
            *target_speed = Seesaw_GetSpeedTarget(*target_speed,
                                                  straight_speed);
            break;
        case ELEMENT_CYLINDER:
            *target_speed = Cylinder_GetSpeedTarget(*target_speed,
                                                    straight_speed);
            if (Cylinder_IsEntryLeftTurnGuardActive())
                *direction_diff = Cylinder_LimitPreEntryDiff(*direction_diff);
            break;
        case ELEMENT_WALL:
            *target_speed = Wall_GetSpeedTarget(*target_speed,
                                                straight_speed);
            *direction_diff = (int16)(*direction_diff +
                                      Wall_GetDirectionBias());
            break;
        default:
            break;
    }
}

/* 根据当前元素类型对左右轮目标速度进行限幅，跷跷板限制差速，墙壁限制总速度。 */
void Element_ClampWheelTargets(int16 center_speed,
                               int16 *left_speed,
                               int16 *right_speed)
{
    if ((element_type_t)element_current_type == ELEMENT_SEESAW)
        Seesaw_ClampWheelTargets(center_speed, left_speed, right_speed);
    else if ((element_type_t)element_current_type == ELEMENT_WALL)
        Wall_ClampWheelTargets(center_speed, left_speed, right_speed);
}

/* 获取当前所在的立体元素类型，供遥测输出等外部模块查询。 */
element_type_t Element_GetCurrent(void)
{
    return (element_type_t)element_current_type;
}

/* 获取当前赛道路线索引，表示当前处于第几个立体元素。 */
uint8 Element_GetRouteIndex(void)
{
    return element_route_index;
}

/* 获取赛道中立体元素的总数量。 */
uint8 Element_GetRouteCount(void)
{
    return ELEMENT_ROUTE_COUNT;
}
