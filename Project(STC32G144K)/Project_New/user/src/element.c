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
    ELEMENT_SEESAW,
    ELEMENT_CYLINDER,
    ELEMENT_HUANDAO,
    ELEMENT_WALL,
};

#define ELEMENT_ROUTE_COUNT \
    ((uint8)(sizeof(element_route) / sizeof(element_route[0])))

static volatile uint8 element_route_index = 0;
static volatile uint8 element_current_type = ELEMENT_DONE;
static volatile uint8 element_lap_count = 0;
static uint8 element_cylinder_fan_boosted = 0;
static uint16 element_cylinder_fan_applied_pwm = 0;
static uint8 element_wall_fan_boosted = 0;
static uint16 element_wall_fan_applied_pwm = 0;

uint16 suction_fan_pwm_cylinder = 6800;
uint16 suction_fan_pwm_wall = 7200;
/* 目标圈数，修改此值即可设置本次运行完成多少圈。 */
uint8 element_lap_target = 1;

static uint8 element_feedforward_is_allowed(void)
{
    if (voltage_battery_is_low() || flag == CAR_STATE_LAUNCH)
        return 0;

    if (normal_speed == 0 && flag != CAR_STATE_SOFT_STOP)
        return 0;

    return 1;
}

// 根据当前元素类型和IMU姿态计算重力前馈PWM，并设置到电机控制器。
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
            feedforward_pwm = Cylinder_CalcGravityFeedforward(pitch_sin);
            break;
        case ELEMENT_WALL:
            feedforward_pwm = Wall_CalcGravityFeedforward(pitch_sin);
            break;
        default:
            break;
    }

    motor_set_feedforward_pwm(feedforward_pwm);
}

// 圆筒加大负压控制逻辑
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
            (normal_speed == 0 && flag != CAR_STATE_SOFT_STOP))
            return;

        suction_fan_on(suction_fan_pwm_start);
        element_cylinder_fan_applied_pwm = 0;
    }
}

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
            (normal_speed == 0 && flag != CAR_STATE_SOFT_STOP))
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

static void element_complete(element_type_t completed_type)
{
    if ((element_type_t)element_current_type != completed_type)
        return;

    motor_set_feedforward_pwm(0);

    if ((uint8)(element_route_index + 1U) >= ELEMENT_ROUTE_COUNT)
    {
        element_lap_count++;
        if (element_lap_count >= element_lap_target)
        {
            element_route_index = ELEMENT_ROUTE_COUNT;
            element_current_type = ELEMENT_DONE;
        }
        else
        {
            element_route_index = 0;
            element_current_type = (uint8)element_route[0];
            element_reset_type((element_type_t)element_current_type);
        }
        return;
    }

    element_route_index++;
    element_current_type = (uint8)element_route[element_route_index];
    element_reset_type((element_type_t)element_current_type);
}

void Element_Init(void)
{
    if (element_lap_target == 0)
        element_lap_target = 1;

    Seesaw_Reset();
    Cylinder_Reset();
    Huandao_DetectReset();
    Huandao_Reset();
    Wall_Reset();

    element_route_index = 0;
    element_current_type = (uint8)element_route[0];
    element_lap_count = 0;
    element_cylinder_fan_boosted = 0;
    element_cylinder_fan_applied_pwm = 0;
    element_wall_fan_boosted = 0;
    element_wall_fan_applied_pwm = 0;
    motor_set_feedforward_pwm(0);
}

void Element_ImuUpdate(const imu_sample_t *sample)
{
    uint8 cylinder_on_surface;
    uint8 wall_state;

    if (sample == (const imu_sample_t *)0)
        return;

    switch ((element_type_t)element_current_type)
    {
        case ELEMENT_SEESAW:
            Seesaw_ImuUpdate(sample, euler.pitch);
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
            Wall_ImuUpdate(sample, euler.pitch, euler.roll);
            wall_state = Wall_GetState();
            element_update_wall_fan(
                (uint8)(wall_state == WALL_STATE_LATERAL ||
                        wall_state == WALL_STATE_DESCENT));
            if (Wall_HasExited())
                element_complete(ELEMENT_WALL);
            break;

        default:
            break;
    }

    element_update_gravity_feedforward();
}

uint8 Element_AdcUpdate(void)
{
    if ((element_type_t)element_current_type == ELEMENT_CYLINDER)
    {
        Cylinder_AdcUpdate();
        return 0;
    }

    if ((element_type_t)element_current_type == ELEMENT_HUANDAO)
        return Huandao_DetectUpdate();

    return 0;
}

uint8 Element_IsStraightHold(void)
{
    return (uint8)(
        (element_type_t)element_current_type == ELEMENT_HUANDAO &&
        Huandao_DetectIsStraightHold());
}

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
        case ELEMENT_HUANDAO:
            Huandao_PrepareControl(straight_speed,
                                   target_speed,
                                   direction_diff);
            break;
        case ELEMENT_WALL:
            *target_speed = Wall_GetSpeedTarget(*target_speed,
                                                straight_speed);
            break;
        default:
            break;
    }
}

void Element_ClampWheelTargets(int16 center_speed,
                               int16 *left_speed,
                               int16 *right_speed)
{
    if ((element_type_t)element_current_type == ELEMENT_SEESAW)
        Seesaw_ClampWheelTargets(center_speed, left_speed, right_speed);
    else if ((element_type_t)element_current_type == ELEMENT_WALL)
        Wall_ClampWheelTargets(center_speed, left_speed, right_speed);
}

element_type_t Element_GetCurrent(void)
{
    return (element_type_t)element_current_type;
}

uint8 Element_GetRouteIndex(void)
{
    return element_route_index;
}

uint8 Element_GetLapCount(void)
{
    return element_lap_count;
}

uint8 Element_GetLapTarget(void)
{
    return element_lap_target;
}
