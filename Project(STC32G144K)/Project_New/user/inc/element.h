#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "zf_common_typedef.h"
#include "quaternion.h"

typedef enum
{
    ELEMENT_SEESAW = 0,
    ELEMENT_CYLINDER = 1,
    ELEMENT_HUANDAO = 2,
    ELEMENT_WALL = 3,
    ELEMENT_DONE = 4
} element_type_t;

void Element_Init(void);
void Element_ImuUpdate(const imu_sample_t *sample);
uint8 Element_AdcUpdate(void);
uint8 Element_IsStraightHold(void);
void Element_PrepareControl(int16 straight_speed,
                            int16 *target_speed,
                            int16 *direction_diff);
void Element_ClampWheelTargets(int16 center_speed,
                               int16 *left_speed,
                               int16 *right_speed);

element_type_t Element_GetCurrent(void);
uint8 Element_GetRouteIndex(void);

extern uint16 suction_fan_pwm_cylinder;

#endif
