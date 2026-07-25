#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "zf_common_typedef.h"
#include "quaternion.h"

/* 0: pure line-following test; 1: enable all element logic. */
#define ELEMENT_ENABLE (1U)

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
void Element_PrepareControl(int16 straight_speed,
                            int16 *target_speed,
                            int16 *direction_diff,
                            int16 *direction_diff_limit);
void Element_ClampWheelTargets(int16 center_speed,
                               int16 *left_speed,
                               int16 *right_speed);

element_type_t Element_GetCurrent(void);
uint8 Element_GetRouteIndex(void);
uint8 Element_GetLapCount(void);
uint8 Element_GetLapTarget(void);
uint8 Element_GetReverseRun(void);

extern uint16 suction_fan_pwm_wall;
extern uint8 element_lap_target;
extern uint8 element_reverse_run;

#endif
