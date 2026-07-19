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

element_type_t Element_GetCurrent(void);
uint8 Element_GetRouteIndex(void);
uint8 Element_GetRouteCount(void);

#endif
