#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "headfile.h"

typedef enum
{
    ELEMENT_STATE_NORMAL = 0,
    ELEMENT_STATE_SUSPECT,
    ELEMENT_STATE_CIRCLE_ACTIVE,
    ELEMENT_STATE_REARM
} element_state_t;

extern volatile uint8 element_state;

extern float element_pre_h_threshold;
extern float element_circle_h_threshold;
extern float element_circle_side_delta;
extern float element_cross_v_exit_threshold;
extern float element_suspect_min_distance;
extern float element_suspect_max_distance;
extern float element_rearm_h_threshold;

uint8 element_process(void);
uint8 element_handler_is_straight(void);
void element_handler_start_rearm(void);

#endif
