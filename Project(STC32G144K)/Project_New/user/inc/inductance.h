#ifndef __INDUCTANCE_H
#define __INDUCTANCE_H
#include "headfile.h"

#define NUM 5 // ADC 采样通道数量
typedef struct
{
    float last_middlle;
    float err_dir;
    float last_err_dir;
} adc_struct;

extern uint8 cnt;
extern float AD_ONE_last;
extern uint8 flag1;

void direction_adc_init(void);
void direction_adc_get(void);
extern  uint16 middle[3];
extern adc_struct aaddcc;
extern uint16 ad_ave[NUM];
extern float AD_ONE[NUM];
extern volatile uint8 flag_adc;
extern float adc_left_dir;
extern float adc_right_dir;

extern float A_, B_, C_;
#endif
