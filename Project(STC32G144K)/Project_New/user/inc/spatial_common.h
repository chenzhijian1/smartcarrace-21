#ifndef __SPATIAL_COMMON_H_
#define __SPATIAL_COMMON_H_

#include "zf_common_typedef.h"

/* 立体元素共用的加速度、滤波和连续帧工具。 */
float spatial_accel_norm_g(float ax_g, float ay_g, float az_g);
uint8 spatial_accel_norm_in_range(float norm_g,
                                  float min_g,
                                  float max_g);
uint8 spatial_accel_vector_norm_in_range(float ax_g,
                                         float ay_g,
                                         float az_g,
                                         float min_g,
                                         float max_g);

/* 条件成立时饱和递增，失败时清零；计数变量由调用方持有。 */
uint8 spatial_confirm_update(uint8 condition,
                             uint16 required_samples,
                             uint16 *count);

float spatial_lowpass_update(float previous,
                             float input,
                             float alpha);

/* enter_min必须大于exit_max。 */
uint8 spatial_hysteresis_high_update(float value,
                                     uint8 state,
                                     float enter_min,
                                     float exit_max);

/* enter_max必须小于exit_min。 */
uint8 spatial_hysteresis_low_update(float value,
                                    uint8 state,
                                    float enter_max,
                                    float exit_min);

#endif /* __SPATIAL_COMMON_H_ */
