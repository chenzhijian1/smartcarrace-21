#ifndef __SPATIAL_FEATURES_H_
#define __SPATIAL_FEATURES_H_

#include "zf_common_typedef.h"

#define SPATIAL_NORM_MIN_G        (0.85f)
#define SPATIAL_NORM_MAX_G        (1.15f)
#define SPATIAL_INVERTED_AZ_MAX_G (-0.65f)
#define SPATIAL_BASELINE_PITCH_MIN_DEG  (-8.0f)
#define SPATIAL_BASELINE_PITCH_MAX_DEG  (8.0f)

float spatial_absf(float value);
int16 spatial_clamp_i16(int16 value, int16 low, int16 high);
uint8 spatial_accel_vector_norm_in_range(float ax_g,
                                         float ay_g,
                                         float az_g,
                                         float min_g,
                                         float max_g);
uint8 spatial_confirm_update(uint8 condition,
                             uint16 required_samples,
                             uint16 *count);

#endif /* __SPATIAL_FEATURES_H_ */