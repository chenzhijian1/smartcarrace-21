#include "spatial_features.h"

float spatial_absf(float value)
{
    return value >= 0.0f ? value : -value;
}

int16 spatial_clamp_i16(int16 value, int16 low, int16 high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

uint8 spatial_accel_vector_norm_in_range(float ax_g,
                                         float ay_g,
                                         float az_g,
                                         float min_g,
                                         float max_g)
{
    float norm_squared;
    float min_squared;
    float max_squared;

    norm_squared = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    min_squared = min_g * min_g;
    max_squared = max_g * max_g;

    return (uint8)(norm_squared >= min_squared &&
                   norm_squared <= max_squared);
}

uint8 spatial_confirm_update(uint8 condition,
                             uint16 required_samples,
                             uint16 *count)
{
    if (required_samples == 0U)
    {
        *count = 0;
        return 1;
    }

    if (!condition)
    {
        *count = 0;
        return 0;
    }

    if (*count < required_samples)
        (*count)++;

    return (uint8)(*count >= required_samples);
}
