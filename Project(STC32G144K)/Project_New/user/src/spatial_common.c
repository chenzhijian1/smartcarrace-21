/**
 * @file    spatial_common.c
 * @brief   立体元素共用的加速度、滤波和连续帧工具函数实现。
 * @note    本文件提供加速度向量模长计算、低通滤波、连续帧确认
 *           以及滞回比较器等基础工具函数，供各传感器处理模块复用。
 */

/*
 * 文件职能：空间特征公共工具实现。
 *
 * 本文件只提供无状态的基础函数，供多个立体元素和姿态模块复用，
 * 包括加速度模长计算、范围有效性判断、低通滤波、连续帧确认以及
 * 带滞回的阈值判断。本文件不保存车辆状态，也不直接控制电机。
 */

#include "spatial_common.h"
#include <math.h>

/**
 * @brief  计算三轴加速度的向量模长（单位：g）。
 * @param  ax_g  X 轴加速度，单位 g。
 * @param  ay_g  Y 轴加速度，单位 g。
 * @param  az_g  Z 轴加速度，单位 g。
 * @return 加速度向量模长，单位 g。
 */
float spatial_accel_norm_g(float ax_g, float ay_g, float az_g)
{
    return (float)sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}

/**
 * @brief  判断加速度模长是否在指定范围内。
 * @param  norm_g  已计算的加速度模长，单位 g。
 * @param  min_g   范围下限，单位 g。
 * @param  max_g   范围上限，单位 g。
 * @return 1 表示在范围内，0 表示不在范围内。
 */
uint8 spatial_accel_norm_in_range(float norm_g,
                                  float min_g,
                                  float max_g)
{
    return (uint8)(norm_g >= min_g && norm_g <= max_g);
}

/**
 * @brief  判断加速度向量模长是否在指定范围内（平方比较，避免开方运算）。
 * @param  ax_g   X 轴加速度，单位 g。
 * @param  ay_g   Y 轴加速度，单位 g。
 * @param  az_g   Z 轴加速度，单位 g。
 * @param  min_g  范围下限，单位 g。
 * @param  max_g  范围上限，单位 g。
 * @return 1 表示在范围内，0 表示不在范围内。
 * @note   通过比较平方值来避免 sqrt() 运算，适合嵌入式实时场景。
 */
uint8 spatial_accel_vector_norm_in_range(float ax_g,
                                         float ay_g,
                                         float az_g,
                                         float min_g,
                                         float max_g)
{
    float norm_squared;  /* 模长平方 */
    float min_squared;   /* 下限平方 */
    float max_squared;   /* 上限平方 */

    norm_squared = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    min_squared = min_g * min_g;
    max_squared = max_g * max_g;

    return (uint8)(norm_squared >= min_squared &&
                   norm_squared <= max_squared);
}

/**
 * @brief  连续帧确认：条件满足时饱和递增计数，失败时清零。
 * @param  condition        当前条件是否满足（1 满足，0 不满足）。
 * @param  required_samples 需要连续满足的帧数。
 * @param  count            当前连续满足帧数的计数器（由调用方持有）。
 * @return 1 表示连续满足达到 required_samples 帧，0 表示未达到。
 * @note   若 required_samples 为 0，则直接返回 1 并清零计数器。
 *          计数器为饱和递增，不会超过 required_samples。
 */
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

/**
 * @brief  一阶低通滤波器（IIR）。
 * @param  previous  上一次滤波器的输出值。
 * @param  input     当前输入的原始值。
 * @param  alpha     滤波系数（0~1），越小平滑程度越高、响应越慢。
 * @return 滤波后的输出值。
 * @note   公式：output = previous + alpha * (input - previous)
 *          alpha 为 0 时输出保持不变，alpha 为 1 时直接输出原始值。
 */
float spatial_lowpass_update(float previous,
                             float input,
                             float alpha)
{
    if (alpha <= 0.0f)
        return previous;
    if (alpha >= 1.0f)
        return input;

    return previous + alpha * (input - previous);
}

/**
 * @brief  高阈值滞回比较器（上升沿触发）。
 * @param  value     当前输入值。
 * @param  state     当前状态（1 已触发，0 未触发）。
 * @param  enter_min 进入触发状态的阈值（必须大于 exit_max）。
 * @param  exit_max  退出触发状态的阈值（必须小于 enter_min）。
 * @return 新的状态值（1 触发，0 未触发）。
 * @note   当 state 为 0 时，value >= enter_min 才进入触发状态；
 *          当 state 为 1 时，value <= exit_max 才退出触发状态。
 *          滞回区间可防止在阈值附近频繁抖动。
 */
uint8 spatial_hysteresis_high_update(float value,
                                     uint8 state,
                                     float enter_min,
                                     float exit_max)
{
    if (state)
        return (uint8)(value > exit_max);

    return (uint8)(value >= enter_min);
}

/**
 * @brief  低阈值滞回比较器（下降沿触发）。
 * @param  value     当前输入值。
 * @param  state     当前状态（1 已触发，0 未触发）。
 * @param  enter_max 进入触发状态的阈值（必须小于 exit_min）。
 * @param  exit_min  退出触发状态的阈值（必须大于 enter_max）。
 * @return 新的状态值（1 触发，0 未触发）。
 * @note   当 state 为 0 时，value <= enter_max 才进入触发状态；
 *          当 state 为 1 时，value >= exit_min 才退出触发状态。
 *          滞回区间可防止在阈值附近频繁抖动。
 */
uint8 spatial_hysteresis_low_update(float value,
                                    uint8 state,
                                    float enter_max,
                                    float exit_min)
{
    if (state)
        return (uint8)(value < exit_min);

    return (uint8)(value <= enter_max);
}
