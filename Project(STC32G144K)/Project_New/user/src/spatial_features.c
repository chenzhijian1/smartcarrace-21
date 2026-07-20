/*
 * IMU angle / ay / az reference (assuming static 1 g and ax ~= 0)
 *
 * The implementation below uses:
 *     angle_deg = atan2(-ay_lowpass_g, az_lowpass_g) * 57.2957795
 * For an integer angle theta in degrees, the ideal sensor values are:
 *     ay = -sin(theta * PI / 180) [g]
 *     az =  cos(theta * PI / 180) [g]
 *     ay^2 + az^2 = 1.0 [g^2]
 * Therefore the correspondence is defined at every 1 degree from -90 deg
 * through +90 deg (and can be extended to +/-180 deg with the same formula):
 *
 *   theta: -90 ... -1, 0, 1 ... 90 (one-degree steps)
 *   ay:    -sin(theta deg), az: cos(theta deg)
 *
 * Key one-degree lookup examples (g):
 *   -90: ay=+1.0000, az= 0.0000    -45: ay=+0.7071, az= 0.7071
 *   -30: ay=+0.5000, az= 0.8660    -10: ay=+0.1736, az= 0.9848
 *    -1: ay=+0.0175, az= 0.9998      0: ay= 0.0000, az= 1.0000
 *     1: ay=-0.0175, az= 0.9998     10: ay=-0.1736, az= 0.9848
 *    30: ay=-0.5000, az= 0.8660     45: ay=-0.7071, az= 0.7071
 *    90: ay=-1.0000, az= 0.0000
 *
 * Sign convention: positive angle means the vehicle nose pitches upward;
 * negative angle means the nose pitches downward. In real motion, use the
 * low-pass values and check norm_valid first, because linear acceleration and
 * vibration make ay/az deviate from the ideal 1 g table.
 */
/*
 * 文件职能：公共IMU空间特征生成。
 *
 * 本文件接收统一格式的IMU样本，保存原始加速度和角速度，执行加速度
 * 低通滤波与模长有效性门控，并发布平面、近竖直、倒置等带滞回的姿态
 * 特征。它只生成观测数据，不选择具体立体元素，也不直接控制电机。
 */

#include "spatial_features.h"
#include <math.h>

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

float spatial_accel_norm_g(float ax_g, float ay_g, float az_g)
{
    return (float)sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}

uint8 spatial_accel_norm_in_range(float norm_g,
                                  float min_g,
                                  float max_g)
{
    return (uint8)(norm_g >= min_g && norm_g <= max_g);
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

/* 立体元素公共特征实现：只发布观测量，不决定当前是哪一种元素。*/
#define SPATIAL_FEATURES_RAD_TO_DEG (57.2957795f) // 弧度转角度的固定比例。

/* 当前发布的特征快照和低通/滞回内部状态。*/
static spatial_features_t spatial_features = {0};
static uint8 spatial_features_initialized = 0;   // 低通滤波器是否已用首帧初始化
static uint8 spatial_features_flat_state = 0;     // 平面姿态滞回内部状态
static uint8 spatial_features_vertical_state = 0; // 近竖直姿态滞回内部状态
static uint8 spatial_features_inverted_state = 0; // 倒置姿态滞回内部状态

/* 清除特征快照、低通启动标志和三个带滞回的姿态状态。*/
void SpatialFeatures_Reset(void)
{
    spatial_features.ax_g = 0.0f;
    spatial_features.ay_g = 0.0f;
    spatial_features.az_g = 0.0f;
    spatial_features.norm_g = 0.0f;
    spatial_features.ax_lowpass_g = 0.0f;
    spatial_features.ay_lowpass_g = 0.0f;
    spatial_features.az_lowpass_g = 0.0f;
    spatial_features.climb_angle_deg = 0.0f;
    spatial_features.gx_dps = 0.0f;
    spatial_features.gx_abs_dps = 0.0f;
    spatial_features.norm_valid = 0;
    spatial_features.flat = 0;
    spatial_features.vertical = 0;
    spatial_features.inverted = 0;
    spatial_features_initialized = 0;
    spatial_features_flat_state = 0;
    spatial_features_vertical_state = 0;
    spatial_features_inverted_state = 0;
}

/* 上电初始化包装函数；实际清理工作由 Reset() 完成。*/
void SpatialFeatures_Init(void)
{
    SpatialFeatures_Reset();
}

/* 处理一帧IMU样本，提取立体元素共用的空间特征。*/
void SpatialFeatures_Update(const imu_sample_t *sample)
{
    float ay_for_angle;
    float az_for_angle;

    if (sample == (const imu_sample_t *)0)
        return;

    /* 阶段1：原始值存储与模长计算
     * 将三轴加速度(g)、角速度(deg/s)直接存入特征结构体，
     * 计算加速度向量模长 norm_g = sqrt(ax^2 + ay^2 + az^2)，正常静止约1.0g。*/
    spatial_features.ax_g = sample->ax_g;
    spatial_features.ay_g = sample->ay_g;
    spatial_features.az_g = sample->az_g;
    spatial_features.norm_g = spatial_accel_norm_g(
        sample->ax_g, sample->ay_g, sample->az_g);
    spatial_features.gx_dps = sample->gx_dps;
    spatial_features.gx_abs_dps = spatial_absf(sample->gx_dps);

    /* 阶段2：一阶低通滤波(alpha=0.20)
     * 首次调用直接用原始值初始化，避免从0缓慢爬升；
     * 后续调用 new = old + alpha * (sample - old)，滤除高频振动。*/
    if (!spatial_features_initialized)
    {
        spatial_features.ax_lowpass_g = sample->ax_g;
        spatial_features.ay_lowpass_g = sample->ay_g;
        spatial_features.az_lowpass_g = sample->az_g;
        spatial_features_initialized = 1;
    }
    else
    {
        spatial_features.ax_lowpass_g = spatial_lowpass_update(
            spatial_features.ax_lowpass_g,
            sample->ax_g,
            SPATIAL_FEATURES_FILTER_ALPHA);
        spatial_features.ay_lowpass_g = spatial_lowpass_update(
            spatial_features.ay_lowpass_g,
            sample->ay_g,
            SPATIAL_FEATURES_FILTER_ALPHA);
        spatial_features.az_lowpass_g = spatial_lowpass_update(
            spatial_features.az_lowpass_g,
            sample->az_g,
            SPATIAL_FEATURES_FILTER_ALPHA);
    }

    /* 阶段3：模长有效性门控
     * norm_valid 当 norm_g 在 [0.85g, 1.15g] 区间内时为真。
     * 所有姿态判定都要求 norm_valid 为真，防止剧烈振动或碰撞时误判。*/
    spatial_features.norm_valid = spatial_accel_norm_in_range(
        spatial_features.norm_g,
        SPATIAL_FEATURES_NORM_MIN_G,
        SPATIAL_FEATURES_NORM_MAX_G);

    /* 阶段4：三种姿态的滞回判定（Schmitt Trigger，防止边界抖动）
     * 坐标系：Z轴垂直车身向上，Y轴指向车头。
     * 平面(flat)   = 正常行驶：进入 |ay|<=0.15g 且 az>=0.85g, 退出 |ay|>0.20g 或 az<0.75g
     * 近竖直(vertical) = 车头上翘(上坡/爬圆筒)：进入 ay<=-0.75g 且 |az|<=0.45g, 退出 ay>-0.60g 或 |az|>0.55g
     * 倒置(inverted)   = 翻车：进入 az<=-0.65g, 退出 az>-0.55g */
    if (spatial_features_flat_state)
    {
        if (!spatial_features.norm_valid ||
            spatial_absf(spatial_features.ay_lowpass_g) >
                SPATIAL_FEATURES_FLAT_AY_EXIT_G ||
            spatial_features.az_lowpass_g < SPATIAL_FEATURES_FLAT_AZ_EXIT_G)
            spatial_features_flat_state = 0;
    }
    else if (spatial_features.norm_valid &&
             spatial_absf(spatial_features.ay_lowpass_g) <=
                 SPATIAL_FEATURES_FLAT_AY_MAX_G &&
             spatial_features.az_lowpass_g >= SPATIAL_FEATURES_FLAT_AZ_MIN_G)
    {
        spatial_features_flat_state = 1;
    }

    if (spatial_features_vertical_state)
    {
        if (!spatial_features.norm_valid ||
            spatial_features.ay_lowpass_g >
                SPATIAL_FEATURES_VERTICAL_AY_EXIT_G ||
            spatial_absf(spatial_features.az_lowpass_g) >
                SPATIAL_FEATURES_VERTICAL_AZ_EXIT_G)
            spatial_features_vertical_state = 0;
    }
    else if (spatial_features.norm_valid &&
             spatial_features.ay_lowpass_g <=
                 SPATIAL_FEATURES_VERTICAL_AY_MAX_G &&
             spatial_absf(spatial_features.az_lowpass_g) <=
                 SPATIAL_FEATURES_VERTICAL_AZ_MAX_G)
    {
        spatial_features_vertical_state = 1;
    }

    if (spatial_features_inverted_state)
    {
        if (!spatial_features.norm_valid ||
            spatial_features.az_lowpass_g >
                SPATIAL_FEATURES_INVERTED_AZ_EXIT_G)
            spatial_features_inverted_state = 0;
    }
    else if (spatial_features.norm_valid &&
             spatial_features.az_lowpass_g <=
                 SPATIAL_FEATURES_INVERTED_AZ_MAX_G)
    {
        spatial_features_inverted_state = 1;
    }

    spatial_features.flat = spatial_features_flat_state;
    spatial_features.vertical = spatial_features_vertical_state;
    spatial_features.inverted = spatial_features_inverted_state;

    /* 阶段5：爬升角度计算
     * climb_angle_deg = atan2(-ay_lowpass, az_lowpass) * 57.3度
     * 正值=车头上仰(爬坡/爬圆筒)，负值=车头下俯。
     * 当 ay、az 都接近 0 时(如自由落体)直接返回0度避免除零异常。*/
    ay_for_angle = spatial_features.ay_lowpass_g;
    az_for_angle = spatial_features.az_lowpass_g;
    if (spatial_absf(ay_for_angle) < 0.0001f &&
        spatial_absf(az_for_angle) < 0.0001f)
    {
        spatial_features.climb_angle_deg = 0.0f;
    }
    else
    {
        spatial_features.climb_angle_deg = (float)atan2(
            -ay_for_angle, az_for_angle) * SPATIAL_FEATURES_RAD_TO_DEG;
    }
}

/* 返回当前只读特征快照；下一次 Update() 前内容保持不变。*/
const spatial_features_t *SpatialFeatures_Get(void)
{
    return &spatial_features;
}

/*
 * IMU倾角逐1度查表（单位：g；静止/匀速且加速度模长约为1g）
 * 正角度=车头上仰，负角度=车头下俯。实际值应使用低通后的ay、az。
 * 角度    ay(g)     az(g)
 *  -90    1.0000    0.0000
 *  -89    0.9998    0.0175
 *  -88    0.9994    0.0349
 *  -87    0.9986    0.0523
 *  -86    0.9976    0.0698
 *  -85    0.9962    0.0872
 *  -84    0.9945    0.1045
 *  -83    0.9925    0.1219
 *  -82    0.9903    0.1392
 *  -81    0.9877    0.1564
 *  -80    0.9848    0.1736
 *  -79    0.9816    0.1908
 *  -78    0.9781    0.2079
 *  -77    0.9744    0.2250
 *  -76    0.9703    0.2419
 *  -75    0.9659    0.2588
 *  -74    0.9613    0.2756
 *  -73    0.9563    0.2924
 *  -72    0.9511    0.3090
 *  -71    0.9455    0.3256
 *  -70    0.9397    0.3420
 *  -69    0.9336    0.3584
 *  -68    0.9272    0.3746
 *  -67    0.9205    0.3907
 *  -66    0.9135    0.4067
 *  -65    0.9063    0.4226
 *  -64    0.8988    0.4384
 *  -63    0.8910    0.4540
 *  -62    0.8829    0.4695
 *  -61    0.8746    0.4848
 *  -60    0.8660    0.5000
 *  -59    0.8572    0.5150
 *  -58    0.8480    0.5299
 *  -57    0.8387    0.5446
 *  -56    0.8290    0.5592
 *  -55    0.8192    0.5736
 *  -54    0.8090    0.5878
 *  -53    0.7986    0.6018
 *  -52    0.7880    0.6157
 *  -51    0.7771    0.6293
 *  -50    0.7660    0.6428
 *  -49    0.7547    0.6561
 *  -48    0.7431    0.6691
 *  -47    0.7314    0.6820
 *  -46    0.7193    0.6947
 *  -45    0.7071    0.7071
 *  -44    0.6947    0.7193
 *  -43    0.6820    0.7314
 *  -42    0.6691    0.7431
 *  -41    0.6561    0.7547
 *  -40    0.6428    0.7660
 *  -39    0.6293    0.7771
 *  -38    0.6157    0.7880
 *  -37    0.6018    0.7986
 *  -36    0.5878    0.8090
 *  -35    0.5736    0.8192
 *  -34    0.5592    0.8290
 *  -33    0.5446    0.8387
 *  -32    0.5299    0.8480
 *  -31    0.5150    0.8572
 *  -30    0.5000    0.8660
 *  -29    0.4848    0.8746
 *  -28    0.4695    0.8829
 *  -27    0.4540    0.8910
 *  -26    0.4384    0.8988
 *  -25    0.4226    0.9063
 *  -24    0.4067    0.9135
 *  -23    0.3907    0.9205
 *  -22    0.3746    0.9272
 *  -21    0.3584    0.9336
 *  -20    0.3420    0.9397
 *  -19    0.3256    0.9455
 *  -18    0.3090    0.9511
 *  -17    0.2924    0.9563
 *  -16    0.2756    0.9613
 *  -15    0.2588    0.9659
 *  -14    0.2419    0.9703
 *  -13    0.2250    0.9744
 *  -12    0.2079    0.9781
 *  -11    0.1908    0.9816
 *  -10    0.1736    0.9848
 *   -9    0.1564    0.9877
 *   -8    0.1392    0.9903
 *   -7    0.1219    0.9925
 *   -6    0.1045    0.9945
 *   -5    0.0872    0.9962
 *   -4    0.0698    0.9976
 *   -3    0.0523    0.9986
 *   -2    0.0349    0.9994
 *   -1    0.0175    0.9998
 *    0    0.0000    1.0000
 *    1   -0.0175    0.9998
 *    2   -0.0349    0.9994
 *    3   -0.0523    0.9986
 *    4   -0.0698    0.9976
 *    5   -0.0872    0.9962
 *    6   -0.1045    0.9945
 *    7   -0.1219    0.9925
 *    8   -0.1392    0.9903
 *    9   -0.1564    0.9877
 *   10   -0.1736    0.9848
 *   11   -0.1908    0.9816
 *   12   -0.2079    0.9781
 *   13   -0.2250    0.9744
 *   14   -0.2419    0.9703
 *   15   -0.2588    0.9659
 *   16   -0.2756    0.9613
 *   17   -0.2924    0.9563
 *   18   -0.3090    0.9511
 *   19   -0.3256    0.9455
 *   20   -0.3420    0.9397
 *   21   -0.3584    0.9336
 *   22   -0.3746    0.9272
 *   23   -0.3907    0.9205
 *   24   -0.4067    0.9135
 *   25   -0.4226    0.9063
 *   26   -0.4384    0.8988
 *   27   -0.4540    0.8910
 *   28   -0.4695    0.8829
 *   29   -0.4848    0.8746
 *   30   -0.5000    0.8660
 *   31   -0.5150    0.8572
 *   32   -0.5299    0.8480
 *   33   -0.5446    0.8387
 *   34   -0.5592    0.8290
 *   35   -0.5736    0.8192
 *   36   -0.5878    0.8090
 *   37   -0.6018    0.7986
 *   38   -0.6157    0.7880
 *   39   -0.6293    0.7771
 *   40   -0.6428    0.7660
 *   41   -0.6561    0.7547
 *   42   -0.6691    0.7431
 *   43   -0.6820    0.7314
 *   44   -0.6947    0.7193
 *   45   -0.7071    0.7071
 *   46   -0.7193    0.6947
 *   47   -0.7314    0.6820
 *   48   -0.7431    0.6691
 *   49   -0.7547    0.6561
 *   50   -0.7660    0.6428
 *   51   -0.7771    0.6293
 *   52   -0.7880    0.6157
 *   53   -0.7986    0.6018
 *   54   -0.8090    0.5878
 *   55   -0.8192    0.5736
 *   56   -0.8290    0.5592
 *   57   -0.8387    0.5446
 *   58   -0.8480    0.5299
 *   59   -0.8572    0.5150
 *   60   -0.8660    0.5000
 *   61   -0.8746    0.4848
 *   62   -0.8829    0.4695
 *   63   -0.8910    0.4540
 *   64   -0.8988    0.4384
 *   65   -0.9063    0.4226
 *   66   -0.9135    0.4067
 *   67   -0.9205    0.3907
 *   68   -0.9272    0.3746
 *   69   -0.9336    0.3584
 *   70   -0.9397    0.3420
 *   71   -0.9455    0.3256
 *   72   -0.9511    0.3090
 *   73   -0.9563    0.2924
 *   74   -0.9613    0.2756
 *   75   -0.9659    0.2588
 *   76   -0.9703    0.2419
 *   77   -0.9744    0.2250
 *   78   -0.9781    0.2079
 *   79   -0.9816    0.1908
 *   80   -0.9848    0.1736
 *   81   -0.9877    0.1564
 *   82   -0.9903    0.1392
 *   83   -0.9925    0.1219
 *   84   -0.9945    0.1045
 *   85   -0.9962    0.0872
 *   86   -0.9976    0.0698
 *   87   -0.9986    0.0523
 *   88   -0.9994    0.0349
 *   89   -0.9998    0.0175
 *   90   -1.0000    0.0000
 * 计算定义：climb_angle_deg=atan2(-ay_lowpass_g,az_lowpass_g)。
 */
