#ifndef __SPATIAL_FEATURES_H_
#define __SPATIAL_FEATURES_H_

#include "zf_common_typedef.h"
#include "quaternion.h"

/*
 * 立体元素共用的 IMU 派生特征。
 *
 * 本模块只负责单位统一后的低通、模长门控和姿态特征发布，不选择具体元素，
 * 也不直接写电机。跷跷板、圆筒和墙面通过同一份快照进行各自状态判断。
 */

/* ---------- 模长门控和低通 ---------- */
#define SPATIAL_FEATURES_NORM_MIN_G       (0.85f)//有效加速度模长下限，低于此值时不信任姿态判定。
#define SPATIAL_FEATURES_NORM_MAX_G       (1.15f)//有效加速度模长上限，高于此值时不信任姿态判定。
#define SPATIAL_FEATURES_FILTER_ALPHA     (0.20f)//一阶低通系数；调大响应快但更容易受振动影响，调小更稳但滞后更大。

/* ---------- 平面姿态滞回 ---------- */
#define SPATIAL_FEATURES_FLAT_AY_MAX_G    (0.15f)//进入平面要求|ay_lowpass|<=0.15g。
#define SPATIAL_FEATURES_FLAT_AZ_MIN_G    (0.85f)//进入平面要求az_lowpass>=0.85g。
#define SPATIAL_FEATURES_FLAT_AY_EXIT_G   (0.20f)//已在平面时|ay|超过0.20g才退出，形成滞回避免抖动。
#define SPATIAL_FEATURES_FLAT_AZ_EXIT_G   (0.75f)//已在平面时az低于0.75g才退出，形成滞回。

/* ---------- 近竖直姿态滞回 ---------- */
#define SPATIAL_FEATURES_VERTICAL_AY_MAX_G (-0.75f)//进入近竖直要求ay<=-0.75g，数值更负会更严格。
#define SPATIAL_FEATURES_VERTICAL_AY_EXIT_G (-0.60f)//已近竖直时ay>-0.60g才退出，避免边界来回跳变。
#define SPATIAL_FEATURES_VERTICAL_AZ_MAX_G (0.45f)//进入近竖直要求|az|<=0.45g。
#define SPATIAL_FEATURES_VERTICAL_AZ_EXIT_G (0.55f)//已近竖直时|az|>0.55g才退出，形成滞回。

/* ---------- 倒置姿态滞回 ---------- */
#define SPATIAL_FEATURES_INVERTED_AZ_MAX_G (-0.65f)//进入倒置要求az<=-0.65g。
#define SPATIAL_FEATURES_INVERTED_AZ_EXIT_G (-0.55f)//已倒置时az>-0.55g才退出，避免边界抖动。

typedef struct
{
    float ax_g;//当前原始转换值，单位g。
    float ay_g;//当前原始转换值，单位g。
    float az_g;//当前原始转换值，单位g。
    float norm_g;//三轴加速度模长，单位g。

    float ax_lowpass_g;//低通后的X轴分量，供慢速姿态判断使用。
    float ay_lowpass_g;//低通后的Y轴分量，供上坡/下坡判断使用。
    float az_lowpass_g;//低通后的Z轴分量，供平面/竖直判断使用。
    float climb_angle_deg;//atan2(-ay_lowpass_g, az_lowpass_g)得到的车头上仰角，单位度。

    float gx_dps;//X轴角速度，单位度/秒。
    float gx_abs_dps;//|gx_dps|，方便阈值比较。

    uint8 norm_valid;//模长是否落在NORM_MIN_G~NORM_MAX_G内。
    uint8 flat;//带滞回的平面姿态标志。
    uint8 vertical;//带滞回的近竖直姿态标志。
    uint8 inverted;//带滞回的倒置姿态标志。
} spatial_features_t;

void SpatialFeatures_Init(void);//上电初始化入口，清空低通初值和所有发布字段。
void SpatialFeatures_Update(const imu_sample_t *sample);//每个新IMU样本调用一次，更新低通、模长门控和滞回标志。
const spatial_features_t *SpatialFeatures_Get(void);//返回只读快照；指针保持有效到下一次Update()。
void SpatialFeatures_Reset(void);//清除低通启动状态、滞回状态和所有输出字段。

#endif /* __SPATIAL_FEATURES_H_ */
