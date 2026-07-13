#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "headfile.h"

// 路径记忆数组大小
#define MAX_PATH_POINTS 300

typedef struct {
    float yaw_absolute; // 转弯处的绝对航向角 (°)
} PathPoint;

// 采样配置 - 基于编码器170=30cm的比例
#define DISTANCE_RATIO (170.0f / 30.0f)        // 编码器比例：5.67
#define DISTANCE_SEG    6.0f
#define SAMPLE_DISTANCE (DISTANCE_SEG * DISTANCE_RATIO) // 6cm采样间隔，约34编码器值

// 路径记录状态宏定义，节省内存
#define RECORD_SAMPLING 0    // 采样阶段
#define RECORD_COMPLETE 1    // 记录完成

extern PathPoint path_points[MAX_PATH_POINTS];

extern uint16 path_point_count;
extern uint16 j; // 用于回放时的路径点索引

extern uint8 flag_end; // 0表示未到达终点,1表示到达终点
extern uint8 send_flag_nav;
extern uint8 send_curvature_flag; // 用于发送曲率数据
extern uint8 flag_fast_start;

// 速度配置
extern float speed_high;
extern float speed_low;
extern float speed_90;
extern float speed_S;

// 函数声明
void Path_record(void);
void fast_tracking(void);
void refresh(void);
void write_path(void);
void read_path(void);

// 曲率分析相关函数
float calculate_simple_angle_ratio(float yaw1, float yaw2);
float angle_ratio_to_speed(float ratio);

#endif /* __NAVIGATION_H__ */
