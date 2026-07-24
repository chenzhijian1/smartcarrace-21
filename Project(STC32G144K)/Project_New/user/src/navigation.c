#include "headfile.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265f
#endif

PathPoint path_points[MAX_PATH_POINTS] = {0};

uint16 path_point_count = 0;
uint16 j = 0; // 用于回放时的路径点索引

static uint8 record_state = RECORD_SAMPLING;
static float last_sample_distance = 0.0f;    // 上次采样距离
static uint8 lap_complete = 0;               // 一圈完成标志

// 回放相关变量
uint8 flag_end = 0;
uint8 send_flag_nav = 0;
uint8 send_curvature_flag = 0; // 用于发送曲率数据
uint8 flag_fast_start = 0;

// 角度比相关宏定义
#define ANGLE_RATIO_STRAIGHT_MAX 0.7f    // 直道最大角度比（度/6cm）
#define ANGLE_RATIO_SMALLCURVE_MAX 2.5f  // 小弯最大角度比（度/6cm）
#define ANGLE_RATIO_RIGHTCURVE_MAX 4.0f  // 直角弯最大角度比（度/6cm）
#define ANGLE_RATIO_STRAIGHT_TO_SMALLCURVE_INV (1.0f/(ANGLE_RATIO_SMALLCURVE_MAX-ANGLE_RATIO_STRAIGHT_MAX)) // 线性插值系数
#define ANGLE_RATIO_SMALLCURVE_TO_RIGHTCURVE_INV (1.0f/(ANGLE_RATIO_RIGHTCURVE_MAX-ANGLE_RATIO_SMALLCURVE_MAX)) // 线性插值系数

// 简化的角度变化率计算函数（固定距离间隔）
float calculate_simple_angle_ratio(float yaw1, float yaw2) {
    // 计算角度变化（直接相减）
    float angle_change = yaw2 - yaw1;
    
    // 取绝对值，避免fabs函数
    float abs_angle_change = (angle_change >= 0) ? angle_change : -angle_change;
    
    // 角度变化率 = 角度变化 / 固定距离间隔（度/6cm）
    // SAMPLE_DISTANCE 约等于34编码器值，对应6cm
    return abs_angle_change / DISTANCE_SEG;
}

// 根据角度比计算速度
float angle_ratio_to_speed(float ratio) {
    float weight;
    if (ratio <= ANGLE_RATIO_STRAIGHT_MAX) {
        return speed_high;
    }
    else if (ratio <= ANGLE_RATIO_SMALLCURVE_MAX) {
        // 线性插值：直道到小弯之间，从600到450
        weight = (ratio - ANGLE_RATIO_STRAIGHT_MAX) * ANGLE_RATIO_STRAIGHT_TO_SMALLCURVE_INV;
        return speed_high - weight * (speed_high - speed_low);
    }
    else if (ratio <= ANGLE_RATIO_RIGHTCURVE_MAX) {
        // 线性插值：小弯到直角弯之间，从450到350
        weight = (ratio - ANGLE_RATIO_SMALLCURVE_MAX) * ANGLE_RATIO_SMALLCURVE_TO_RIGHTCURVE_INV;
        return speed_low - weight * (speed_low - speed_90);
    }
    else {
        return speed_S; // 急弯
    }
}

// 路径记录函数 - 每隔6cm采样
void Path_record(void) {
    float current_ratio = 0.0f;
    
    // 检查是否需要采样新点（每6cm一次）
    // if (record_state == RECORD_SAMPLING && 
    if (encoder_ave - last_sample_distance >= SAMPLE_DISTANCE) {
        
        // 采样新的路径点
        if (path_point_count < MAX_PATH_POINTS) {
            path_points[path_point_count].yaw_absolute = euler.yaw;
            path_point_count++;
            last_sample_distance = encoder_ave;
            
            send_curvature_flag = 1;
        }
    }
    
    // 检查是否完成一圈（通过磁钢检测或其他方式）
    // if (flag == 5 && record_state == RECORD_SAMPLING) {
    //     record_state = RECORD_COMPLETE;
    //     flag_end = 1; // 标记可以开始回放
    //     send_flag_nav = 1; // 导航结束标志位
    //     nav_end_flag_sent = 0;
    //     write_path();
    // }
}

// 大幅简化的快速循迹函数（带前瞻）
void fast_tracking(void) {
    uint16 current_index;
    uint16 i;
    float angle_ratio;
    float max_ratio = 0.0f;  // 前瞻范围内的最大角度比
    uint8 lookahead_points = (uint8)(normal_speed / 60.0f); // 前瞻点数，基于速度计算
    float current_distance;

    // 只有有路径点时才开始回放
    if (path_point_count == 0) return;
    
    // 检查是否已经完成回放
    // 如果已完成一圈回放，重置状态，重新开始记录
    if (j >= path_point_count) {
        refresh();
        return;
    }
    
    // 计算当前应该对应的距离位置
    current_distance = encoder_ave;
    
    // 寻找当前位置对应的路径点（基于固定间隔）
    current_index = (uint16)(current_distance / SAMPLE_DISTANCE);
    if (current_index >= path_point_count) {
        current_index = path_point_count - 1;
    }
    
    // 更新当前路径点索引
    j = current_index;
    
    // 前瞻速度决策：检查当前点及前方几个点的角度比
    if (current_index < path_point_count - 1) {
        // 在前瞻范围内寻找最大的角度比（最急的弯道）
        for (i = current_index; i < path_point_count - 1 && i < current_index + lookahead_points; i++) {
            angle_ratio = calculate_simple_angle_ratio(path_points[i].yaw_absolute, path_points[i + 1].yaw_absolute);

            // 保留最大的比值（最急的转弯）
            if (angle_ratio > max_ratio) {
                max_ratio = angle_ratio;
            }
        }
        
        // 根据前瞻范围内最急的弯道来设定速度
        normal_speed = angle_ratio_to_speed(max_ratio);
    }
    else {
        // 最后一个点，设为中等速度
        normal_speed = speed_low;
    }
}

void refresh(void) {
    encoder_clear();
    Yaw_Reset();
    
    // 重置记录状态
    record_state = RECORD_SAMPLING;
    last_sample_distance = 0.0f;
    lap_complete = 0;
    path_point_count = 0;
    j = 0;
    flag_end = 0;
}

void write_path(void) {
    // 将路径数据写入 EEPROM，起始地址选择 0x200，避免与 my_peripheral.c 中参数区冲突
    uint16 addr = 0x200;
    uint16 page_addr;
    uint16 i;

    // 计算最多可能用到的页并提前擦除，最多 500 个点，大约 8 kB
    // 循环擦除 EEPROM 页面
    for (page_addr = 0x200; page_addr <= 0x2600; page_addr += 0x200)
        iap_erase_page(page_addr);

    /*
     * 先存储路径点数量，方便读取复原
     * 采用 eeprom_write_float_ascii 写入：整数位 3 位，小数 0 位
     * 长度 = num + pointnum + 3 = 3 + 0 + 3 = 6 字节
     */
    eeprom_write_float_ascii((float)path_point_count, 3, 0, addr);
    addr = 0x400;

    /*
     * 依次写入各个路径点：
     * yaw      : 整数 3 位，小数 1 位  -> 3 + 1 + 3 = 7 字节
     * 总计每点 7 字节（只存储yaw，不存储distance）
     */
    for (i = 0; i < path_point_count; i++) {
        // 只写入角度数据
        eeprom_write_float_ascii((float)path_points[i].yaw_absolute, 3, 1, addr);
        addr += 7;

        // 每写入73个点后，强制跳转到下一个EEPROM页的起始地址（加上初始偏移）
        // 73个点 * 7字节 = 511字节，接近512字节页面大小
        if ((i + 1) % 73 == 0 && (i + 1) < path_point_count) {
            addr = 0x400 + ((i + 1) / 73) * 0x200;
        }
    }
}

void read_path(void) {
    uint16 addr = 0x200;
    uint16 i;

    /* 读取路径点数量 */
    path_point_count = (uint16)Config_ReadFloat(6, addr);
    if (path_point_count > MAX_PATH_POINTS) {
        path_point_count = MAX_PATH_POINTS;
    }
    if (!debug_mode)
        printf("%d\r\n", path_point_count);
    addr = 0x400;

    /* 按顺序读取各路径点数据 */
    for (i = 0; i < path_point_count; i++) {
        /* yaw 3 整 1 小 : 7 字节 */
        path_points[i].yaw_absolute = Config_ReadFloat(7, addr);
        addr += 7;

        // 每读取73个点后，强制跳转到下一个EEPROM页的起始地址（加上初始偏移）
        // 73个点 * 7字节 = 511字节
        if ((i + 1) % 73 == 0 && (i + 1) < path_point_count) {
            addr = 0x400 + ((i + 1) / 73) * 0x200;
        }
    }
    for (i = 0; i < path_point_count; i++)
        if (!debug_mode)
            printf("Point %d: Yaw=%.1f\r\n", i, path_points[i].yaw_absolute);

    // 读取完成后，设置相关状态
    record_state = RECORD_COMPLETE;
    flag_end = 0;
    j = 0; // 重置回放索引
}
