/*
 * 模块功能：激光距离滤波核心：范围校正、统计离群点剔除和扇区最小值计算。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 *
 * 直译自 native/laser_filter_core.hpp（P5）。全 float 规范（P4.5）：
 * double→float、数学函数 f 后缀、字面量 f 后缀。
 * 注意：原版统计累加器（sum/squared_sum/mean/variance/stddev）为 double，
 * 按全库 float 硬要求转 float；刀锋翻转按 P4.5 方法论在 parity 审查。
 */
#ifndef DWA_LASER_FILTER_CORE_H
#define DWA_LASER_FILTER_CORE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FilterConfig 直译。 */
typedef struct {
  int window_size;
  float stddev_multiplier;
  int min_valid_samples;
  /* Reject isolated returns that have no spatially continuous neighbour support. */
  int min_cluster_size;
  float max_neighbor_gap;
} dwa_laser_filter_config_t;

/* 默认值：window_size=5 stddev_multiplier=1.0 min_valid_samples=3
 * min_cluster_size=3 max_neighbor_gap=0.12。 */
dwa_laser_filter_config_t dwa_laser_filter_config_default(void);

/* FilterStats 直译。 */
typedef struct {
  size_t input_count;
  size_t invalid_count;
  size_t near_range_clamped_count;
  size_t statistical_outlier_count;
  size_t output_count;
} dwa_laser_filter_stats_t;

/* 判断单个雷达距离值是否是有限数，且落在该帧声明的有效量程内。
 * 功能：判断valid、距离是否满足当前模块的有效性或安全条件。 */
bool dwa_laser_valid_range(float value, float range_min, float range_max);

/*
 * 对一帧 LaserScan 距离数组做基础量程过滤和局部统计离群点过滤。
 * 功能：对一圈距离数据进行范围校正、滑动窗口统计离群剔除并返回滤波结果。
 *
 * input/input_count：输入距离数组（可为 NaN/Inf）；output：调用方提供的输出
 * 缓冲（长度 >= input_count），返回时全部写满（NaN=unknown, +Inf=no_return,
 * 有限值=保留）；stats 可为 NULL（对应 C++ 默认参数 nullptr）。
 * 返回值恒为 input_count（输出长度与输入一致，对应 C++ vector 同长）。
 */
size_t dwa_filter_ranges(
  const float *input,
  size_t input_count,
  float range_min,
  float range_max,
  const dwa_laser_filter_config_t *config,
  float *output,
  dwa_laser_filter_stats_t *stats);

/*
 * 将完整 360 度距离数组切成多个角度扇区，输出每个扇区的最近障碍距离。
 * 功能：执行`sector_minimums`，负责本模块对应的扇区、minimums处理。
 *
 * sectors：调用方缓冲（长度 >= sector_count），每个扇区写入最近距离或 +Inf；
 * 返回写入的扇区数（sector_count <= 0 或 sector_end <= sector_start 时返回 0，
 * 对应 C++ 返回空 vector）。
 */
int dwa_sector_minimums(
  const float *ranges,
  size_t range_count,
  float angle_min,
  float angle_increment,
  float sector_start,
  float sector_end,
  int sector_count,
  float *sectors);

#ifdef __cplusplus
}
#endif

#endif /* DWA_LASER_FILTER_CORE_H */
