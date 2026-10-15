/*
 * 模块功能：感知链路（lidar 滤波预处理 + 激光融合）的参数加载。
 * 数据来源：
 *   - config/dwa_planner.yaml 的 laser_scan_fusion 段（ros__parameters）
 *   - config/lidar_filter.yaml 的 left_lidar_preprocessor / right_lidar_preprocessor 段
 * 默认值逐条对应：
 *   - native/laser_scan_fusion.cpp 的 LaserScanFusionNode::configure_mount
 *   - native/laser_scan_preprocessor.cpp 的 LaserScanPreprocessor 构造函数
 * （P5；HANDOFF 遗留疑点 3：laser_scan_fusion 的 yaml 段。）
 */
#ifndef DWA_PERCEPTION_PARAMS_H
#define DWA_PERCEPTION_PARAMS_H

#include <stdbool.h>

#include "dwa_yaml.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DWA_PERCEPTION_STRING_MAX 96

/*
 * laser_scan_fusion 段。
 * mount 的 offset 已从 deg 转 rad（对应 configure_mount 里
 * declare_parameter(...angle_offset_deg, 0.0) * kPi / 180.0）。
 */
typedef struct {
  char left_scan_topic[DWA_PERCEPTION_STRING_MAX];
  char right_scan_topic[DWA_PERCEPTION_STRING_MAX];
  char output_topic[DWA_PERCEPTION_STRING_MAX];
  char frame_id[DWA_PERCEPTION_STRING_MAX];
  float left_laser_x, left_laser_y, left_laser_yaw;
  bool left_laser_angle_reversed;
  float left_laser_angle_offset_deg;   /* yaml 原值 deg */
  float left_laser_angle_offset;       /* 已转 rad */
  float right_laser_x, right_laser_y, right_laser_yaw;
  bool right_laser_angle_reversed;
  float right_laser_angle_offset_deg;  /* yaml 原值 deg */
  float right_laser_angle_offset;      /* 已转 rad */
} dwa_fusion_params_t;

/* 融合节点 declare_parameter 默认值（注意：x/y 默认 0.50/±0.27，
 * 与 DWA 节点的 LaserMount 默认 0.48/±0.25 不同）。 */
void dwa_fusion_params_defaults(dwa_fusion_params_t *p);

/* 从 yaml（dwa_planner.yaml）加载 laser_scan_fusion 段；
 * 文件打不开返回 -1，其余 0。 */
int dwa_fusion_params_load(const char *yaml_path, dwa_fusion_params_t *p);

/*
 * lidar_filter.yaml 单侧 preprocessor 段。
 * sector_start/end 已从 deg 转 rad；window_size 已应用"偶数则 +1"的
 * 构造函数语义（奇数化），sector_count 已应用 "<0 归 0"。
 */
typedef struct {
  char input_topic[DWA_PERCEPTION_STRING_MAX];
  char output_topic[DWA_PERCEPTION_STRING_MAX];
  char sector_topic[DWA_PERCEPTION_STRING_MAX];
  int window_size;                 /* 已奇数化 */
  float stddev_multiplier;
  int min_valid_samples;
  int min_cluster_size;            /* filter_min_cluster_size */
  float max_neighbor_gap;          /* filter_max_neighbor_gap */
  int sector_count;
  float sector_start_deg, sector_end_deg;
  float sector_start_rad, sector_end_rad;
} dwa_lidar_filter_params_t;

/* preprocessor declare_parameter 默认值。 */
void dwa_lidar_filter_params_defaults(dwa_lidar_filter_params_t *p);

/* 从 lidar_filter.yaml 加载指定侧（is_left：true=left_lidar_preprocessor，
 * false=right_lidar_preprocessor）；文件打不开返回 -1。 */
int dwa_lidar_filter_params_load(
  const char *yaml_path, bool is_left, dwa_lidar_filter_params_t *p);

#ifdef __cplusplus
}
#endif

#endif /* DWA_PERCEPTION_PARAMS_H */
