/*
 * 模块功能：感知链路（lidar 滤波预处理 + 激光融合）的参数加载。
 * 详见 perception_params.h。默认值与转换语义逐条直译自
 * native/laser_scan_fusion.cpp 与 native/laser_scan_preprocessor.cpp。
 */
#include "perception_params.h"

#include <string.h>

#include "../common/dwa_types.h"

#ifndef DWA_PI
#define DWA_PI ((float)3.14159265358979323846)
#endif

static void copy_str(char *dst, const char *src)
{
  strncpy(dst, src, DWA_PERCEPTION_STRING_MAX - 1);
  dst[DWA_PERCEPTION_STRING_MAX - 1] = '\0';
}

void dwa_fusion_params_defaults(dwa_fusion_params_t *p)
{
  memset(p, 0, sizeof(*p));
  copy_str(p->left_scan_topic, "/dwa/scan_left_filtered");
  copy_str(p->right_scan_topic, "/dwa/scan_right_filtered");
  copy_str(p->output_topic, "/dwa/fused_scan_base");
  copy_str(p->frame_id, "base_link");
  /* configure_mount 默认：x 0.50（左右同）、y 左 0.27 右 -0.27、yaw 0、
   * reversed false、offset 0。 */
  p->left_laser_x = 0.50f;   p->left_laser_y = 0.27f;   p->left_laser_yaw = 0.0f;
  p->left_laser_angle_reversed = false;
  p->left_laser_angle_offset_deg = 0.0f;
  p->left_laser_angle_offset = 0.0f;
  p->right_laser_x = 0.50f;  p->right_laser_y = -0.27f; p->right_laser_yaw = 0.0f;
  p->right_laser_angle_reversed = false;
  p->right_laser_angle_offset_deg = 0.0f;
  p->right_laser_angle_offset = 0.0f;
}

int dwa_fusion_params_load(const char *yaml_path, dwa_fusion_params_t *p)
{
  dwa_yaml_doc_t doc;
  const dwa_yaml_doc_t *d = &doc;

  dwa_fusion_params_defaults(p);
  if (dwa_yaml_load_file(yaml_path, "laser_scan_fusion", &doc) < 0) return -1;

  dwa_yaml_get_string(d, "left_scan_topic", p->left_scan_topic, DWA_PERCEPTION_STRING_MAX);
  dwa_yaml_get_string(d, "right_scan_topic", p->right_scan_topic, DWA_PERCEPTION_STRING_MAX);
  dwa_yaml_get_string(d, "output_topic", p->output_topic, DWA_PERCEPTION_STRING_MAX);
  dwa_yaml_get_string(d, "frame_id", p->frame_id, DWA_PERCEPTION_STRING_MAX);

  p->left_laser_x = dwa_yaml_get_double(d, "left_laser_x", p->left_laser_x);
  p->left_laser_y = dwa_yaml_get_double(d, "left_laser_y", p->left_laser_y);
  p->left_laser_yaw = dwa_yaml_get_double(d, "left_laser_yaw", p->left_laser_yaw);
  p->left_laser_angle_reversed =
    dwa_yaml_get_bool(d, "left_laser_angle_reversed", p->left_laser_angle_reversed);
  p->left_laser_angle_offset_deg =
    dwa_yaml_get_double(d, "left_laser_angle_offset_deg", p->left_laser_angle_offset_deg);
  /* configure_mount: offset_deg * kPi / 180.0 */
  p->left_laser_angle_offset = p->left_laser_angle_offset_deg * DWA_PI / 180.0f;

  p->right_laser_x = dwa_yaml_get_double(d, "right_laser_x", p->right_laser_x);
  p->right_laser_y = dwa_yaml_get_double(d, "right_laser_y", p->right_laser_y);
  p->right_laser_yaw = dwa_yaml_get_double(d, "right_laser_yaw", p->right_laser_yaw);
  p->right_laser_angle_reversed =
    dwa_yaml_get_bool(d, "right_laser_angle_reversed", p->right_laser_angle_reversed);
  p->right_laser_angle_offset_deg =
    dwa_yaml_get_double(d, "right_laser_angle_offset_deg", p->right_laser_angle_offset_deg);
  p->right_laser_angle_offset = p->right_laser_angle_offset_deg * DWA_PI / 180.0f;
  return 0;
}

void dwa_lidar_filter_params_defaults(dwa_lidar_filter_params_t *p)
{
  memset(p, 0, sizeof(*p));
  copy_str(p->input_topic, "/scan_raw");
  copy_str(p->output_topic, "/scan_filtered");
  copy_str(p->sector_topic, "/scan_sectors");
  p->window_size = 5;
  p->stddev_multiplier = 1.0f;
  p->min_valid_samples = 3;
  p->min_cluster_size = 3;        /* filter_min_cluster_size 默认 */
  p->max_neighbor_gap = 0.12f;    /* filter_max_neighbor_gap 默认 */
  p->sector_count = 24;
  p->sector_start_deg = -180.0f;
  p->sector_end_deg = 180.0f;
  p->sector_start_rad = p->sector_start_deg * DWA_PI / 180.0f;
  p->sector_end_rad = p->sector_end_deg * DWA_PI / 180.0f;
}

int dwa_lidar_filter_params_load(
  const char *yaml_path, bool is_left, dwa_lidar_filter_params_t *p)
{
  dwa_yaml_doc_t doc;
  const dwa_yaml_doc_t *d = &doc;

  dwa_lidar_filter_params_defaults(p);
  if (dwa_yaml_load_file(
        yaml_path, is_left ? "left_lidar_preprocessor" : "right_lidar_preprocessor",
        &doc) < 0) {
    return -1;
  }

  dwa_yaml_get_string(d, "input_topic", p->input_topic, DWA_PERCEPTION_STRING_MAX);
  dwa_yaml_get_string(d, "output_topic", p->output_topic, DWA_PERCEPTION_STRING_MAX);
  dwa_yaml_get_string(d, "sector_topic", p->sector_topic, DWA_PERCEPTION_STRING_MAX);

  p->window_size = dwa_yaml_get_int(d, "window_size", p->window_size);
  p->stddev_multiplier = dwa_yaml_get_double(d, "stddev_multiplier", p->stddev_multiplier);
  p->min_valid_samples = dwa_yaml_get_int(d, "min_valid_samples", p->min_valid_samples);
  p->min_cluster_size = dwa_yaml_get_int(d, "filter_min_cluster_size", p->min_cluster_size);
  p->max_neighbor_gap = dwa_yaml_get_double(d, "filter_max_neighbor_gap", p->max_neighbor_gap);
  p->sector_count = dwa_yaml_get_int(d, "sector_count", p->sector_count);
  p->sector_start_deg = dwa_yaml_get_double(d, "sector_start_deg", p->sector_start_deg);
  p->sector_end_deg = dwa_yaml_get_double(d, "sector_end_deg", p->sector_end_deg);

  /* 构造函数语义：window_size 偶数则 +1（RCLCPP_WARN 只在 ROS 层）；sector_count<0 归 0。 */
  if (p->window_size > 0 && p->window_size % 2 == 0) ++p->window_size;
  if (p->sector_count < 0) p->sector_count = 0;

  /* degrees_to_radians（preprocessor 本地 kPi double 版；此处 float π，差异在
   * parity 输入量化下不可见，记 TRANSLATION_NOTES）。 */
  p->sector_start_rad = p->sector_start_deg * DWA_PI / 180.0f;
  p->sector_end_rad = p->sector_end_deg * DWA_PI / 180.0f;
  return 0;
}
