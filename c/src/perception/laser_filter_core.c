/*
 * 模块功能：激光距离滤波核心：范围校正、统计离群点剔除和扇区最小值计算。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 *
 * 逐行直译自 native/laser_filter_core.hpp（P5），全 float 规范（P4.5）。
 */
#include "laser_filter_core.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

dwa_laser_filter_config_t dwa_laser_filter_config_default(void)
{
  dwa_laser_filter_config_t config;
  config.window_size = 5;
  config.stddev_multiplier = 1.0f;
  config.min_valid_samples = 3;
  config.min_cluster_size = 3;
  config.max_neighbor_gap = 0.12f;
  return config;
}

/* 判断单个雷达距离值是否是有限数，且落在该帧声明的有效量程内。
 * 功能：判断valid、距离是否满足当前模块的有效性或安全条件。 */
bool dwa_laser_valid_range(float value, float range_min, float range_max)
{
  return isfinite(value) && value >= range_min && value <= range_max;
}

/*
 * 对一帧 LaserScan 距离数组做基础量程过滤和局部统计离群点过滤。
 * 功能：对一圈距离数据进行范围校正、滑动窗口统计离群剔除并返回滤波结果。
 */
size_t dwa_filter_ranges(
  const float *input,
  size_t input_count,
  float range_min,
  float range_max,
  const dwa_laser_filter_config_t *config,
  float *output,
  dwa_laser_filter_stats_t *stats)
{
  const float unknown = (float)NAN;
  const float no_return = (float)INFINITY;
  unsigned char *valid = NULL;
  unsigned char *protected_near_range = NULL;
  dwa_laser_filter_stats_t local_stats;
  size_t i;

  local_stats.input_count = input_count;
  local_stats.invalid_count = 0;
  local_stats.near_range_clamped_count = 0;
  local_stats.statistical_outlier_count = 0;
  local_stats.output_count = 0;

  if (input_count == 0) {
    if (stats != NULL) *stats = local_stats;
    return 0;
  }

  valid = (unsigned char *)calloc(input_count, sizeof(unsigned char));
  protected_near_range = (unsigned char *)calloc(input_count, sizeof(unsigned char));
  if (valid == NULL || protected_near_range == NULL) {
    /* calloc 失败按未知全部输出（防御，原版 vector 不会失败）。 */
    for (i = 0; i < input_count; ++i) output[i] = unknown;
    free(valid);
    free(protected_near_range);
    if (stats != NULL) *stats = local_stats;
    return input_count;
  }

  for (i = 0; i < input_count; ++i) {
    output[i] = unknown;
    if (dwa_laser_valid_range(input[i], range_min, range_max)) {
      valid[i] = 1;
      output[i] = input[i];
    } else if (isfinite(input[i]) && input[i] > 0.0f && input[i] < range_min) {
      /* 近距有限回波不能被当成空闲空间，钳位到最小量程保留为危险点。 */
      valid[i] = 1;
      protected_near_range[i] = 1;
      output[i] = range_min;
      ++local_stats.near_range_clamped_count;
    } else if (isinf(input[i]) && input[i] > 0.0f) {
      /* 真实正无穷表示无回波，保留给下游执行正常自由空间射线清除。 */
      output[i] = no_return;
    } else {
      /* NaN、负值和其他无效值只能表示未知，禁止伪装成无回波。 */
      output[i] = unknown;
      ++local_stats.invalid_count;
    }
  }

  {
    int window_size = config->window_size;
    if (window_size > 0 && window_size < 3) window_size = 3;
    if (window_size > 0 && window_size % 2 == 0) ++window_size;
    {
      const int min_valid_samples =
        config->min_valid_samples > 1 ? config->min_valid_samples : 1;
      if (window_size > 0 && config->stddev_multiplier > 0.0f) {
        const size_t half_window = (size_t)(window_size / 2);
        size_t center;
        for (center = 0; center < input_count; ++center) {
          size_t begin;
          size_t end;
          float sum = 0.0f;
          float squared_sum = 0.0f;
          int count = 0;
          float mean;
          float variance;
          float standard_deviation;
          size_t j;

          if (!valid[center] || protected_near_range[center]) continue;

          begin = center > half_window ? center - half_window : 0;
          end = center + half_window + 1;
          if (end > input_count) end = input_count;
          for (j = begin; j < end; ++j) {
            if (!valid[j]) continue;
            {
              const float value = input[j];
              sum += value;
              squared_sum += value * value;
              ++count;
            }
          }

          if (count < min_valid_samples) continue;

          mean = sum / (float)count;
          {
            const float raw = squared_sum / (float)count - mean * mean;
            variance = raw > 0.0f ? raw : 0.0f;
          }
          standard_deviation = sqrtf(variance);
          if (standard_deviation < 1e-6f) continue;

          if (fabsf(input[center] - mean) >
              config->stddev_multiplier * standard_deviation) {
            /* 离群点改为未知，不能触发下游自由空间清除。 */
            output[center] = unknown;
            ++local_stats.statistical_outlier_count;
          }
        }
      }
    }
  }

  /* D6 in open space can produce isolated finite echoes. Keep only returns that
   * belong to a short contiguous angular cluster; this does not alter range values. */
  {
    const int min_cluster_size =
      config->min_cluster_size > 1 ? config->min_cluster_size : 1;
    const float raw_gap = config->max_neighbor_gap > 0.0f ? config->max_neighbor_gap : 0.0f;
    const float max_neighbor_gap = raw_gap;
    if (min_cluster_size > 1 && max_neighbor_gap > 0.0f) {
      unsigned char *supported = (unsigned char *)calloc(input_count, sizeof(unsigned char));
      if (supported != NULL) {
        size_t i2;
        for (i2 = 0; i2 < input_count; ++i2) {
          if (!isfinite(output[i2])) continue;
          {
            int cluster = 1;
            size_t j;
            for (j = i2; j > 0; --j) {
              const size_t k = j - 1;
              if (!isfinite(output[k]) ||
                  fabsf(output[j] - output[k]) > max_neighbor_gap) break;
              ++cluster;
            }
            for (j = i2 + 1; j < input_count; ++j) {
              if (!isfinite(output[j]) ||
                  fabsf(output[j] - output[j - 1]) > max_neighbor_gap) break;
              ++cluster;
            }
            if (cluster >= min_cluster_size) supported[i2] = 1;
          }
        }
        for (i2 = 0; i2 < input_count; ++i2) {
          if (isfinite(output[i2]) && !supported[i2]) {
            output[i2] = unknown;
            ++local_stats.statistical_outlier_count;
          }
        }
        free(supported);
      }
    }
  }

  {
    size_t i3;
    for (i3 = 0; i3 < input_count; ++i3) {
      if (isfinite(output[i3])) ++local_stats.output_count;
    }
  }

  free(valid);
  free(protected_near_range);
  if (stats != NULL) *stats = local_stats;
  return input_count;
}

/*
 * 将完整 360 度距离数组切成多个角度扇区，输出每个扇区的最近障碍距离。
 * 功能：执行`sector_minimums`，负责本模块对应的扇区、minimums处理。
 */
int dwa_sector_minimums(
  const float *ranges,
  size_t range_count,
  float angle_min,
  float angle_increment,
  float sector_start,
  float sector_end,
  int sector_count,
  float *sectors)
{
  const float invalid = (float)INFINITY;
  int s;

  if (sector_count <= 0 || sector_end <= sector_start) return 0;

  for (s = 0; s < sector_count; ++s) sectors[s] = invalid;
  {
    const float span = sector_end - sector_start;
    float angle = angle_min;
    size_t i;
    for (i = 0; i < range_count; ++i) {
      if (isfinite(ranges[i]) && angle >= sector_start && angle <= sector_end) {
        int index = (int)((angle - sector_start) / span * (float)sector_count);
        if (index < 0) index = 0;
        if (index > sector_count - 1) index = sector_count - 1;
        if (ranges[i] < sectors[index]) sectors[index] = ranges[i];
      }
      angle += angle_increment;
    }
  }
  return sector_count;
}
