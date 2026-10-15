/*
 * 模块功能：前方安全区域几何计算实现。
 * 直译自 native/safety/front_safety.hpp。
 */
#include "front_safety.h"

static inline float fts_max(float a, float b) { return a > b ? a : b; }

float dwa_collision_front_edge(const dwa_footprint_t *footprint)
{
  float physical_front = footprint->front;
  if (footprint->footrest_front > footprint->footrest_start &&
    footprint->footrest_front > physical_front)
  {
    physical_front = footprint->footrest_front;
  }
  if (footprint->toe_front > physical_front) {
    physical_front = footprint->toe_front;
  }
  return physical_front + footprint->padding;
}

float dwa_front_trigger_half_width_at(
  float x,
  const dwa_front_safety_config_t *config)
{
  const float dx = fts_max(0.0f, x - config->trigger_start_x);
  const float expand_angle = config->trigger_expand_angle_deg *
    DWA_PI / 180.0f;
  return 0.5f * config->trigger_base_width + dx * tanf(expand_angle);
}

bool dwa_point_inside_front_trigger_zone(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config)
{
  if (point->x < config->trigger_start_x) return false;
  return fabsf(point->y) <= dwa_front_trigger_half_width_at(point->x, config);
}

bool dwa_point_inside_front_safety_lane(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config,
  const dwa_footprint_t *footprint)
{
  float physical_half_width = footprint->half_width;
  const float padding = footprint->padding > 0.0f ? footprint->padding : 0.0f;
  if (footprint->footrest_front > footprint->footrest_start &&
    point->x > footprint->front && point->x <= footprint->footrest_start)
  {
    const float span = footprint->footrest_start - footprint->front;
    const float ratio = span > 0.0f ?
      (point->x - footprint->front) / span : 1.0f;
    physical_half_width = footprint->half_width + ratio *
      (footprint->footrest_half_width - footprint->half_width);
  } else if (footprint->footrest_front > footprint->footrest_start &&
    point->x > footprint->footrest_start &&
    point->x <= footprint->footrest_front)
  {
    physical_half_width = footprint->footrest_half_width;
  } else if (footprint->toe_front > footprint->footrest_front &&
    point->x > footprint->footrest_front)
  {
    physical_half_width = footprint->toe_half_width;
  }
  return point->x >= config->trigger_start_x &&
    fabsf(point->y) <= physical_half_width + padding;
}


bool dwa_point_is_front_safety_candidate(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config,
  const dwa_footprint_t *footprint,
  float self_filter_release_m)
{
  const float release = self_filter_release_m > 0.0f ? self_filter_release_m : 0.0f;
  if (point == NULL || config == NULL || footprint == NULL ||
    footprint->toe_front <= 0.0f)
  {
    return false;
  }
  return point->x >= footprint->toe_front - release &&
    dwa_point_inside_front_safety_lane(point, config, footprint);
}
