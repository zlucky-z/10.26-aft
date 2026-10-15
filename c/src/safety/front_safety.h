/*
 * 模块功能：前方安全区域几何计算：车头前缘、外扩触发区和点命中判断。
 * 直译自 native/safety/front_safety.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 */
#ifndef DWA_FRONT_SAFETY_H
#define DWA_FRONT_SAFETY_H

#include <math.h>
#include <stdbool.h>

#include "../common/dwa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Parameters used by the front trigger zone and front-edge distance checks. */
typedef struct {
  float trigger_start_x;
  float trigger_base_width;
  float trigger_expand_angle_deg;
} dwa_front_safety_config_t;

/* FrontSafetyConfig{} 成员默认值直译。 */
static inline dwa_front_safety_config_t dwa_front_safety_config_default(void)
{
  dwa_front_safety_config_t config;
  config.trigger_start_x = 0.48f;
  config.trigger_base_width = 0.54f;
  config.trigger_expand_angle_deg = 5.0f;
  return config;
}

/* collision_front_edge(footprint) 直译。 */
float dwa_collision_front_edge(const dwa_footprint_t *footprint);

/* front_trigger_half_width_at(x, config) 直译。 */
float dwa_front_trigger_half_width_at(
  float x,
  const dwa_front_safety_config_t *config);

/* point_inside_front_trigger_zone(point, config) 直译。 */
bool dwa_point_inside_front_trigger_zone(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config);

/* Narrow front-safety lane follows the actual front footprint width, not the broad trigger wedge. */
bool dwa_point_inside_front_safety_lane(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config,
  const dwa_footprint_t *footprint);
/* Candidate preservation boundary matching the self-filter release strip. */
bool dwa_point_is_front_safety_candidate(
  const dwa_point_t *point,
  const dwa_front_safety_config_t *config,
  const dwa_footprint_t *footprint,
  float self_filter_release_m);

#ifdef __cplusplus
}
#endif

#endif /* DWA_FRONT_SAFETY_H */
