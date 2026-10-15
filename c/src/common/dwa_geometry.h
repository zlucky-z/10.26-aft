/*
 * 模块功能：DWA几何基础工具：角度归一化、车体包络、距离计算和运动积分。
 * 直译自 native/common/dwa_geometry.hpp（逐行对应）。
 * C++ 默认参数 include_toe=true → C 侧显式传参；提供带 _full 的便捷包装。
 */
#ifndef DWA_GEOMETRY_H
#define DWA_GEOMETRY_H

#include <stdbool.h>

#include "dwa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

float dwa_clamp_value(float value, float low, float high);

float dwa_normalize_angle(float angle);

float dwa_footprint_front_extent(const dwa_footprint_t *footprint, bool include_toe);
float dwa_footprint_front_extent_full(const dwa_footprint_t *footprint);

float dwa_footprint_half_width_extent(
  const dwa_footprint_t *footprint, bool include_toe);
float dwa_footprint_half_width_extent_full(const dwa_footprint_t *footprint);

bool dwa_inside_footprint_rectangle(
  float x,
  float y,
  float min_x,
  float max_x,
  float half_width,
  float padding);

/* Self-reflection filter: apply lateral padding, but never extend past toe_front. */
bool dwa_point_inside_footprint_self_filter(
  float x, float y, const dwa_footprint_t *footprint, float padding);

bool dwa_footprint_contains_local(
  float x,
  float y,
  const dwa_footprint_t *footprint,
  bool include_toe);
bool dwa_footprint_contains_local_full(
  float x, float y, const dwa_footprint_t *footprint);

float dwa_distance_to_rectangle(
  float x,
  float y,
  float min_x,
  float max_x,
  float half_width,
  float padding);

float dwa_distance_to_expanded_footprint(
  const dwa_point_t *point,
  const dwa_footprint_t *footprint);

float dwa_footprint_half_width_at_x(
  float x,
  const dwa_footprint_t *footprint,
  bool include_toe);
float dwa_footprint_half_width_at_x_full(float x, const dwa_footprint_t *footprint);

/* motion_step：按差速车运动模型向前积分一步；结果写入 *next。 */
void dwa_motion_step(
  const dwa_state_t *state, float v, float w, float dt, dwa_state_t *next);

#ifdef __cplusplus
}
#endif

#endif /* DWA_GEOMETRY_H */
