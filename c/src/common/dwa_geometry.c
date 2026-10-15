/* 直译自 native/common/dwa_geometry.hpp，逻辑/顺序不变。 */
#include "dwa_geometry.h"

static float dwa_min_double(float a, float b) { return b < a ? b : a; }
static float dwa_max_double(float a, float b) { return a < b ? b : a; }

float dwa_clamp_value(float value, float low, float high)
{
  return dwa_max_double(low, dwa_min_double(high, value));
}

float dwa_normalize_angle(float angle)
{
  while (angle > DWA_PI) angle -= 2.0f * DWA_PI;
  while (angle < -DWA_PI) angle += 2.0f * DWA_PI;
  return angle;
}

float dwa_footprint_front_extent(const dwa_footprint_t *footprint, bool include_toe)
{
  float front = footprint->front;
  if (footprint->footrest_front > footprint->footrest_start) {
    front = dwa_max_double(front, footprint->footrest_front);
  }
  if (include_toe && footprint->toe_front > 0.0f) {
    front = dwa_max_double(front, footprint->toe_front);
  }
  return front;
}

float dwa_footprint_front_extent_full(const dwa_footprint_t *footprint)
{
  return dwa_footprint_front_extent(footprint, true);
}

float dwa_footprint_half_width_extent(
  const dwa_footprint_t *footprint, bool include_toe)
{
  float half_width = footprint->half_width;
  if (footprint->footrest_front > footprint->footrest_start) {
    half_width = dwa_max_double(half_width, footprint->footrest_half_width);
  }
  if (include_toe && footprint->toe_front > 0.0f) {
    half_width = dwa_max_double(half_width, footprint->toe_half_width);
  }
  return half_width;
}

float dwa_footprint_half_width_extent_full(const dwa_footprint_t *footprint)
{
  return dwa_footprint_half_width_extent(footprint, true);
}

bool dwa_inside_footprint_rectangle(
  float x,
  float y,
  float min_x,
  float max_x,
  float half_width,
  float padding)
{
  return x >= min_x - padding && x <= max_x + padding &&
    fabsf(y) <= half_width + padding;
}

bool dwa_point_inside_footprint_self_filter(
  float x, float y, const dwa_footprint_t *footprint, float padding)
{
  float half_width;
  const float trapezoid_start_x = footprint->front;
  const float trapezoid_shoulder_x = footprint->footrest_start;
  const float trapezoid_end_x = footprint->toe_front;

  padding = dwa_max_double(0.0f, padding);
  if (dwa_inside_footprint_rectangle(
      x, y, -footprint->back, footprint->front,
      footprint->half_width, padding))
  {
    return true;
  }
  if (trapezoid_end_x <= trapezoid_start_x ||
    trapezoid_shoulder_x < trapezoid_start_x ||
    trapezoid_shoulder_x > trapezoid_end_x)
  {
    return false;
  }
  half_width = footprint->toe_half_width;
  if (x <= trapezoid_shoulder_x) {
    const float span = trapezoid_shoulder_x - trapezoid_start_x;
    const float ratio = span > 0.0f ?
      (x - trapezoid_start_x) / span : 1.0f;
    half_width = footprint->half_width + ratio *
      (footprint->footrest_half_width - footprint->half_width);
  }
  /* Release only the physical toe-width forward strip beyond the toe edge.
   * Keep the existing lateral-padding strip filtered so this fix does not add
   * bend-wall points to DWA; side-wall handling remains on its existing path. */
  if (x > trapezoid_end_x && x <= trapezoid_end_x + padding &&
    fabsf(y) > footprint->toe_half_width &&
    fabsf(y) <= footprint->toe_half_width + padding)
  {
    return true;
  }
  return x >= trapezoid_start_x - padding &&
    x <= trapezoid_end_x && fabsf(y) <= half_width + padding;
}

bool dwa_footprint_contains_local(
  float x,
  float y,
  const dwa_footprint_t *footprint,
  bool include_toe)
{
  const float padding = dwa_max_double(0.0f, footprint->padding);
  if (dwa_inside_footprint_rectangle(
      x, y, -footprint->back, footprint->front, footprint->half_width, padding)) {
    return true;
  }
  if (footprint->footrest_front > footprint->footrest_start &&
    dwa_inside_footprint_rectangle(
      x, y, footprint->footrest_start, footprint->footrest_front,
      footprint->footrest_half_width, padding)) {
    return true;
  }
  return include_toe && footprint->toe_front > footprint->footrest_front &&
    dwa_inside_footprint_rectangle(
      x, y, footprint->footrest_front, footprint->toe_front,
      footprint->toe_half_width, padding);
}

bool dwa_footprint_contains_local_full(
  float x, float y, const dwa_footprint_t *footprint)
{
  return dwa_footprint_contains_local(x, y, footprint, true);
}

float dwa_distance_to_rectangle(
  float x,
  float y,
  float min_x,
  float max_x,
  float half_width,
  float padding)
{
  const float dx = dwa_max_double(dwa_max_double(min_x - padding - x, 0.0f),
    x - max_x - padding);
  const float dy = dwa_max_double(fabsf(y) - half_width - padding, 0.0f);
  return sqrtf(dx * dx + dy * dy);
}

float dwa_distance_to_expanded_footprint(
  const dwa_point_t *point,
  const dwa_footprint_t *footprint)
{
  if (dwa_footprint_contains_local(point->x, point->y, footprint, true)) return -1.0f;

  const float padding = dwa_max_double(0.0f, footprint->padding);
  float distance = dwa_distance_to_rectangle(
    point->x, point->y, -footprint->back, footprint->front,
    footprint->half_width, padding);
  if (footprint->footrest_front > footprint->footrest_start) {
    distance = dwa_min_double(distance, dwa_distance_to_rectangle(
      point->x, point->y, footprint->footrest_start, footprint->footrest_front,
      footprint->footrest_half_width, padding));
  }
  if (footprint->toe_front > footprint->footrest_front) {
    distance = dwa_min_double(distance, dwa_distance_to_rectangle(
      point->x, point->y, footprint->footrest_front, footprint->toe_front,
      footprint->toe_half_width, padding));
  }
  return distance;
}

float dwa_footprint_half_width_at_x(
  float x,
  const dwa_footprint_t *footprint,
  bool include_toe)
{
  const float padding = dwa_max_double(0.0f, footprint->padding);
  float half_width = 0.0f;
  if (x >= -footprint->back - padding && x <= footprint->front + padding) {
    half_width = footprint->half_width;
  }
  if (footprint->footrest_front > footprint->footrest_start &&
    x >= footprint->footrest_start - padding && x <= footprint->footrest_front + padding) {
    half_width = dwa_max_double(half_width, footprint->footrest_half_width);
  }
  if (include_toe && footprint->toe_front > footprint->footrest_front &&
    x >= footprint->footrest_front - padding && x <= footprint->toe_front + padding) {
    half_width = dwa_max_double(half_width, footprint->toe_half_width);
  }
  return half_width;
}

float dwa_footprint_half_width_at_x_full(float x, const dwa_footprint_t *footprint)
{
  return dwa_footprint_half_width_at_x(x, footprint, true);
}

void dwa_motion_step(
  const dwa_state_t *state, float v, float w, float dt, dwa_state_t *next)
{
  *next = *state;
  next->yaw = dwa_normalize_angle(state->yaw + w * dt);
  next->x += v * cosf(next->yaw) * dt;
  next->y += v * sinf(next->yaw) * dt;
  next->v = v;
  next->w = w;
}
