#ifndef STATIONARY_TURN_SPEED_LIMITER_H
#define STATIONARY_TURN_SPEED_LIMITER_H

#include <stdbool.h>
#include <stddef.h>

#include "../common/dwa_types.h"

typedef struct {
  bool enabled;
  int controller_center_x;
  int controller_center_y;
  int controller_x_dead_low;
  int controller_x_dead_high;
  int controller_y_dead_low;
  float forward_side_angle_deg;
  float outer_radius_m;
  float inner_radius_m;
  float gear1_speed_mps;
  float inner_speed_ratio;
} stsl_config_t;

typedef struct {
  bool joystick_in_trigger_region;
  bool speed_limited;
  float nearest_obstacle_distance_m;
  float speed_limit_mps;
} stsl_result_t;

stsl_result_t stsl_evaluate(
  const stsl_config_t *config,
  int controller_x,
  int controller_y,
  bool controller_fresh,
  bool perception_fresh,
  float nearest_obstacle_distance_m);

float stsl_nearest_obstacle_distance(
  const dwa_point_t *obstacles, size_t obstacle_count, float current_min_m);

float stsl_apply_linear_limit(float linear_x, float speed_limit_mps);

#endif
