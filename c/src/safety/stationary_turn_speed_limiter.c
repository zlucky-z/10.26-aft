#include "stationary_turn_speed_limiter.h"

#include <math.h>

#define STSL_PI 3.14159265358979323846f

static bool axis_outside_deadband(int value, int low, int high)
{
  return value < low || value > high;
}

static bool joystick_in_trigger_region(
  const stsl_config_t *config,
  int controller_x,
  int controller_y)
{
  const bool x_active = axis_outside_deadband(controller_x,
    config->controller_x_dead_low, config->controller_x_dead_high);
  const float x_delta = (float)(controller_x - config->controller_center_x);
  const float y_delta = (float)(controller_y - config->controller_center_y);

  /* Rearward stick beyond its deadzone is reverse; ignore command mapping. */
  if (controller_y < config->controller_y_dead_low) return true;
  if (!x_active) return false;

  if (controller_y <= config->controller_center_y) return true;
  if (fabsf(x_delta) <= 1e-6f) return false;

  /* Use raw stick geometry, not axis deadband, for the forward red-sector edge. */
  {
    const float angle = config->forward_side_angle_deg * STSL_PI / 180.0f;
    return fabsf(y_delta) <= fabsf(x_delta) * tanf(angle);
  }
}

stsl_result_t stsl_evaluate(
  const stsl_config_t *config,
  int controller_x,
  int controller_y,
  bool controller_fresh,
  bool perception_fresh,
  float nearest_obstacle_distance_m)
{
  stsl_result_t result;

  result.joystick_in_trigger_region = false;
  result.speed_limited = false;
  result.nearest_obstacle_distance_m = nearest_obstacle_distance_m;
  result.speed_limit_mps = INFINITY;
  if (config == NULL || !config->enabled || !controller_fresh ||
      !perception_fresh) {
    return result;
  }

  result.joystick_in_trigger_region = joystick_in_trigger_region(
    config, controller_x, controller_y);
  if (!result.joystick_in_trigger_region) return result;

  if (result.nearest_obstacle_distance_m <= config->inner_radius_m) {
    result.speed_limit_mps = config->gear1_speed_mps * config->inner_speed_ratio;
  } else if (result.nearest_obstacle_distance_m <= config->outer_radius_m) {
    result.speed_limit_mps = config->gear1_speed_mps;
  } else {
    return result;
  }
  result.speed_limited = isfinite(result.speed_limit_mps) &&
    result.speed_limit_mps >= 0.0f;
  return result;
}

float stsl_nearest_obstacle_distance(
  const dwa_point_t *obstacles, size_t obstacle_count, float current_min_m)
{
  size_t i;
  float nearest = current_min_m;
  if (obstacles == NULL) return nearest;
  for (i = 0; i < obstacle_count; ++i) {
    const float distance = hypotf(obstacles[i].x, obstacles[i].y);
    if (isfinite(distance) && distance < nearest) nearest = distance;
  }
  return nearest;
}

float stsl_apply_linear_limit(float linear_x, float speed_limit_mps)
{
  if (!isfinite(linear_x) || !isfinite(speed_limit_mps) ||
      speed_limit_mps < 0.0f || fabsf(linear_x) <= speed_limit_mps) {
    return linear_x;
  }
  return copysignf(speed_limit_mps, linear_x);
}
