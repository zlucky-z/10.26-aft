/*
 * 模块功能：脱困方向选择实现。
 * 直译自 native/safety/escape_controller.hpp。
 */
#include "escape_controller.h"

static inline float esc_max(float a, float b) { return a > b ? a : b; }

float dwa_joystick_direction_angle_deg(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config)
{
  const float forward_ratio = esc_max(
    0.0f, intent->v / esc_max(config->forward_speed_limit, 1e-6f));
  const float right_ratio = -intent->w / esc_max(config->max_yawrate, 1e-6f);
  return atan2f(forward_ratio, right_ratio) * 180.0f / DWA_PI;
}
planner_manual_turn_direction_t dwa_manual_turn_direction(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config,
  float *angle_deg)
{
  *angle_deg = 90.0f;
  if (intent->v < -config->deadband_linear ||
    (fabsf(intent->v) <= config->deadband_linear &&
    fabsf(intent->w) <= config->deadband_angular))
  {
    return PLANNER_TURN_NONE;
  }

  *angle_deg = dwa_joystick_direction_angle_deg(intent, config);
  if (*angle_deg < config->forward_min_angle_deg) {
    return PLANNER_TURN_RIGHT;
  }
  if (*angle_deg > config->forward_max_angle_deg) {
    return PLANNER_TURN_LEFT;
  }
  return PLANNER_TURN_NONE;
}
/* End of turn classification helpers. */
planner_manual_turn_direction_t dwa_manual_turn_direction_with_reverse(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config,
  float *angle_deg)
{
  *angle_deg = dwa_joystick_direction_angle_deg(intent, config);
  if (intent->v >= -config->deadband_linear) {
    return dwa_manual_turn_direction(intent, config, angle_deg);
  }
  if (intent->v < -config->deadband_linear &&
    fabsf(intent->w) > config->deadband_angular)
  {
    /* Reverse commands use the requested front-heading side; the reverse
     * escape correction is applied in the opposite yaw direction by the node. */
    return intent->w > 0.0f ? PLANNER_TURN_LEFT : PLANNER_TURN_RIGHT;
  }
  return PLANNER_TURN_NONE;
}
