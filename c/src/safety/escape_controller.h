/*
 * 模块功能：前方/侧方硬停止脱困方向选择和手动转向方向分类。
 * 直译自 native/safety/escape_controller.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 */
#ifndef DWA_ESCAPE_CONTROLLER_H
#define DWA_ESCAPE_CONTROLLER_H

#include <math.h>
#include <stdbool.h>

#include "../common/dwa_types.h"
#include "../runtime/planner_data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float forward_speed_limit;
  float max_yawrate;
  float deadband_linear;
  float deadband_angular;
  float forward_min_angle_deg;
  float forward_max_angle_deg;
} dwa_turn_classification_config_t;

/* TurnClassificationConfig{} 成员默认值直译。 */
static inline dwa_turn_classification_config_t
dwa_turn_classification_config_default(void)
{
  dwa_turn_classification_config_t config;
  config.forward_speed_limit = 0.70f;
  config.max_yawrate = 1.40f;
  config.deadband_linear = 0.02f;
  config.deadband_angular = 0.02f;
  config.forward_min_angle_deg = 65.0f;
  config.forward_max_angle_deg = 115.0f;
  return config;
}

/* joystick_direction_angle_deg(intent, config) 直译。 */
float dwa_joystick_direction_angle_deg(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config);

/* manual_turn_direction(intent, config, &angle_deg) 直译。 */
planner_manual_turn_direction_t dwa_manual_turn_direction(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config,
  float *angle_deg);

/* manual_turn_direction_with_reverse(intent, config, &angle_deg) 直译。 */
planner_manual_turn_direction_t dwa_manual_turn_direction_with_reverse(
  const dwa_velocity_t *intent,
  const dwa_turn_classification_config_t *config,
  float *angle_deg);

typedef struct {
  bool allowed;
  planner_manual_turn_direction_t direction;
} dwa_front_escape_decision_t;

/* FrontEscapeDecision{} 默认值直译。 */
static inline dwa_front_escape_decision_t dwa_front_escape_decision_default(void)
{
  dwa_front_escape_decision_t decision;
  decision.allowed = false;
  decision.direction = PLANNER_TURN_NONE;
  return decision;
}

/* choose_hard_stop_escape_direction(clearance, enabled, threshold) 直译。 */
dwa_front_escape_decision_t dwa_choose_hard_stop_escape_direction(
  const planner_side_clearance_diagnostics_t *clearance,
  bool enabled,
  float blocked_clearance_threshold);

/* choose_front_escape_direction（旧接口名，语义已统一）直译。 */
dwa_front_escape_decision_t dwa_choose_front_escape_direction(
  const planner_side_clearance_diagnostics_t *clearance,
  bool enabled,
  float minimum_clearance);

#ifdef __cplusplus
}
#endif

#endif /* DWA_ESCAPE_CONTROLLER_H */
