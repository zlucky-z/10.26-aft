/*
 * 模块功能：侧边三角转向保护和扫掠预测保护计算。
 * 直译自 native/safety/side_safety.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 */
#ifndef DWA_SIDE_SAFETY_H
#define DWA_SIDE_SAFETY_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "../common/dwa_types.h"
#include "../runtime/planner_data_types.h"
#include "escape_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool triangle_enabled;
  bool circle_enabled;
  bool sweep_enabled;
  float triangle_stop_gap_m;
  dwa_footprint_t footprint;
  float circle_center_x;
  float circle_center_y;
  float circle_radius;
  float wheel_track_m;
  float sweep_prediction_sec;
  float sweep_brake_sec;
  float sweep_min_yawrate;
  float deadband_linear;
  float deadband_angular;
  dwa_turn_classification_config_t turn;
} dwa_side_safety_config_t;

/* SideSafetyConfig{} 成员默认值直译。 */
static inline dwa_side_safety_config_t dwa_side_safety_config_default(void)
{
  dwa_side_safety_config_t config;
  config.triangle_enabled = true;
  config.circle_enabled = true;
  config.sweep_enabled = true;
  config.triangle_stop_gap_m = 0.20f;
  config.footprint = dwa_footprint_default();
  config.circle_center_x = 0.51f;
  config.circle_center_y = 0.0f;
  config.circle_radius = 0.44f;
  config.wheel_track_m = 0.50f;
  config.sweep_prediction_sec = 0.45f;
  config.sweep_brake_sec = 0.25f;
  config.sweep_min_yawrate = 0.15f;
  config.deadband_linear = 0.02f;
  config.deadband_angular = 0.02f;
  config.turn = dwa_turn_classification_config_default();
  return config;
}

/* 当前控制链使用三角区域保护；圆形/扫掠接口保留给历史测试和兼容代码。 */
planner_side_hard_stop_decision_t dwa_evaluate_side_triangle_stop(
  const dwa_velocity_t *command,
  bool perception_fresh,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_side_safety_config_t *config);

/* predict_side_circle_entry(...) 历史扫掠几何接口。 */
bool dwa_predict_side_circle_entry(
  const dwa_point_t *point,
  float linear_speed,
  float yawrate,
  float horizon_sec,
  const dwa_side_safety_config_t *config,
  float *entry_time_sec,
  float *entry_radius);

/*
 * evaluate_side_hard_stop_circle(intent, perception_fresh, diagnostics, config)
 * 直译，结果按值返回。
 */
planner_side_hard_stop_decision_t dwa_evaluate_side_hard_stop_circle(
  const dwa_velocity_t *intent,
  bool perception_fresh,
  const planner_side_hard_stop_circle_diagnostics_t *diagnostics,
  const dwa_side_safety_config_t *config);

/*
 * evaluate_side_sweep_guard(command, manual_intent, motor_feedback,
 *                           perception_fresh, safety_points, count, config)
 * 直译，结果按值返回。
 */
planner_side_hard_stop_decision_t dwa_evaluate_side_sweep_guard(
  const dwa_velocity_t *command,
  const dwa_velocity_t *manual_intent,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  bool perception_fresh,
  const dwa_point_t *safety_points,
  size_t safety_points_count,
  const dwa_side_safety_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* DWA_SIDE_SAFETY_H */
