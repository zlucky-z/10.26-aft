/*
 * 模块功能：dwa_node_tick.c / dwa_node_safety.c / dwa_node_perception.c
 * 三个实现文件之间的内部接口（对应 C++ node 类的私有成员函数）。
 * 不对外暴露；前缀 dnp_ = node perception，dns_ = node safety。
 */
#ifndef DWA_NODE_INTERNAL_H
#define DWA_NODE_INTERNAL_H

#include "dwa_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== dwa_node_perception.c（node_perception_grid/scan.inc 直译） ===== */

void dnp_reset_occupancy_grid(const dwa_node_t *node, dwa_occupancy_grid_t *grid);

void dnp_project_base_cloud_to_grid(
  const dwa_node_t *node,
  const dwa_fused_base_point_t *base_points,
  size_t base_point_count,
  planner_side_clearance_diagnostics_t *side_clearance,
  dwa_occupancy_grid_t *grid,
  dwa_point_buf_t *points,
  dwa_geometric_scan_buf_t *left_geometric_scan_points,
  dwa_geometric_scan_buf_t *right_geometric_scan_points,
  planner_side_hard_stop_circle_diagnostics_t *side_hard_stop_circle,
  planner_front_no_turn_stop_diagnostics_t *front_no_turn_stop,
  dwa_point_buf_t *side_safety_points);

void dnp_mark_fused_obstacles_in_grid(
  const dwa_node_t *node, const dwa_point_buf_t *points, dwa_occupancy_grid_t *grid);

void dnp_inflate_occupied_cells(
  const dwa_node_t *node, dwa_occupancy_grid_t *grid);

void dnp_clear_current_footprint(
  const dwa_node_t *node, dwa_occupancy_grid_t *grid);

planner_obstacle_diagnostics_t dnp_diagnose_obstacles(
  const dwa_node_t *node, const dwa_point_buf_t *obstacles);

planner_grid_diagnostics_t dnp_diagnose_grid(const dwa_occupancy_grid_t *grid);

/* 骑乘者身体区自点判定（P9 新增）：x ∈ [start, front] 且 |y| ≤ half_width。
 * 只供速度模块清距诊断使用（dnp_diagnose_obstacles 前方触发区块、
 * tick 的 sweep_clearance 计算）；不改 DWA 障碍图、避障和几何选簇。 */
bool dnp_point_inside_rider_self_filter(
  const dwa_node_t *node, float x, float y);

/* ===== dwa_node_safety.c（node_safety_control.inc + node_diagnostics.inc 直译） ===== */

void dns_log_tick_timing(dwa_node_t *node, dwa_tick_timing_t *timing);

float dns_collision_front_edge(const dwa_node_t *node);

float dns_front_trigger_half_width_at(const dwa_node_t *node, float x);

bool dns_point_inside_front_trigger_zone(
  const dwa_node_t *node, const dwa_point_t *point);
bool dns_point_inside_front_safety_lane(
  const dwa_node_t *node, const dwa_point_t *point);

bool dns_point_inside_gear_front_trigger_zone(
  const dwa_node_t *node, const dwa_point_t *point);

float dns_update_moving_prebrake_y_down_rate(
  dwa_node_t *node,
  const dwa_dynamic_velocity_input_t *input,
  const dwa_dynamic_velocity_decision_t *decision);

void dns_reset_static_dwa_command_tracking(dwa_node_t *node);

float dns_static_dwa_planning_hold_target(
  const dwa_node_t *node,
  const dwa_dynamic_velocity_input_t *input,
  float runtime_max_velocity);

void dns_note_static_dwa_command_published(dwa_node_t *node, float target_linear);

bool dns_publish_regular_command(
  dwa_node_t *node,
  const dwa_twist_t *output,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  bool hard_stop);

void dns_publish_command_without_trajectory(
  dwa_node_t *node,
  const dwa_twist_t *output,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  bool hard_stop);

bool dns_publish_motion_command_safely(
  dwa_node_t *node,
  const dwa_twist_t *requested,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  bool perception_fresh,
  const dwa_point_buf_t *safety_points,
  const dwa_velocity_t *manual_intent,
  const char *source,
  const planner_side_clearance_diagnostics_t *side_clearance);

void dns_publish_motion_command_without_trajectory_safely(
  dwa_node_t *node,
  const dwa_twist_t *output,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  bool perception_fresh,
  const dwa_point_buf_t *safety_points,
  const dwa_velocity_t *manual_intent,
  const char *source,
  const planner_side_clearance_diagnostics_t *side_clearance);

bool dns_handle_manual_bypass(
  dwa_node_t *node,
  const dwa_velocity_t *raw_intent,
  const dwa_velocity_t *guarded_intent,
  const dwa_side_wall_hold_result_t *side_wall_hold,
  const dwa_dynamic_velocity_decision_t *side_decision,
  bool perception_fresh,
  bool hard_stop_latched,
  float front_clearance,
  float front_nearest_radar_distance,
  float nearest_radar_distance,
  const planner_side_clearance_diagnostics_t *side_clearance,
  const planner_side_hard_stop_circle_diagnostics_t *side_hard_stop_circle,
  const planner_front_no_turn_stop_diagnostics_t *front_no_turn_stop,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  const dwa_point_buf_t *side_safety_points);

bool dns_handle_direct_moving_control(
  dwa_node_t *node,
  const dwa_velocity_t *intent,
  const dwa_velocity_t *manual_intent,
  const dwa_dynamic_velocity_input_t *dynamic_input,
  const dwa_dynamic_velocity_decision_t *decision,
  float runtime_max_velocity,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  const dwa_point_buf_t *side_safety_points);

float dns_update_runtime_velocity_limit(
  dwa_node_t *node, const dwa_dynamic_velocity_decision_t *decision);

void dns_log_gear_closed_loop_state(
  dwa_node_t *node,
  const dwa_dynamic_velocity_decision_t *decision,
  const dwa_dynamic_velocity_input_t *input,
  const dwa_velocity_t *intent);

void dns_log_trajectory_scores(
  dwa_node_t *node, const dwa_plan_result_t *result, dwa_tick_timing_t *timing);

#ifdef __cplusplus
}
#endif

#endif /* DWA_NODE_INTERNAL_H */
