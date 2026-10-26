/*
 * 模块功能：DWA 运行时节点的构造/析构、输入注入与输入状态处理。
 * 直译自 native/nodes/impl/node_members.inc（成员初始化语义）、
 * node_constructor.inc（config 组装语义）、node_input_state.inc（读取逻辑）。
 * P6；全 float 规范。
 */
#include "dwa_node.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../hal/hal_log.h"
#include "../hal/hal_time.h"
#include "../common/dwa_geometry.h"

#define NODE_PI ((float)3.14159265358979323846)

/* ---------- 动态缓冲 ---------- */

void dwa_point_buf_destroy(dwa_point_buf_t *buf)
{
  free(buf->points);
  buf->points = NULL;
  buf->count = 0;
  buf->capacity = 0;
}

void dwa_geometric_scan_buf_destroy(dwa_geometric_scan_buf_t *buf)
{
  free(buf->items);
  buf->items = NULL;
  buf->count = 0;
  buf->capacity = 0;
}

/* ---------- TickTiming（planner_runtime.hpp 直译） ---------- */

void dwa_tick_timing_init(dwa_tick_timing_t *timing)
{
  timing->started_sec = hal_steady_now_sec();
  timing->last_transition_sec = timing->started_sec;
  timing->active = DWA_TICK_SEGMENT_OTHER;
  timing->stack_count = 0;
  timing->total_ms = 0.0;
  timing->pointcloud_ms = 0.0;
  timing->dwa_ms = 0.0;
  timing->trajectory_publish_ms = 0.0;
  timing->finished = false;
}

static void dwa_tick_timing_account(dwa_tick_timing_t *timing, double stamp_sec)
{
  const double elapsed_ms = (stamp_sec - timing->last_transition_sec) * 1e3;
  switch (timing->active) {
    case DWA_TICK_SEGMENT_POINT_CLOUD:
      timing->pointcloud_ms += elapsed_ms;
      break;
    case DWA_TICK_SEGMENT_DWA_PLANNING:
      timing->dwa_ms += elapsed_ms;
      break;
    case DWA_TICK_SEGMENT_TRAJECTORY_PUBLISH:
      timing->trajectory_publish_ms += elapsed_ms;
      break;
    case DWA_TICK_SEGMENT_OTHER:
      break;
  }
  timing->last_transition_sec = stamp_sec;
}

void dwa_tick_timing_enter(dwa_tick_timing_t *timing, dwa_tick_segment_t segment)
{
  const double stamp = hal_steady_now_sec();
  dwa_tick_timing_account(timing, stamp);
  if (timing->stack_count < 8) {
    timing->stack[timing->stack_count++] = segment;
  }
  timing->active = segment;
}

void dwa_tick_timing_leave(dwa_tick_timing_t *timing)
{
  const double stamp = hal_steady_now_sec();
  dwa_tick_timing_account(timing, stamp);
  if (timing->stack_count > 0) {
    --timing->stack_count;
  }
  timing->active = timing->stack_count == 0 ?
    DWA_TICK_SEGMENT_OTHER : timing->stack[timing->stack_count - 1];
}

void dwa_tick_timing_finish(dwa_tick_timing_t *timing)
{
  if (timing->finished) return;
  const double stamp = hal_steady_now_sec();
  dwa_tick_timing_account(timing, stamp);
  timing->total_ms = (stamp - timing->started_sec) * 1e3;
  timing->finished = true;
}

double dwa_tick_timing_other_ms(const dwa_tick_timing_t *timing)
{
  const double other = timing->total_ms - timing->pointcloud_ms -
    timing->dwa_ms - timing->trajectory_publish_ms;
  return other > 0.0 ? other : 0.0;
}

/* ---------- 时间基准 ---------- */

double dwa_node_now_sec(void)
{
  return hal_wall_now_sec();
}

bool dwa_node_stale(double data_time, float timeout_sec)
{
  /* rclcpp::Time 未设置（0）与超时同样视为过期。 */
  if (data_time <= 0.0) return true;
  return (dwa_node_now_sec() - data_time) > (double)timeout_sec;
}

/* ---------- config 组装（node_constructor.inc 语义） ---------- */

static void assemble_dynamic_config(
  const dwa_planner_params_t *p, const dwa_config_t *config,
  dwa_dynamic_velocity_config_t *c)
{
  /* 结构默认值（dwa_dynamic_velocity_config_default）再逐字段覆盖：
   * constructor 语义 = declare(name, 当前值) → yaml 有值用 yaml。 */
  *c = dwa_dynamic_velocity_config_default();
  c->forward_speed_limit = config->max_velocity; /* 固定映射，非 yaml */
  c->moving_prebrake_base_distance = p->moving_prebrake_base_distance;
  c->moving_prebrake_speed_ratio = p->moving_prebrake_speed_ratio;
  c->moving_prebrake_target_speed = p->moving_prebrake_target_speed;
  c->moving_prebrake_deceleration = p->moving_prebrake_deceleration;
  c->moving_prebrake_profile_far_distance = p->moving_prebrake_profile_far_distance;
  c->moving_prebrake_profile_zero_distance = p->moving_prebrake_profile_zero_distance;
  c->moving_prebrake_recovery_acceleration = p->moving_prebrake_recovery_acceleration;
  c->bypass_near_zone_release_hold_cycles = p->bypass_near_zone_release_hold_cycles;
  c->bypass_near_zone_far_distance = p->bypass_near_zone_far_distance;
  /* A3(10.09)：释放侧净距门控。 */
  c->bypass_near_zone_release_side_distance = p->bypass_near_zone_release_side_distance;
  c->moving_dwa_speed_limit = p->moving_dwa_speed_limit;
  c->moving_dwa_yawrate_limit = p->moving_dwa_yawrate_limit;
  c->moving_dwa_force_gear = p->moving_dwa_force_gear;
  c->moving_dwa_gear_level = p->moving_dwa_gear_level;
  c->near_obstacle_gear_enabled = p->near_obstacle_gear_enabled;
  c->near_obstacle_gear_center_x = p->near_obstacle_gear_center_x;
  c->near_obstacle_gear_center_y = p->near_obstacle_gear_center_y;
  c->near_obstacle_gear_radius = p->near_obstacle_gear_radius;
  c->near_obstacle_gear_release_radius = p->near_obstacle_gear_release_radius;
  c->near_obstacle_gear_level = p->near_obstacle_gear_level;
  c->moving_dwa_trigger_distance = p->moving_dwa_trigger_distance;
  c->moving_dwa_entry_speed_max = p->moving_dwa_entry_speed_max;
  c->moving_release_hysteresis = p->moving_release_hysteresis;
  c->static_start_distance = p->static_start_distance;
  c->static_vdwa_gain = p->static_vdwa_gain;
  c->release_margin = p->dynamic_release_margin;
  c->release_clear_sec = p->dynamic_release_clear_sec;
  c->speed_restore_side_clearance = p->speed_restore_side_clearance;
  c->speed_restore_sweep_clearance = p->speed_restore_sweep_clearance;
  c->speed_restore_stable_sec = p->speed_restore_stable_sec;
  c->speed_restore_max_yawrate = p->speed_restore_max_yawrate;
  c->static_recalculate_sec = p->static_vdwa_recalculate_sec;
  c->fallback_gear = p->controller_gear_fallback;
  c->gear_closed_loop_enabled = p->gear_closed_loop_enabled;
  c->gear_restore_hysteresis = p->gear_restore_hysteresis;
  c->gear_force_level = p->gear_force_level;
  c->gear_force_forward_only = p->gear_force_forward_only;
  c->gear_front_trigger_extra_angle_deg = p->gear_front_trigger_extra_angle_deg;
  c->gear_restore_hold_cycles = p->gear_restore_hold_cycles;
  c->gear_transition_step_sec = p->gear_transition_step_sec;
  c->gear_transition_safety_margin = p->gear_transition_safety_margin;
  c->gear_speed_transition_enabled = p->gear_speed_transition_enabled;
  c->gear_speed_transition_duration_sec = p->gear_speed_transition_duration_sec;
  c->side_speed_trigger = p->side_speed_trigger;
  c->side_speed_target = p->side_speed_target;
  c->side_limit_distance = p->side_limit_distance;
  c->side_prediction_reaction_sec = p->side_prediction_reaction_sec;
  c->side_prediction_deceleration = p->side_prediction_deceleration;
  c->side_closing_rate_filter_alpha = p->side_closing_rate_filter_alpha;
  c->side_closing_rate_max = p->side_closing_rate_max;
  c->side_cap_release_distance = p->side_cap_release_distance;
  c->side_cap_release_clear_sec = p->side_cap_release_clear_sec;
  c->side_target_down_rate = p->side_target_down_rate;
  c->side_target_up_rate = p->side_target_up_rate;
  /* gear_peak_speeds：固定 5 档（原 vector size()==5 校验） */
  if (p->gear_peak_speeds_count == DWA_FUSION_GEAR_COUNT) {
    int i;
    for (i = 0; i < DWA_FUSION_GEAR_COUNT; ++i) {
      c->gear_peak_speeds[i] = p->gear_peak_speeds[i];
    }
  }
}

static void assemble_side_wall_config(
  const dwa_planner_params_t *p, dwa_side_wall_hold_config_t *c)
{
  *c = dwa_side_wall_hold_config_default();
  c->enabled = p->side_wall_hold_enabled;
  c->hold_distance = p->side_wall_hold_distance;
  c->correction_distance = p->side_wall_hold_correction_distance;
  c->prediction_sec = p->side_wall_hold_prediction_sec;
  c->invalid_clearance_hold_sec = p->side_wall_hold_invalid_hold_sec;
  /* constructor：closing_rate_* 复用 dynamic_config 的 side_closing_rate_*。 */
  c->closing_rate_filter_alpha = p->side_closing_rate_filter_alpha;
  c->closing_rate_max = p->side_closing_rate_max;
  c->release_distance = p->side_wall_hold_release_distance;
  c->proportional_gain = p->side_wall_hold_kp;
  c->min_yawrate = p->side_wall_hold_min_yawrate;
  c->max_yawrate = p->side_wall_hold_max_yawrate;
  c->yawrate_acceleration = p->side_wall_hold_yaw_accel;
  c->release_clear_sec = p->side_wall_hold_release_sec;
}

static void assemble_corridor_config(
  const dwa_planner_params_t *p, dwa_corridor_guide_config_t *c)
{
  *c = dwa_corridor_guide_config_default();
  c->enabled = p->dwa_corridor_guide_enabled;
  c->search_start_x = p->dwa_corridor_search_start_x;
  c->search_end_x = p->dwa_corridor_search_end_x;
  c->minimum_width_margin = p->dwa_corridor_min_width_margin;
  c->maximum_width = p->dwa_corridor_max_width;
  c->search_half_width = p->dwa_corridor_search_half_width;
  c->approach_start_x = p->dwa_corridor_approach_start_x;
  c->cluster_gap_distance = p->dwa_corridor_cluster_gap_distance;
  c->cluster_min_points = p->dwa_corridor_cluster_min_points;
  c->gate_max_depth_difference = p->dwa_corridor_gate_max_depth_difference;
  c->gate_blocking_clearance = p->dwa_corridor_gate_blocking_clearance;
  c->gate_endpoint_exclusion = p->dwa_corridor_gate_endpoint_exclusion;
  c->gate_dedup_distance = p->dwa_corridor_gate_dedup_distance;
  c->gate_projection_length = p->dwa_corridor_gate_projection_length;
  c->explicit_intent_ratio = p->dwa_corridor_explicit_intent_ratio;
  c->maximum_heading = p->dwa_corridor_max_heading;
  c->intent_lateral_range = p->dwa_corridor_intent_lateral_range;
  c->stable_cycles = p->dwa_corridor_stable_cycles;
  c->lost_hold_cycles = p->dwa_corridor_lost_hold_cycles;
  c->track_lateral_tolerance = p->dwa_corridor_track_lateral_tolerance;
  c->track_heading_tolerance = p->dwa_corridor_track_heading_tolerance;
  c->path_filter_alpha = p->dwa_corridor_path_filter_alpha;
  c->trajectory_lookahead = p->dwa_corridor_trajectory_lookahead;
  c->lateral_error_weight = p->dwa_corridor_lateral_weight;
  c->heading_error_weight = p->dwa_corridor_heading_weight;
  c->progress_weight = p->dwa_corridor_progress_weight;
}

static void assemble_geometric_config(
  const dwa_planner_params_t *p, dwa_geometric_selector_config_t *c)
{
  *c = dwa_geometric_selector_config_default();
  c->enabled = p->dwa_geometric_selector_enabled;
  c->lookahead_distance = p->dwa_geometric_lookahead_distance;
  c->minimum_lookahead_distance = p->dwa_geometric_min_lookahead_distance;
  c->minimum_obstacle_forward_offset = p->dwa_geometric_min_forward_offset;
  c->cluster_gap_distance = p->dwa_geometric_cluster_gap_distance;
  c->cluster_forward_window = p->dwa_geometric_cluster_forward_window;
  c->edge_front_band = p->dwa_geometric_edge_front_band;
  c->cluster_track_tolerance = p->dwa_geometric_cluster_track_tolerance;
  c->centered_entry_max_width = p->dwa_geometric_centered_entry_max_width;
  c->handoff_stable_cycles = p->dwa_geometric_handoff_stable_cycles;
  c->handoff_heading_agree = p->dwa_geometric_handoff_heading_agree;
  c->minimum_candidate_clearance = p->dwa_geometric_min_candidate_clearance;
  c->maximum_clearance_loss = p->dwa_geometric_max_clearance_loss;
  c->raw_override_yawrate = p->dwa_geometric_raw_entry_yawrate;
  c->minimum_target_heading = p->dwa_geometric_min_target_heading;
  c->maximum_heading_error = p->dwa_geometric_max_error;
  c->manual_override_yawrate = p->dwa_geometric_manual_override_yawrate;
  c->target_filter_alpha = p->dwa_geometric_target_filter_alpha;
  c->heading_tie_tolerance = p->dwa_geometric_heading_tie;
  c->near_small_angle_distance = p->dwa_geometric_near_small_angle_distance;
  c->small_target_heading = p->dwa_geometric_small_target_heading;
  c->near_target_heading_compensation = p->dwa_geometric_near_target_heading_compensation;
  c->maximum_heading_growth = p->dwa_geometric_max_heading_growth;
}

/* ---------- 构造/析构 ---------- */

int dwa_node_init(dwa_node_t *node, const dwa_planner_params_t *params)
{
  const char *reason = NULL;

  memset(node, 0, sizeof(*node));
  node->params = &node->params_snapshot;
  node->params_snapshot = *params;

  /* DwaConfig（含 footprint/path_reference 覆盖，constructor 尾部同步）。 */
  dwa_planner_params_to_config(&node->params_snapshot, &node->config);

  /* LaserMount（constructor：deg→rad offset）。 */
  node->left_mount.x = params->left_laser_x;
  node->left_mount.y = params->left_laser_y;
  node->left_mount.yaw = params->left_laser_yaw;
  node->left_mount.angle_reversed = params->left_laser_angle_reversed;
  node->left_mount.angle_offset = params->left_laser_angle_offset_deg * NODE_PI / 180.0f;
  node->right_mount.x = params->right_laser_x;
  node->right_mount.y = params->right_laser_y;
  node->right_mount.yaw = params->right_laser_yaw;
  node->right_mount.angle_reversed = params->right_laser_angle_reversed;
  node->right_mount.angle_offset = params->right_laser_angle_offset_deg * NODE_PI / 180.0f;

  /* 工作区。 */
  dwa_plan_result_init(&node->dwa_plan_workspace);
  dwa_obstacle_spatial_index_init(&node->dwa_spatial_index_workspace);
  dwa_grid_blocking_index_init(&node->dwa_blocking_index_workspace);
  memset(&node->occupancy_grid_workspace, 0, sizeof(node->occupancy_grid_workspace));
  memset(&node->obstacles_workspace, 0, sizeof(node->obstacles_workspace));
  memset(&node->side_safety_points_workspace, 0, sizeof(node->side_safety_points_workspace));
  memset(&node->left_geometric_scan_points_workspace, 0,
    sizeof(node->left_geometric_scan_points_workspace));
  memset(&node->right_geometric_scan_points_workspace, 0,
    sizeof(node->right_geometric_scan_points_workspace));

  /* 控制状态默认值（node_members 类内初始化器）。 */
  node->last_output_linear = 0.0f;
  node->last_valid_dwa_linear = 0.0f;
  node->forced_straight_active = false;
  node->moving_dwa_command_published = false;
  node->hard_stop_latched = false;
  node->hard_stop_clear_cycles = 0;
  node->near_obstacle_gear_latched = false;
  node->near_obstacle_gear_nearest_distance = (float)INFINITY;
  node->stationary_turn_ring_speed_limit_mps = (float)INFINITY;
  node->stationary_turn_ring_nearest_distance_m = (float)INFINITY;
  node->moving_prebrake_y_down_rate = params->moving_prebrake_y_rate_release;
  node->output_effective_gear = 1;
  node->output_gear_closed_loop_active = false;
  node->static_dwa_hall_target = 0.0f;
  node->static_dwa_hall_target_valid = false;
  node->tick_count = 0;
  node->body_masked_points_this_tick = 0;
  node->side_flank_points_this_tick = 0;
  node->front_unclipped_clearance_this_tick = (float)INFINITY; /* V4d：未截断前向净距初始为 ∞ */
  node->local_foot_masked_points_this_tick = 0;
  node->left_angle_cropped_points_this_tick = 0;
  node->right_angle_cropped_points_this_tick = 0;
  /* Fix D：侧净距诊断初始为 ∞（memset 归零会把首 tick 侧净距误判为 0）。 */
  node->diag_side_clear_left = INFINITY;
  node->diag_side_clear_right = INFINITY;

  /* Hall 反馈状态。 */
  node->last_wheel_left_mps = 0.0f;
  node->last_wheel_right_mps = 0.0f;
  node->filtered_actual_forward_speed = 0.0f;
  node->moving_hall_speed_filter_initialized = false;
  node->wheel_feedback_valid = false;
  node->last_wheel_feedback_time = 0.0;
  node->forward_start_rising_latched = false;
  node->last_controller_x = 100;
  node->controller_x_valid = false;
  node->last_controller_y = 100;
  node->controller_y_valid = false;
  node->last_controller_axes_time = 0.0;
  node->raw_gear = 1;
  node->raw_gear_valid = false;

  /* 档位日志状态（node_members 直译：未初始化 + 无效占位）。 */
  node->gear_log_initialized = false;
  node->last_logged_raw_gear = 0;
  node->last_logged_effective_gear = 0;
  node->last_logged_gear_target = 0;
  node->last_logged_gear_closed_loop_active = false;
  node->last_logged_gear_transition_active = false;

  /* 模块实例（constructor 组装 + 校验 + 尾部同步）。 */
  assemble_dynamic_config(&node->params_snapshot, &node->config, &node->dynamic_config);
  if (!dwa_validate_dynamic_velocity_config(&node->dynamic_config, &reason)) {
    hal_log(HAL_LOG_ERROR, "invalid dynamic velocity config: %s",
      reason ? reason : "unknown");
    dwa_node_destroy(node);
    return -1;
  }
  dwa_dynamic_velocity_fusion_init(&node->dynamic_fusion, &node->dynamic_config);
  /* constructor 尾部：config_.max_velocity = forward_speed_limit（恒等回写）。 */
  node->config.max_velocity = node->dynamic_config.forward_speed_limit;
  node->runtime_velocity_limit = node->config.max_velocity;
  node->runtime_velocity_limit_initialized = true;

  assemble_side_wall_config(&node->params_snapshot, &node->side_wall_hold_config);
  dwa_side_wall_hold_controller_init(
    &node->side_wall_hold_controller, &node->side_wall_hold_config);

  assemble_corridor_config(&node->params_snapshot, &node->corridor_guide_config);
  dwa_corridor_guide_init(&node->corridor_guide, &node->corridor_guide_config);

  assemble_geometric_config(&node->params_snapshot, &node->geometric_selector_config);
  dwa_geometric_selector_init(&node->geometric_selector, &node->geometric_selector_config);

  /* 输入/输出面。 */
  hal_mutex_init(&node->scan_mutex);
  hal_mutex_init(&node->cmd_mutex);
  hal_mutex_init(&node->motor_feedback_mutex);
  node->fused_cloud_points = NULL;
  node->fused_cloud_count = 0;
  node->fused_cloud_capacity = 0;
  node->fused_cloud_time = 0.0;
  dwa_twist_reset(&node->last_cmd);
  node->last_cmd_time = 0.0;
  node->current_velocity.v = 0.0f;
  node->current_velocity.w = 0.0f;
  node->odom_time = 0.0;
  dwa_twist_reset(&node->last_published_cmd);
  node->last_published_tick = 0;
  node->hall_speed_control_active = true;

  node->debug_nominal_speed = params->debug_nominal_speed;
  node->force_debug_intent = params->force_debug_intent;

  /* 诊断/日志开关（node_members 类内初始化器直译）。 */
  node->publish_debug_visualization = true;
  node->log_trajectory_scores = false;
  node->trajectory_score_top_n = 10;
  node->trajectory_score_log_interval_ms = 1000;
  node->tick_timing_warn_ms = 120.0f;

  /* 输出 sink。 */
  node->cmd_sink = NULL;
  node->cmd_sink_user_data = NULL;

  return 0;
}

void dwa_node_destroy(dwa_node_t *node)
{
  dwa_plan_result_destroy(&node->dwa_plan_workspace);
  dwa_obstacle_spatial_index_destroy(&node->dwa_spatial_index_workspace);
  dwa_grid_blocking_index_destroy(&node->dwa_blocking_index_workspace);
  dwa_occupancy_grid_destroy(&node->occupancy_grid_workspace);
  dwa_point_buf_destroy(&node->obstacles_workspace);
  dwa_point_buf_destroy(&node->side_safety_points_workspace);
  dwa_geometric_scan_buf_destroy(&node->left_geometric_scan_points_workspace);
  dwa_geometric_scan_buf_destroy(&node->right_geometric_scan_points_workspace);
  /* fusion / side_wall / geometric_selector 均无动态成员（纯标量状态），无 destroy。 */
  dwa_corridor_guide_destroy(&node->corridor_guide);
  free(node->fused_cloud_points);
  node->fused_cloud_points = NULL;
  node->fused_cloud_count = 0;
  node->fused_cloud_capacity = 0;
  hal_mutex_destroy(&node->scan_mutex);
  hal_mutex_destroy(&node->cmd_mutex);
  hal_mutex_destroy(&node->motor_feedback_mutex);
  memset(node, 0, sizeof(*node));
}

/* ---------- 输入注入（ROS 回调直译） ---------- */

void dwa_node_on_fused_cloud(
  dwa_node_t *node, const dwa_fused_base_point_t *points, size_t count)
{
  hal_mutex_lock(&node->scan_mutex);
  if (node->fused_cloud_capacity < count) {
    dwa_fused_base_point_t *fresh = (dwa_fused_base_point_t *)realloc(
      node->fused_cloud_points, count * sizeof(dwa_fused_base_point_t));
    if (fresh == NULL) {
      hal_mutex_unlock(&node->scan_mutex);
      return;
    }
    node->fused_cloud_points = fresh;
    node->fused_cloud_capacity = count;
  }
  if (count > 0) {
    memcpy(node->fused_cloud_points, points,
      count * sizeof(dwa_fused_base_point_t));
  }
  node->fused_cloud_count = count;
  node->fused_cloud_time = dwa_node_now_sec();
  hal_mutex_unlock(&node->scan_mutex);
}

void dwa_node_on_cmd(dwa_node_t *node, const dwa_twist_t *cmd)
{
  hal_mutex_lock(&node->cmd_mutex);
  node->last_cmd = *cmd;
  node->last_cmd_time = dwa_node_now_sec();
  hal_mutex_unlock(&node->cmd_mutex);
}

void dwa_node_on_motor_feedback(
  dwa_node_t *node, const float *data, size_t data_count)
{
  int controller_x;
  int controller_y;
  bool previous_neutral;

  /* on_motor_feedback 直译（node_input_state.inc）。 */
  if (data_count < 2 || !isfinite(data[0]) || !isfinite(data[1])) return;

  hal_mutex_lock(&node->motor_feedback_mutex);
  controller_x = (int)lroundf(data[0]);
  controller_y = (int)lroundf(data[1]);
  previous_neutral = node->controller_y_valid &&
    node->last_controller_y >= node->params_snapshot.controller_y_dead_low &&
    node->last_controller_y <= node->params_snapshot.controller_y_dead_high;
  if (previous_neutral && controller_y > node->params_snapshot.controller_y_dead_high) {
    node->forward_start_rising_latched = true;
  } else if (controller_y <= node->params_snapshot.controller_y_dead_high) {
    node->forward_start_rising_latched = false;
  }
  node->last_controller_x = controller_x;
  node->controller_x_valid = true;
  node->last_controller_y = controller_y;
  node->controller_y_valid = true;
  node->last_controller_axes_time = dwa_node_now_sec();

  node->raw_gear_valid = false;
  if (data_count >= 7 && isfinite(data[6])) {
    const int gear = (int)lroundf(data[6]);
    if (gear >= 1 && gear <= 5) {
      node->raw_gear = gear;
      node->raw_gear_valid = true;
    }
  }

  if (data_count >= 6 && isfinite(data[4]) && isfinite(data[5])) {
    const float actual_forward_speed =
      0.5f * (data[4] + data[5]) > 0.0f ? 0.5f * (data[4] + data[5]) : 0.0f;
    node->last_wheel_left_mps = data[4];
    node->last_wheel_right_mps = data[5];
    if (!node->moving_hall_speed_filter_initialized) {
      node->filtered_actual_forward_speed = actual_forward_speed;
      node->moving_hall_speed_filter_initialized = true;
    } else {
      node->filtered_actual_forward_speed =
        node->params_snapshot.moving_hall_speed_filter_alpha * actual_forward_speed +
        (1.0f - node->params_snapshot.moving_hall_speed_filter_alpha) *
        node->filtered_actual_forward_speed;
    }
    node->wheel_feedback_valid = true;
    node->last_wheel_feedback_time = dwa_node_now_sec();
  }
  hal_mutex_unlock(&node->motor_feedback_mutex);
}

void dwa_node_on_odom(dwa_node_t *node, const dwa_velocity_t *velocity)
{
  hal_mutex_lock(&node->cmd_mutex);
  node->current_velocity = *velocity;
  node->odom_time = dwa_node_now_sec();
  hal_mutex_unlock(&node->cmd_mutex);
}

void dwa_node_set_cmd_sink(
  dwa_node_t *node, dwa_node_cmd_sink_fn sink, void *user_data)
{
  node->cmd_sink = sink;
  node->cmd_sink_user_data = user_data;
}

/* ---------- 输入状态（node_input_state.inc 直译） ---------- */

dwa_velocity_t dwa_node_read_intent(const dwa_node_t *node)
{
  dwa_velocity_t intent;
  dwa_twist_t latest_cmd;
  double latest_cmd_time;

  intent.v = 0.0f;
  intent.w = 0.0f;
  /* C++ 侧 cmd_mutex_ 为 mutable（const 成员函数内加锁）；
   * C 直译用去 const 转换保持同一语义。 */
  hal_mutex_lock((hal_mutex_t *)&node->cmd_mutex);
  latest_cmd = node->last_cmd;
  latest_cmd_time = node->last_cmd_time;
  hal_mutex_unlock((hal_mutex_t *)&node->cmd_mutex);

  if (node->force_debug_intent && node->debug_nominal_speed > 0.0f) {
    intent.v = node->debug_nominal_speed;
    intent.w = 0.0f;
  } else if (!dwa_node_stale(latest_cmd_time, node->params_snapshot.cmd_timeout_sec)) {
    intent.v = latest_cmd.linear_x;
    intent.w = latest_cmd.angular_z;
  } else if (node->debug_nominal_speed > 0.0f) {
    intent.v = node->debug_nominal_speed;
    intent.w = 0.0f;
  }
  intent.v = dwa_clamp_value(
    intent.v, -node->params_snapshot.max_reverse_velocity,
    node->dynamic_config.forward_speed_limit);
  intent.w = dwa_clamp_value(
    intent.w, -node->config.max_yawrate, node->config.max_yawrate);
  return intent;
}

static planner_motor_feedback_snapshot_t motor_feedback_snapshot_locked(
  const dwa_node_t *node)
{
  planner_motor_feedback_snapshot_t snapshot =
    planner_motor_feedback_snapshot_default();
  snapshot.left_mps = node->last_wheel_left_mps;
  snapshot.right_mps = node->last_wheel_right_mps;
  snapshot.forward_speed = node->filtered_actual_forward_speed;
  snapshot.raw_gear = node->raw_gear;
  snapshot.controller_x = node->last_controller_x;
  snapshot.controller_y = node->last_controller_y;
  snapshot.controller_axes_valid =
    node->controller_x_valid && node->controller_y_valid &&
    node->last_controller_axes_time > 0.0 &&
    dwa_node_now_sec() - node->last_controller_axes_time <=
      (double)node->params_snapshot.cmd_timeout_sec;
  snapshot.valid = node->wheel_feedback_valid;
  if (snapshot.valid) {
    const double age = dwa_node_now_sec() - node->last_wheel_feedback_time;
    snapshot.age = age > 0.0 ? age : 0.0;
    snapshot.fresh = snapshot.age <=
      (double)node->params_snapshot.hall_feedback_timeout_sec;
  }
  snapshot.raw_gear_valid = node->raw_gear_valid && snapshot.fresh;
  return snapshot;
}

planner_motor_feedback_snapshot_t dwa_node_motor_feedback_snapshot(
  const dwa_node_t *node)
{
  planner_motor_feedback_snapshot_t snapshot;
  hal_mutex_lock((hal_mutex_t *)&node->motor_feedback_mutex);
  snapshot = motor_feedback_snapshot_locked(node);
  hal_mutex_unlock((hal_mutex_t *)&node->motor_feedback_mutex);
  return snapshot;
}

planner_motor_feedback_snapshot_t dwa_node_consume_motor_feedback(
  dwa_node_t *node, bool forward_intent)
{
  planner_motor_feedback_snapshot_t snapshot;
  hal_mutex_lock(&node->motor_feedback_mutex);
  snapshot = motor_feedback_snapshot_locked(node);
  snapshot.forward_start_rising = forward_intent && node->forward_start_rising_latched;
  if (forward_intent) node->forward_start_rising_latched = false;
  hal_mutex_unlock(&node->motor_feedback_mutex);
  return snapshot;
}

bool dwa_node_pure_forward_intent(const dwa_node_t *node,
  const dwa_velocity_t *intent)
{
  return intent->v > node->params_snapshot.deadband_linear &&
    dwa_node_joystick_forward_angle_deg(node, intent) <=
    node->params_snapshot.forced_straight_manual_cone_deg;
}

float dwa_node_joystick_forward_angle_deg(const dwa_node_t *node,
  const dwa_velocity_t *intent)
{
  const float speed_floor = node->dynamic_config.forward_speed_limit > 1e-6f ?
    node->dynamic_config.forward_speed_limit : 1e-6f;
  const float yawrate_floor = node->config.max_yawrate > 1e-6f ?
    node->config.max_yawrate : 1e-6f;
  const float forward_ratio =
    (intent->v / speed_floor) > 0.0f ? intent->v / speed_floor : 0.0f;
  const float turn_ratio = fabsf(intent->w) / yawrate_floor;
  return atan2f(turn_ratio, forward_ratio) * 180.0f / NODE_PI;
}

float dwa_node_manual_yawrate_for_straight_override(const dwa_node_t *node,
  const dwa_velocity_t *intent)
{
  if (fabsf(intent->w) <= node->params_snapshot.forced_straight_yaw_tolerance ||
    node->params_snapshot.forced_straight_manual_cone_deg <= 1e-6f)
  {
    return 0.0f;
  }
  {
    const float ratio = dwa_clamp_value(
      dwa_node_joystick_forward_angle_deg(node, intent) /
      node->params_snapshot.forced_straight_manual_cone_deg, 0.0f, 1.0f);
    return (intent->w >= 0.0f ? 1.0f : -1.0f) *
      node->params_snapshot.forced_straight_manual_max_yawrate * ratio;
  }
}

const dwa_trajectory_t *dwa_node_find_valid_straight_trajectory(
  const dwa_plan_result_t *result)
{
  const dwa_trajectory_t *best = NULL;
  size_t i;
  for (i = 0; i < result->trajectory_count; ++i) {
    const dwa_trajectory_t *trajectory = &result->trajectories[i];
    if (!trajectory->valid || fabsf(trajectory->command.w) > 1e-6f) continue;
    if (best == NULL || trajectory->total_cost < best->total_cost) {
      best = trajectory;
    }
  }
  return best;
}

float dwa_node_preview_slew_output_linear(const dwa_node_t *node, float target)
{
  const float delta = target - node->last_output_linear;
  const float rate = delta >= 0.0f ?
    node->config.max_acceleration : node->config.max_deceleration;
  const float positive_rate = rate > 0.0f ? rate : 0.0f;
  const float positive_period = node->config.control_period > 0.0f ?
    node->config.control_period : 0.0f;
  const float max_step = positive_rate * positive_period;
  if (fabsf(delta) <= max_step || max_step <= 0.0f) {
    return target;
  }
  return node->last_output_linear + (delta > 0.0f ? max_step : -max_step);
}

float dwa_node_slew_output_linear(dwa_node_t *node, float target)
{
  node->last_output_linear = dwa_node_preview_slew_output_linear(node, target);
  return node->last_output_linear;
}

void dwa_node_reset_planning_control(dwa_node_t *node)
{
  node->moving_dwa_command_published = false;
  node->static_dwa_hall_target = 0.0f;
  node->static_dwa_hall_target_valid = false;
  node->last_valid_dwa_linear = 0.0f;
}
