/*
 * 模块功能：DWA 安全控制编排：前方急停、侧边保护、手动旁路、脱困和安全发布。
 * 直译自 native/nodes/impl/node_safety_control.inc 与 node_diagnostics.inc
 * （逐行对应，不改逻辑/默认值/执行顺序）。
 * P6；全 float 规范。
 *
 * ROS 发布（cmd_pub_ / hall_speed_control_pub_）直译为 cmd_sink 回调 +
 * node->last_published_cmd 记录；RViz 可视化发布为 no-op（publish_debug_
 * visualization 仅作门控，数据面无 RViz）。log_trajectory_scores 的 top-N
 * 排序在 C++ 侧为 std::sort（不稳定），此处为确定性最小值扫描，仅日志展示
 * 用，不影响控制行为。
 */
#include "dwa_node_internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../common/dwa_geometry.h"
#include "../hal/hal_log.h"
#include "../hal/hal_time.h"
#include "../safety/escape_controller.h"
#include "../safety/front_safety.h"
#include "../safety/side_safety.h"
#include "../safety/stationary_turn_speed_limiter.h"

#define NODE_PI ((float)3.14159265358979323846)

/* log_trajectory_scores 的 top-N 有效轨迹指针栈上限（仅日志用；
 * 超出部分不进入 top-N 统计，不影响控制行为）。 */
#define DWA_TRAJECTORY_SCORE_MAX_VALID 512

/* ---------- 日志节流 key（每个调用点唯一字符串地址） ---------- */

static const char *const kKeySweepYawLimit = "dns:sweep_yaw_limit";
static const char *const kKeySweepEscape = "dns:sweep_escape";
static const char *const kKeySweepForward = "dns:sweep_forward";
static const char *const kKeySweepReverse = "dns:sweep_reverse";
static const char *const kKeySweepTurnStop = "dns:sweep_turn_stop";
static const char *const kKeySweepHardStop = "dns:sweep_hard_stop";
static const char *const kKeyNeutralBefore = "dns:neutral_before";
static const char *const kKeyNeutralDone = "dns:neutral_done";
static const char *const kKeyFrontEscape = "dns:front_escape";
static const char *const kKeyFrontHardStop = "dns:front_hard_stop";
static const char *const kKeySideReverseStop = "dns:side_reverse_stop";
static const char *const kKeyReverseBypass = "dns:reverse_bypass";
static const char *const kKeyTurnSpeedGuard = "dns:turn_speed_guard";
static const char *const kKeyTurnSideWall = "dns:turn_side_wall";
static const char *const kKeyTurnDwa = "dns:turn_dwa";
static const char *const kKeyTurnSideCap = "dns:turn_side_cap";
static const char *const kKeyTurnSafetyAware = "dns:turn_safety_aware";
static const char *const kKeyGearTransition = "dns:gear_transition";
static const char *const kKeyGearLog = "dns:gear_log";
static const char *const kKeyStationaryTurnRing = "dns:stationary_turn_ring";
static const char *const kKeyScoreLog = "dns:score_log";


static const char *manual_turn_direction_name(
  planner_manual_turn_direction_t direction);

/* 记录一次连续的 DWA 硬停止脱困过程；同一过程的多个控制周期只计一次。 */
static void dns_record_hard_stop_escape_event(
  dwa_node_t *node, const char *type,
  planner_manual_turn_direction_t direction,
  const planner_side_clearance_diagnostics_t *side_clearance)
{
  if (node->hard_stop_escape_event_last_tick == 0 ||
      node->hard_stop_escape_event_last_tick + 1 != node->tick_count) {
    ++node->hard_stop_escape_event_count;
    hal_log(HAL_LOG_WARN,
      "脱困统计 source=dwa_hard_stop event=%llu type=%s direction=%s "
      "side_clear(L=%.3f,R=%.3f)",
      (unsigned long long)node->hard_stop_escape_event_count,
      type, manual_turn_direction_name(direction),
      side_clearance != NULL ? side_clearance->left : (float)INFINITY,
      side_clearance != NULL ? side_clearance->right : (float)INFINITY);
  }
  node->hard_stop_escape_event_last_tick = node->tick_count;
}

/* ---------- node_diagnostics.inc 直译 ---------- */

/* log_tick_timing(timing) 直译。 */
void dns_log_tick_timing(dwa_node_t *node, dwa_tick_timing_t *timing)
{
  const double classified_ms = timing->pointcloud_ms + timing->dwa_ms +
    timing->trajectory_publish_ms;
  const bool slow = timing->total_ms >= (double)node->tick_timing_warn_ms;

  dwa_tick_timing_finish(timing);
  if (!slow && node->tick_count % 5 != 0) return;

  hal_log(HAL_LOG_INFO,
    "tick_timing%s tick=%llu total_ms=%.1f pointcloud_ms=%.1f "
    "dwa_ms=%.1f trajectory_pub_ms=%.1f other_ms=%.1f classified_ms=%.1f",
    slow ? " SLOW" : "",
    (unsigned long long)node->tick_count,
    timing->total_ms, timing->pointcloud_ms, timing->dwa_ms,
    timing->trajectory_publish_ms, dwa_tick_timing_other_ms(timing),
    classified_ms);
}

/* log_gear_closed_loop_state(decision, input, intent) 直译。 */
void dns_log_gear_closed_loop_state(
  dwa_node_t *node,
  const dwa_dynamic_velocity_decision_t *decision,
  const dwa_dynamic_velocity_input_t *input,
  const dwa_velocity_t *intent)
{
  const bool changed = !node->gear_log_initialized ||
    node->last_logged_raw_gear != decision->raw_gear ||
    node->last_logged_effective_gear != decision->effective_gear ||
    node->last_logged_gear_target != decision->gear_target ||
    node->last_logged_gear_closed_loop_active !=
    decision->gear_closed_loop_active ||
    node->last_logged_gear_transition_active != decision->gear_transition_active;
  if (changed) {
    hal_log(HAL_LOG_INFO,
      "[档位闭环] 状态=%s 原始档位=%d 有效档位=%d 目标档位=%d 过渡中=%d "
      "原始有效=%d 前方距离=%.3fm S_DWA=%.3fm S_gear=%.3fm gear_t=%.3fs "
      "gear_comp=%.3fm H=%.3fm 原始意图(v=%.3f,w=%.3f) DWA上限=%.3fm/s "
      "发送档位=%d",
      decision->gear_closed_loop_active ? "强制1档" : "跟随实际档位",
      decision->raw_gear, decision->effective_gear, decision->gear_target,
      decision->gear_transition_active ? 1 : 0, decision->raw_gear_valid ? 1 : 0,
      input->front_clearance, decision->moving_prebrake_distance,
      decision->gear_trigger_distance, decision->gear_transition_remaining_sec,
      decision->gear_transition_compensation_distance,
      node->dynamic_config.gear_restore_hysteresis, intent->v, intent->w,
      decision->target_max_velocity, decision->effective_gear);
  } else {
    hal_log_throttle(hal_steady_now_sec(), 1000, kKeyGearLog, HAL_LOG_INFO,
      "[档位闭环] 原始档位=%d 有效档位=%d 目标档位=%d 过渡中=%d "
      "原始有效=%d 前方距离=%.3fm S_DWA=%.3fm S_gear=%.3fm gear_t=%.3fs "
      "gear_comp=%.3fm H=%.3fm 状态=%s 意图(v=%.3f,w=%.3f) DWA上限=%.3fm/s "
      "发送档位=%d DWA=%d",
      decision->raw_gear, decision->effective_gear, decision->gear_target,
      decision->gear_transition_active ? 1 : 0, decision->raw_gear_valid ? 1 : 0,
      input->front_clearance, decision->moving_prebrake_distance,
      decision->gear_trigger_distance, decision->gear_transition_remaining_sec,
      decision->gear_transition_compensation_distance,
      node->dynamic_config.gear_restore_hysteresis,
      decision->gear_closed_loop_active ? "强制1档" : "跟随实际档位",
      intent->v, intent->w, decision->target_max_velocity,
      decision->effective_gear,
      decision->dwa_takeover ? 1 : 0);
  }
  node->gear_log_initialized = true;
  node->last_logged_raw_gear = decision->raw_gear;
  node->last_logged_effective_gear = decision->effective_gear;
  node->last_logged_gear_target = decision->gear_target;
  node->last_logged_gear_closed_loop_active = decision->gear_closed_loop_active;
  node->last_logged_gear_transition_active = decision->gear_transition_active;
}

/* update_runtime_velocity_limit(decision) 直译。 */
float dns_update_runtime_velocity_limit(
  dwa_node_t *node, const dwa_dynamic_velocity_decision_t *decision)
{
  const float target = decision->target_max_velocity > 0.0f ?
    decision->target_max_velocity : 0.0f;
  if (decision->mode != DYNAMIC_CONTROL_HAND) {
    node->runtime_velocity_limit = target;
    node->runtime_velocity_limit_initialized = true;
    return node->runtime_velocity_limit;
  }
  if (!node->runtime_velocity_limit_initialized) {
    node->runtime_velocity_limit = target;
    node->runtime_velocity_limit_initialized = true;
    return node->runtime_velocity_limit;
  }
  {
    const float delta = target - node->runtime_velocity_limit;
    const float rate = delta >= 0.0f ?
      node->config.max_acceleration : node->config.max_deceleration;
    const float positive_rate = rate > 0.0f ? rate : 0.0f;
    const float positive_period = node->config.control_period > 0.0f ?
      node->config.control_period : 0.0f;
    const float max_step = positive_rate * positive_period;
    if (fabsf(delta) <= max_step || max_step <= 0.0f) {
      node->runtime_velocity_limit = target;
    } else {
      node->runtime_velocity_limit +=
        delta > 0.0f ? max_step : -max_step;
    }
    return node->runtime_velocity_limit;
  }
}

/* log_trajectory_scores(result) 直译（top-N 用确定性最小值扫描；
 * std::map reject 计数用小数组）。 */
void dns_log_trajectory_scores(
  dwa_node_t *node, const dwa_plan_result_t *result, dwa_tick_timing_t *timing)
{
  enum { MAX_REJECT_REASONS = 16 };
  const dwa_trajectory_t *valid_head[DWA_TRAJECTORY_SCORE_MAX_VALID];
  const char *reject_reasons[MAX_REJECT_REASONS];
  int reject_counts[MAX_REJECT_REASONS];
  int reject_unique = 0;
  size_t valid_count = 0;
  size_t i;
  int count;
  int top;
  char stream[2048];
  int offset = 0;

  if (!node->log_trajectory_scores) return;
  dwa_tick_timing_enter(timing, DWA_TICK_SEGMENT_DWA_PLANNING);

  for (i = 0; i < result->trajectory_count; ++i) {
    const dwa_trajectory_t *trajectory = &result->trajectories[i];
    if (trajectory->valid) {
      if (valid_count < DWA_TRAJECTORY_SCORE_MAX_VALID) {
        valid_head[valid_count++] = trajectory;
      }
    } else {
      const char *reason = trajectory->reject_reason == NULL ?
        "invalid_no_reason" : trajectory->reject_reason;
      int slot = -1;
      int j;
      for (j = 0; j < reject_unique; ++j) {
        if (strcmp(reject_reasons[j], reason) == 0) { slot = j; break; }
      }
      if (slot < 0 && reject_unique < MAX_REJECT_REASONS) {
        slot = reject_unique++;
        reject_reasons[slot] = reason;
        reject_counts[slot] = 0;
      }
      if (slot >= 0) ++reject_counts[slot];
    }
  }

  offset += snprintf(stream + offset, sizeof(stream) - (size_t)offset,
    "dwa_score valid=%d selected(v=%.3f,w=%.3f)",
    result->valid_count, result->selected.v, result->selected.w);

  count = node->trajectory_score_top_n < (int)valid_count ?
    node->trajectory_score_top_n : (int)valid_count;
  for (top = 0; top < count; ++top) {
    /* 确定性 top-N：每轮取剩余最小 total_cost（平局取先出现者）。 */
    int best = -1;
    int k;
    for (k = 0; k < (int)valid_count; ++k) {
      if (valid_head[k] == NULL) continue;
      if (best < 0 || valid_head[k]->total_cost < valid_head[best]->total_cost) {
        best = k;
      }
    }
    if (best < 0) break;
    {
      const dwa_trajectory_t *trajectory = valid_head[best];
      const dwa_state_t *end = trajectory->state_count == 0 ?
        NULL : &trajectory->states[trajectory->state_count - 1];
      valid_head[best] = NULL;
      offset += snprintf(stream + offset, sizeof(stream) - (size_t)offset,
        " top%d{v=%.3f,w=%.3f,yaw_deg=%.3f,end=(%.3f,%.3f)"
        ",clear=%.3f,total=%.3f,obs=%.3f,goal=%.3f,speed=%.3f,path=%.3f}",
        top,
        trajectory->command.v, trajectory->command.w,
        end ? end->yaw * 180.0f / NODE_PI : 0.0f,
        end ? end->x : 0.0f, end ? end->y : 0.0f,
        trajectory->min_clearance, trajectory->total_cost,
        trajectory->obstacle_cost, trajectory->to_goal_cost,
        trajectory->speed_cost, trajectory->path_cost);
      if (offset <= 0 || (size_t)offset >= sizeof(stream) - 1) break;
    }
  }

  if (reject_unique > 0) {
    offset += snprintf(stream + offset, sizeof(stream) - (size_t)offset,
      " reject{");
    for (top = 0; top < reject_unique; ++top) {
      offset += snprintf(stream + offset, sizeof(stream) - (size_t)offset,
        "%s%s=%d", top == 0 ? "" : ",", reject_reasons[top], reject_counts[top]);
    }
    offset += snprintf(stream + offset, sizeof(stream) - (size_t)offset, "}");
  }

  hal_log_throttle(hal_steady_now_sec(),
    node->trajectory_score_log_interval_ms, kKeyScoreLog, HAL_LOG_INFO,
    "%s", stream);

  dwa_tick_timing_leave(timing);
}

/* ---------- 安全配置组装（node_safety_control.inc 头部） ---------- */

/* front_safety_config() 直译。 */
static dwa_front_safety_config_t front_safety_config(const dwa_node_t *node)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  dwa_front_safety_config_t value = dwa_front_safety_config_default();
  value.trigger_start_x = p->front_trigger_start_x;
  value.trigger_base_width = p->front_trigger_base_width;
  value.trigger_expand_angle_deg = p->front_trigger_expand_angle_deg;
  return value;
}

/* turn_classification_config() 直译。 */
static dwa_turn_classification_config_t turn_classification_config(
  const dwa_node_t *node)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  dwa_turn_classification_config_t value = dwa_turn_classification_config_default();
  value.forward_speed_limit = node->dynamic_config.forward_speed_limit;
  value.max_yawrate = node->config.max_yawrate;
  value.deadband_linear = p->deadband_linear;
  value.deadband_angular = p->deadband_angular;
  value.forward_min_angle_deg = p->manual_turn_forward_min_angle_deg;
  value.forward_max_angle_deg = p->manual_turn_forward_max_angle_deg;
  return value;
}

/* side_safety_config() 直译。 */
static dwa_side_safety_config_t side_safety_config(const dwa_node_t *node)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  dwa_side_safety_config_t value = dwa_side_safety_config_default();
  value.circle_enabled = p->side_hard_stop_circle_enabled;
  value.circle_center_x = p->side_hard_stop_circle_center_x;
  value.circle_center_y = p->side_hard_stop_circle_center_y;
  value.circle_radius = p->side_hard_stop_circle_radius;
  value.wheel_track_m = p->side_hard_stop_wheel_track_m;
  value.sweep_prediction_sec = p->side_hard_stop_sweep_prediction_sec;
  value.sweep_brake_sec = p->side_hard_stop_sweep_brake_sec;
  value.sweep_min_yawrate = p->side_hard_stop_sweep_min_yawrate;
  value.deadband_linear = p->deadband_linear;
  value.deadband_angular = p->deadband_angular;
  value.turn = turn_classification_config(node);
  return value;
}

/* collision_front_edge() 直译。 */
float dns_collision_front_edge(const dwa_node_t *node)
{
  return dwa_collision_front_edge(&node->config.footprint);
}

/* front_trigger_half_width_at(x) 直译。 */
float dns_front_trigger_half_width_at(const dwa_node_t *node, float x)
{
  const dwa_front_safety_config_t config = front_safety_config(node);
  return dwa_front_trigger_half_width_at(x, &config);
}

/* point_inside_front_trigger_zone(point) 直译。 */
bool dns_point_inside_front_trigger_zone(
  const dwa_node_t *node, const dwa_point_t *point)
{
  const dwa_front_safety_config_t config = front_safety_config(node);
  return dwa_point_inside_front_trigger_zone(point, &config);
}

bool dns_point_inside_front_safety_lane(
  const dwa_node_t *node, const dwa_point_t *point)
{
  const dwa_front_safety_config_t config = front_safety_config(node);
  return dwa_point_inside_front_safety_lane(
    point, &config, &node->config.footprint);
}

/* point_inside_gear_front_trigger_zone(point) 直译。 */
bool dns_point_inside_gear_front_trigger_zone(
  const dwa_node_t *node, const dwa_point_t *point)
{
  dwa_front_safety_config_t value = front_safety_config(node);
  value.trigger_expand_angle_deg +=
    node->dynamic_config.gear_front_trigger_extra_angle_deg;
  return dwa_point_inside_front_trigger_zone(point, &value);
}

/* update_moving_prebrake_y_down_rate(input, decision) 直译。 */
float dns_update_moving_prebrake_y_down_rate(
  dwa_node_t *node,
  const dwa_dynamic_velocity_input_t *input,
  const dwa_dynamic_velocity_decision_t *decision)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const bool prebrake_active =
    decision->mode == DYNAMIC_CONTROL_MOVING_PREBRAKE ||
    decision->mode == DYNAMIC_CONTROL_MOVING_HOLD;
  float requested_rate;

  if (!prebrake_active) {
    node->moving_prebrake_y_down_rate = p->moving_prebrake_y_down_rate_min;
    return node->moving_prebrake_y_down_rate;
  }

  requested_rate = p->moving_prebrake_y_down_rate_min;
  if (input->perception_fresh && input->hall_speed_fresh &&
    isfinite(input->front_clearance) &&
    input->actual_forward_speed > node->dynamic_config.moving_dwa_entry_speed_max)
  {
    const float speed = input->actual_forward_speed > 0.0f ?
      input->actual_forward_speed : 0.0f;
    const float reserve_distance =
      speed * p->moving_prebrake_reaction_sec + p->moving_prebrake_safety_margin;
    const float available_distance = input->front_clearance -
      node->dynamic_config.moving_dwa_trigger_distance - reserve_distance;
    if (available_distance <= 1e-3f) {
      requested_rate = p->moving_prebrake_y_down_rate_max;
    } else {
      const float entry_max = node->dynamic_config.moving_dwa_entry_speed_max;
      const float speed_square_delta =
        speed * speed - entry_max * entry_max > 0.0f ?
        speed * speed - entry_max * entry_max : 0.0f;
      const float required_deceleration =
        speed_square_delta / (2.0f * available_distance);
      const float decel_floor =
        node->dynamic_config.moving_prebrake_deceleration > 1e-6f ?
        node->dynamic_config.moving_prebrake_deceleration : 1e-6f;
      const float urgency = dwa_clamp_value(
        required_deceleration / decel_floor, 0.0f, 1.0f);
      requested_rate = p->moving_prebrake_y_down_rate_min + urgency *
        (p->moving_prebrake_y_down_rate_max - p->moving_prebrake_y_down_rate_min);
    }
  }

  /* 制动力可以立即增强，但只能缓慢回落，避免点云抖动造成减速度反复跳变。 */
  if (requested_rate >= node->moving_prebrake_y_down_rate) {
    node->moving_prebrake_y_down_rate = requested_rate;
  } else {
    const float positive_period = node->config.control_period > 0.0f ?
      node->config.control_period : 0.0f;
    const float release_step =
      p->moving_prebrake_y_rate_release * positive_period;
    const float floor = node->moving_prebrake_y_down_rate - release_step;
    node->moving_prebrake_y_down_rate =
      requested_rate > floor ? requested_rate : floor;
  }
  return node->moving_prebrake_y_down_rate;
}

/* ---------- 发布面（ROS publish → cmd_sink 数据面） ---------- */

/* publish_hall_speed_control(active, target_linear, mode, hard_stop) 直译。 */
static void publish_hall_speed_control(
  dwa_node_t *node,
  bool active,
  float target_linear,
  dwa_dynamic_control_mode_t mode,
  bool hard_stop)
{
  dwa_hall_speed_control_msg_t message;
  message.data[0] = active ? 1.0f : 0.0f;
  message.data[1] = target_linear;
  message.data[2] = (float)(int)mode;
  message.data[3] = node->dynamic_config.moving_prebrake_target_speed;
  message.data[4] = node->moving_prebrake_y_down_rate;
  message.data[5] = (float)node->output_effective_gear;
  message.data[6] = node->output_gear_closed_loop_active ? 1.0f : 0.0f;
  message.data[7] = hard_stop ? 1.0f : 0.0f;
  message.escape_request = node->escape_request_pending ? 1 : 0;
  if (node->cmd_sink != NULL) {
    node->cmd_sink(node->cmd_sink_user_data, &node->last_published_cmd, &message);
  }
  node->escape_request_pending = false;
}

/* publish_regular_command(output, ...) 直译。 */
bool dns_publish_regular_command(
  dwa_node_t *node,
  const dwa_twist_t *output,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  bool hard_stop)
{
  dwa_twist_t published = *output;
  const float previous_linear = published.linear_x;
  published.linear_x = stsl_apply_linear_limit(
    published.linear_x, node->stationary_turn_ring_speed_limit_mps);
  if (isfinite(node->stationary_turn_ring_speed_limit_mps)) {
    if (fabsf(previous_linear) > fabsf(published.linear_x)) {
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyStationaryTurnRing,
        HAL_LOG_INFO,
        "stationary-turn speed ring active nearest=%.3fm cap=%.3fm/s "
        "linear=%.3f->%.3f yaw=%.3f",
        node->stationary_turn_ring_nearest_distance_m,
        node->stationary_turn_ring_speed_limit_mps,
        previous_linear, published.linear_x, published.angular_z);
    }
  }
  node->last_published_cmd = published;
  node->last_published_tick = node->tick_count;
  publish_hall_speed_control(
    node, hall_speed_control_active, published.linear_x, mode, hard_stop);
  return true;
}

/* publish_command_without_trajectory(output, ...) 直译。
 * （publish_empty_selected / publish_candidates({}) / publish_geometric_
 * selection({}) 为 RViz 轨迹清理，数据面 no-op。） */
void dns_publish_command_without_trajectory(
  dwa_node_t *node,
  const dwa_twist_t *output,
  bool hall_speed_control_active,
  dwa_dynamic_control_mode_t mode,
  bool hard_stop)
{
  dns_publish_regular_command(
    node, output, hall_speed_control_active, mode, hard_stop);
}

/* reset_static_dwa_command_tracking() 直译。 */
void dns_reset_static_dwa_command_tracking(dwa_node_t *node)
{
  node->static_dwa_hall_target = 0.0f;
  node->static_dwa_hall_target_valid = false;
}

/* static_dwa_planning_hold_target(input, runtime_max_velocity) 直译。 */
float dns_static_dwa_planning_hold_target(
  const dwa_node_t *node,
  const dwa_dynamic_velocity_input_t *input,
  float runtime_max_velocity)
{
  float target = 0.0f;
  if (node->static_dwa_hall_target_valid) {
    target = node->static_dwa_hall_target;
  } else if (input->hall_speed_fresh) {
    target = input->actual_forward_speed;
  }
  return dwa_clamp_value(target, 0.0f, runtime_max_velocity);
}

/* note_static_dwa_command_published(target_linear) 直译。 */
void dns_note_static_dwa_command_published(dwa_node_t *node, float target_linear)
{
  node->static_dwa_hall_target =
    target_linear > 0.0f ? target_linear : 0.0f;
  node->static_dwa_hall_target_valid = true;
}

/* manual_turn_direction_name(direction) 直译。 */
static const char *manual_turn_direction_name(
  planner_manual_turn_direction_t direction)
{
  switch (direction) {
    case PLANNER_TURN_RIGHT: return "RIGHT";
    case PLANNER_TURN_LEFT: return "LEFT";
    case PLANNER_TURN_NONE: return "NONE";
  }
  return "NONE";
}

/* ---------- 硬停止脱困状态 ---------- */

/* reset_hard_stop_escape_state() 直译。 */
static void reset_hard_stop_escape_state(dwa_node_t *node)
{
  node->hard_stop_escape_eval_cycles = 0;
  node->hard_stop_escape_direction = PLANNER_TURN_NONE;
  node->hard_stop_escape_last_active_tick = 0;
  node->hard_stop_escape_started_sec = (double)NAN;
  node->hard_stop_escape_direction_latched_sec = (double)NAN;
}

/* limit_near_obstacle_yawrate(yawrate, nearby_obstacle) 直译。 */
static float limit_near_obstacle_yawrate(
  const dwa_node_t *node, float yawrate, bool nearby_obstacle)
{
  if (!node->params_snapshot.near_obstacle_yawrate_limit_enabled ||
      !nearby_obstacle || !isfinite(yawrate)) {
    return yawrate;
  }
  {
    const float cap = node->params_snapshot.near_obstacle_max_yawrate;
    return -cap > yawrate ? -cap : (cap < yawrate ? cap : yawrate);
  }
}

/* ---------- 安全发布 ---------- */

/* publish_motion_command_safely(...) 直译。 */
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
  const planner_side_clearance_diagnostics_t *side_clearance)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const dwa_velocity_t command = { requested->linear_x, requested->angular_z };
  dwa_side_safety_config_t config = side_safety_config(node);
  const planner_side_hard_stop_decision_t guard = dwa_evaluate_side_sweep_guard(
    &command, manual_intent, motor_feedback, perception_fresh,
    safety_points->points, safety_points->count, &config);
  if (!guard.stop) {
    dwa_twist_t safe_command = *requested;
    if (guard.yaw_limited) {
      safe_command.angular_z = guard.limited_yawrate;
      {
        const float actual_yawrate = motor_feedback->fresh ?
          (motor_feedback->right_mps - motor_feedback->left_mps) /
          p->side_hard_stop_wheel_track_m : 0.0f;
        hal_log_throttle(hal_steady_now_sec(), 200, kKeySweepYawLimit,
          HAL_LOG_WARN,
          "side sweep yaw limit source=%s direction=%s side=%s "
          "predicted_t=%.2fs point=(%.3f,%.3f) requested(v=%.3f,w=%+.3f) "
          "safe_w=%+.3f actual_w=%+.3f",
          source, manual_turn_direction_name(guard.direction),
          dwa_side_wall_hold_side_name(guard.side),
          guard.prediction_time_sec, guard.point.x, guard.point.y,
          command.v, command.w, guard.limited_yawrate, actual_yawrate);
      }
    }
    dns_publish_regular_command(
      node, &safe_command, hall_speed_control_active, mode, false);
    return true;
  }

  /* DWA 只报告侧向硬停止脱困请求；安全方向与输出统一交由电机桥模块决定。 */
  if (side_clearance != NULL && command.v > p->deadband_linear) {
    dwa_twist_t stop;
    dwa_twist_reset(&stop);
    node->last_output_linear = 0.0f;
    node->escape_request_pending = true;
    dns_record_hard_stop_escape_event(
      node, "SIDE_SWEEP", PLANNER_TURN_NONE, side_clearance);
    dns_publish_command_without_trajectory(node, &stop, false, mode, true);
    hal_log_throttle(hal_steady_now_sec(), 250, kKeySweepEscape,
      HAL_LOG_WARN,
      "side sweep hard stop escape delegated to motor bridge "
      "side_clear(L=%.3f,R=%.3f) requested(v=%.3f,w=%+.3f)",
      side_clearance->left, side_clearance->right, command.v, command.w);
    return true;
  }
  /* 侧扫掠保护不做跨周期锁存：后退保持原有旁路，前进时只取消朝障碍侧的转向。
   * 这里必须依据最终待发布的 command.v，而不是 manual_intent.v：DWA接管时
   * 手柄意图可能已经为零，但DWA仍可能给出安全的正向线速度；此时不能把
   * 侧边扫掠保护误升级为整车停车。
   * 侧边圆是转弯扫掠保护，不等价于车体正前方碰撞；前方保护仍由前方硬停止
   * 和轨迹碰撞负责。 */
  {
    const bool forward_intent = command.v > p->deadband_linear;
    const bool reverse_intent = command.v < -p->deadband_linear;
    if (forward_intent) {
      dwa_twist_t straight = *requested;
      straight.angular_z = 0.0f;
      node->last_output_linear = straight.linear_x;
      dns_publish_regular_command(
        node, &straight, hall_speed_control_active, mode, false);
      hal_log_throttle(hal_steady_now_sec(), 250, kKeySweepForward,
        HAL_LOG_WARN,
        "侧扫掠保护：前进保留线速度，仅取消危险侧转向 "
        "source=%s side=%s radius=%.3fm point=(%.3f,%.3f) out(v=%.3f,w=0.000)",
        source, dwa_side_wall_hold_side_name(guard.side),
        guard.radius, guard.point.x, guard.point.y, straight.linear_x);
      return true;
    }
    if (!forward_intent) {
      if (reverse_intent) {
        dns_publish_regular_command(
          node, requested, hall_speed_control_active, mode, false);
        hal_log_throttle(hal_steady_now_sec(), 250, kKeySweepReverse,
          HAL_LOG_WARN,
          "侧扫掠保护：后退保持原始指令 raw(v=%.3f,w=%+.3f)",
          manual_intent->v, manual_intent->w);
        return true;
      }
      {
        dwa_twist_t turn_stop;
        dwa_twist_reset(&turn_stop);
        dns_publish_regular_command(node, &turn_stop, false, mode, false);
        hal_log_throttle(hal_steady_now_sec(), 250, kKeySweepTurnStop,
          HAL_LOG_WARN,
          "侧扫掠保护：当前周期禁止危险转向 raw(v=%.3f,w=%+.3f)",
          manual_intent->v, manual_intent->w);
      }
      return false;
    }
    {
      dwa_twist_t stop;
      dwa_twist_reset(&stop);
      node->last_output_linear = 0.0f;
      node->escape_request_pending = true;
      dns_record_hard_stop_escape_event(
        node, "SIDE_SWEEP", PLANNER_TURN_NONE, side_clearance);
      dns_publish_regular_command(node, &stop, false, mode, true);
      hal_log_throttle(hal_steady_now_sec(), 250, kKeySweepHardStop,
        HAL_LOG_ERROR,
        "side sweep hard stop; escape request delegated to bridge "
        "side_clear(L=%.3f,R=%.3f)",
        side_clearance->left, side_clearance->right);
    }
    return true;
  }
}

/* publish_motion_command_without_trajectory_safely(...) 直译。 */
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
  const planner_side_clearance_diagnostics_t *side_clearance)
{
  dns_publish_motion_command_safely(
    node, output, hall_speed_control_active, mode, motor_feedback,
    perception_fresh, safety_points, manual_intent, source, side_clearance);
}

/* ---------- 手动旁路 ---------- */

/* handle_manual_bypass(...) 直译。 */
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
  const dwa_point_buf_t *side_safety_points)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const dwa_dynamic_velocity_config_t *dc = &node->dynamic_config;
  dwa_twist_t output;
  dwa_side_safety_config_t sconfig = side_safety_config(node);
  (void)nearest_radar_distance;
  planner_side_hard_stop_decision_t side_hard_stop;
  bool forward_hard_stop;
  bool front_no_turn_hard_stop;
  bool forward_hard_stop_active;
  bool side_hard_stop_active;
  bool nearby_front_obstacle;
  bool nearby_side_obstacle;
  bool nearby_obstacle;

  dwa_twist_reset(&output);
  /* 中立意味着本次硬停止事件结束，清除两周期方向保持状态。 */
  if (dwa_manual_command_is_neutral(raw_intent, p->deadband_linear,
      p->deadband_angular))
  {
    reset_hard_stop_escape_state(node);
  }
  if (dwa_manual_command_is_neutral(raw_intent, p->deadband_linear,
      p->deadband_angular))
  {
    /* 记录回中处理前的所有可观测状态，便于确认旧的脱困/侧边保护指令
     * 是否仍在生效。 */
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyNeutralBefore,
      HAL_LOG_WARN,
      "控制器回中处理前：原始意图(v=%.3f,w=%.3f) 模式=%s DWA接管=%d "
      "侧方限速锁存=%d 侧墙锁存(左=%d,右=%d) "
      "运动DWA已发布=%d 强制直行=%d",
      raw_intent->v, raw_intent->w,
      dwa_dynamic_control_mode_name(side_decision->mode),
      side_decision->dwa_takeover ? 1 : 0,
      side_decision->side_cap_active ? 1 : 0,
      side_wall_hold->left_latched ? 1 : 0,
      side_wall_hold->right_latched ? 1 : 0,
      node->moving_dwa_command_published ? 1 : 0,
      node->forced_straight_active ? 1 : 0);

    dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
    dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
    dwa_node_reset_planning_control(node);
    node->forced_straight_active = false;
    node->last_output_linear = 0.0f;
    dns_publish_command_without_trajectory(
      node, &output, false, DYNAMIC_CONTROL_HAND, hard_stop_latched);
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyNeutralDone,
      HAL_LOG_WARN,
      "控制器回中处理完成：意图已清零(v=0.000,w=0.000) "
      "DWA接管=0 侧方限速锁存=0 侧墙锁存(左=0,右=0) "
      "运动DWA已发布=0 强制直行=0 最终输出(v=0.000,w=0.000)");
    return true;
  }

  /* D6在近于最小稳定量程时会饱和到约6cm；前方/侧方硬停止
   * 仍先按原有保护条件判定，再由统一脱困策略决定是否允许低速转向。 */
  {
    const bool front_radar_contact =
      isfinite(front_nearest_radar_distance) &&
      front_nearest_radar_distance <= p->radar_contact_distance;
    const bool front_envelope_intrusion = isfinite(front_clearance) &&
      front_clearance <= 0.0f;
    forward_hard_stop = raw_intent->v > p->deadband_linear &&
      (hard_stop_latched || (perception_fresh &&
      (front_radar_contact || front_envelope_intrusion)));
    front_no_turn_hard_stop = perception_fresh &&
      front_no_turn_stop->point_count > 0 &&
      raw_intent->v >= -p->deadband_linear;
    side_hard_stop = dwa_evaluate_side_hard_stop_circle(
      raw_intent, perception_fresh, side_hard_stop_circle, &sconfig);
    forward_hard_stop_active = forward_hard_stop || front_no_turn_hard_stop;
    side_hard_stop_active = side_hard_stop.stop;
    nearby_front_obstacle = perception_fresh &&
      ((isfinite(front_clearance) &&
      front_clearance <= dc->moving_dwa_trigger_distance) ||
      front_no_turn_stop->point_count > 0 || forward_hard_stop_active);
    nearby_side_obstacle = perception_fresh &&
      ((isfinite(side_clearance->left) &&
      side_clearance->left <= dc->side_limit_distance) ||
      (isfinite(side_clearance->right) &&
      side_clearance->right <= dc->side_limit_distance) ||
      side_hard_stop_active);
    nearby_obstacle = nearby_front_obstacle || nearby_side_obstacle;
  }

  if (!forward_hard_stop_active && !side_hard_stop_active) {
    /* 没有任何硬停止的周期结束上一事件，避免下一次侧扫掠沿用旧方向。 */
    reset_hard_stop_escape_state(node);
  }

  if (forward_hard_stop_active ||
    (side_hard_stop_active && raw_intent->v >= -p->deadband_linear))
  {
    dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
    dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
    dwa_node_reset_planning_control(node);
    node->forced_straight_active = false;
    node->last_output_linear = 0.0f;
    {
      const bool request_escape = raw_intent->v > p->deadband_linear;
      if (request_escape) {
        node->escape_request_pending = true;
        dns_record_hard_stop_escape_event(
          node, forward_hard_stop_active ? "FRONT" : "SIDE",
          PLANNER_TURN_NONE, side_clearance);
        hal_log_throttle(hal_steady_now_sec(), 250, kKeyFrontEscape,
          HAL_LOG_WARN,
          "hard stop escape delegated type=%s side_clear(L=%.3f,R=%.3f)",
          forward_hard_stop_active ? "FRONT" : "SIDE",
          side_clearance->left, side_clearance->right);
      }
      dns_publish_command_without_trajectory(
        node, &output, false, DYNAMIC_CONTROL_HAND, request_escape);
    }
    if (raw_intent->v <= p->deadband_linear && forward_hard_stop_active) {
      hal_log_throttle(hal_steady_now_sec(), 250, kKeyFrontHardStop,
        HAL_LOG_ERROR,
        "front hard stop: no live forward intent; escape not requested "
        "front_clear=%.3f side_clear(L=%.3f,R=%.3f)",
        front_clearance, side_clearance->left, side_clearance->right);
    }
    return true;
  }

  if (side_hard_stop_active && raw_intent->v < -p->deadband_linear) {
    dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
    dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
    dwa_node_reset_planning_control(node);
    node->forced_straight_active = false;
    node->last_output_linear = 0.0f;
    dns_publish_command_without_trajectory(
      node, &output, false, DYNAMIC_CONTROL_HAND, true);
    hal_log_throttle(hal_steady_now_sec(), 250, kKeySideReverseStop,
      HAL_LOG_ERROR,
      "side hard stop reverse: remain stopped; no forward escape request "
      "side_clear(L=%.3f,R=%.3f)",
      side_clearance->left, side_clearance->right);
    return true;
  }

  if (raw_intent->v < -p->deadband_linear) {
    dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
    dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
    dwa_node_reset_planning_control(node);
    node->forced_straight_active = false;
    output.linear_x = raw_intent->v;
    output.angular_z = limit_near_obstacle_yawrate(
      node, raw_intent->w, nearby_obstacle);
    node->last_output_linear = output.linear_x;
    dns_publish_command_without_trajectory(node, &output, false,
      DYNAMIC_CONTROL_HAND, false);
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyReverseBypass,
      HAL_LOG_INFO,
      "dwa reverse bypass raw(v=%.2f,w=%.2f) out(v=%.2f,w=%.2f)",
      raw_intent->v, raw_intent->w, output.linear_x, output.angular_z);
    return true;
  }

  {
    float turn_angle_deg = 90.0f;
    const dwa_turn_classification_config_t tconfig =
      turn_classification_config(node);
    const planner_manual_turn_direction_t turn_direction =
      dwa_manual_turn_direction(raw_intent, &tconfig, &turn_angle_deg);
    float same_side_clearance = (float)INFINITY;
    if (turn_direction != PLANNER_TURN_NONE) {
      same_side_clearance = turn_direction == PLANNER_TURN_RIGHT ?
        side_clearance->right : side_clearance->left;
    }
    if (turn_direction != PLANNER_TURN_NONE) {
      const bool moving_speed_guard = raw_intent->v > p->deadband_linear &&
        (side_decision->mode == DYNAMIC_CONTROL_MOVING_PREBRAKE ||
        side_decision->mode == DYNAMIC_CONTROL_MOVING_HOLD);
      if (moving_speed_guard) {
        hal_log_throttle(hal_steady_now_sec(), 250, kKeyTurnSpeedGuard,
          HAL_LOG_INFO,
          "manual turn handed to moving speed guard direction=%s "
          "angle=%.1fdeg mode=%s raw(v=%.2f,w=%+.2f)",
          manual_turn_direction_name(turn_direction), turn_angle_deg,
          dwa_dynamic_control_mode_name(side_decision->mode),
          raw_intent->v, raw_intent->w);
        return false;
      }

      {
        const bool near_forward_angle =
          turn_angle_deg >=
          p->manual_turn_forward_min_angle_deg -
          p->side_wall_hold_forward_margin_deg &&
          turn_angle_deg <=
          p->manual_turn_forward_max_angle_deg +
          p->side_wall_hold_forward_margin_deg;
        const bool forward_turn_guarded =
          near_forward_angle && raw_intent->v > p->deadband_linear &&
          side_wall_hold->active &&
          fabsf(guarded_intent->w - raw_intent->w) > 1e-6f;
        if (forward_turn_guarded) {
          hal_log_throttle(hal_steady_now_sec(), 250, kKeyTurnSideWall,
            HAL_LOG_INFO,
            "manual turn handled by side wall hold direction=%s "
            "angle=%.1fdeg raw(v=%.2f,w=%+.2f) guarded_w=%+.2f",
            manual_turn_direction_name(turn_direction), turn_angle_deg,
            raw_intent->v, raw_intent->w, guarded_intent->w);
          return false;
        }
      }

      if (side_decision->dwa_takeover) {
        hal_log_throttle(hal_steady_now_sec(), 250, kKeyTurnDwa,
          HAL_LOG_INFO,
          "manual turn handed to DWA direction=%s angle=%.1fdeg "
          "raw(v=%.2f,w=%+.2f)",
          manual_turn_direction_name(turn_direction), turn_angle_deg,
          raw_intent->v, raw_intent->w);
        return false;
      }

      if (side_decision->side_cap_active) {
        const float cap_target = side_decision->target_max_velocity > 0.0f ?
          side_decision->target_max_velocity : 0.0f;
        output.linear_x = raw_intent->v > 0.0f ?
          (raw_intent->v < cap_target ? raw_intent->v : cap_target) :
          (raw_intent->v < 0.0f ? 0.0f : raw_intent->v);
        output.angular_z = limit_near_obstacle_yawrate(
          node, guarded_intent->w, nearby_obstacle);
        node->last_output_linear = output.linear_x;
        dns_publish_motion_command_without_trajectory_safely(
          node, &output, side_decision->hall_speed_control_active,
          side_decision->mode, motor_feedback, perception_fresh,
          side_safety_points, raw_intent, "manual_side_cap", side_clearance);
        hal_log_throttle(hal_steady_now_sec(), 250, kKeyTurnSideCap,
          HAL_LOG_INFO,
          "manual turn side-capped direction=%s angle=%.1fdeg "
          "clearance=%.3fm raw(v=%.2f,w=%+.2f) out(v=%.2f,w=%.2f)",
          manual_turn_direction_name(turn_direction), turn_angle_deg,
          side_decision->side_clearance, raw_intent->v, raw_intent->w,
          output.linear_x, output.angular_z);
        return true;
      }

      dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
      dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
      dwa_node_reset_planning_control(node);
      node->forced_straight_active = false;

      /* 手动转向旁路仍必须沿用侧墙保护后的方向，不能恢复原始危险角速度。
       * 没有侧墙约束时 guarded_intent 与 raw_intent 相同，不影响正常手动
       * 转向。 */
      output.linear_x = guarded_intent->v;
      output.angular_z = limit_near_obstacle_yawrate(
        node, guarded_intent->w, nearby_obstacle);
      node->last_output_linear = output.linear_x;
      dns_publish_motion_command_without_trajectory_safely(
        node, &output, false, DYNAMIC_CONTROL_HAND, motor_feedback,
        perception_fresh, side_safety_points, raw_intent,
        "manual_turn_safety_aware", side_clearance);
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyTurnSafetyAware,
        HAL_LOG_INFO,
        "dwa safety-aware turn direction=%s angle=%.1fdeg side=%.3fm "
        "raw(v=%.2f,w=%.2f) out(v=%.2f,w=%.2f)",
        manual_turn_direction_name(turn_direction), turn_angle_deg,
        same_side_clearance, raw_intent->v, raw_intent->w,
        output.linear_x, output.angular_z);
      return true;
    }
  }

  return false;
}

/* handle_direct_moving_control(...) 直译。 */
bool dns_handle_direct_moving_control(
  dwa_node_t *node,
  const dwa_velocity_t *intent,
  const dwa_velocity_t *manual_intent,
  const dwa_dynamic_velocity_input_t *dynamic_input,
  const dwa_dynamic_velocity_decision_t *decision,
  float runtime_max_velocity,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  const dwa_point_buf_t *side_safety_points)
{
  const dwa_dynamic_velocity_config_t *dc = &node->dynamic_config;
  const bool speed_control_wait =
    decision->mode == DYNAMIC_CONTROL_MOVING_PREBRAKE ||
    decision->mode == DYNAMIC_CONTROL_MOVING_HOLD;
  const bool before_dwa_distance =
    !isfinite(dynamic_input->front_clearance) ||
    dynamic_input->front_clearance >= dc->moving_dwa_trigger_distance;
  const bool direct_control = !decision->dwa_takeover &&
    dynamic_input->perception_fresh &&
    (speed_control_wait || before_dwa_distance);
  bool hall_control_active;
  dwa_twist_t output;
  (void)runtime_max_velocity;

  if (!direct_control) return false;

  dwa_twist_reset(&output);
  {
    const bool gear_speed_transition =
      decision->mode == DYNAMIC_CONTROL_HAND &&
      !decision->dwa_takeover && decision->gear_speed_transition_active;
    /* 速度交接归电机桥所有。DWA 只提供原始控制器/意图指令；桥负责
     * 预减速、霍尔环、加减速和档位过渡。 */
    if (decision->mode == DYNAMIC_CONTROL_MOVING_PREBRAKE ||
      decision->mode == DYNAMIC_CONTROL_MOVING_HOLD)
    {
      /* 9.27：预减速直控输出改用融合层的距离比例目标（净距远高近低、
       * 限速率平滑），不再发恒定 moving_prebrake_target_speed——两档制
       * 下该常量 0.6944 与满速几乎相等，正前障碍接近时观察不到减速。
       * 速度模块只限速不加速：目标高于用户当前油门时按用户意图输出
       * （慢速骑行接近远处障碍不得被模块反向提速）。 */
      const float prebrake_cmd = decision->target_max_velocity;
      output.linear_x =
        (manual_intent->v > 0.02f && manual_intent->v < prebrake_cmd) ?
        manual_intent->v : prebrake_cmd;
    } else {
      output.linear_x = manual_intent->v;
    }
    output.angular_z = intent->w;
    hall_control_active =
      decision->mode == DYNAMIC_CONTROL_MOVING_PREBRAKE ||
      decision->mode == DYNAMIC_CONTROL_MOVING_HOLD;
    if (gear_speed_transition) {
      hal_log_throttle(hal_steady_now_sec(), 200, kKeyGearTransition,
        HAL_LOG_INFO,
        "档位速度过渡：模式=手动 原始档位=%d 有效档位=%d 目标档位=%d "
        "目标速度=%.3f 进度=%.3f 输出速度=%.3f",
        decision->raw_gear, decision->effective_gear, decision->gear_target,
        decision->gear_speed_transition_target,
        decision->gear_speed_transition_progress, output.linear_x);
    }
  }
  dns_publish_motion_command_without_trajectory_safely(
    node, &output, hall_control_active, decision->mode, motor_feedback,
    dynamic_input->perception_fresh, side_safety_points, manual_intent,
    "direct_moving_control", NULL);
  return true;
}
