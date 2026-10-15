/*
 * 模块功能：侧边圆形硬停和扫掠预测保护实现。
 * 直译自 native/safety/side_safety.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * C++ lambda command_is_safe → command_is_safe_ctx_t 上下文结构 + 静态函数。
 */
#include "side_safety.h"

static inline float ssf_min(float a, float b) { return a < b ? a : b; }
static inline float ssf_max(float a, float b) { return a > b ? a : b; }

bool dwa_predict_side_circle_entry(
  const dwa_point_t *point,
  float linear_speed,
  float yawrate,
  float horizon_sec,
  const dwa_side_safety_config_t *config,
  float *entry_time_sec,
  float *entry_radius)
{
  const float sample_dt = 0.05f;
  float pose_x = 0.0f;
  float pose_y = 0.0f;
  float pose_yaw = 0.0f;
  float elapsed = 0.0f;
  while (elapsed <= horizon_sec + 1e-9f) {
    const float cos_yaw = cosf(pose_yaw);
    const float sin_yaw = sinf(pose_yaw);
    const float relative_x = point->x - pose_x;
    const float relative_y = point->y - pose_y;
    const float local_x = cos_yaw * relative_x + sin_yaw * relative_y;
    const float local_y = -sin_yaw * relative_x + cos_yaw * relative_y;
    const float radius = hypotf(
      local_x - config->circle_center_x,
      local_y - config->circle_center_y);
    if (radius <= config->circle_radius) {
      *entry_time_sec = elapsed;
      *entry_radius = radius;
      return true;
    }

    if (elapsed >= horizon_sec - 1e-9f) break;
    const float dt = ssf_min(sample_dt, horizon_sec - elapsed);
    pose_x += linear_speed * cosf(pose_yaw) * dt;
    pose_y += linear_speed * sinf(pose_yaw) * dt;
    pose_yaw += yawrate * dt;
    elapsed += dt;
  }
  return false;
}

/* set_side_collision(target, point_on_left, point, time_sec, radius) 直译。 */
static void ssf_set_side_collision(
  planner_side_hard_stop_decision_t *target,
  bool point_on_left,
  const dwa_point_t *point,
  float time_sec,
  float radius)
{
  target->direction = point_on_left ?
    PLANNER_TURN_LEFT : PLANNER_TURN_RIGHT;
  target->side = point_on_left ?
    DWA_SIDE_HOLD_LEFT : DWA_SIDE_HOLD_RIGHT;
  target->point = *point;
  target->prediction_time_sec = time_sec;
  target->radius = radius;
}

planner_side_hard_stop_decision_t dwa_evaluate_side_hard_stop_circle(
  const dwa_velocity_t *intent,
  bool perception_fresh,
  const planner_side_hard_stop_circle_diagnostics_t *diagnostics,
  const dwa_side_safety_config_t *config)
{
  planner_side_hard_stop_decision_t decision =
    planner_side_hard_stop_decision_default();
  if (!config->circle_enabled || !perception_fresh) return decision;

  float angle_deg = 90.0f;
  const planner_manual_turn_direction_t direction =
    dwa_manual_turn_direction_with_reverse(intent, &config->turn, &angle_deg);
  if (direction == PLANNER_TURN_NONE) return decision;

  const bool check_left = direction == PLANNER_TURN_LEFT;
  const float radius = check_left ? diagnostics->left_radius : diagnostics->right_radius;
  if (!isfinite(radius) || radius > config->circle_radius) return decision;

  decision.stop = true;
  decision.direction = direction;
  decision.side = check_left ?
    DWA_SIDE_HOLD_LEFT : DWA_SIDE_HOLD_RIGHT;
  decision.radius = radius;
  decision.point = check_left ? diagnostics->left_point : diagnostics->right_point;
  return decision;
}

/* lambda command_is_safe 捕获展开（evaluate_left/prediction_speed/
 * safety_points/config 均为只读捕获）。 */
typedef struct {
  const dwa_side_safety_config_t *config;
  const dwa_point_t *safety_points;
  size_t safety_points_count;
  bool evaluate_left;
  float prediction_speed;
} command_is_safe_ctx_t;

static bool ssf_command_is_safe(
  const command_is_safe_ctx_t *ctx,
  float yawrate,
  planner_side_hard_stop_decision_t *collision)
{
  bool safe = true;
  float earliest_time = INFINITY;
  for (size_t i = 0; i < ctx->safety_points_count; ++i) {
    const dwa_point_t *point = &ctx->safety_points[i];
    const bool point_on_requested_side = ctx->evaluate_left ?
      point->y > ctx->config->circle_center_y :
      point->y < ctx->config->circle_center_y;
    if (!point_on_requested_side) continue;

    float entry_time = 0.0f;
    float entry_radius = INFINITY;
    if (!dwa_predict_side_circle_entry(
        point, ctx->prediction_speed, yawrate, ctx->config->sweep_prediction_sec,
        ctx->config, &entry_time, &entry_radius))
    {
      continue;
    }
    if (entry_time < earliest_time) {
      earliest_time = entry_time;
      ssf_set_side_collision(
        collision, ctx->evaluate_left, point, entry_time, entry_radius);
    }
    safe = false;
  }
  return safe;
}

planner_side_hard_stop_decision_t dwa_evaluate_side_sweep_guard(
  const dwa_velocity_t *command,
  const dwa_velocity_t *manual_intent,
  const planner_motor_feedback_snapshot_t *motor_feedback,
  bool perception_fresh,
  const dwa_point_t *safety_points,
  size_t safety_points_count,
  const dwa_side_safety_config_t *config)
{
  planner_side_hard_stop_decision_t decision =
    planner_side_hard_stop_decision_default();
  if (!config->circle_enabled || !perception_fresh ||
    command->v < -config->deadband_linear)
  {
    return decision;
  }

  const float actual_yawrate = motor_feedback->fresh ?
    (motor_feedback->right_mps - motor_feedback->left_mps) /
    config->wheel_track_m : 0.0f;
  const bool actual_left = actual_yawrate >= config->sweep_min_yawrate;
  const bool actual_right = actual_yawrate <= -config->sweep_min_yawrate;
  const bool requested_left = command->w > config->deadband_angular;
  const bool requested_right = command->w < -config->deadband_angular;
  const bool manual_left = manual_intent->w > config->deadband_angular;
  const bool manual_right = manual_intent->w < -config->deadband_angular;
  /* The circle guard responds only to an explicit manual turn. A pure forward
   * command must not be stopped by DWA's own angular correction. */
  const bool protect_left = manual_left && (actual_left || requested_left);
  const bool protect_right = manual_right && (actual_right || requested_right);
  if (!protect_left && !protect_right) return decision;

  const float measured_speed = motor_feedback->fresh ?
    ssf_max(0.0f, 0.5f * (motor_feedback->left_mps + motor_feedback->right_mps)) :
    ssf_max(0.0f, command->v);

  for (size_t i = 0; i < safety_points_count; ++i) {
    const dwa_point_t *point = &safety_points[i];
    const bool point_on_left = point->y > config->circle_center_y;
    const bool point_on_right = point->y < config->circle_center_y;
    if ((!point_on_left || !protect_left) && (!point_on_right || !protect_right)) {
      continue;
    }

    const float current_radius = hypotf(
      point->x - config->circle_center_x,
      point->y - config->circle_center_y);
    if (current_radius <= config->circle_radius) {
      decision.stop = true;
      ssf_set_side_collision(&decision, point_on_left, point, 0.0f, current_radius);
      return decision;
    }

    const bool actual_toward_point = point_on_left ? actual_left : actual_right;
    if (!actual_toward_point) continue;
    float entry_time = 0.0f;
    float entry_radius = INFINITY;
    if (dwa_predict_side_circle_entry(
        point, measured_speed, actual_yawrate, config->sweep_brake_sec,
        config, &entry_time, &entry_radius))
    {
      decision.stop = true;
      ssf_set_side_collision(&decision, point_on_left, point, entry_time, entry_radius);
      return decision;
    }
  }

  const bool command_turns_left = manual_left && requested_left;
  const bool command_turns_right = manual_right && requested_right;
  if (!command_turns_left && !command_turns_right) return decision;

  const bool evaluate_left = command_turns_left;
  const float prediction_speed = ssf_max(measured_speed, ssf_max(0.0f, command->v));

  command_is_safe_ctx_t ctx;
  ctx.config = config;
  ctx.safety_points = safety_points;
  ctx.safety_points_count = safety_points_count;
  ctx.evaluate_left = evaluate_left;
  ctx.prediction_speed = prediction_speed;

  planner_side_hard_stop_decision_t full_command_collision =
    planner_side_hard_stop_decision_default();
  if (ssf_command_is_safe(&ctx, command->w, &full_command_collision)) return decision;

  const int kYawSearchSamples = 16;
  float limited_yawrate = 0.0f;
  bool safe_yawrate_found = false;
  planner_side_hard_stop_decision_t candidate_collision = full_command_collision;
  for (int sample = 1; sample <= kYawSearchSamples; ++sample) {
    const float scale = 1.0f -
      (float)sample / (float)kYawSearchSamples;
    const float candidate_yawrate = command->w * scale;
    planner_side_hard_stop_decision_t collision =
      planner_side_hard_stop_decision_default();
    if (ssf_command_is_safe(&ctx, candidate_yawrate, &collision)) {
      limited_yawrate = candidate_yawrate;
      safe_yawrate_found = true;
      break;
    }
    candidate_collision = collision;
  }

  if (!safe_yawrate_found &&
    candidate_collision.prediction_time_sec <= config->sweep_brake_sec)
  {
    candidate_collision.stop = true;
    return candidate_collision;
  }

  decision = full_command_collision;
  decision.yaw_limited = true;
  decision.limited_yawrate = limited_yawrate;
  return decision;
}
