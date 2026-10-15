/*
 * 模块功能：侧墙保持控制器实现。
 * 直译自 native/side_wall_hold_control.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 */
#include <string.h>

#include "side_wall_hold_control.h"

/* std::min/std::max 双参直译（NaN 语义与 libstdc++ 一致：返回未定参数之一）。 */
static inline float swc_min(float a, float b) { return a < b ? a : b; }
static inline float swc_max(float a, float b) { return a > b ? a : b; }

dwa_side_wall_hold_config_t dwa_side_wall_hold_config_default(void)
{
  dwa_side_wall_hold_config_t config;
  config.enabled = true;
  config.hold_distance = 0.20f;
  config.correction_distance = 0.20f;
  config.prediction_sec = 0.60f;
  config.invalid_clearance_hold_sec = 0.25f;
  config.closing_rate_filter_alpha = 0.35f;
  config.closing_rate_max = 1.00f;
  config.release_distance = 0.22f;
  config.proportional_gain = 0.80f;
  config.min_yawrate = 0.03f;
  config.max_yawrate = 0.12f;
  config.yawrate_acceleration = 0.25f;
  config.release_clear_sec = 0.30f;
  return config;
}

bool dwa_validate_side_wall_hold_config(
  const dwa_side_wall_hold_config_t *config,
  const char **reason)
{
  const char *message = NULL;
  if (!isfinite(config->hold_distance) || config->hold_distance <= 0.0f) {
    message = "side wall hold distance must be positive";
  } else if (!isfinite(config->correction_distance) ||
    config->correction_distance <= 0.0f ||
    config->correction_distance > config->hold_distance)
  {
    message = "side wall hold correction distance must be within the hold distance";
  } else if (!isfinite(config->prediction_sec) || config->prediction_sec < 0.0f) {
    message = "side wall hold prediction time must be non-negative";
  } else if (!isfinite(config->invalid_clearance_hold_sec) ||
    config->invalid_clearance_hold_sec < 0.0f)
  {
    message = "side wall hold invalid-clearance hold time must be non-negative";
  } else if (!isfinite(config->closing_rate_filter_alpha) ||
    config->closing_rate_filter_alpha < 0.0f ||
    config->closing_rate_filter_alpha > 1.0f ||
    !isfinite(config->closing_rate_max) || config->closing_rate_max <= 0.0f)
  {
    message = "side wall hold closing-rate parameters are invalid";
  } else if (!isfinite(config->release_distance) ||
    config->release_distance <= config->hold_distance)
  {
    message = "side wall hold release distance must exceed the hold distance";
  } else if (!isfinite(config->proportional_gain) ||
    config->proportional_gain <= 0.0f)
  {
    message = "side wall hold proportional gain must be positive";
  } else if (!isfinite(config->min_yawrate) || config->min_yawrate < 0.0f ||
    !isfinite(config->max_yawrate) || config->max_yawrate <= 0.0f ||
    config->min_yawrate > config->max_yawrate)
  {
    message = "side wall hold yawrate limits are invalid";
  } else if (!isfinite(config->yawrate_acceleration) ||
    config->yawrate_acceleration <= 0.0f)
  {
    message = "side wall hold yawrate acceleration must be positive";
  } else if (!isfinite(config->release_clear_sec) || config->release_clear_sec < 0.0f) {
    message = "side wall hold release time must be non-negative";
  }
  if (message != NULL) {
    if (reason != NULL) *reason = message;
    return false;
  }
  return true;
}

const char *dwa_side_wall_hold_side_name(dwa_side_wall_hold_side_t side)
{
  switch (side) {
    case DWA_SIDE_HOLD_LEFT: return "LEFT";
    case DWA_SIDE_HOLD_RIGHT: return "RIGHT";
    case DWA_SIDE_HOLD_BOTH: return "BOTH";
    case DWA_SIDE_HOLD_NONE: return "NONE";
  }
  return "NONE";
}

float dwa_apply_side_wall_hold_yawrate(
  const dwa_side_wall_hold_result_t *result,
  float requested_yawrate)
{
  if (!result->active) return requested_yawrate;
  /* Stop turning only when both sides are actually in the danger zone. */
  if (result->left_active && result->right_active) return 0.0f;

  float guarded_yawrate = requested_yawrate;
  if (result->left_active) guarded_yawrate = swc_min(guarded_yawrate, 0.0f);
  if (result->right_active) guarded_yawrate = swc_max(guarded_yawrate, 0.0f);

  if (result->correction_yawrate < 0.0f) {
    return result->right_active ? result->correction_yawrate :
           swc_min(guarded_yawrate, result->correction_yawrate);
  }
  if (result->correction_yawrate > 0.0f) {
    return result->left_active ? result->correction_yawrate :
           swc_max(guarded_yawrate, result->correction_yawrate);
  }
  return guarded_yawrate;
}

/* move_toward 直译。 */
static float swc_move_toward(float current, float target, float max_step)
{
  if (max_step <= 0.0f || current == target) return current;
  const float delta = target - current;
  if (fabsf(delta) <= max_step) return target;
  return current + (delta < 0.0f ? -max_step : max_step);
}

void dwa_side_wall_hold_controller_init(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_config_t *config)
{
  controller->config = *config;
  controller->left_active = false;
  controller->right_active = false;
  controller->left_clear_since_sec = -1.0f;
  controller->right_clear_since_sec = -1.0f;
  controller->correction_yawrate = 0.0f;
  controller->left_clearance_state = dwa_side_clearance_state_default();
  controller->right_clearance_state = dwa_side_clearance_state_default();
}

void dwa_side_wall_hold_controller_init_default(
  dwa_side_wall_hold_controller_t *controller)
{
  const dwa_side_wall_hold_config_t config = dwa_side_wall_hold_config_default();
  dwa_side_wall_hold_controller_init(controller, &config);
}

bool dwa_side_wall_hold_controller_set_config(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_config_t *config,
  const char **reason)
{
  if (!dwa_validate_side_wall_hold_config(config, reason)) return false;
  controller->config = *config;
  dwa_side_wall_hold_controller_reset(controller);
  return true;
}

void dwa_side_wall_hold_controller_reset(dwa_side_wall_hold_controller_t *controller)
{
  controller->left_active = false;
  controller->right_active = false;
  controller->left_clear_since_sec = -1.0f;
  controller->right_clear_since_sec = -1.0f;
  controller->correction_yawrate = 0.0f;
  controller->left_clearance_state = dwa_side_clearance_state_default();
  controller->right_clearance_state = dwa_side_clearance_state_default();
}

/* update_clearance_estimate(clearance, now_sec, &state) 直译。 */
static dwa_side_clearance_estimate_t swc_update_clearance_estimate(
  const dwa_side_wall_hold_controller_t *controller,
  float clearance,
  float now_sec,
  dwa_side_clearance_state_t *state)
{
  dwa_side_clearance_estimate_t estimate = dwa_side_clearance_estimate_default();
  if (isfinite(clearance)) {
    if (state->initialized) {
      const float dt = now_sec - state->last_update_sec;
      if (dt > 1e-3f && dt <= 0.50f) {
        const float raw_rate = swc_max(
          -controller->config.closing_rate_max,
          swc_min(
            controller->config.closing_rate_max,
            (state->last_clearance - clearance) / dt));
        const float alpha = controller->config.closing_rate_filter_alpha;
        state->closing_rate =
          alpha * raw_rate + (1.0f - alpha) * state->closing_rate;
      } else {
        state->closing_rate = 0.0f;
      }
    } else {
      state->closing_rate = 0.0f;
      state->initialized = true;
    }
    state->last_clearance = clearance;
    state->last_update_sec = now_sec;
    estimate.clearance = clearance;
  } else if (
    state->initialized && now_sec >= state->last_update_sec &&
    now_sec - state->last_update_sec <=
      controller->config.invalid_clearance_hold_sec)
  {
    estimate.clearance = state->last_clearance;
    estimate.held = true;
  } else {
    *state = dwa_side_clearance_state_default();
    return estimate;
  }

  estimate.closing_rate = swc_max(0.0f, state->closing_rate);
  estimate.predicted_clearance = swc_max(
    0.0f,
    estimate.clearance - estimate.closing_rate * controller->config.prediction_sec);
  return estimate;
}

/* update_latch(activation, release, now, &active, &clear_since) 直译。 */
static void swc_update_latch(
  const dwa_side_wall_hold_controller_t *controller,
  float activation_clearance,
  float release_clearance,
  float now_sec,
  bool *active,
  float *clear_since_sec)
{
  if (isfinite(activation_clearance) &&
    activation_clearance <= controller->config.hold_distance)
  {
    *active = true;
    *clear_since_sec = -1.0f;
    return;
  }
  if (!*active) return;

  const bool release_clear = !isfinite(release_clearance) ||
    release_clearance >= controller->config.release_distance;
  if (!release_clear) {
    *clear_since_sec = -1.0f;
    return;
  }
  if (*clear_since_sec < 0.0f) {
    *clear_since_sec = now_sec;
    return;
  }
  if (now_sec - *clear_since_sec + 1e-9f < controller->config.release_clear_sec) return;

  *active = false;
  *clear_since_sec = -1.0f;
}

dwa_side_wall_hold_result_t dwa_side_wall_hold_controller_update(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_input_t *input)
{
  const dwa_side_wall_hold_config_t *config = &controller->config;
  dwa_side_wall_hold_result_t result = dwa_side_wall_hold_result_default();
  result.output_yawrate = input->requested_yawrate;
  if (!config->enabled || !input->forward || !input->perception_fresh) {
    dwa_side_wall_hold_controller_reset(controller);
    return result;
  }

  const dwa_side_clearance_estimate_t left = swc_update_clearance_estimate(
    controller, input->left_clearance, input->now_sec,
    &controller->left_clearance_state);
  const dwa_side_clearance_estimate_t right = swc_update_clearance_estimate(
    controller, input->right_clearance, input->now_sec,
    &controller->right_clearance_state);
  result.left_clearance = left.clearance;
  result.right_clearance = right.clearance;
  result.left_predicted_clearance = left.predicted_clearance;
  result.right_predicted_clearance = right.predicted_clearance;
  result.left_closing_rate = left.closing_rate;
  result.right_closing_rate = right.closing_rate;
  result.left_clearance_held = left.held;
  result.right_clearance_held = right.held;

  /* Latch with predicted clearance and release with realtime clearance. */
  swc_update_latch(
    controller, left.predicted_clearance, left.clearance, input->now_sec,
    &controller->left_active, &controller->left_clear_since_sec);
  swc_update_latch(
    controller, right.predicted_clearance, right.clearance, input->now_sec,
    &controller->right_active, &controller->right_clear_since_sec);

  result.left_latched = controller->left_active;
  result.right_latched = controller->right_active;
  bool effective_left_active = controller->left_active;
  bool effective_right_active = controller->right_active;
  if (effective_left_active && effective_right_active) {
    const bool left_actual_danger = isfinite(left.clearance) &&
      left.clearance <= config->hold_distance;
    const bool right_actual_danger = isfinite(right.clearance) &&
      right.clearance <= config->hold_distance;

    /* Keep the safer turn direction unless both sides are physically dangerous. */
    if (!(left_actual_danger && right_actual_danger)) {
      if (left_actual_danger != right_actual_danger) {
        effective_left_active = left_actual_danger;
        effective_right_active = right_actual_danger;
      } else {
        const float left_risk = isfinite(left.predicted_clearance) ?
          left.predicted_clearance : left.clearance;
        const float right_risk = isfinite(right.predicted_clearance) ?
          right.predicted_clearance : right.clearance;
        if (left_risk <= right_risk) {
          effective_right_active = false;
        } else {
          effective_left_active = false;
        }
      }
    }
  }

  result.left_active = effective_left_active;
  result.right_active = effective_right_active;
  result.active = effective_left_active || effective_right_active;
  if (!result.active) {
    controller->correction_yawrate = 0.0f;
    return result;
  }

  const bool both_sides_danger = effective_left_active && effective_right_active;
  const bool left_needs_correction =
    !both_sides_danger && effective_left_active &&
    isfinite(left.predicted_clearance) &&
    left.predicted_clearance < config->correction_distance &&
    (!isfinite(right.clearance) ||
    right.clearance > config->correction_distance);
  const bool right_needs_correction =
    !both_sides_danger && effective_right_active &&
    isfinite(right.predicted_clearance) &&
    right.predicted_clearance < config->correction_distance &&
    (!isfinite(left.clearance) ||
    left.clearance > config->correction_distance);
  const bool correcting_left = left_needs_correction && !right_needs_correction;
  const bool correcting_right = right_needs_correction && !left_needs_correction;
  const float clearance = correcting_left ? left.predicted_clearance :
    (correcting_right ? right.predicted_clearance : config->correction_distance);
  const float error = isfinite(clearance) ?
    swc_max(0.0f, config->correction_distance - clearance) : 0.0f;
  float correction_magnitude = 0.0f;
  if (error > 0.0f) {
    correction_magnitude = swc_min(
      config->max_yawrate,
      swc_max(config->min_yawrate, config->proportional_gain * error));
  }
  const float desired_correction = correcting_left ? -correction_magnitude :
    (correcting_right ? correction_magnitude : 0.0f);

  /* Leave active correction once the side is outside the emergency band. */
  if (!correcting_left && !correcting_right) {
    controller->correction_yawrate = 0.0f;
  } else {
    /* Clear the old correction before switching sides. */
    if (controller->correction_yawrate * desired_correction < 0.0f) {
      controller->correction_yawrate = 0.0f;
    }
    const float dt = swc_max(0.0f, swc_min(0.25f, input->dt_sec));
    const float max_step = config->yawrate_acceleration * dt;
    controller->correction_yawrate = swc_move_toward(
      controller->correction_yawrate, desired_correction, max_step);
  }

  if (both_sides_danger) {
    result.side = DWA_SIDE_HOLD_BOTH;
  } else if (correcting_left) {
    result.side = DWA_SIDE_HOLD_LEFT;
  } else if (correcting_right) {
    result.side = DWA_SIDE_HOLD_RIGHT;
  } else {
    result.side = effective_left_active ?
      DWA_SIDE_HOLD_LEFT : DWA_SIDE_HOLD_RIGHT;
  }
  result.correction_yawrate = controller->correction_yawrate;

  result.output_yawrate = dwa_apply_side_wall_hold_yawrate(
    &result, input->requested_yawrate);
  return result;
}
