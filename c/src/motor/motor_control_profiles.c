/*
 * 模块功能：motor_control_profiles.h 对应实现（motor_control_profiles.hpp 直译）。
 */
#include "motor_control_profiles.h"

#include <math.h>
#include <string.h>

#include "motor_protocol.h"

/* —— 判定函数 —— */

int mc_hall_control_should_follow_raw(
  int mode, int raw_recent, double raw_linear, double linear_deadband)
{
  if (!raw_recent || raw_linear < -linear_deadband) return 1;
  return !(mode == MP_HALL_MODE_MOVING_DWA ||
           mode == MP_HALL_MODE_STATIC_DYNAMIC_DWA);
}

int mc_controller_neutral_stop_required(
  double sample_time, double now, double sample_timeout,
  int controller_x, int controller_y, int dead_low, int dead_high)
{
  const double max_timeout = sample_timeout > 0.0 ? sample_timeout : 0.0;
  if (sample_time <= 0.0 || now - sample_time > max_timeout) return 0;
  return (dead_low <= controller_x && controller_x <= dead_high &&
          dead_low <= controller_y && controller_y <= dead_high);
}

int mc_linear_output_hard_stop_required(
  int x, int y, const char *state, int raw_forward_active)
{
  if (strcmp(state, "controller_neutral_stop") == 0 ||
      strcmp(state, "timeout_stop") == 0 ||
      strcmp(state, "hall_fault_stop") == 0 ||
      strcmp(state, "hall_control_timeout_stop") == 0 ||
      strcmp(state, "hall_feedback_timeout_stop") == 0 ||
      strcmp(state, "hall_moving_hold_emergency_stop") == 0) {
    return 1;
  }
  return x == MP_CENTER && y == MP_CENTER && raw_forward_active;
}

int mc_linear_output_uses_protective_rates(int mode, int control_fresh)
{
  return control_fresh &&
    (mode == MP_HALL_MODE_MOVING_PREBRAKE || mode == MP_HALL_MODE_MOVING_HOLD ||
     mode == MP_HALL_MODE_MOVING_DWA || mode == MP_HALL_MODE_STATIC_DYNAMIC_DWA ||
     mode == MP_HALL_MODE_SIDE_CAP);
}

/* —— HallSpeedController —— */

void mc_hall_speed_controller_init(
  mc_hall_speed_controller_t *c,
  double gain, double max_down_rate, double max_up_rate,
  double deadband, double minimum_y)
{
  c->gain = gain;
  c->max_down_rate = max_down_rate;
  c->max_up_rate = max_up_rate;
  c->deadband = deadband;
  c->minimum_y = minimum_y;
  mc_hall_speed_controller_reset(c);
}

void mc_hall_speed_controller_reset(mc_hall_speed_controller_t *c)
{
  c->output_y = (double)MP_CENTER;
  c->initialized = 0;
  c->last_time = 0.0;
  c->last_actual = 0.0;
  c->has_last_actual = 0;
  c->last_actual_time = 0.0;
  c->actual_rate = 0.0;
  c->has_last_info = 0;
}

void mc_hall_speed_controller_start(
  mc_hall_speed_controller_t *c, double initial_y, double now)
{
  c->output_y = mp_clamp_double(initial_y, (double)MP_CENTER, 200.0);
  c->initialized = 1;
  c->last_time = now;
}

int mc_hall_speed_controller_update(
  mc_hall_speed_controller_t *c,
  double target, double actual, double raw_cap_y, double initial_y,
  double now, double default_dt, double predict_sec)
{
  const double trend_dt = (c->last_actual_time <= 0.0)
    ? default_dt : mp_clamp_double(now - c->last_actual_time, 0.001, 0.25);
  if (c->has_last_actual) {
    const double measured_rate =
      mp_clamp_double((actual - c->last_actual) / trend_dt, -3.0, 3.0);
    c->actual_rate = 0.5 * measured_rate + 0.5 * c->actual_rate;
  }
  c->last_actual = actual;
  c->has_last_actual = 1;
  c->last_actual_time = now;
  {
    const double predict_pos = predict_sec > 0.0 ? predict_sec : 0.0;
    const double predicted_actual =
      (actual + c->actual_rate * predict_pos > 0.0)
      ? actual + c->actual_rate * predict_pos : 0.0;

    if (target <= 0.0 || raw_cap_y <= (double)MP_CENTER) {
      c->output_y = (double)MP_CENTER;
      c->initialized = 1;
      c->last_time = now;
      c->last_info.target = target;
      c->last_info.actual = actual;
      c->last_info.predicted = predicted_actual;
      c->last_info.rate = c->actual_rate;
      c->last_info.error = target - predicted_actual;
      c->last_info.y = (double)MP_CENTER;
      c->has_last_info = 1;
      return MP_CENTER;
    }

    if (!c->initialized) {
      const double start_y = initial_y < raw_cap_y ? initial_y : raw_cap_y;
      mc_hall_speed_controller_start(c, start_y, now);
    }

    {
      const double dt = (c->last_time <= 0.0)
        ? default_dt : mp_clamp_double(now - c->last_time, 0.001, 0.25);
      double delta;
      double maximum_y;
      double minimum_y;
      c->last_time = now;
      {
        const double error = target - predicted_actual;
        const double effective_error =
          (fabs(error) <= c->deadband) ? 0.0 : error;
        delta = c->gain * effective_error * dt;
        delta = mp_clamp_double(
          delta, -c->max_down_rate * dt, c->max_up_rate * dt);
      }
      c->output_y += delta;
      /* maximum_y = std::max(kCenter, std::min(200.0, raw_cap_y)) */
      maximum_y = (raw_cap_y < 200.0) ? raw_cap_y : 200.0;
      if (maximum_y < (double)MP_CENTER) maximum_y = (double)MP_CENTER;
      minimum_y = (c->minimum_y < maximum_y) ? c->minimum_y : maximum_y;
      c->output_y = mp_clamp_double(c->output_y, minimum_y, maximum_y);
      {
        const double error = target - predicted_actual;
        c->last_info.target = target;
        c->last_info.actual = actual;
        c->last_info.predicted = predicted_actual;
        c->last_info.rate = c->actual_rate;
        c->last_info.error = error;
        c->last_info.y = c->output_y;
      }
      c->has_last_info = 1;
      return mp_clamp_coord(c->output_y);
    }
  }
}

/* —— MovingHoldController —— */

void mc_moving_hold_controller_init(
  mc_moving_hold_controller_t *c,
  double gain, double max_down_rate, double max_up_rate,
  double deadband, double minimum_y, double predict_sec,
  double maximum_y, double emergency_speed, double release_margin)
{
  mc_hall_speed_controller_init(
    &c->speed_controller, gain, max_down_rate, max_up_rate, deadband, minimum_y);
  c->predict_sec = predict_sec;
  c->maximum_y = maximum_y;
  c->emergency_speed = emergency_speed;
  c->release_margin = release_margin;
  c->emergency_active = 0;
}

void mc_moving_hold_controller_reset(mc_moving_hold_controller_t *c)
{
  mc_hall_speed_controller_reset(&c->speed_controller);
  c->emergency_active = 0;
}

int mc_moving_hold_controller_seed(
  mc_moving_hold_controller_t *c, double initial_y, double now)
{
  const double seed_y = initial_y < c->maximum_y ? initial_y : c->maximum_y;
  mc_hall_speed_controller_start(&c->speed_controller, seed_y, now);
  c->emergency_active = 0;
  return mp_clamp_coord(seed_y);
}

mc_moving_hold_result_t mc_moving_hold_controller_update(
  mc_moving_hold_controller_t *c,
  double target, double actual, double raw_cap_y, double initial_y,
  double now, double default_dt)
{
  mc_moving_hold_result_t result;
  const double maximum_y = raw_cap_y < c->maximum_y ? raw_cap_y : c->maximum_y;
  const int controlled_y = mc_hall_speed_controller_update(
    &c->speed_controller, target, actual, maximum_y,
    (initial_y < maximum_y ? initial_y : maximum_y),
    now, default_dt, c->predict_sec);
  const double rate = c->speed_controller.has_last_info
    ? c->speed_controller.last_info.rate : 0.0;

  if (c->emergency_active) {
    if (actual <= target + c->release_margin && rate <= 0.0) {
      double restart_y = (double)controlled_y;
      c->emergency_active = 0;
      if (restart_y > maximum_y) restart_y = maximum_y;
      mc_hall_speed_controller_start(&c->speed_controller, restart_y, now);
    } else {
      result.emergency_stop = 1;
      result.controlled_y = MP_CENTER;
      return result;
    }
  }

  if (actual >= c->emergency_speed && rate > 0.0) {
    c->emergency_active = 1;
    result.emergency_stop = 1;
    result.controlled_y = MP_CENTER;
    return result;
  }
  result.emergency_stop = 0;
  result.controlled_y = controlled_y;
  return result;
}

/* —— HallPrebrakeStopController —— */

void mc_prebrake_stop_controller_init(
  mc_prebrake_stop_controller_t *c,
  double predict_sec, double deadzone_y, double release_margin)
{
  c->predict_sec = predict_sec;
  c->deadzone_y = deadzone_y;
  c->release_margin = release_margin;
  mc_prebrake_stop_controller_reset(c);
}

void mc_prebrake_stop_controller_reset(mc_prebrake_stop_controller_t *c)
{
  c->active = 0;
  c->released = 0;
  c->hold_y = (double)MP_CENTER;
  c->has_last_speed = 0;
  c->last_speed = 0.0;
  c->last_time = 0.0;
  c->deceleration_rate = 0.0;
  c->has_last_info = 0;
}

void mc_prebrake_stop_controller_start(
  mc_prebrake_stop_controller_t *c,
  double target, double actual, double raw_cap_y, double target_command_y,
  double now)
{
  double entry_speed = target > actual ? target : actual;
  if (entry_speed < 1e-6) entry_speed = 1e-6;
  {
    const double usable_y = raw_cap_y - c->deadzone_y > 0.0
      ? raw_cap_y - c->deadzone_y : 0.0;
    const double estimated_y = c->deadzone_y + usable_y * target / entry_speed;
    double upper_y = c->deadzone_y + 1.0;
    /* std::max(deadzone+1, std::min(raw_cap_y, target_command_y)) */
    double inner = raw_cap_y < target_command_y ? raw_cap_y : target_command_y;
    if (upper_y < inner) upper_y = inner;
    c->hold_y = mp_clamp_double(estimated_y, c->deadzone_y + 1.0, upper_y);
  }
  c->active = 1;
  c->released = 0;
  c->last_speed = actual;
  c->has_last_speed = 1;
  c->last_time = now;
  c->deceleration_rate = 0.0;
}

mc_prebrake_result_t mc_prebrake_stop_controller_update(
  mc_prebrake_stop_controller_t *c,
  double target, double actual, double raw_cap_y, double target_command_y,
  double now, double default_dt)
{
  mc_prebrake_result_t result;
  if (!c->active) {
    mc_prebrake_stop_controller_start(
      c, target, actual, raw_cap_y, target_command_y, now);
  }

  {
    const double dt = (c->last_time <= 0.0)
      ? default_dt : mp_clamp_double(now - c->last_time, 0.001, 0.25);
    if (c->has_last_speed) {
      const double raw_deceleration =
        mp_clamp_double((c->last_speed - actual) / dt, 0.0, 3.0);
      c->deceleration_rate = 0.5 * raw_deceleration + 0.5 * c->deceleration_rate;
    }
    c->last_speed = actual;
    c->has_last_speed = 1;
    c->last_time = now;
    {
      const double decel_pos =
        c->deceleration_rate * c->predict_sec > 0.0
        ? c->deceleration_rate * c->predict_sec : 0.0;
      const double predicted_speed = actual - decel_pos > 0.0
        ? actual - decel_pos : 0.0;
      const double release_speed = target + c->release_margin;
      c->released = (actual <= release_speed);

      /* hold_y_ = std::min(std::min(hold_y_, raw_cap_y), target_command_y) */
      {
        double h = c->hold_y;
        if (h > raw_cap_y) h = raw_cap_y;
        if (h > target_command_y) h = target_command_y;
        c->hold_y = h;
      }
      c->last_info.target = target;
      c->last_info.release_speed = release_speed;
      c->last_info.actual = actual;
      c->last_info.predicted = predicted_speed;
      c->last_info.deceleration = c->deceleration_rate;
      c->last_info.hold_y = c->hold_y;
      c->last_info.released = c->released;
      c->has_last_info = 1;
      result.released = c->released;
      result.hold_y = mp_clamp_coord(c->hold_y);
      return result;
    }
  }
}

/* —— LinearOutputProfile —— */

void mc_linear_output_profile_init(
  mc_linear_output_profile_t *c, double minimum_y)
{
  c->minimum_y = mp_clamp_double(minimum_y, (double)(MP_CENTER + 1), 200.0);
  mc_linear_output_profile_reset(c);
}

void mc_linear_output_profile_reset(mc_linear_output_profile_t *c)
{
  c->output_y = (double)MP_CENTER;
  c->initialized = 0;
  c->last_time = 0.0;
  c->has_last_info = 0;
}

void mc_linear_output_profile_start(
  mc_linear_output_profile_t *c, double initial_y, double now)
{
  double y = mp_clamp_double(initial_y, (double)MP_CENTER, 200.0);
  if (y > (double)MP_CENTER) y = (y > c->minimum_y) ? y : c->minimum_y;
  c->output_y = y;
  c->initialized = (y > (double)MP_CENTER);
  c->last_time = now;
  c->has_last_info = 0;
}

int mc_linear_output_profile_hard_stop(
  mc_linear_output_profile_t *c, double now)
{
  const double previous_y = c->output_y;
  c->output_y = (double)MP_CENTER;
  c->initialized = 0;
  c->last_time = now;
  c->last_info.target = (double)MP_CENTER;
  c->last_info.previous = previous_y;
  c->last_info.output = (double)MP_CENTER;
  c->last_info.delta = (double)MP_CENTER - previous_y;
  c->last_info.dt = 0.0;
  c->last_info.rate_limit = 0.0;
  c->last_info.limited = 0;
  c->last_info.hard_stop = 1;
  c->has_last_info = 1;
  return MP_CENTER;
}

int mc_linear_output_profile_update(
  mc_linear_output_profile_t *c,
  double target_y, double now, double default_dt,
  double up_rate, double down_rate)
{
  double target = mp_clamp_double(target_y, (double)MP_CENTER, 200.0);
  if (target <= (double)MP_CENTER) {
    return mc_linear_output_profile_hard_stop(c, now);
  }
  target = (target > c->minimum_y) ? target : c->minimum_y;

  if (!c->initialized) {
    const double start_y = target < c->minimum_y ? target : c->minimum_y;
    mc_linear_output_profile_start(c, start_y, now);
    c->last_info.target = target;
    c->last_info.previous = (double)MP_CENTER;
    c->last_info.output = c->output_y;
    c->last_info.delta = c->output_y - (double)MP_CENTER;
    c->last_info.dt = 0.0;
    c->last_info.rate_limit = up_rate;
    c->last_info.limited = (c->output_y < target);
    c->last_info.hard_stop = 0;
    c->has_last_info = 1;
    return mp_clamp_coord(c->output_y);
  }

  {
    const double dt = (c->last_time <= 0.0)
      ? default_dt : mp_clamp_double(now - c->last_time, 0.001, 0.25);
    const double previous_y = c->output_y;
    const double delta =
      mp_clamp_double(target - previous_y, -down_rate * dt, up_rate * dt);
    c->last_time = now;
    c->output_y = mp_clamp_double(previous_y + delta, c->minimum_y, 200.0);
    c->last_info.target = target;
    c->last_info.previous = previous_y;
    c->last_info.output = c->output_y;
    c->last_info.delta = c->output_y - previous_y;
    c->last_info.dt = dt;
    c->last_info.rate_limit = (target >= previous_y) ? up_rate : down_rate;
    c->last_info.limited = (fabs(c->output_y - target) > 1e-9);
    c->last_info.hard_stop = 0;
    c->has_last_info = 1;
    return mp_clamp_coord(c->output_y);
  }
}
