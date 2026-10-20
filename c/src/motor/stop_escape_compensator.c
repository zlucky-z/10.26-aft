/*
 * 模块功能：stop_escape_compensator.h 对应实现
 * （stop_escape_compensator.hpp 直译）。
 */
#include "stop_escape_compensator.h"

#include <math.h>

/* tie_margin/forward_min_x/forward_max_x/max_lateral 使用 C++ 构造默认值
 * （bridge 侧只传 5 个参数，其余走默认实参 0.03 / -0.20 / 1.20 / 1.20）。 */
#define SE_DEFAULT_TIE_MARGIN 0.03
#define SE_DEFAULT_FORWARD_MIN_X (-0.20)
#define SE_DEFAULT_FORWARD_MAX_X 1.20
#define SE_DEFAULT_MAX_LATERAL 1.20
#define SE_ESCAPE_SIDE_MIN_CLEARANCE 0.03
#define SE_DEFAULT_MAX_DURATION_SEC 2.0
#define SE_DWA_HANDOFF_COMMAND_HOLD_SEC 0.30
#define SE_DWA_HANDOFF_MOTION_HOLD_SEC 0.25
#define SE_DWA_HANDOFF_TIMEOUT_SEC 0.60
#define SE_FRONT_BEARING_DEADBAND 0.05
/* Keep forward-facing near-corner returns out of lateral gap measurement.
 * abs(y) >= 0.5*abs(x) corresponds to the lateral-facing sectors beyond
 * about 27 degrees from the front/rear axis. */
#define SE_SIDE_SECTOR_LATERAL_RATIO 0.50

void se_stop_escape_compensator_init(
  se_stop_escape_compensator_t *c,
  double stop_hold_sec, double stop_speed, double yawrate,
  double cloud_timeout, double half_width)
{
  c->stop_hold_sec = stop_hold_sec > 0.0 ? stop_hold_sec : 0.0;
  c->stop_speed = stop_speed > 0.0 ? stop_speed : 0.0;
  c->yawrate = fabs(yawrate);
  c->cloud_timeout = cloud_timeout > 0.05 ? cloud_timeout : 0.05;
  c->half_width = half_width > 0.0 ? half_width : 0.0;
  c->max_duration_sec = SE_DEFAULT_MAX_DURATION_SEC;
  c->tie_margin = SE_DEFAULT_TIE_MARGIN > 0.0 ? SE_DEFAULT_TIE_MARGIN : 0.0;
  c->forward_min_x = SE_DEFAULT_FORWARD_MIN_X;
  c->forward_max_x = SE_DEFAULT_FORWARD_MAX_X;
  {
    double max_lat = SE_DEFAULT_MAX_LATERAL;
    if (c->half_width > max_lat) max_lat = c->half_width;
    c->max_lateral = max_lat;
  }
  c->cloud_time = 0.0;
  c->left_clearance = INFINITY;
  c->right_clearance = INFINITY;
  c->front_obstacle_away_direction = SE_DIRECTION_NONE;
  c->stop_since = -1.0;
  c->started_at = -1.0;
  c->recovery_since = -1.0;
  c->forward_command_since = -1.0;
  c->handoff_since = -1.0;
  c->forward_motion_since = -1.0;
  c->handoff_active = 0;
  c->inhibited_until_rearm = 0;
  c->direction = SE_DIRECTION_NONE;
  c->last_direction = SE_DIRECTION_NONE;
  c->active = 0;
}

void se_stop_escape_compensator_reset(se_stop_escape_compensator_t *c)
{
  c->stop_since = -1.0;
  c->started_at = -1.0;
  c->recovery_since = -1.0;
  c->forward_command_since = -1.0;
  c->handoff_since = -1.0;
  c->forward_motion_since = -1.0;
  c->handoff_active = 0;
  c->inhibited_until_rearm = 0;
  c->direction = SE_DIRECTION_NONE;
  c->active = 0;
}

void se_stop_escape_compensator_update_cloud(
  se_stop_escape_compensator_t *c, const se_point_t *points, size_t count,
  double now)
{
  double left = INFINITY;
  double right = INFINITY;
  double nearest_front_left_x = INFINITY;
  double nearest_front_right_x = INFINITY;
  size_t i;
  for (i = 0; i < count; ++i) {
    const double x = points[i].x;
    const double y = points[i].y;
    if (!(isfinite(x) && isfinite(y))) continue;
    if (x < c->forward_min_x || x > c->forward_max_x) continue;
    /* For tied side clearances, compare the nearest frontal returns on each
     * half of the footprint. A >5 cm range skew identifies the nearer side. */
    if (x >= 0.0 && fabs(y) <= c->half_width) {
      if (y > SE_FRONT_BEARING_DEADBAND && x < nearest_front_left_x) {
        nearest_front_left_x = x;
      } else if (y < -SE_FRONT_BEARING_DEADBAND &&
        x < nearest_front_right_x)
      {
        nearest_front_right_x = x;
      }
    }
    if (fabs(y) > c->max_lateral || fabs(y) <= c->half_width) continue;
    /* Side clearance is for lateral sectors, not a frontal wall's near-corner
     * returns. Those points move out of this sector when backing up, which
     * otherwise makes a wide-open side appear blocked until the chair reverses. */
    if (fabs(y) < SE_SIDE_SECTOR_LATERAL_RATIO * fabs(x)) continue;
    {
      const double clearance = fabs(y) - c->half_width;
      if (y >= 0.0) left = left < clearance ? left : clearance;
      else right = right < clearance ? right : clearance;
    }
  }
  c->left_clearance = left;
  c->right_clearance = right;
  c->front_obstacle_away_direction = SE_DIRECTION_NONE;
  if (isfinite(nearest_front_left_x) &&
    (!isfinite(nearest_front_right_x) ||
    nearest_front_left_x + SE_FRONT_BEARING_DEADBAND <
      nearest_front_right_x))
  {
    c->front_obstacle_away_direction = SE_DIRECTION_RIGHT;
  } else if (isfinite(nearest_front_right_x) &&
    (!isfinite(nearest_front_left_x) ||
    nearest_front_right_x + SE_FRONT_BEARING_DEADBAND <
      nearest_front_left_x))
  {
    c->front_obstacle_away_direction = SE_DIRECTION_LEFT;
  }
  c->cloud_time = now;
}

se_direction_t se_stop_escape_compensator_choose_direction(
  const se_stop_escape_compensator_t *c)
{
  const double left = c->left_clearance;
  const double right = c->right_clearance;
  const int left_finite = isfinite(left);
  const int right_finite = isfinite(right);
  const int left_safe = left_finite && left >= SE_ESCAPE_SIDE_MIN_CLEARANCE;
  const int right_safe = right_finite && right >= SE_ESCAPE_SIDE_MIN_CLEARANCE;

  if (!left_safe && !right_safe) return SE_DIRECTION_NONE;
  if (!left_safe) return SE_DIRECTION_RIGHT;
  if (!right_safe) return SE_DIRECTION_LEFT;
  if (fabs(left - right) > c->tie_margin) {
    return (left > right) ? SE_DIRECTION_LEFT : SE_DIRECTION_RIGHT;
  }
  if (c->front_obstacle_away_direction != SE_DIRECTION_NONE) {
    return c->front_obstacle_away_direction;
  }
  return SE_DIRECTION_NONE;
}

static void se_stop_escape_inhibit(se_stop_escape_compensator_t *c)
{
  c->stop_since = -1.0;
  c->started_at = -1.0;
  c->recovery_since = -1.0;
  c->forward_command_since = -1.0;
  c->handoff_since = -1.0;
  c->forward_motion_since = -1.0;
  c->handoff_active = 0;
  c->active = 0;
  c->direction = SE_DIRECTION_NONE;
  c->inhibited_until_rearm = 1;
}

se_stop_escape_decision_t se_stop_escape_compensator_update_with_turn(
  se_stop_escape_compensator_t *c,
  double now,
  int controller_neutral,
  int forward_intent,
  int vehicle_stopped,
  int normal_output_stopped,
  int positive_dwa_command,
  int positive_forward_motion,
  se_direction_t manual_turn_direction)
{
  se_stop_escape_decision_t d;
  const int cloud_fresh = c->cloud_time > 0.0 &&
    now >= c->cloud_time && now - c->cloud_time <= c->cloud_timeout;

  d.active = 0;
  d.command_enabled = 0;
  d.yield_to_dwa = 0;
  d.yawrate = 0.0;
  d.direction = SE_DIRECTION_NONE;
  d.left_clearance = c->left_clearance;
  d.right_clearance = c->right_clearance;
  d.reason = "";

  /* Explicit user release always cancels immediately. */
  if (controller_neutral || !forward_intent) {
    se_stop_escape_compensator_reset(c);
    d.reason = "conditions_not_met";
    return d;
  }

  if (manual_turn_direction != SE_DIRECTION_NONE) {
    if (c->active || (vehicle_stopped && normal_output_stopped)) {
      se_stop_escape_inhibit(c);
    }
    d.direction = manual_turn_direction;
    d.reason = "manual_turn_takeover";
    return d;
  }

  if (c->inhibited_until_rearm) {
    if (!vehicle_stopped || !normal_output_stopped) {
      c->inhibited_until_rearm = 0;
    } else {
      d.reason = "escape_locked_out";
      return d;
    }
  }

  if (c->active) {
    if (c->max_duration_sec > 0.0 && c->started_at >= 0.0 &&
        now - c->started_at >= c->max_duration_sec) {
      se_stop_escape_inhibit(c);
      d.reason = "duration_limit";
      return d;
    }
    /* A brief sensing dropout pauses output. The total escape timer continues
     * to run, so stale perception cannot preserve a maneuver indefinitely. */
    if (!cloud_fresh) {
      c->recovery_since = -1.0;
      d.active = 1;
      d.direction = c->direction;
      d.reason = "paused_cloud_stale";
      return d;
    }
    /* Never keep or reverse a latched escape after its selected side becomes
     * unsafe or unobservable. Stop and require the trigger to clear first. */
    {
      const double latched = c->direction == SE_DIRECTION_LEFT ?
        c->left_clearance : c->right_clearance;
      if (!isfinite(latched) || latched < SE_ESCAPE_SIDE_MIN_CLEARANCE) {
        se_stop_escape_inhibit(c);
        d.reason = "selected_side_blocked";
        return d;
      }
    }

    /* Handoff only after a fresh positive DWA command remains available.
     * During the trial, let the normal DWA output pass through. End the escape
     * only after measured forward wheel motion is sustained; otherwise resume
     * the same latched turn without creating a new event. */
    if (c->handoff_active) {
      if (!positive_dwa_command) {
        c->handoff_active = 0;
        c->handoff_since = -1.0;
        c->forward_motion_since = -1.0;
        c->forward_command_since = -1.0;
      } else {
        if (positive_forward_motion) {
          if (c->forward_motion_since < 0.0) c->forward_motion_since = now;
          if (now - c->forward_motion_since >=
              SE_DWA_HANDOFF_MOTION_HOLD_SEC) {
            se_stop_escape_compensator_reset(c);
            d.reason = "recovered";
            return d;
          }
        } else {
          c->forward_motion_since = -1.0;
        }
        if (now - c->handoff_since < SE_DWA_HANDOFF_TIMEOUT_SEC ||
            positive_forward_motion) {
          d.active = 1;
          d.yield_to_dwa = 1;
          d.direction = c->direction;
          d.reason = "handoff_to_dwa";
          return d;
        }
        c->handoff_active = 0;
        c->handoff_since = -1.0;
        c->forward_motion_since = -1.0;
        c->forward_command_since = -1.0;
      }
    } else if (positive_dwa_command) {
      if (c->forward_command_since < 0.0) c->forward_command_since = now;
      if (now - c->forward_command_since >=
          SE_DWA_HANDOFF_COMMAND_HOLD_SEC) {
        c->handoff_active = 1;
        c->handoff_since = now;
        c->forward_motion_since = -1.0;
        d.active = 1;
        d.yield_to_dwa = 1;
        d.direction = c->direction;
        d.reason = "handoff_to_dwa";
        return d;
      }
    } else {
      c->forward_command_since = -1.0;
    }

    c->last_direction = c->direction;
    d.active = 1;
    d.command_enabled = 1;
    d.yawrate = c->direction == SE_DIRECTION_LEFT ? c->yawrate : -c->yawrate;
    d.direction = c->direction;
    d.left_clearance = c->left_clearance;
    d.right_clearance = c->right_clearance;
    d.reason = "escaping";
    return d;
  }

  /* Initial activation requires fresh sensing and a confirmed stopped state,
   * so the fallback cannot take over ordinary movement. */
  if (!cloud_fresh) {
    d.reason = "conditions_not_met";
    return d;
  }
  if (!vehicle_stopped || !normal_output_stopped) {
    c->stop_since = -1.0;
    c->recovery_since = -1.0;
    d.reason = "conditions_not_met";
    return d;
  }

  if (c->stop_since < 0.0) c->stop_since = now;
  if (now - c->stop_since < c->stop_hold_sec) {
    d.reason = "waiting_stop_hold";
    return d;
  }

  c->direction = se_stop_escape_compensator_choose_direction(c);
  if (c->direction == SE_DIRECTION_NONE || c->yawrate <= 0.0) {
    if (vehicle_stopped && normal_output_stopped) {
      c->inhibited_until_rearm = 1;
    }
    d.reason = "no_safe_side";
    return d;
  }
  c->active = 1;
  c->started_at = now;
  c->recovery_since = -1.0;
  c->last_direction = c->direction;
  d.active = 1;
  d.command_enabled = 1;
  d.yawrate = c->direction == SE_DIRECTION_LEFT ? c->yawrate : -c->yawrate;
  d.direction = c->direction;
  d.left_clearance = c->left_clearance;
  d.right_clearance = c->right_clearance;
  d.reason = "escaping";
  return d;
}

se_stop_escape_decision_t se_stop_escape_compensator_update(
  se_stop_escape_compensator_t *c,
  double now,
  int controller_neutral,
  int forward_intent,
  int vehicle_stopped,
  int normal_output_stopped,
  int positive_dwa_command,
  int positive_forward_motion)
{
  return se_stop_escape_compensator_update_with_turn(c, now,
    controller_neutral, forward_intent, vehicle_stopped,
    normal_output_stopped, positive_dwa_command, positive_forward_motion,
    SE_DIRECTION_NONE);
}

double se_stop_escape_compensator_left_clearance(
  const se_stop_escape_compensator_t *c)
{
  return c->left_clearance;
}

double se_stop_escape_compensator_right_clearance(
  const se_stop_escape_compensator_t *c)
{
  return c->right_clearance;
}
