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
  c->stop_since = -1.0;
  c->direction = SE_DIRECTION_NONE;
  c->active = 0;
}

void se_stop_escape_compensator_reset(se_stop_escape_compensator_t *c)
{
  c->stop_since = -1.0;
  c->direction = SE_DIRECTION_NONE;
  c->active = 0;
}

void se_stop_escape_compensator_update_cloud(
  se_stop_escape_compensator_t *c, const se_point_t *points, size_t count,
  double now)
{
  double left = INFINITY;
  double right = INFINITY;
  size_t i;
  for (i = 0; i < count; ++i) {
    const double x = points[i].x;
    const double y = points[i].y;
    if (!(isfinite(x) && isfinite(y))) continue;
    if (x < c->forward_min_x || x > c->forward_max_x) continue;
    if (fabs(y) > c->max_lateral || fabs(y) <= c->half_width) continue;
    {
      const double clearance = fabs(y) - c->half_width;
      if (y >= 0.0) left = left < clearance ? left : clearance;
      else right = right < clearance ? right : clearance;
    }
  }
  c->left_clearance = left;
  c->right_clearance = right;
  c->cloud_time = now;
}

se_direction_t se_stop_escape_compensator_choose_direction(
  const se_stop_escape_compensator_t *c)
{
  const double left = c->left_clearance;
  const double right = c->right_clearance;
  const int left_finite = isfinite(left);
  const int right_finite = isfinite(right);
  if (!left_finite && !right_finite) return SE_DIRECTION_NONE;
  if (!left_finite) return SE_DIRECTION_LEFT;
  if (!right_finite) return SE_DIRECTION_RIGHT;
  if (fabs(left - right) <= c->tie_margin) return SE_DIRECTION_NONE;
  return (left > right) ? SE_DIRECTION_LEFT : SE_DIRECTION_RIGHT;
}

se_stop_escape_decision_t se_stop_escape_compensator_update(
  se_stop_escape_compensator_t *c,
  double now,
  int controller_neutral,
  int forward_intent,
  int vehicle_stopped,
  int normal_output_stopped)
{
  se_stop_escape_decision_t d;
  const int cloud_fresh =
    (c->cloud_time > 0.0 && now - c->cloud_time <= c->cloud_timeout);

  d.active = 0;
  d.yawrate = 0.0;
  d.direction = SE_DIRECTION_NONE;
  d.left_clearance = c->left_clearance;
  d.right_clearance = c->right_clearance;
  d.reason = "";

  if (controller_neutral || !forward_intent || !vehicle_stopped ||
      !normal_output_stopped || !cloud_fresh) {
    se_stop_escape_compensator_reset(c);
    d.reason = "conditions_not_met";
    return d;
  }

  if (c->stop_since < 0.0) c->stop_since = now;
  if (!c->active && now - c->stop_since < c->stop_hold_sec) {
    d.reason = "waiting_stop_hold";
    return d;
  }

  if (!c->active) {
    c->direction = se_stop_escape_compensator_choose_direction(c);
    if (c->direction == SE_DIRECTION_NONE || c->yawrate <= 0.0) {
      d.reason = "no_safe_side";
      return d;
    }
    c->active = 1;
  }

  d.active = 1;
  d.yawrate = (c->direction == SE_DIRECTION_LEFT) ? c->yawrate : -c->yawrate;
  d.direction = c->direction;
  d.left_clearance = c->left_clearance;
  d.right_clearance = c->right_clearance;
  d.reason = "escaping";
  return d;
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
