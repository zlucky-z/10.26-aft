/* predictive_FNC.c::predictive_escape_step 的米制/模块接口移植。
 * 这里仅做 Point2D/单位/调用接口适配；判断顺序和常量语义保持备份版。 */
#include "stop_escape_compensator.h"

#include <math.h>
#include <string.h>

#define ESC_WBOOST_ENABLE 1
#define ESC_WBOOS_LIMIT_TICKS 2.0
#define ESC_WBOOS_STEP 0.05
#define ESC_TOTAL_LIMIT_TICKS 30.0
#define ESC_ESCAPE_YAWRATE 0.65
#define ESC_ESCAPE_MIN_SIDE_CLEARANCE 0.10
#define ESC_SETTLE_WHEEL_EPS_MPS 0.01
#define GEAR_REV_TURN_DEAD_RAD 0.10

static int se_side_tri_stop_hit(
  const se_stop_escape_compensator_t *c,
  const se_point_t *points,
  size_t count,
  int turn_dir,
  double *gap)
{
  const double dx1 = c->front_x - c->rear_x;
  /* Backup call passes ESC_ESCAPE_MIN_SIDE_CLEARANCE (10cm) here.  The
   * footrest-front check below separately uses GEAR_REV_SIDE_STOP_GAP_CM
   * (20cm); these two distances must not be collapsed. */
  const double th1 = atan2(c->triangle_gap, dx1);
  const double hyp1 = dx1 / cos(th1);
  size_t i;

  if (gap != NULL) *gap = 1e9;
  if (points == NULL || count == 0) return 0;
  for (i = 0; i < count; ++i) {
    const double px = points[i].x;
    const double raw_y = points[i].y;
    const double y = turn_dir > 0 ? raw_y : -raw_y;
    double rx;
    double ry;
    if (px < c->rear_x || px > c->footrest_front_x) continue;
    if (y < c->footrest_half_width) continue;
    if (turn_dir > 0 && raw_y < 0.0) continue;
    if (turn_dir < 0 && raw_y > 0.0) continue;
    if (px <= c->front_x && y > c->half_width) {
      rx = px - c->rear_x;
      ry = y - c->half_width;
      if (atan2(ry, rx) <= th1 && sqrt(rx * rx + ry * ry) <= hyp1)
        return -1;
    }
    if (px >= c->front_x && y > c->footrest_half_width) {
      ry = y - c->footrest_half_width;
      if (ry < c->stop_gap) return -1;
    }
    if (px >= c->front_x && y > c->half_width && gap != NULL &&
        y < *gap) *gap = y;
  }
  return 0;
}

static void se_decision_reset(se_stop_escape_decision_t *d,
  const se_stop_escape_compensator_t *c)
{
  memset(d, 0, sizeof(*d));
  d->direction = SE_DIRECTION_NONE;
  d->left_clearance = c->left_clearance;
  d->right_clearance = c->right_clearance;
  d->reason = "conditions_not_met";
}

void se_stop_escape_compensator_init(
  se_stop_escape_compensator_t *c,
  double stop_hold_sec, double stop_speed, double yawrate,
  double cloud_timeout, double half_width)
{
  (void)half_width; /* Backup geometry is fixed by planner_common.h. */
  memset(c, 0, sizeof(*c));
  c->stop_hold_sec = stop_hold_sec > 0.0 ? stop_hold_sec : 0.0;
  c->stop_speed = stop_speed > 0.0 ? stop_speed : ESC_SETTLE_WHEEL_EPS_MPS;
  c->yawrate = fabs(yawrate) > 0.0 ? fabs(yawrate) : ESC_ESCAPE_YAWRATE;
  c->cloud_timeout = cloud_timeout > 0.05 ? cloud_timeout : 0.60;
  /* Backup planner_common.h: CAR_WIDTH=63cm, CAR_HALF_WID=31.5cm. */
  c->half_width = 0.315;
  c->max_duration_sec = 3.0;
  c->rear_x = -0.14;
  c->front_x = 0.50;
  c->footrest_front_x = 0.90;
  /* Backup dwa_plan_local.h: LEFT_RADAR_Y(28)-BLIND_MAX_JX(9)=19cm;
   * predictive_side_tri_stop_hit uses 10cm for the rear-body triangle and
   * GEAR_REV_SIDE_STOP_GAP_CM=20cm for the footrest-front edge. */
  c->footrest_half_width = 0.19;
  c->triangle_gap = ESC_ESCAPE_MIN_SIDE_CLEARANCE;
  c->stop_gap = 0.20;
  c->cloud_time = 0.0;
  c->left_clearance = 1e9;
  c->right_clearance = 1e9;
  c->escape_yawrate = c->yawrate;
  se_stop_escape_compensator_reset(c);
}

void se_stop_escape_compensator_reset(se_stop_escape_compensator_t *c)
{
  c->total_ticks = 0.0;
  c->settle_tick = 0.0;
  c->escape_yawrate = c->yawrate;
  c->session = 0;
  c->giveup = 0;
  c->joy_reset = 0;
}

void se_stop_escape_compensator_update_cloud(
  se_stop_escape_compensator_t *c, const se_point_t *points, size_t count,
  double now)
{
  c->left_hit = se_side_tri_stop_hit(c, points, count, 1,
    &c->left_clearance) == -1;
  c->right_hit = se_side_tri_stop_hit(c, points, count, -1,
    &c->right_clearance) == -1;
  c->cloud_time = now;
}

se_direction_t se_stop_escape_compensator_choose_direction(
  const se_stop_escape_compensator_t *c)
{
  /* This intentionally preserves the backup version's selection conditions,
   * including its original right-side comparison semantics. */
  const int left_stop = c->left_hit ? -1 : 0;
  const int right_stop = c->right_hit ? -1 : 0;
  if (left_stop != -1 && right_stop != -1) {
    return c->left_clearance > c->right_clearance ?
      SE_DIRECTION_LEFT : SE_DIRECTION_RIGHT;
  }
  if (left_stop != -1) return SE_DIRECTION_LEFT;
  if (c->right_clearance != -1) return SE_DIRECTION_RIGHT;
  return SE_DIRECTION_NONE;
}

se_stop_escape_decision_t se_stop_escape_compensator_step(
  se_stop_escape_compensator_t *c,
  double now,
  double joy_v,
  double joy_w,
  double left_mps,
  double right_mps,
  double w_max,
  int has_path)
{
  se_stop_escape_decision_t d;
  int esc_req_dir = 0;
  int stop_lr;
  int stop_dir;

  (void)now;
  se_decision_reset(&d, c);

  if (has_path) {
    c->session = 0;
    c->settle_tick = 0.0;
    if (fabs(left_mps) > ESC_SETTLE_WHEEL_EPS_MPS ||
        fabs(right_mps) > ESC_SETTLE_WHEEL_EPS_MPS) {
      c->total_ticks = 0.0;
      c->giveup = 0;
      c->joy_reset = 0;
    }
    c->escape_yawrate = c->yawrate;
    return d;
  }

  if (!c->session) {
    if (fabs(left_mps) > ESC_SETTLE_WHEEL_EPS_MPS ||
        fabs(right_mps) > ESC_SETTLE_WHEEL_EPS_MPS) {
      d.reason = "waiting_wheel_settle";
      return d;
    }
    c->settle_tick = 0.0;
    c->session = 1;
  }

  if (c->giveup) {
    if (fabs(joy_v) <= 0.01 && fabs(joy_w) < 0.05) {
      c->joy_reset = 1;
    } else if (c->joy_reset && joy_v > 0.01) {
      c->giveup = 0;
      c->joy_reset = 0;
      c->total_ticks = 0.0;
      c->escape_yawrate = c->yawrate;
    }
  } else {
    c->total_ticks += 1.0;
    if (c->total_ticks > ESC_TOTAL_LIMIT_TICKS) c->giveup = 1;
  }

  if (fabs(joy_w) >= GEAR_REV_TURN_DEAD_RAD)
    esc_req_dir = joy_w > 0.0 ? 1 : -1;

  if (esc_req_dir == 0) {
    const se_direction_t selected =
      se_stop_escape_compensator_choose_direction(c);
    stop_dir = selected == SE_DIRECTION_LEFT ? 1 :
      (selected == SE_DIRECTION_RIGHT ? -1 : 0);
  } else {
    /* Same branch as predictive_escape_step: requested side is accepted only
     * when predictive_side_tri_stop_hit() did not return -1. */
    stop_lr = esc_req_dir > 0 ? (c->left_hit ? -1 : 0) :
      (c->right_hit ? -1 : 0);
    stop_dir = stop_lr != -1 ? esc_req_dir : 0;
  }

  if (stop_dir != 0 && !c->giveup) {
#if ESC_WBOOST_ENABLE
    if (c->total_ticks >= ESC_WBOOS_LIMIT_TICKS) {
      c->escape_yawrate += ESC_WBOOS_STEP;
      if (w_max > 0.0 && c->escape_yawrate > w_max)
        c->escape_yawrate = w_max;
    } else {
      c->escape_yawrate = c->yawrate;
    }
#endif
    d.active = 1;
    d.command_enabled = 1;
    d.yawrate = stop_dir > 0 ? c->escape_yawrate : -c->escape_yawrate;
    d.direction = stop_dir > 0 ? SE_DIRECTION_LEFT : SE_DIRECTION_RIGHT;
    d.reason = "escaping";
    return d;
  }
  d.reason = c->giveup ? "duration_limit" : "no_safe_side";
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
