/*
 * 独立脱困模块：移植备份版 predictive_escape_step 的状态与输出语义。
 * 该模块只在前进手柄意图、DWA 本拍无有效路径且车辆停稳时输出 v=0 的
 * 原地转向；DWA 恢复路径后立即交还。输入点云来自当前统一融合后的点云。
 */
#ifndef DWA_STOP_ESCAPE_COMPENSATOR_H
#define DWA_STOP_ESCAPE_COMPENSATOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SE_DIRECTION_NONE = 0,
  SE_DIRECTION_LEFT = 1,
  SE_DIRECTION_RIGHT = 2
} se_direction_t;

typedef struct {
  int active;
  int command_enabled;
  double yawrate;
  se_direction_t direction;
  double left_clearance;
  double right_clearance;
  const char *reason;
} se_stop_escape_decision_t;

typedef struct {
  double x;
  double y;
} se_point_t;

typedef struct {
  double stop_hold_sec;
  double stop_speed;
  double yawrate;
  double cloud_timeout;
  double half_width;
  double max_duration_sec;
  double rear_x;
  double front_x;
  double footrest_front_x;
  double footrest_half_width;
  double triangle_gap;
  double stop_gap;
  double cloud_time;
  double left_clearance;
  double right_clearance;
  int left_hit;
  int right_hit;
  double total_ticks;
  double settle_tick;
  double escape_yawrate;
  int session;
  int giveup;
  int joy_reset;
} se_stop_escape_compensator_t;

void se_stop_escape_compensator_init(
  se_stop_escape_compensator_t *c,
  double stop_hold_sec, double stop_speed, double yawrate,
  double cloud_timeout, double half_width);

void se_stop_escape_compensator_reset(se_stop_escape_compensator_t *c);

void se_stop_escape_compensator_update_cloud(
  se_stop_escape_compensator_t *c, const se_point_t *points, size_t count,
  double now);

se_stop_escape_decision_t se_stop_escape_compensator_step(
  se_stop_escape_compensator_t *c,
  double now,
  double joy_v,
  double joy_w,
  double left_mps,
  double right_mps,
  double w_max,
  int has_path);

se_direction_t se_stop_escape_compensator_choose_direction(
  const se_stop_escape_compensator_t *c);

double se_stop_escape_compensator_left_clearance(
  const se_stop_escape_compensator_t *c);
double se_stop_escape_compensator_right_clearance(
  const se_stop_escape_compensator_t *c);

#ifdef __cplusplus
}
#endif

#endif /* DWA_STOP_ESCAPE_COMPENSATOR_H */
