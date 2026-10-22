/*
 * 模块功能：独立脱困补偿：只决定停止且前向指令下的原地转向，
 * 不评估轨迹、不改 DWA、不改感知。
 * 直译自 motor_bridge_cpp/native/stop_escape_compensator.hpp
 * （namespace hu_m40_stop_escape → se_ 前缀）。保持 double 运算。
 * 方向字符串 "NONE"/"LEFT"/"RIGHT" → 枚举（无拼接语义，等价）。
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
  int yield_to_dwa;
  double yawrate;
  se_direction_t direction;
  double left_clearance;
  double right_clearance;
  const char *reason; /* conditions_not_met / waiting_stop_hold /
                         no_safe_side / escaping / paused_cloud_stale /
                         selected_side_blocked / manual_turn_takeover /
                         escape_locked_out / duration_limit / recovered */
} se_stop_escape_decision_t;

typedef struct {
  double x;
  double y;
} se_point_t;

typedef struct {
  /* 构造参数（经 clamp 后不变） */
  double stop_hold_sec;
  double stop_speed;
  double yawrate;
  double cloud_timeout;
  double half_width;
  double max_duration_sec;
  double tie_margin;
  double forward_min_x;
  double forward_max_x;
  double max_lateral;
  /* 状态 */
  double cloud_time;
  double left_clearance;
  double right_clearance;
  se_direction_t front_obstacle_away_direction;
  double stop_since;
  double started_at;
  double recovery_since;
  double forward_command_since;
  double handoff_since;
  double forward_motion_since;
  int handoff_active;
  int inhibited_until_rearm;
  int stop_while_locked_out;
  se_direction_t direction;
  se_direction_t last_direction;
  int active;
} se_stop_escape_compensator_t;

/*
 * 构造（对齐 C++ 默认实参版本；bridge 传入 escape_* 配置）。
 * 构造体内对参数做 std::max / std::abs 钳制。
 */
void se_stop_escape_compensator_init(
  se_stop_escape_compensator_t *c,
  double stop_hold_sec, double stop_speed, double yawrate,
  double cloud_timeout, double half_width);

void se_stop_escape_compensator_reset(se_stop_escape_compensator_t *c);

void se_stop_escape_compensator_update_cloud(
  se_stop_escape_compensator_t *c, const se_point_t *points, size_t count,
  double now);

se_direction_t se_stop_escape_compensator_choose_direction(
  const se_stop_escape_compensator_t *c);

se_stop_escape_decision_t se_stop_escape_compensator_update(
  se_stop_escape_compensator_t *c,
  double now,
  int controller_neutral,
  int forward_intent,
  int vehicle_stopped,
  int normal_output_stopped,
  int positive_dwa_command,
  int positive_forward_motion);

se_stop_escape_decision_t se_stop_escape_compensator_update_with_turn(
  se_stop_escape_compensator_t *c,
  double now,
  int controller_neutral,
  int forward_intent,
  int vehicle_stopped,
  int normal_output_stopped,
  int positive_dwa_command,
  int positive_forward_motion,
  se_direction_t manual_turn_direction);

double se_stop_escape_compensator_left_clearance(
  const se_stop_escape_compensator_t *c);
double se_stop_escape_compensator_right_clearance(
  const se_stop_escape_compensator_t *c);

#ifdef __cplusplus
}
#endif

#endif /* DWA_STOP_ESCAPE_COMPENSATOR_H */
