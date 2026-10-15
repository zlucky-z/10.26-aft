/*
 * 模块功能：规划运行时的数据结构：雷达安装位、障碍诊断、侧距、网格和霍尔反馈快照。
 * 直译自 native/runtime/planner_data_types.hpp（纯数据头文件，P3 阶段提前移植，
 * 因为 safety 模块依赖其中的 ManualTurnDirection / 各 Diagnostics / 决策结构）。
 * 注意：LaserMount / ObstacleDiagnostics / GridDiagnostics 目前尚无消费方（P5/P6 使用），
 * 一并直译以保证与 native 数据契约一一对应。
 */
#ifndef PLANNER_DATA_TYPES_H
#define PLANNER_DATA_TYPES_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "../common/dwa_types.h"
#include "../safety/side_wall_hold_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float yaw;
  /* Whether this lidar publishes its scan in the reverse angular order. */
  bool angle_reversed;
  /* Fixed angular correction applied after the scan direction is normalized. */
  float angle_offset;
} planner_laser_mount_t;

typedef struct {
  /* 当前周期参与诊断的障碍点总数及落入当前车体包络的数量。 */
  size_t total;
  size_t inside_current_footprint;
  /* 车体坐标系下的最近前方、左右两侧和最近雷达点距离。 */
  float nearest_front;
  float nearest_gear_front;
  float nearest_left;
  float nearest_right;
  float nearest_radar_distance;
  float nearest_front_radar_distance;
  /* 对应上述距离的代表点，便于日志和可视化定位。 */
  dwa_point_t nearest_radar_point;
  dwa_point_t nearest_front_point;
  dwa_point_t nearest_left_point;
  dwa_point_t nearest_right_point;
} planner_obstacle_diagnostics_t;

static inline planner_obstacle_diagnostics_t planner_obstacle_diagnostics_default(void)
{
  planner_obstacle_diagnostics_t d;
  d.total = 0;
  d.inside_current_footprint = 0;
  d.nearest_front = INFINITY;
  d.nearest_gear_front = INFINITY;
  d.nearest_left = INFINITY;
  d.nearest_right = INFINITY;
  d.nearest_radar_distance = INFINITY;
  d.nearest_front_radar_distance = INFINITY;
  d.nearest_radar_point.x = 0.0f; d.nearest_radar_point.y = 0.0f;
  d.nearest_front_point.x = 0.0f; d.nearest_front_point.y = 0.0f;
  d.nearest_left_point.x = 0.0f; d.nearest_left_point.y = 0.0f;
  d.nearest_right_point.x = 0.0f; d.nearest_right_point.y = 0.0f;
  return d;
}

typedef struct {
  /* 左右侧相对车体外沿的最小净距及对应障碍点。 */
  float left;
  float right;
  dwa_point_t left_point;
  dwa_point_t right_point;
} planner_side_clearance_diagnostics_t;

static inline planner_side_clearance_diagnostics_t
planner_side_clearance_diagnostics_default(void)
{
  planner_side_clearance_diagnostics_t d;
  d.left = INFINITY;
  d.right = INFINITY;
  d.left_point.x = 0.0f; d.left_point.y = 0.0f;
  d.right_point.x = 0.0f; d.right_point.y = 0.0f;
  return d;
}

typedef struct {
  /* 左右侧硬停圆的最近半径及对应障碍点。 */
  float left_radius;
  float right_radius;
  dwa_point_t left_point;
  dwa_point_t right_point;
} planner_side_hard_stop_circle_diagnostics_t;

static inline planner_side_hard_stop_circle_diagnostics_t
planner_side_hard_stop_circle_diagnostics_default(void)
{
  planner_side_hard_stop_circle_diagnostics_t d;
  d.left_radius = INFINITY;
  d.right_radius = INFINITY;
  d.left_point.x = 0.0f; d.left_point.y = 0.0f;
  d.right_point.x = 0.0f; d.right_point.y = 0.0f;
  return d;
}

typedef struct {
  /* 前方禁止转向区域内的命中数量、最近x坐标和代表点。 */
  size_t point_count;
  float nearest_x;
  dwa_point_t nearest_point;
} planner_front_no_turn_stop_diagnostics_t;

static inline planner_front_no_turn_stop_diagnostics_t
planner_front_no_turn_stop_diagnostics_default(void)
{
  planner_front_no_turn_stop_diagnostics_t d;
  d.point_count = 0;
  d.nearest_x = INFINITY;
  d.nearest_point.x = 0.0f; d.nearest_point.y = 0.0f;
  return d;
}

/* enum class ManualTurnDirection { kNone, kRight, kLeft } */
typedef enum {
  PLANNER_TURN_NONE = 0,
  PLANNER_TURN_RIGHT = 1,
  PLANNER_TURN_LEFT = 2
} planner_manual_turn_direction_t;

typedef struct {
  /* 侧边保护最终给出的停止、限角速度和方向决定。 */
  bool stop;
  bool yaw_limited;
  planner_manual_turn_direction_t direction;
  dwa_side_wall_hold_side_t side;
  float radius;
  float prediction_time_sec;
  float limited_yawrate;
  dwa_point_t point;
} planner_side_hard_stop_decision_t;

static inline planner_side_hard_stop_decision_t
planner_side_hard_stop_decision_default(void)
{
  planner_side_hard_stop_decision_t d;
  d.stop = false;
  d.yaw_limited = false;
  d.direction = PLANNER_TURN_NONE;
  d.side = DWA_SIDE_HOLD_NONE;
  d.radius = INFINITY;
  d.prediction_time_sec = 0.0f;
  d.limited_yawrate = 0.0f;
  d.point.x = 0.0f; d.point.y = 0.0f;
  return d;
}

typedef struct {
  /* 当前局部栅格中的空闲、占用和未知单元统计。 */
  size_t free_cells;
  size_t occupied_cells;
  size_t unknown_cells;
} planner_grid_diagnostics_t;

typedef struct {
  /* 左右轮反馈、车体前进速度、控制器原始档位以及反馈的新鲜度状态。 */
  float left_mps;
  float right_mps;
  float forward_speed;
  int raw_gear;
  bool raw_gear_valid;
  bool valid;
  bool fresh;
  float age;
  bool forward_start_rising;
} planner_motor_feedback_snapshot_t;

static inline planner_motor_feedback_snapshot_t
planner_motor_feedback_snapshot_default(void)
{
  planner_motor_feedback_snapshot_t d;
  d.left_mps = 0.0f;
  d.right_mps = 0.0f;
  d.forward_speed = 0.0f;
  d.raw_gear = 1;
  d.raw_gear_valid = false;
  d.valid = false;
  d.fresh = false;
  d.age = INFINITY;
  d.forward_start_rising = false;
  return d;
}

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_DATA_TYPES_H */
