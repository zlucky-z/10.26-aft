/*
 * 模块功能：DWA轨迹代价计算、碰撞有效性判定和代价归一化。
 * 直译自 native/planning/dwa_scoring.hpp。
 * C++ lambda evaluate_obstacle → evaluate_obstacle_ctx_t + dwa_evaluate_obstacle。
 */
#ifndef DWA_SCORING_H
#define DWA_SCORING_H

#include "dwa_collision.h"
#include "dwa_config.h"
#include "dwa_trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * evaluate_trajectory(trajectory, obstacles, obstacle_count, local_goal,
 *                    dynamic_max_velocity, config, grid, spatial_index, blocking_index)
 * 直译；grid/spatial_index/blocking_index 可为 NULL。
 */
void dwa_evaluate_trajectory(
  dwa_trajectory_t *trajectory,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_point_t *local_goal,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  const dwa_occupancy_grid_t *grid,
  const dwa_obstacle_spatial_index_t *spatial_index,
  const dwa_grid_blocking_index_t *blocking_index);

/* evaluate_command_trajectory_into 直译。 */
void dwa_evaluate_command_trajectory_into(
  const dwa_velocity_t *command,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *intent,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  dwa_trajectory_t *trajectory,
  const dwa_occupancy_grid_t *grid,
  const dwa_obstacle_spatial_index_t *spatial_index,
  const dwa_grid_blocking_index_t *blocking_index);

/* evaluate_forward_clearance_trajectory 直译（结果写入 *trajectory）。 */
void dwa_evaluate_forward_clearance_trajectory(
  float lookahead_distance,
  float probe_speed,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *intent,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  const dwa_occupancy_grid_t *grid,
  dwa_trajectory_t *trajectory);

float dwa_normalize_cost_component(float value, float minimum, float maximum);

void dwa_normalize_trajectory_costs(
  dwa_trajectory_t *trajectories,
  size_t trajectory_count,
  const dwa_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* DWA_SCORING_H */
