/*
 * 模块功能：DWA速度窗口和候选轨迹采样、评估、排序入口。
 * 直译自 native/planning/dwa_sampling.hpp。
 * 说明：C++ 版的 OpenMP 并行候选循环为候选间无共享写、静态分片且结果
 * 按索引写回，串行执行结果完全一致；C 版为确定性串行（见 TRANSLATION_NOTES）。
 */
#ifndef DWA_SAMPLING_H
#define DWA_SAMPLING_H

#include "dwa_config.h"
#include "dwa_scoring.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * plan_dwa_into(obstacles, current_velocity, user_intent, config, result,
 *               grid, spatial_index_workspace, blocking_index_workspace)
 * 直译；grid 与两个工作区可为 NULL。
 */
void dwa_plan_dwa_into(
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *current_velocity,
  const dwa_velocity_t *user_intent,
  const dwa_config_t *config,
  dwa_plan_result_t *result,
  const dwa_occupancy_grid_t *grid,
  dwa_obstacle_spatial_index_t *spatial_index_workspace,
  dwa_grid_blocking_index_t *blocking_index_workspace);

/* dwa_turn_direction(yawrate, epsilon) 直译。 */
int dwa_turn_direction(float yawrate, float epsilon);

#ifdef __cplusplus
}
#endif

#endif /* DWA_SAMPLING_H */
