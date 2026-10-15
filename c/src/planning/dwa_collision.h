/*
 * 模块功能：DWA碰撞检测：判断预测车体包络与占用栅格或障碍物的交叠。
 * 直译自 native/planning/dwa_collision.hpp。
 */
#ifndef DWA_COLLISION_H
#define DWA_COLLISION_H

#include <stdbool.h>

#include "dwa_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * footprint_hits_grid(state, grid, config, cos_yaw, sin_yaw, &hit_unknown, blocking_index)
 * 直译；hit_unknown / blocking_index 可为 NULL（对应 C++ 默认参数）。
 */
bool dwa_footprint_hits_grid(
  const dwa_state_t *state,
  const dwa_occupancy_grid_t *grid,
  const dwa_config_t *config,
  float cos_yaw,
  float sin_yaw,
  bool *hit_unknown,
  const dwa_grid_blocking_index_t *blocking_index);

#ifdef __cplusplus
}
#endif

#endif /* DWA_COLLISION_H */
