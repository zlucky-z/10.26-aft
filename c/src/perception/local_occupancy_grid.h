/*
 * 模块功能：局部占用栅格基础操作：初始化、射线空闲标记、障碍标记、
 * 膨胀和自车区域清理。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 *
 * 直译自 native/perception/local_occupancy_grid.hpp（P5）。
 * 全 float 规范（P4.5）。
 */
#ifndef DWA_LOCAL_OCCUPANCY_GRID_H
#define DWA_LOCAL_OCCUPANCY_GRID_H

#include <stdbool.h>

#include "../common/dwa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration for the local grid projection. Values retain the existing
 * node defaults. */
typedef struct {
  /* 局部栅格的分辨率、空间范围和未知单元处理策略。 */
  float resolution;
  float min_x;
  float max_x;
  float max_abs_y;
  float inflation_radius;
} dwa_occupancy_grid_config_t;

dwa_occupancy_grid_config_t dwa_occupancy_grid_config_default(void);

/* 功能：重置或清理reset、occupancy、占用栅格的内部状态。
 * 对应 C++ grid.cells.assign(w*h, -1)；内部负责 cells 缓冲分配/扩容。
 * 分配失败返回 false（原版 vector 不会失败）。 */
bool dwa_reset_occupancy_grid(
  dwa_occupancy_grid_t *grid,
  const dwa_occupancy_grid_config_t *config);

/* 功能：执行`mark_grid_free`，负责本模块对应的mark、占用栅格、free处理。 */
void dwa_mark_grid_free(dwa_occupancy_grid_t *grid, int cell_x, int cell_y);

/* 功能：执行`mark_grid_occupied`，负责本模块对应的mark、占用栅格、occupied处理。 */
void dwa_mark_grid_occupied(dwa_occupancy_grid_t *grid, int cell_x, int cell_y);

/* 功能：执行`raytrace_free`，负责本模块对应的raytrace、free处理。 */
void dwa_raytrace_free(
  dwa_occupancy_grid_t *grid,
  float start_x,
  float start_y,
  float end_x,
  float end_y);

/* 功能：执行`inflate_occupied_cells`，负责本模块对应的inflate、occupied、
 * cells处理。radius<=0 或栅格无效直接返回。内部临时缓冲自管。 */
void dwa_inflate_occupied_cells(
  dwa_occupancy_grid_t *grid,
  float inflation_radius);

/* 功能：重置或清理clear、current、车体包络的内部状态。 */
void dwa_clear_current_footprint(
  dwa_occupancy_grid_t *grid,
  const dwa_footprint_t *footprint);

#ifdef __cplusplus
}
#endif

#endif /* DWA_LOCAL_OCCUPANCY_GRID_H */
