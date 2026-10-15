/*
 * 模块功能：局部占用栅格基础操作：初始化、射线空闲标记、障碍标记、
 * 膨胀和自车区域清理。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 *
 * 逐行直译自 native/perception/local_occupancy_grid.hpp（P5），
 * 全 float 规范（P4.5）。
 */
#include "local_occupancy_grid.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../common/dwa_geometry.h"

dwa_occupancy_grid_config_t dwa_occupancy_grid_config_default(void)
{
  dwa_occupancy_grid_config_t config;
  config.resolution = 0.05f;
  config.min_x = -0.80f;
  config.max_x = 3.60f;
  config.max_abs_y = 3.00f;
  config.inflation_radius = 0.0f;
  return config;
}

/* 功能：重置或清理reset、occupancy、占用栅格的内部状态。 */
bool dwa_reset_occupancy_grid(
  dwa_occupancy_grid_t *grid,
  const dwa_occupancy_grid_config_t *config)
{
  size_t needed;

  grid->resolution = config->resolution > 0.02f ? config->resolution : 0.02f;
  grid->origin_x = config->min_x;
  grid->origin_y = -config->max_abs_y;
  {
    int w = (int)ceilf((config->max_x - config->min_x) / grid->resolution);
    grid->width = w > 1 ? w : 1;
  }
  {
    int h = (int)ceilf(2.0f * config->max_abs_y / grid->resolution);
    grid->height = h > 1 ? h : 1;
  }

  needed = (size_t)grid->width * (size_t)grid->height;
  if (grid->cells_capacity < needed) {
    int8_t *fresh = (int8_t *)realloc(grid->cells, needed);
    if (fresh == NULL) {
      grid->cells_size = 0;
      return false;
    }
    grid->cells = fresh;
    grid->cells_capacity = needed;
  }
  grid->cells_size = needed;
  memset(grid->cells, -1, needed);
  return true;
}

/* 功能：执行`mark_grid_free`，负责本模块对应的mark、占用栅格、free处理。 */
void dwa_mark_grid_free(dwa_occupancy_grid_t *grid, int cell_x, int cell_y)
{
  int8_t *cell;
  if (cell_x < 0 || cell_x >= grid->width || cell_y < 0 || cell_y >= grid->height) return;
  cell = &grid->cells[(size_t)(cell_y * grid->width + cell_x)];
  if (*cell < 100) *cell = 0;
}

/* 功能：执行`mark_grid_occupied`，负责本模块对应的mark、占用栅格、occupied处理。 */
void dwa_mark_grid_occupied(dwa_occupancy_grid_t *grid, int cell_x, int cell_y)
{
  if (cell_x < 0 || cell_x >= grid->width || cell_y < 0 || cell_y >= grid->height) return;
  grid->cells[(size_t)(cell_y * grid->width + cell_x)] = 100;
}

/* 功能：执行`raytrace_free`，负责本模块对应的raytrace、free处理。 */
void dwa_raytrace_free(
  dwa_occupancy_grid_t *grid,
  float start_x,
  float start_y,
  float end_x,
  float end_y)
{
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  int x;
  int y;
  int dx;
  int sx;
  int dy;
  int sy;
  int error;

  if (!dwa_grid_world_to_cell(grid, start_x, start_y, &x0, &y0) ||
      !dwa_grid_world_to_cell(grid, end_x, end_y, &x1, &y1)) return;

  x = x0;
  y = y0;
  dx = abs(x1 - x0);
  sx = x0 < x1 ? 1 : -1;
  dy = -abs(y1 - y0);
  sy = y0 < y1 ? 1 : -1;
  error = dx + dy;
  while (x != x1 || y != y1) {
    dwa_mark_grid_free(grid, x, y);
    {
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y += sy;
      }
    }
  }
}

/* 功能：执行`inflate_occupied_cells`，负责本模块对应的inflate、occupied、
 * cells处理。 */
void dwa_inflate_occupied_cells(
  dwa_occupancy_grid_t *grid,
  float inflation_radius)
{
  int8_t *original;
  int radius;
  int y;

  if (!dwa_grid_valid(grid) || inflation_radius <= 0.0f) return;
  original = (int8_t *)malloc(grid->cells_size);
  if (original == NULL) return; /* 防御：原版 vector 拷贝不会失败 */
  memcpy(original, grid->cells, grid->cells_size);
  radius = (int)ceilf(inflation_radius / grid->resolution);
  for (y = 0; y < grid->height; ++y) {
    int x;
    for (x = 0; x < grid->width; ++x) {
      if (original[(size_t)(y * grid->width + x)] < 100) continue;
      {
        int offset_y;
        for (offset_y = -radius; offset_y <= radius; ++offset_y) {
          int offset_x;
          for (offset_x = -radius; offset_x <= radius; ++offset_x) {
            if (offset_x * offset_x + offset_y * offset_y > radius * radius) continue;
            dwa_mark_grid_occupied(grid, x + offset_x, y + offset_y);
          }
        }
      }
    }
  }
  free(original);
}

/* 功能：重置或清理clear、current、车体包络的内部状态。 */
void dwa_clear_current_footprint(
  dwa_occupancy_grid_t *grid,
  const dwa_footprint_t *footprint)
{
  const float front = dwa_footprint_front_extent(footprint, false) +
    footprint->padding;
  const float back = footprint->back + footprint->padding;
  const float half_width = dwa_footprint_half_width_extent(
    footprint, false) + footprint->padding;
  int min_x = 0;
  int min_y = 0;
  int max_x = 0;
  int max_y = 0;
  int y;

  if (!dwa_grid_world_to_cell(grid, -back, -half_width, &min_x, &min_y) ||
      !dwa_grid_world_to_cell(grid, front, half_width, &max_x, &max_y)) return;
  for (y = min_y; y <= max_y; ++y) {
    const float world_y = grid->origin_y +
      ((float)y + 0.5f) * grid->resolution;
    int x;
    for (x = min_x; x <= max_x; ++x) {
      const float world_x = grid->origin_x +
        ((float)x + 0.5f) * grid->resolution;
      if (dwa_footprint_contains_local(
          world_x, world_y, footprint, false)) {
        dwa_mark_grid_free(grid, x, y);
      }
    }
  }
}
