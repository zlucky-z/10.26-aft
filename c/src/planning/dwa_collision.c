/* 直译自 native/planning/dwa_collision.hpp。 */
#include "dwa_collision.h"

#include "../common/dwa_geometry.h"

/* 10.09 编译卫生：dwa_min_double 自 9.27 起未被使用，删除消警告。 */
static float dwa_max_double(float a, float b) { return a < b ? b : a; }

bool dwa_footprint_hits_grid(
  const dwa_state_t *state,
  const dwa_occupancy_grid_t *grid,
  const dwa_config_t *config,
  float cos_yaw,
  float sin_yaw,
  bool *hit_unknown,
  const dwa_grid_blocking_index_t *blocking_index)
{
  if (hit_unknown) *hit_unknown = false;
  if (!dwa_grid_valid(grid)) return false;

  const float front = dwa_footprint_front_extent(&config->footprint, true) +
    config->footprint.padding;
  const float back = config->footprint.back + config->footprint.padding;
  const float half_width = dwa_footprint_half_width_extent(&config->footprint, true) +
    config->footprint.padding;
  const float radius_x = fabsf(cos_yaw) * dwa_max_double(front, back) +
    fabsf(sin_yaw) * half_width + grid->resolution;
  const float radius_y = fabsf(sin_yaw) * dwa_max_double(front, back) +
    fabsf(cos_yaw) * half_width + grid->resolution;

  int min_x = 0;
  int min_y = 0;
  int max_x = 0;
  int max_y = 0;
  if (!dwa_grid_world_to_cell(grid, state->x - radius_x, state->y - radius_y, &min_x, &min_y) ||
    !dwa_grid_world_to_cell(grid, state->x + radius_x, state->y + radius_y, &max_x, &max_y)) {
    if (hit_unknown) *hit_unknown = true;
    return config->unknown_is_obstacle;
  }

  const bool use_blocking_index = blocking_index != NULL &&
    dwa_grid_blocking_index_valid(blocking_index) &&
    blocking_index->grid == grid &&
    blocking_index->unknown_is_obstacle == config->unknown_is_obstacle;
  if (use_blocking_index) {
    for (int cell_y = min_y; cell_y <= max_y; ++cell_y) {
      const float world_y = grid->origin_y +
        ((float)cell_y + 0.5f) * grid->resolution;
      const dwa_int_vec_t *row = &blocking_index->rows[(size_t)cell_y];
      /* std::lower_bound(row, min_x)：行内有序，线性跳过 < min_x 等价。 */
      for (size_t idx = 0; idx < row->count; ++idx) {
        const int cell_x = row->data[idx];
        if (cell_x < min_x) continue;
        if (cell_x > max_x) break;
        const float world_x = grid->origin_x +
          ((float)cell_x + 0.5f) * grid->resolution;
        const float dx = world_x - state->x;
        const float dy = world_y - state->y;
        const float local_x = cos_yaw * dx + sin_yaw * dy;
        const float local_y = -sin_yaw * dx + cos_yaw * dy;
        if (!dwa_footprint_contains_local(local_x, local_y, &config->footprint, true)) {
          continue;
        }

        const int8_t value = grid->cells[(size_t)(cell_y * grid->width + cell_x)];
        if (value >= 100) return true;
        if (value < 0 && config->unknown_is_obstacle) {
          if (hit_unknown) *hit_unknown = true;
          return true;
        }
      }
    }
    return false;
  }

  for (int cell_y = min_y; cell_y <= max_y; ++cell_y) {
    const float world_y = grid->origin_y +
      ((float)cell_y + 0.5f) * grid->resolution;
    for (int cell_x = min_x; cell_x <= max_x; ++cell_x) {
      const float world_x = grid->origin_x +
        ((float)cell_x + 0.5f) * grid->resolution;
      const float dx = world_x - state->x;
      const float dy = world_y - state->y;
      const float local_x = cos_yaw * dx + sin_yaw * dy;
      const float local_y = -sin_yaw * dx + cos_yaw * dy;
      if (!dwa_footprint_contains_local(local_x, local_y, &config->footprint, true)) continue;

      const int8_t value = grid->cells[(size_t)(cell_y * grid->width + cell_x)];
      if (value >= 100) return true;
      if (value < 0 && config->unknown_is_obstacle) {
        if (hit_unknown) *hit_unknown = true;
        return true;
      }
    }
  }
  return false;
}
