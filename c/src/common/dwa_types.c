/*
 * 直译自 native/common/dwa_types.hpp 中的非模板函数与默认值。
 * 所有逻辑/顺序/默认值与 C++ 版一致，不做"优化"。
 */
#include "dwa_types.h"

#include <stdlib.h>
#include <string.h>

dwa_path_reference_t dwa_path_reference_default(void)
{
  dwa_path_reference_t ref;
  memset(&ref, 0, sizeof(ref));
  ref.enabled = false;
  ref.target_y = 0.0f;
  ref.target_heading = 0.0f;
  ref.lookahead = 1.20f;
  ref.lateral_weight = 2.00f;
  ref.heading_weight = 0.70f;
  ref.progress_weight = 0.35f;
  return ref;
}

void dwa_trajectory_init(dwa_trajectory_t *trajectory)
{
  memset(trajectory, 0, sizeof(*trajectory));
  trajectory->min_clearance = INFINITY;
  trajectory->total_cost = INFINITY;
  trajectory->obstacle_cost = 0.0f;
  trajectory->to_goal_cost = 0.0f;
  trajectory->speed_cost = 0.0f;
  trajectory->path_cost = 0.0f;
  trajectory->reject_reason = NULL;
}

void dwa_trajectory_destroy(dwa_trajectory_t *trajectory)
{
  free(trajectory->states);
  trajectory->states = NULL;
  trajectory->state_count = 0;
  trajectory->state_capacity = 0;
}

void dwa_trajectory_states_reserve(dwa_trajectory_t *trajectory, size_t capacity)
{
  if (trajectory->state_capacity < capacity) {
    size_t next = trajectory->state_capacity > 0 ? trajectory->state_capacity : 16;
    while (next < capacity) next *= 2;
    dwa_state_t *grown =
      (dwa_state_t *)realloc(trajectory->states, next * sizeof(dwa_state_t));
    trajectory->states = grown;
    trajectory->state_capacity = next;
  }
}

void dwa_trajectory_states_push_back(dwa_trajectory_t *trajectory, const dwa_state_t *state)
{
  dwa_trajectory_states_reserve(trajectory, trajectory->state_count + 1);
  trajectory->states[trajectory->state_count++] = *state;
}

void dwa_trajectory_copy(dwa_trajectory_t *dst, const dwa_trajectory_t *src)
{
  /* 深拷贝；states 缓冲区按 dst 已有容量复用（对应 vector 赋值时保留内存的场景）。 */
  if (dst->state_capacity < src->state_count) {
    dwa_trajectory_states_reserve(dst, src->state_count);
  }
  if (src->state_count > 0) {
    memcpy(dst->states, src->states, src->state_count * sizeof(dwa_state_t));
  }
  dst->state_count = src->state_count;
  dst->command = src->command;
  dst->valid = src->valid;
  dst->min_clearance = src->min_clearance;
  dst->total_cost = src->total_cost;
  dst->obstacle_cost = src->obstacle_cost;
  dst->to_goal_cost = src->to_goal_cost;
  dst->speed_cost = src->speed_cost;
  dst->path_cost = src->path_cost;
  dst->reject_reason = src->reject_reason;
}

dwa_footprint_t dwa_footprint_default(void)
{
  dwa_footprint_t fp;
  memset(&fp, 0, sizeof(fp));
  fp.front = 0.34f;
  fp.back = 0.32f;
  fp.half_width = 0.285f;
  fp.padding = 0.0f;
  fp.footrest_start = 0.0f;
  fp.footrest_front = 0.0f;
  fp.footrest_half_width = 0.0f;
  fp.toe_front = 0.0f;
  fp.toe_half_width = 0.0f;
  return fp;
}

dwa_occupancy_grid_t dwa_occupancy_grid_default(void)
{
  dwa_occupancy_grid_t grid;
  memset(&grid, 0, sizeof(grid));
  grid.resolution = 0.05f;
  grid.origin_x = -0.80f;
  grid.origin_y = -3.00f;
  grid.width = 0;
  grid.height = 0;
  grid.cells = NULL;
  grid.cells_size = 0;
  grid.cells_capacity = 0;
  return grid;
}

void dwa_occupancy_grid_destroy(dwa_occupancy_grid_t *grid)
{
  free(grid->cells);
  grid->cells = NULL;
  grid->cells_size = 0;
  grid->cells_capacity = 0;
}

void dwa_int_vec_destroy(dwa_int_vec_t *vec)
{
  free(vec->data);
  vec->data = NULL;
  vec->count = 0;
  vec->capacity = 0;
}

void dwa_grid_blocking_index_init(dwa_grid_blocking_index_t *index)
{
  memset(index, 0, sizeof(*index));
}

void dwa_grid_blocking_index_destroy(dwa_grid_blocking_index_t *index)
{
  if (index->rows != NULL) {
    for (size_t i = 0; i < index->row_capacity; ++i) {
      dwa_int_vec_destroy(&index->rows[i]);
    }
    free(index->rows);
  }
  memset(index, 0, sizeof(*index));
}

/* static helpers: min/max（与 std::min/std::max 语义一致）。 */
static int dwa_min_int(int a, int b) { return b < a ? b : a; }
static int dwa_max_int(int a, int b) { return a < b ? b : a; }
static float dwa_min_double(float a, float b) { return b < a ? b : a; }
static float dwa_max_double(float a, float b) { return a < b ? b : a; }

void dwa_build_grid_blocking_index(
  const dwa_occupancy_grid_t *grid,
  bool unknown_is_obstacle,
  dwa_grid_blocking_index_t *index)
{
  index->grid = grid;
  index->unknown_is_obstacle = unknown_is_obstacle;
  if (!dwa_grid_valid(grid)) {
    /* blocking_x_by_row.clear()：保留已分配行的容量。 */
    index->row_count = 0;
    return;
  }

  /* blocking_x_by_row.resize(grid->height)：新行默认构造（空），旧行保留内容（随后 clear）。 */
  if (index->row_capacity < (size_t)grid->height) {
    size_t next = index->row_capacity > 0 ? index->row_capacity : 8;
    while (next < (size_t)grid->height) next *= 2;
    dwa_int_vec_t *grown = (dwa_int_vec_t *)realloc(
      index->rows, next * sizeof(dwa_int_vec_t));
    for (size_t i = index->row_capacity; i < next; ++i) {
      grown[i].data = NULL;
      grown[i].count = 0;
      grown[i].capacity = 0;
    }
    index->rows = grown;
    index->row_capacity = next;
  }
  index->row_count = (size_t)grid->height;
  for (int cell_y = 0; cell_y < grid->height; ++cell_y) {
    dwa_int_vec_t *row = &index->rows[(size_t)cell_y];
    row->count = 0; /* row.clear()：容量保留 */
    if (row->capacity < (size_t)(grid->width / 8)) {
      /* row.reserve(width/8)：C++ 可能不缩减容量，这里等价扩容。 */
      size_t want = (size_t)(grid->width / 8);
      int *grown = (int *)realloc(row->data, want * sizeof(int));
      row->data = grown;
      row->capacity = want;
    }
    for (int cell_x = 0; cell_x < grid->width; ++cell_x) {
      const int8_t value = grid->cells[(size_t)(cell_y * grid->width + cell_x)];
      if (value >= 100 || (unknown_is_obstacle && value < 0)) {
        /* row.push_back(cell_x) */
        if (row->count >= row->capacity) {
          size_t want = row->capacity > 0 ? row->capacity * 2 : 8;
          int *grown = (int *)realloc(row->data, want * sizeof(int));
          row->data = grown;
          row->capacity = want;
        }
        row->data[row->count++] = cell_x;
      }
    }
  }
}

void dwa_obstacle_spatial_index_init(dwa_obstacle_spatial_index_t *index)
{
  memset(index, 0, sizeof(*index));
}

void dwa_obstacle_spatial_index_destroy(dwa_obstacle_spatial_index_t *index)
{
  free(index->offsets);
  free(index->write_positions);
  free(index->point_indices);
  memset(index, 0, sizeof(*index));
}

/* offsets.assign(n, 0)：复用缓冲，尺寸= n，全零。 */
static void dwa_size_vec_assign_zero(
  size_t **buf, size_t *size_field, size_t *capacity_field, size_t n)
{
  if (*capacity_field < n) {
    size_t next = *capacity_field > 0 ? *capacity_field : 16;
    while (next < n) next *= 2;
    size_t *grown = (size_t *)realloc(*buf, next * sizeof(size_t));
    *buf = grown;
    *capacity_field = next;
  }
  *size_field = n;
  if (n > 0) memset(*buf, 0, n * sizeof(size_t));
}

/* point_indices.resize(offsets.back())：只增不减内容，尺寸对齐。 */
static void dwa_size_vec_resize(
  size_t **buf, size_t *size_field, size_t *capacity_field, size_t n)
{
  if (*capacity_field < n) {
    size_t next = *capacity_field > 0 ? *capacity_field : 16;
    while (next < n) next *= 2;
    size_t *grown = (size_t *)realloc(*buf, next * sizeof(size_t));
    *buf = grown;
    *capacity_field = next;
  }
  *size_field = n;
}

void dwa_build_obstacle_spatial_index(
  const dwa_point_t *points,
  size_t points_size,
  float bucket_size,
  dwa_obstacle_spatial_index_t *index)
{
  index->points = points;
  index->points_size = points_size;
  index->bucket_size = bucket_size;
  if (points_size == 0 || !isfinite(bucket_size) || bucket_size <= 0.0f) {
    index->width = 0;
    index->height = 0;
    /* 保留 capacity，下一周期继续复用已分配内存。 */
    index->offsets_size = 0;
    index->write_positions_size = 0;
    index->point_indices_size = 0;
    return;
  }

  float min_x = INFINITY;
  float min_y = INFINITY;
  float max_x = -INFINITY;
  float max_y = -INFINITY;
  for (size_t i = 0; i < points_size; ++i) {
    const dwa_point_t *point = &points[i];
    if (!isfinite(point->x) || !isfinite(point->y)) continue;
    min_x = dwa_min_double(min_x, point->x);
    min_y = dwa_min_double(min_y, point->y);
    max_x = dwa_max_double(max_x, point->x);
    max_y = dwa_max_double(max_y, point->y);
  }
  if (!isfinite(min_x) || !isfinite(min_y) || !isfinite(max_x) || !isfinite(max_y)) {
    index->width = 0;
    index->height = 0;
    index->offsets_size = 0;
    index->write_positions_size = 0;
    index->point_indices_size = 0;
    return;
  }

  index->origin_x = floorf(min_x / bucket_size) * bucket_size;
  index->origin_y = floorf(min_y / bucket_size) * bucket_size;
  index->width = dwa_max_int(
    1, (int)floorf((max_x - index->origin_x) / bucket_size) + 1);
  index->height = dwa_max_int(
    1, (int)floorf((max_y - index->origin_y) / bucket_size) + 1);
  const size_t bucket_count = (size_t)index->width * (size_t)index->height;

  /* 计数、前缀和、连续写入。 */
  dwa_size_vec_assign_zero(
    &index->offsets, &index->offsets_size, &index->offsets_capacity,
    bucket_count + 1U);
  for (size_t i = 0; i < points_size; ++i) {
    const dwa_point_t *point = &points[i];
    if (!isfinite(point->x) || !isfinite(point->y)) continue;
    const int bucket_x = (int)floorf((point->x - index->origin_x) / bucket_size);
    const int bucket_y = (int)floorf((point->y - index->origin_y) / bucket_size);
    if (bucket_x < 0 || bucket_x >= index->width ||
      bucket_y < 0 || bucket_y >= index->height) continue;
    const size_t bucket = (size_t)(bucket_y * index->width + bucket_x);
    ++index->offsets[bucket + 1U];
  }
  for (size_t bucket = 1; bucket <= bucket_count; ++bucket) {
    index->offsets[bucket] += index->offsets[bucket - 1U];
  }
  dwa_size_vec_resize(
    &index->point_indices, &index->point_indices_size,
    &index->point_indices_capacity, index->offsets[index->offsets_size - 1U]);
  /* write_positions.assign(offsets.begin(), offsets.end() - 1) */
  dwa_size_vec_resize(
    &index->write_positions, &index->write_positions_size,
    &index->write_positions_capacity, bucket_count);
  if (bucket_count > 0) {
    memcpy(index->write_positions, index->offsets, bucket_count * sizeof(size_t));
  }

  for (size_t point_index = 0; point_index < points_size; ++point_index) {
    const dwa_point_t *point = &points[point_index];
    if (!isfinite(point->x) || !isfinite(point->y)) continue;
    const int bucket_x = (int)floorf((point->x - index->origin_x) / bucket_size);
    const int bucket_y = (int)floorf((point->y - index->origin_y) / bucket_size);
    if (bucket_x < 0 || bucket_x >= index->width ||
      bucket_y < 0 || bucket_y >= index->height) continue;
    const size_t bucket = (size_t)(bucket_y * index->width + bucket_x);
    index->point_indices[index->write_positions[bucket]++] = point_index;
  }
}

bool dwa_spatial_index_visit_square(
  const dwa_obstacle_spatial_index_t *index,
  float x,
  float y,
  float radius,
  dwa_point_visitor_t visitor,
  void *user)
{
  if (!dwa_spatial_index_valid(index) || !isfinite(radius) || radius < 0.0f) return false;

  int min_x = (int)floorf((x - radius - index->origin_x) / index->bucket_size);
  int max_x = (int)floorf((x + radius - index->origin_x) / index->bucket_size);
  int min_y = (int)floorf((y - radius - index->origin_y) / index->bucket_size);
  int max_y = (int)floorf((y + radius - index->origin_y) / index->bucket_size);
  if (max_x < 0 || max_y < 0 || min_x >= index->width || min_y >= index->height) return false;

  min_x = dwa_max_int(0, min_x);
  min_y = dwa_max_int(0, min_y);
  max_x = dwa_min_int(index->width - 1, max_x);
  max_y = dwa_min_int(index->height - 1, max_y);
  for (int bucket_y = min_y; bucket_y <= max_y; ++bucket_y) {
    for (int bucket_x = min_x; bucket_x <= max_x; ++bucket_x) {
      const size_t bucket = (size_t)(bucket_y * index->width + bucket_x);
      const size_t begin = index->offsets[bucket];
      const size_t end = index->offsets[bucket + 1U];
      for (size_t i = begin; i < end; ++i) {
        if (visitor(&index->points[index->point_indices[i]], user)) return true;
      }
    }
  }
  return false;
}
