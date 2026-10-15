/*
 * 模块功能：DWA公共数据结构：点、速度、轨迹、车体包络、占用栅格和空间索引。
 * 直译自 native/common/dwa_types.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * C++ vector/string → C 动态数组/静态字符串字面量，见 c/docs/TRANSLATION_NOTES.md。
 */
#ifndef DWA_TYPES_H
#define DWA_TYPES_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* constexpr double kDwaPi */
#define DWA_PI 3.14159265358979323846f

typedef struct {
  float x;
  float y;
} dwa_point_t;

typedef struct {
  bool enabled;
  float target_y;
  float target_heading;
  float lookahead;
  float lateral_weight;
  float heading_weight;
  float progress_weight;
} dwa_path_reference_t;

/* 返回 DwaPathReference 的成员默认值（C++ 成员初始化器直译）。 */
dwa_path_reference_t dwa_path_reference_default(void);

typedef struct {
  float v;
  float w;
} dwa_velocity_t;

/* manual_command_is_neutral 直译；std::max(0.0,x) → x<0?0:x 保持求值语义。 */
static inline float dwa_max0(float value)
{
  return value > 0.0f ? value : 0.0f;
}

static inline bool dwa_manual_command_is_neutral(
  const dwa_velocity_t *command,
  float linear_deadband,
  float angular_deadband)
{
  return isfinite(command->v) && isfinite(command->w) &&
    fabsf(command->v) <= dwa_max0(linear_deadband) &&
    fabsf(command->w) <= dwa_max0(angular_deadband);
}

typedef struct {
  float x;
  float y;
  float yaw;
  float v;
  float w;
} dwa_state_t;

/*
 * DwaTrajectory：states 为复用容量的动态数组（对应 std::vector<DwaState>）。
 * reject_reason 对应 std::string，但原代码只赋固定字面量/清空，
 * 故用 const char*（指向静态字面量）或 NULL（= clear() 后的空串）。
 */
typedef struct {
  dwa_state_t *states;        /* 元素数组 */
  size_t state_count;         /* vector::size() */
  size_t state_capacity;      /* vector::capacity() */
  dwa_velocity_t command;
  bool valid;
  float min_clearance;
  float total_cost;
  float obstacle_cost;
  float to_goal_cost;
  float speed_cost;
  float path_cost;
  const char *reject_reason;  /* NULL == 空串 */
} dwa_trajectory_t;

/* 构造一个全默认值的 DwaTrajectory（对应默认构造 + 成员初始化器）。 */
void dwa_trajectory_init(dwa_trajectory_t *trajectory);
void dwa_trajectory_destroy(dwa_trajectory_t *trajectory);
/* vector::reserve */
void dwa_trajectory_states_reserve(dwa_trajectory_t *trajectory, size_t capacity);
/* vector::push_back */
void dwa_trajectory_states_push_back(dwa_trajectory_t *trajectory, const dwa_state_t *state);
/* 深拷贝（states 内存按 dst 现有容量复用）。 */
void dwa_trajectory_copy(dwa_trajectory_t *dst, const dwa_trajectory_t *src);

typedef struct {
  float front;
  float back;
  float half_width;
  float padding;
  float footrest_start;
  float footrest_front;
  float footrest_half_width;
  float toe_front;
  float toe_half_width;
} dwa_footprint_t;

dwa_footprint_t dwa_footprint_default(void);

/* DwaOccupancyGrid：-1 未知、0 空闲、100 占用。 */
typedef struct {
  float resolution;
  float origin_x;
  float origin_y;
  int width;
  int height;
  int8_t *cells;        /* vector<int8_t> */
  size_t cells_size;
  size_t cells_capacity;
} dwa_occupancy_grid_t;

dwa_occupancy_grid_t dwa_occupancy_grid_default(void);
void dwa_occupancy_grid_destroy(dwa_occupancy_grid_t *grid);

/* grid.valid() 直译。 */
static inline bool dwa_grid_valid(const dwa_occupancy_grid_t *grid)
{
  return grid->resolution > 0.0f && grid->width > 0 && grid->height > 0 &&
    grid->cells_size == (size_t)(grid->width * grid->height);
}

/* grid.world_to_cell(x, y, &cell_x, &cell_y) 直译。 */
static inline bool dwa_grid_world_to_cell(
  const dwa_occupancy_grid_t *grid, float x, float y, int *cell_x, int *cell_y)
{
  if (!dwa_grid_valid(grid)) return false;
  *cell_x = (int)floorf((x - grid->origin_x) / grid->resolution);
  *cell_y = (int)floorf((y - grid->origin_y) / grid->resolution);
  return *cell_x >= 0 && *cell_x < grid->width && *cell_y >= 0 && *cell_y < grid->height;
}

/* grid.cell(cell_x, cell_y) 直译：越界按未知返回。 */
static inline int8_t dwa_grid_cell(const dwa_occupancy_grid_t *grid, int cell_x, int cell_y)
{
  if (cell_x < 0 || cell_x >= grid->width || cell_y < 0 || cell_y >= grid->height) return -1;
  return grid->cells[(size_t)(cell_y * grid->width + cell_x)];
}

/* int 动态数组（对应 vector<int> 行）。 */
typedef struct {
  int *data;
  size_t count;
  size_t capacity;
} dwa_int_vec_t;

void dwa_int_vec_destroy(dwa_int_vec_t *vec);

/* DwaGridBlockingIndex 直译。row_count 对应 blocking_x_by_row.size()。 */
typedef struct {
  const dwa_occupancy_grid_t *grid;
  bool unknown_is_obstacle;
  dwa_int_vec_t *rows;
  size_t row_count;
  size_t row_capacity;
} dwa_grid_blocking_index_t;

void dwa_grid_blocking_index_init(dwa_grid_blocking_index_t *index);
void dwa_grid_blocking_index_destroy(dwa_grid_blocking_index_t *index);

static inline bool dwa_grid_blocking_index_valid(const dwa_grid_blocking_index_t *index)
{
  return index->grid != NULL && dwa_grid_valid(index->grid) &&
    index->row_count == (size_t)index->grid->height;
}

/* build_grid_blocking_index 直译。 */
void dwa_build_grid_blocking_index(
  const dwa_occupancy_grid_t *grid,
  bool unknown_is_obstacle,
  dwa_grid_blocking_index_t *index);

/* DwaObstacleSpatialIndex 直译。points/points_size 为外部点云（只读）。 */
typedef struct {
  float bucket_size;
  float origin_x;
  float origin_y;
  int width;
  int height;
  const dwa_point_t *points;
  size_t points_size;
  size_t *offsets;          size_t offsets_size;        size_t offsets_capacity;
  size_t *write_positions;  size_t write_positions_size; size_t write_positions_capacity;
  size_t *point_indices;    size_t point_indices_size;   size_t point_indices_capacity;
} dwa_obstacle_spatial_index_t;

void dwa_obstacle_spatial_index_init(dwa_obstacle_spatial_index_t *index);
void dwa_obstacle_spatial_index_destroy(dwa_obstacle_spatial_index_t *index);

static inline bool dwa_spatial_index_valid(const dwa_obstacle_spatial_index_t *index)
{
  const size_t bucket_count = index->width > 0 && index->height > 0 ?
    (size_t)index->width * (size_t)index->height : (size_t)0U;
  return index->points != NULL && index->bucket_size > 0.0f && bucket_count > 0 &&
    index->offsets_size == bucket_count + 1U &&
    index->offsets[index->offsets_size - 1U] <= index->point_indices_size;
}

/*
 * visit_square 直译。visitor 返回 true 时立即返回 true（对应 C++ visitor 返回 true）。
 * user 为 visitor 上下文（替代 C++ lambda 捕获）。
 */
typedef bool (*dwa_point_visitor_t)(const dwa_point_t *point, void *user);

bool dwa_spatial_index_visit_square(
  const dwa_obstacle_spatial_index_t *index,
  float x,
  float y,
  float radius,
  dwa_point_visitor_t visitor,
  void *user);

/* build_obstacle_spatial_index 直译（内存跨周期复用语义保持）。 */
void dwa_build_obstacle_spatial_index(
  const dwa_point_t *points,
  size_t points_size,
  float bucket_size,
  dwa_obstacle_spatial_index_t *index);

#ifdef __cplusplus
}
#endif

#endif /* DWA_TYPES_H */
