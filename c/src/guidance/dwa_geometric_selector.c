/*
 * 模块功能：几何二次选轨迹实现。
 * 直译自 native/dwa_geometric_selector.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * lambda append_points / edge_for_direction / RankedCandidate 排序捕获
 * → 静态函数 + 上下文参数；std::sort → 带上下文的稳定归并排序。
 */
#include <stdlib.h>
#include <string.h>

#include "dwa_geometric_selector.h"
#include "../common/dwa_geometry.h"
#include "../planning/dwa_sampling.h"

static inline float gs_min(float a, float b) { return a < b ? a : b; }
static inline float gs_max(float a, float b) { return a > b ? a : b; }

dwa_geometric_selector_config_t dwa_geometric_selector_config_default(void)
{
  dwa_geometric_selector_config_t config;
  config.enabled = true;
  config.lookahead_distance = 0.50f;
  config.minimum_lookahead_distance = 0.15f;
  config.minimum_obstacle_forward_offset = 0.05f;
  config.cluster_gap_distance = 0.18f;
  config.cluster_forward_window = 0.40f;
  config.edge_front_band = 0.10f;
  config.cluster_track_tolerance = 0.25f;
  config.centered_entry_max_width = 0.45f;
  config.handoff_stable_cycles = 3;
  config.handoff_heading_agree = 12.0f * DWA_PI / 180.0f;
  config.minimum_candidate_clearance = 0.05f;
  config.maximum_clearance_loss = 0.08f;
  config.raw_override_yawrate = 0.30f;
  config.minimum_target_heading = 0.0f;
  config.maximum_heading_error = 10.0f * DWA_PI / 180.0f;
  config.manual_override_yawrate = 0.0f;
  config.target_filter_alpha = 0.45f;
  config.heading_tie_tolerance = 2.0f * DWA_PI / 180.0f;
  config.near_small_angle_distance = 0.50f;
  config.small_target_heading = 10.0f * DWA_PI / 180.0f;
  config.near_target_heading_compensation = 7.0f * DWA_PI / 180.0f;
  config.maximum_heading_growth = 8.0f * DWA_PI / 180.0f;
  return config;
}

bool dwa_validate_dwa_geometric_selector_config(
  const dwa_geometric_selector_config_t *config,
  const char **reason)
{
  const bool invalid =
    !isfinite(config->lookahead_distance) || config->lookahead_distance <= 0.0f ||
    !isfinite(config->minimum_lookahead_distance) ||
    config->minimum_lookahead_distance <= 0.0f ||
    config->minimum_lookahead_distance > config->lookahead_distance ||
    !isfinite(config->minimum_obstacle_forward_offset) ||
    config->minimum_obstacle_forward_offset < 0.0f ||
    !isfinite(config->cluster_gap_distance) ||
    config->cluster_gap_distance <= 0.0f ||
    !isfinite(config->cluster_forward_window) ||
    config->cluster_forward_window <= 0.0f ||
    !isfinite(config->edge_front_band) ||
    config->edge_front_band <= 0.0f ||
    !isfinite(config->cluster_track_tolerance) ||
    config->cluster_track_tolerance <= 0.0f ||
    !isfinite(config->centered_entry_max_width) ||
    config->centered_entry_max_width <= 0.0f ||
    config->handoff_stable_cycles < 1 ||
    !isfinite(config->handoff_heading_agree) ||
    config->handoff_heading_agree <= 0.0f ||
    config->handoff_heading_agree > DWA_PI ||
    !isfinite(config->minimum_candidate_clearance) ||
    config->minimum_candidate_clearance < 0.0f ||
    !isfinite(config->maximum_clearance_loss) ||
    config->maximum_clearance_loss < 0.0f ||
    !isfinite(config->raw_override_yawrate) ||
    config->raw_override_yawrate < 0.0f ||
    !isfinite(config->minimum_target_heading) ||
    config->minimum_target_heading < 0.0f ||
    config->minimum_target_heading >= 0.5f * DWA_PI ||
    !isfinite(config->maximum_heading_error) ||
    config->maximum_heading_error <= 0.0f ||
    config->maximum_heading_error > DWA_PI ||
    !isfinite(config->manual_override_yawrate) ||
    config->manual_override_yawrate < 0.0f ||
    !isfinite(config->target_filter_alpha) ||
    config->target_filter_alpha <= 0.0f || config->target_filter_alpha > 1.0f ||
    !isfinite(config->heading_tie_tolerance) ||
    config->heading_tie_tolerance <= 0.0f ||
    config->heading_tie_tolerance > config->maximum_heading_error ||
    !isfinite(config->near_small_angle_distance) ||
    config->near_small_angle_distance < 0.0f ||
    !isfinite(config->small_target_heading) ||
    config->small_target_heading < 0.0f ||
    config->small_target_heading >= 0.5f * DWA_PI ||
    !isfinite(config->near_target_heading_compensation) ||
    config->near_target_heading_compensation < 0.0f ||
    config->near_target_heading_compensation >= 0.5f * DWA_PI ||
    !isfinite(config->maximum_heading_growth) ||
    config->maximum_heading_growth < 0.0f ||
    config->maximum_heading_growth >= 0.5f * DWA_PI;
  if (invalid) {
    if (reason != NULL) *reason = "invalid DWA geometric selector configuration";
    return false;
  }
  return true;
}

void dwa_geometric_selection_reset(dwa_geometric_selection_t *selection)
{
  selection->applied = false;
  selection->geometry_valid = false;
  selection->obstacle_edge_valid = false;
  selection->cluster_valid = false;
  selection->handoff_ready = false;
  selection->tracking_active = false;
  selection->manual_reverse = false;
  selection->raw_base_direction = 0;
  selection->base_direction = 0;
  selection->requested_direction = 0;
  selection->eligible_count = 0;
  selection->obstacle_segment_size = 0;
  selection->edge_candidate_count = 0;
  selection->obstacle_segment_start = -1;
  selection->obstacle_segment_end = -1;
  selection->obstacle_edge_index = -1;
  selection->obstacle_from_left_scan = false;
  selection->cluster_point_count = 0;
  selection->stable_cycles = 0;
  selection->selected_index = (size_t)-1;
  selection->obstacle_point.x = 0.0f; selection->obstacle_point.y = 0.0f;
  selection->cluster_centroid.x = 0.0f; selection->cluster_centroid.y = 0.0f;
  selection->radar_point.x = 0.0f; selection->radar_point.y = 0.0f;
  selection->reference_origin.x = 0.0f; selection->reference_origin.y = 0.0f;
  selection->selected_lookahead_point.x = 0.0f;
  selection->selected_lookahead_point.y = 0.0f;
  selection->selected.v = 0.0f;
  selection->selected.w = 0.0f;
  /* selected_trajectory：默认构造语义，容量保留。 */
  {
    dwa_velocity_t zero;
    zero.v = 0.0f;
    zero.w = 0.0f;
    dwa_reset_trajectory(&selection->selected_trajectory, &zero);
  }
  selection->raw_target_heading = 0.0f;
  selection->target_heading = 0.0f;
  selection->target_heading_compensation = 0.0f;
  selection->selected_heading = 0.0f;
  selection->heading_error = INFINITY;
  selection->edge_radar_distance = INFINITY;
  selection->edge_front_clearance = INFINITY;
  selection->cluster_min_x = INFINITY;
  selection->cluster_max_x = -INFINITY;
  selection->cluster_min_y = INFINITY;
  selection->cluster_max_y = -INFINITY;
  selection->raw_heading = 0.0f;
  selection->raw_heading_error = INFINITY;
  selection->lookahead_distance = 0.0f;
  selection->target_heading_compensated = false;
  selection->reason = "disabled";
}

void dwa_geometric_selection_init(dwa_geometric_selection_t *selection)
{
  selection->selected_trajectory.states = NULL;
  selection->selected_trajectory.state_count = 0;
  selection->selected_trajectory.state_capacity = 0;
  dwa_geometric_selection_reset(selection);
}

void dwa_geometric_selection_destroy(dwa_geometric_selection_t *selection)
{
  dwa_trajectory_destroy(&selection->selected_trajectory);
}

void dwa_geometric_fused_obstacle_cluster_init(
  dwa_geometric_fused_obstacle_cluster_t *cluster)
{
  cluster->valid = false;
  cluster->points = NULL;
  cluster->points_size = 0;
  cluster->points_capacity = 0;
  cluster->centroid.x = 0.0f;
  cluster->centroid.y = 0.0f;
  cluster->min_x = INFINITY;
  cluster->max_x = -INFINITY;
  cluster->min_y = INFINITY;
  cluster->max_y = -INFINITY;
  cluster->nearest_front_x = INFINITY;
  cluster->front_clearance = INFINITY;
}

void dwa_geometric_fused_obstacle_cluster_destroy(
  dwa_geometric_fused_obstacle_cluster_t *cluster)
{
  free(cluster->points);
  cluster->points = NULL;
  cluster->points_size = 0;
  cluster->points_capacity = 0;
}

void dwa_update_geometric_cluster_bounds(
  dwa_geometric_fused_obstacle_cluster_t *cluster,
  const dwa_point_t *point)
{
  cluster->min_x = gs_min(cluster->min_x, point->x);
  cluster->max_x = gs_max(cluster->max_x, point->x);
  cluster->min_y = gs_min(cluster->min_y, point->y);
  cluster->max_y = gs_max(cluster->max_y, point->y);
  cluster->nearest_front_x = gs_min(cluster->nearest_front_x, point->x);
  cluster->centroid.x += point->x;
  cluster->centroid.y += point->y;
}

float dwa_geometric_front_trigger_half_width_at(
  float x,
  float front_trigger_start_x,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle)
{
  const float dx = gs_max(0.0f, x - front_trigger_start_x);
  return front_trigger_base_half_width + dx * tanf(front_trigger_expand_angle);
}

bool dwa_point_blocks_front_trigger(
  const dwa_point_t *point,
  float front_trigger_start_x,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle)
{
  const float half_width = dwa_geometric_front_trigger_half_width_at(
    point->x, front_trigger_start_x,
    front_trigger_base_half_width, front_trigger_expand_angle);
  return point->x >= front_trigger_start_x &&
    fabsf(point->y) <= half_width;
}

/* ---------------- 空间桶哈希（与 corridor_guide.c 相同的私有实现） ---------------- */

typedef struct {
  bool used;
  int bx;
  int by;
  size_t *items;
  size_t count;
  size_t capacity;
} gs_bucket_t;

typedef struct {
  gs_bucket_t *slots;
  size_t slot_count;
  size_t used_count;
} gs_bucket_map_t;

/* 9.24 性能修复：聚类桶表改为进程级持久缓冲。原实现每周期
 * calloc/free 桶表和桶内数组；现在只复位标记和计数，容量跨周期
 * 保留，仅在增长时 realloc。该函数只被主控制线程调用，无并发问题。 */
static gs_bucket_map_t gs_cluster_buckets;

static void gs_bucket_map_init(gs_bucket_map_t *map)
{
  if (map->slots == NULL) {
    map->slot_count = 16;
    map->used_count = 0;
    map->slots = (gs_bucket_t *)calloc(map->slot_count, sizeof(gs_bucket_t));
    return;
  }
  for (size_t i = 0; i < map->slot_count; ++i) {
    map->slots[i].used = false;
    map->slots[i].count = 0;
  }
  map->used_count = 0;
}

static size_t gs_bucket_hash(int bx, int by, size_t slot_count)
{
  unsigned int h = (unsigned int)bx * 73856093u ^ (unsigned int)by * 19349663u;
  return (size_t)(h & (unsigned int)(slot_count - 1));
}

static gs_bucket_t *gs_bucket_map_find(
  const gs_bucket_map_t *map, int bx, int by)
{
  size_t slot = gs_bucket_hash(bx, by, map->slot_count);
  for (size_t probe = 0; probe < map->slot_count; ++probe) {
    gs_bucket_t *bucket = &map->slots[(slot + probe) & (map->slot_count - 1)];
    if (!bucket->used) return NULL;
    if (bucket->bx == bx && bucket->by == by) return bucket;
  }
  return NULL;
}

static void gs_bucket_push(gs_bucket_t *bucket, size_t index)
{
  if (bucket->count == bucket->capacity) {
    size_t next = bucket->capacity > 0 ? bucket->capacity * 2 : 4;
    bucket->items = (size_t *)realloc(bucket->items, next * sizeof(size_t));
    bucket->capacity = next;
  }
  bucket->items[bucket->count++] = index;
}

static void gs_bucket_map_grow(gs_bucket_map_t *map)
{
  const size_t old_count = map->slot_count;
  gs_bucket_t *old_slots = map->slots;
  map->slot_count = old_count * 2;
  map->slots = (gs_bucket_t *)calloc(map->slot_count, sizeof(gs_bucket_t));
  for (size_t i = 0; i < old_count; ++i) {
    if (old_slots[i].used) {
      size_t slot = gs_bucket_hash(old_slots[i].bx, old_slots[i].by, map->slot_count);
      while (map->slots[slot].used) slot = (slot + 1) & (map->slot_count - 1);
      map->slots[slot] = old_slots[i];
    }
  }
  free(old_slots);
}

static gs_bucket_t *gs_bucket_map_insert(gs_bucket_map_t *map, int bx, int by)
{
  if (map->used_count * 10 >= map->slot_count * 7) {
    gs_bucket_map_grow(map);
  }
  size_t slot = gs_bucket_hash(bx, by, map->slot_count);
  for (size_t probe = 0; probe < map->slot_count; ++probe) {
    gs_bucket_t *bucket = &map->slots[(slot + probe) & (map->slot_count - 1)];
    if (!bucket->used) {
      bucket->used = true;
      bucket->bx = bx;
      bucket->by = by;
      bucket->count = 0; /* 复用上周期已分配的 items 容量 */
      ++map->used_count;
      return bucket;
    }
    if (bucket->bx == bx && bucket->by == by) return bucket;
  }
  return NULL;
}

/* ---------------- build_nearest_fused_front_cluster ---------------- */

static void gs_points_push(
  dwa_point_t **points, size_t *size, size_t *capacity, const dwa_point_t *point)
{
  if (*size == *capacity) {
    size_t next = *capacity > 0 ? *capacity * 2 : 16;
    *points = (dwa_point_t *)realloc(*points, next * sizeof(dwa_point_t));
    *capacity = next;
  }
  (*points)[(*size)++] = *point;
}

/* lambda append_points 捕获展开。 */
static void gs_append_points(
  const dwa_geometric_scan_point_t *scan_points,
  size_t scan_points_count,
  float front_trigger_start_x,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle,
  dwa_point_t **points,
  size_t *size,
  size_t *capacity)
{
  for (size_t i = 0; i < scan_points_count; ++i) {
    const dwa_geometric_scan_point_t *scan_point = &scan_points[i];
    /* 只用与原始DWA相同的前方触发区，避免侧墙和远处背景串成一个障碍簇。 */
    if (scan_point->valid && dwa_finite_geometric_point(&scan_point->point) &&
      dwa_point_blocks_front_trigger(
        &scan_point->point, front_trigger_start_x,
        front_trigger_base_half_width, front_trigger_expand_angle))
    {
      gs_points_push(points, size, capacity, &scan_point->point);
    }
  }
}

bool dwa_build_nearest_fused_front_cluster(
  const dwa_geometric_scan_point_t *left_scan_points,
  size_t left_scan_points_count,
  const dwa_geometric_scan_point_t *right_scan_points,
  size_t right_scan_points_count,
  float front_trigger_start_x,
  float clearance_front_edge,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle,
  const dwa_geometric_selector_config_t *config,
  dwa_geometric_fused_obstacle_cluster_t *selected_cluster,
  const char **reason)
{
  if (!isfinite(front_trigger_start_x) ||
    !isfinite(clearance_front_edge) ||
    !isfinite(front_trigger_base_half_width) ||
    front_trigger_base_half_width <= 0.0f ||
    !isfinite(front_trigger_expand_angle) ||
    front_trigger_expand_angle < 0.0f ||
    front_trigger_expand_angle >= 0.5f * DWA_PI)
  {
    if (reason != NULL) *reason = "invalid_cluster_input";
    return false;
  }

  /* 9.24 性能修复：points/visited/queue 为进程级持久缓冲，容量跨周期
   * 复用，每周期只复位计数；不再 malloc/free。 */
  static dwa_point_t *points = NULL;
  static size_t points_capacity = 0;
  size_t points_size = 0;
  gs_append_points(
    left_scan_points, left_scan_points_count, front_trigger_start_x,
    front_trigger_base_half_width, front_trigger_expand_angle,
    &points, &points_size, &points_capacity);
  gs_append_points(
    right_scan_points, right_scan_points_count, front_trigger_start_x,
    front_trigger_base_half_width, front_trigger_expand_angle,
    &points, &points_size, &points_capacity);
  if (points_size == 0) {
    if (reason != NULL) *reason = "no_fused_points";
    return false;
  }

  float nearest_front_x = INFINITY;
  for (size_t i = 0; i < points_size; ++i) {
    nearest_front_x = gs_min(nearest_front_x, points[i].x);
  }
  const float local_cluster_max_x = nearest_front_x + config->cluster_forward_window;
  /* points.erase(remove_if(...))：保序过滤。 */
  {
    size_t write = 0;
    for (size_t read = 0; read < points_size; ++read) {
      if (!(points[read].x > local_cluster_max_x)) {
        points[write++] = points[read];
      }
    }
    points_size = write;
  }
  if (points_size == 0) {
    if (reason != NULL) *reason = "no_local_front_points";
    return false;
  }

  const float gap_squared = config->cluster_gap_distance * config->cluster_gap_distance;
  gs_bucket_map_init(&gs_cluster_buckets);
  for (size_t index = 0; index < points_size; ++index) {
    const int bx = (int)floorf(points[index].x / config->cluster_gap_distance);
    const int by = (int)floorf(points[index].y / config->cluster_gap_distance);
    gs_bucket_t *bucket = gs_bucket_map_insert(&gs_cluster_buckets, bx, by);
    gs_bucket_push(bucket, index);
  }

  static char *visited = NULL;
  static size_t visited_capacity = 0;
  if (visited_capacity < points_size) {
    size_t next = visited_capacity > 0 ? visited_capacity : 64;
    while (next < points_size) next *= 2;
    visited = (char *)realloc(visited, next);
    visited_capacity = next;
  }
  memset(visited, 0, points_size);
  static size_t *queue = NULL;
  static size_t queue_capacity = 0;
  bool saw_front_cluster = false;
  for (size_t start = 0; start < points_size; ++start) {
    if (visited[start]) continue;
    dwa_geometric_fused_obstacle_cluster_t cluster;
    dwa_geometric_fused_obstacle_cluster_init(&cluster);
    size_t queue_size = 0;
    if (queue_capacity == 0) {
      queue_capacity = 16;
      queue = (size_t *)realloc(queue, queue_capacity * sizeof(size_t));
    }
    queue[queue_size++] = start;
    visited[start] = 1;
    for (size_t cursor = 0; cursor < queue_size; ++cursor) {
      const size_t index = queue[cursor];
      gs_points_push(&cluster.points, &cluster.points_size,
        &cluster.points_capacity, &points[index]);
      dwa_update_geometric_cluster_bounds(&cluster, &points[index]);
      const int center_bx = (int)floorf(points[index].x / config->cluster_gap_distance);
      const int center_by = (int)floorf(points[index].y / config->cluster_gap_distance);
      for (int offset_x = -1; offset_x <= 1; ++offset_x) {
        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
          const gs_bucket_t *bucket = gs_bucket_map_find(
            &gs_cluster_buckets, center_bx + offset_x, center_by + offset_y);
          if (bucket == NULL) continue;
          for (size_t k = 0; k < bucket->count; ++k) {
            const size_t next = bucket->items[k];
            if (visited[next]) continue;
            const float dx = points[next].x - points[index].x;
            const float dy = points[next].y - points[index].y;
            if (dx * dx + dy * dy <= gap_squared) {
              visited[next] = 1;
              if (queue_size == queue_capacity) {
                queue_capacity *= 2;
                queue = (size_t *)realloc(queue, queue_capacity * sizeof(size_t));
              }
              queue[queue_size++] = next;
            }
          }
        }
      }
    }

    if (cluster.points_size == 0) {
      dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
      continue;
    }
    cluster.centroid.x /= (float)cluster.points_size;
    cluster.centroid.y /= (float)cluster.points_size;

    float front_x = INFINITY;
    for (size_t i = 0; i < cluster.points_size; ++i) {
      if (dwa_point_blocks_front_trigger(
          &cluster.points[i], front_trigger_start_x,
          front_trigger_base_half_width, front_trigger_expand_angle))
      {
        front_x = gs_min(front_x, cluster.points[i].x);
      }
    }
    if (!isfinite(front_x)) {
      dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
      continue;
    }
    saw_front_cluster = true;
    cluster.valid = true;
    cluster.nearest_front_x = front_x;
    /* 触发区从雷达线开始，但净距仍以真实车体前缘为基准。 */
    cluster.front_clearance = gs_max(0.0f, front_x - clearance_front_edge);
    if (!selected_cluster->valid ||
      cluster.nearest_front_x + 1e-12f < selected_cluster->nearest_front_x ||
      (fabsf(cluster.nearest_front_x - selected_cluster->nearest_front_x) <= 1e-12f &&
      fabsf(cluster.centroid.y) < fabsf(selected_cluster->centroid.y)))
    {
      /* selected_cluster = std::move(cluster) */
      dwa_geometric_fused_obstacle_cluster_destroy(selected_cluster);
      *selected_cluster = cluster;
    } else {
      dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
    }
  }

  if (!selected_cluster->valid) {
    if (reason != NULL) {
      *reason = saw_front_cluster ? "no_selected_cluster" : "no_front_cluster";
    }
    return false;
  }
  if (reason != NULL) *reason = "selected_cluster";
  return true;
}

/* ---------------- select_cluster_edge_for_direction ---------------- */

dwa_geometric_obstacle_edge_t dwa_select_cluster_edge_for_direction(
  const dwa_geometric_fused_obstacle_cluster_t *cluster,
  const dwa_point_t *radar_point,
  int direction,
  float front_trigger_start_x,
  const dwa_geometric_selector_config_t *config)
{
  dwa_geometric_obstacle_edge_t edge = dwa_geometric_obstacle_edge_default();
  if (!cluster->valid || cluster->points_size == 0 || direction == 0 ||
    !dwa_finite_geometric_point(radar_point) || !isfinite(front_trigger_start_x))
  {
    edge.reason = "invalid_cluster_edge_input";
    return edge;
  }

  float nearest_candidate_x = INFINITY;
  for (size_t i = 0; i < cluster->points_size; ++i) {
    const dwa_point_t *candidate = &cluster->points[i];
    if (candidate->x >= front_trigger_start_x &&
      candidate->x - radar_point->x > config->minimum_obstacle_forward_offset)
    {
      nearest_candidate_x = gs_min(nearest_candidate_x, candidate->x);
    }
  }
  if (!isfinite(nearest_candidate_x)) {
    edge.reason = "no_cluster_edge_candidate";
    return edge;
  }

  const float edge_max_x = nearest_candidate_x + config->edge_front_band;
  for (size_t index = 0; index < cluster->points_size; ++index) {
    const dwa_point_t *candidate = &cluster->points[index];
    if (candidate->x < front_trigger_start_x ||
      candidate->x - radar_point->x <= config->minimum_obstacle_forward_offset ||
      candidate->x > edge_max_x)
    {
      continue;
    }
    ++edge.candidate_count;
    const float dx = candidate->x - radar_point->x;
    const float dy = candidate->y - radar_point->y;
    const float distance = hypotf(dx, dy);
    const bool farther_to_requested_side =
      direction > 0 ? candidate->y > edge.point.y : candidate->y < edge.point.y;
    const bool first = !edge.valid;
    const bool better_side = first || farther_to_requested_side;
    const bool same_side =
      !first && fabsf(candidate->y - edge.point.y) <= 1e-12f;
    const bool nearer_radar = distance + 1e-12f < edge.radar_distance;
    if (better_side || (same_side && nearer_radar)) {
      edge.valid = true;
      edge.point = *candidate;
      edge.edge_index = (int)index;
      edge.radar_distance = distance;
    }
  }

  if (!edge.valid) {
    edge.reason = "no_cluster_edge_candidate";
    return edge;
  }
  edge.segment_size = (int)cluster->points_size;
  edge.segment_start = 0;
  edge.segment_end = edge.segment_size - 1 > 0 ? edge.segment_size - 1 : 0;
  edge.front_clearance = cluster->front_clearance;
  edge.reason = "selected_cluster_edge";
  return edge;
}

dwa_point_t dwa_geometric_reference_point(
  const dwa_state_t *state,
  const dwa_point_t *reference)
{
  const float cos_yaw = cosf(state->yaw);
  const float sin_yaw = sinf(state->yaw);
  dwa_point_t result;
  result.x = state->x + cos_yaw * reference->x - sin_yaw * reference->y;
  result.y = state->y + sin_yaw * reference->x + cos_yaw * reference->y;
  return result;
}

bool dwa_geometric_heading_at_arc_distance(
  const dwa_trajectory_t *trajectory,
  const dwa_point_t *reference_origin,
  float lookahead_distance,
  float *heading,
  dwa_point_t *lookahead_point)
{
  if (trajectory->state_count == 0 || !dwa_finite_geometric_point(reference_origin) ||
    !isfinite(lookahead_distance) || lookahead_distance <= 0.0f)
  {
    return false;
  }

  const float linear = trajectory->command.v;
  const float angular = trajectory->command.w;
  if (!isfinite(linear) || !isfinite(angular) || linear <= 1e-6f) {
    return false;
  }

  float chord_x = lookahead_distance;
  float chord_y = 0.0f;
  if (fabsf(angular) > 1e-6f) {
    const float yaw = angular * lookahead_distance / linear;
    if (fabsf(yaw) > DWA_PI) return false;
    const float radius = linear / angular;
    chord_x = radius * sinf(yaw);
    chord_y = radius * (1.0f - cosf(yaw));
  }

  if (hypotf(chord_x, chord_y) <= 1e-9f) return false;
  *heading = atan2f(chord_y, chord_x);
  if (lookahead_point != NULL) {
    lookahead_point->x = reference_origin->x + chord_x;
    lookahead_point->y = reference_origin->y + chord_y;
  }
  return true;
}

/* ---------------- 类实现 ---------------- */

/* reset_handoff() 直译。 */
static void gs_reset_handoff(dwa_geometric_trajectory_selector_t *selector)
{
  selector->tracking_active = false;
  selector->handoff_committed = false;
  selector->centered_entry_active = false;
  selector->tracked_direction = 0;
  selector->stable_cycles = 0;
  selector->tracked_centroid.x = 0.0f;
  selector->tracked_centroid.y = 0.0f;
  selector->tracked_target_heading = 0.0f;
}

/* reset_geometry_target() 直译。 */
static void gs_reset_geometry_target(dwa_geometric_trajectory_selector_t *selector)
{
  selector->filtered_target_valid = false;
  selector->filtered_direction = 0;
  selector->filtered_target_heading = 0.0f;
  selector->filtered_target_compensated = false;
  gs_reset_handoff(selector);
}

/* tick（dwa_node_tick.c）在无有效轨迹时调用的公共包装。 */
void dwa_geometric_selector_reset_geometry_target(
  dwa_geometric_trajectory_selector_t *selector)
{
  gs_reset_geometry_target(selector);
}

/* update_handoff(cluster, requested_direction, target_heading) 直译。 */
static void gs_update_handoff(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_geometric_fused_obstacle_cluster_t *cluster,
  int requested_direction,
  float target_heading)
{
  const bool same_track = selector->tracking_active &&
    requested_direction == selector->tracked_direction &&
    hypotf(
      cluster->centroid.x - selector->tracked_centroid.x,
      cluster->centroid.y - selector->tracked_centroid.y) <=
      selector->config.cluster_track_tolerance;
  if (!same_track) {
    selector->tracking_active = true;
    selector->handoff_committed = false;
    selector->tracked_direction = requested_direction;
    selector->tracked_centroid = cluster->centroid;
    selector->tracked_target_heading = target_heading;
    selector->stable_cycles = 1;
    return;
  }

  const float target_delta = fabsf(dwa_normalize_angle(
    target_heading - selector->tracked_target_heading));
  selector->tracked_centroid = cluster->centroid;
  selector->tracked_target_heading = target_heading;
  selector->stable_cycles = target_delta <= selector->config.handoff_heading_agree ?
    selector->stable_cycles + 1 : 1;
}

void dwa_geometric_selector_init(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_geometric_selector_config_t *config)
{
  selector->config = *config;
  selector->filtered_target_valid = false;
  selector->filtered_direction = 0;
  selector->filtered_target_heading = 0.0f;
  selector->filtered_target_compensated = false;
  selector->tracking_active = false;
  selector->handoff_committed = false;
  selector->centered_entry_active = false;
  selector->tracked_direction = 0;
  selector->stable_cycles = 0;
  selector->tracked_centroid.x = 0.0f;
  selector->tracked_centroid.y = 0.0f;
  selector->tracked_target_heading = 0.0f;
}

void dwa_geometric_selector_init_default(dwa_geometric_trajectory_selector_t *selector)
{
  const dwa_geometric_selector_config_t config =
    dwa_geometric_selector_config_default();
  dwa_geometric_selector_init(selector, &config);
}

bool dwa_geometric_selector_set_config(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_geometric_selector_config_t *config,
  const char **reason)
{
  if (!dwa_validate_dwa_geometric_selector_config(config, reason)) return false;
  selector->config = *config;
  dwa_geometric_selector_reset(selector);
  return true;
}

void dwa_geometric_selector_reset(dwa_geometric_trajectory_selector_t *selector)
{
  gs_reset_geometry_target(selector);
}

/* lambda edge_for_direction(direction, &edge) 直译。 */
static bool gs_edge_for_direction(
  const dwa_geometric_selector_input_t *input,
  const dwa_geometric_selector_config_t *config,
  const dwa_geometric_fused_obstacle_cluster_t *cluster,
  int direction,
  dwa_geometric_obstacle_edge_t *edge)
{
  if (direction == 0) return false;
  const dwa_point_t *radar_point = direction > 0 ?
    &input->left_laser : &input->right_laser;
  *edge = dwa_select_cluster_edge_for_direction(
    cluster, radar_point, direction, input->front_trigger_start_x, config);
  return edge->valid;
}

/* RankedCandidate。 */
typedef struct {
  size_t index;
  float heading;
  dwa_point_t lookahead_point;
  float heading_error;
  float speed_change;
  float original_cost;
} gs_ranked_candidate_t;

/* RankedCandidate 比较器直译（捕获 config）。 */
static bool gs_ranked_less(
  const gs_ranked_candidate_t *left,
  const gs_ranked_candidate_t *right,
  const dwa_geometric_selector_config_t *config)
{
  const float error_delta = left->heading_error - right->heading_error;
  if (fabsf(error_delta) > config->heading_tie_tolerance) {
    return error_delta < 0.0f;
  }
  if (left->speed_change != right->speed_change) {
    return left->speed_change < right->speed_change;
  }
  if (left->original_cost != right->original_cost) {
    return left->original_cost < right->original_cost;
  }
  return left->index < right->index;
}

/* ---- std::sort 复刻（libstdc++ introsort）----
 * 原版用 std::sort 对 RankedCandidate 排序。比较器带 heading_tie_tolerance，
 * 近似相等不可传递（非严格弱序），排序结果依赖具体算法实现；
 * merge sort 与 std::sort 在这种比较器下会产生不同顺序。
 * 为保证与 C++ 基线 bit-exact，此处逐行复刻 libstdc++（gcc 11.2）的
 * std::__sort 全套算法：__introsort_loop（快排 + 深度限制转堆排）+
 * __final_insertion_sort（插入排序收尾）。_S_threshold = 16。 */

#define GS_SORT_THRESHOLD 16

static size_t gs_sort_lg(size_t n)
{
  size_t k = 0;
  while (n > 1) {
    n >>= 1;
    ++k;
  }
  return k;
}

/* __push_heap(first, holeIndex, topIndex, value, comp) 直译。 */
static void gs_push_heap(
  gs_ranked_candidate_t *first, size_t hole_index, size_t top_index,
  gs_ranked_candidate_t value,
  const dwa_geometric_selector_config_t *config)
{
  size_t parent = hole_index > 0 ? (hole_index - 1) / 2 : 0;
  while (hole_index > top_index &&
    gs_ranked_less(&first[parent], &value, config))
  {
    first[hole_index] = first[parent];
    hole_index = parent;
    parent = hole_index > 0 ? (hole_index - 1) / 2 : 0;
  }
  first[hole_index] = value;
}

/* __adjust_heap(first, holeIndex, len, value, comp) 直译。 */
static void gs_adjust_heap(
  gs_ranked_candidate_t *first, size_t hole_index, size_t len,
  gs_ranked_candidate_t value,
  const dwa_geometric_selector_config_t *config)
{
  const size_t top_index = hole_index;
  size_t second_child = hole_index;
  while (second_child < (len - 1) / 2) {
    second_child = 2 * (second_child + 1);
    if (gs_ranked_less(&first[second_child], &first[second_child - 1], config)) {
      --second_child;
    }
    first[hole_index] = first[second_child];
    hole_index = second_child;
  }
  if ((len & 1) == 0 && second_child == (len - 2) / 2) {
    second_child = 2 * (second_child + 1);
    first[hole_index] = first[second_child - 1];
    hole_index = second_child - 1;
  }
  gs_push_heap(first, hole_index, top_index, value, config);
}

/* __pop_heap(first, last, result, comp) 直译（result 为迭代器下标）。 */
static void gs_pop_heap(
  gs_ranked_candidate_t *first, size_t heap_last, size_t result,
  const dwa_geometric_selector_config_t *config)
{
  gs_ranked_candidate_t value = first[result];
  first[result] = first[0];
  gs_adjust_heap(first, 0, heap_last, value, config);
}

/* __make_heap(first, last, comp) 直译。 */
static void gs_make_heap(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  if (len < 2) return;
  size_t parent = (len - 2) / 2;
  while (true) {
    gs_ranked_candidate_t value = first[parent];
    gs_adjust_heap(first, parent, len, value, config);
    if (parent == 0) return;
    --parent;
  }
}

/* __sort_heap(first, last, comp) 直译。 */
static void gs_sort_heap(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  while (len > 1) {
    --len;
    gs_pop_heap(first, len, len, config);
  }
}

/* __move_median_to_first(result, a, b, c, comp) 直译。 */
static void gs_move_median_to_first(
  gs_ranked_candidate_t *first, size_t result,
  size_t a, size_t b, size_t c,
  const dwa_geometric_selector_config_t *config)
{
  if (gs_ranked_less(&first[a], &first[b], config)) {
    if (gs_ranked_less(&first[b], &first[c], config)) {
      gs_ranked_candidate_t tmp = first[result];
      first[result] = first[b];
      first[b] = tmp;
    } else if (gs_ranked_less(&first[a], &first[c], config)) {
      gs_ranked_candidate_t tmp = first[result];
      first[result] = first[c];
      first[c] = tmp;
    } else {
      gs_ranked_candidate_t tmp = first[result];
      first[result] = first[a];
      first[a] = tmp;
    }
  } else if (gs_ranked_less(&first[a], &first[c], config)) {
    gs_ranked_candidate_t tmp = first[result];
    first[result] = first[a];
    first[a] = tmp;
  } else if (gs_ranked_less(&first[b], &first[c], config)) {
    gs_ranked_candidate_t tmp = first[result];
    first[result] = first[c];
    first[c] = tmp;
  } else {
    gs_ranked_candidate_t tmp = first[result];
    first[result] = first[b];
    first[b] = tmp;
  }
}

/* __unguarded_partition(first, last, pivot, comp) 直译；返回切分点。
 * first/last/pivot 均为绝对下标（C++ 侧 first 传入的是 begin+1）。 */
static size_t gs_unguarded_partition(
  gs_ranked_candidate_t *base, size_t first, size_t last, size_t pivot,
  const dwa_geometric_selector_config_t *config)
{
  while (true) {
    while (gs_ranked_less(&base[first], &base[pivot], config)) ++first;
    --last;
    while (gs_ranked_less(&base[pivot], &base[last], config)) --last;
    if (!(first < last)) return first;
    gs_ranked_candidate_t tmp = base[first];
    base[first] = base[last];
    base[last] = tmp;
    ++first;
  }
}

/* __unguarded_partition_pivot(first, last, comp) 直译。 */
static size_t gs_unguarded_partition_pivot(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  const size_t mid = len / 2;
  gs_move_median_to_first(first, 0, 1, mid, len - 1, config);
  /* C++: __unguarded_partition(__first + 1, __last, __first, __comp) */
  return gs_unguarded_partition(first, 1, len, 0, config);
}

/* __introsort_loop(first, last, depth_limit, comp) 直译。 */
static void gs_introsort_loop(
  gs_ranked_candidate_t *first, size_t len, size_t depth_limit,
  const dwa_geometric_selector_config_t *config)
{
  while (len > GS_SORT_THRESHOLD) {
    if (depth_limit == 0) {
      /* __partial_sort(first, last, last)：make_heap + sort_heap。 */
      gs_make_heap(first, len, config);
      gs_sort_heap(first, len, config);
      return;
    }
    --depth_limit;
    const size_t cut = gs_unguarded_partition_pivot(first, len, config);
    gs_introsort_loop(first + cut, len - cut, depth_limit, config);
    len = cut;
  }
}

/* __unguarded_linear_insert(last, comp) 直译。 */
static void gs_unguarded_linear_insert(
  gs_ranked_candidate_t *first, size_t last,
  const dwa_geometric_selector_config_t *config)
{
  gs_ranked_candidate_t value = first[last];
  size_t next = last - 1;
  while (gs_ranked_less(&value, &first[next], config)) {
    first[last] = first[next];
    last = next;
    --next;
  }
  first[last] = value;
}

/* __insertion_sort(first, last, comp) 直译。 */
static void gs_insertion_sort(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  if (len == 0) return;
  for (size_t i = 1; i < len; ++i) {
    if (gs_ranked_less(&first[i], &first[0], config)) {
      /* val = *i; move_backward(first, i, i+1); *first = val; */
      gs_ranked_candidate_t value = first[i];
      for (size_t k = i; k > 0; --k) {
        first[k] = first[k - 1];
      }
      first[0] = value;
    } else {
      gs_unguarded_linear_insert(first, i, config);
    }
  }
}

/* __final_insertion_sort(first, last, comp) 直译。 */
static void gs_final_insertion_sort(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  if (len > GS_SORT_THRESHOLD) {
    gs_insertion_sort(first, GS_SORT_THRESHOLD, config);
    for (size_t i = GS_SORT_THRESHOLD; i < len; ++i) {
      gs_unguarded_linear_insert(first, i, config);
    }
  } else {
    gs_insertion_sort(first, len, config);
  }
}

/* std::__sort(first, last, comp) 直译：即 std::sort 的 libstdc++ 实现。 */
static void gs_std_sort(
  gs_ranked_candidate_t *first, size_t len,
  const dwa_geometric_selector_config_t *config)
{
  if (len != 0) {
    gs_introsort_loop(first, len, gs_sort_lg(len) * 2, config);
    gs_final_insertion_sort(first, len, config);
  }
}

void dwa_geometric_selector_select(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_plan_result_t *plan,
  const dwa_geometric_selector_input_t *input,
  dwa_geometric_selection_t *selection)
{
  const dwa_geometric_selector_config_t *config = &selector->config;
  dwa_geometric_selection_reset(selection);
  selection->raw_base_direction = input->base_direction > 0 ? 1 :
    (input->base_direction < 0 ? -1 : 0);
  selection->base_direction = selection->raw_base_direction;
  selection->reference_origin.x =
    0.5f * (input->left_laser.x + input->right_laser.x);
  selection->reference_origin.y =
    0.5f * (input->left_laser.y + input->right_laser.y);
  selection->lookahead_distance = config->lookahead_distance;

  if (!config->enabled) {
    gs_reset_geometry_target(selector);
    selection->reason = "disabled";
    return;
  }
  if (!plan->selected_trajectory.valid) {
    gs_reset_geometry_target(selector);
    selection->reason = "no_selected_trajectory";
    return;
  }
  if (input->left_scan_points == NULL || input->right_scan_points == NULL ||
    !dwa_finite_geometric_point(&input->left_laser) ||
    !dwa_finite_geometric_point(&input->right_laser) ||
    !isfinite(input->front_trigger_start_x) ||
    !isfinite(input->clearance_front_edge) ||
    !isfinite(input->front_trigger_base_half_width) ||
    input->front_trigger_base_half_width <= 0.0f ||
    !isfinite(input->front_trigger_expand_angle) ||
    input->front_trigger_expand_angle < 0.0f ||
    input->front_trigger_expand_angle >= 0.5f * DWA_PI)
  {
    gs_reset_geometry_target(selector);
    selection->reason = "invalid_geometry_input";
    return;
  }

  /* 手柄反向只使用控制器死区判断，避免额外阈值挡住对称反转。 */
  const int manual_direction = dwa_turn_direction(
    input->intent.w, input->direction_epsilon);
  dwa_geometric_fused_obstacle_cluster_t cluster;
  dwa_geometric_fused_obstacle_cluster_init(&cluster);
  const char *cluster_reason = NULL;
  bool cluster_built = false;
  {
    /* C++ 里 cluster 为函数局部默认构造；此处直接构建到局部再转移。 */
    dwa_geometric_fused_obstacle_cluster_t built;
    dwa_geometric_fused_obstacle_cluster_init(&built);
    cluster_built = dwa_build_nearest_fused_front_cluster(
      input->left_scan_points, input->left_scan_points_count,
      input->right_scan_points, input->right_scan_points_count,
      input->front_trigger_start_x, input->clearance_front_edge,
      input->front_trigger_base_half_width, input->front_trigger_expand_angle,
      config, &built, &cluster_reason);
    if (cluster_built) {
      cluster = built; /* 所有权转移 */
    } else {
      dwa_geometric_fused_obstacle_cluster_destroy(&built);
    }
  }
  if (!cluster_built) {
    gs_reset_geometry_target(selector);
    selection->reason = cluster_reason;
    return;
  }
  selection->cluster_valid = true;
  selection->cluster_centroid = cluster.centroid;
  selection->cluster_point_count = (int)cluster.points_size;
  selection->cluster_min_x = cluster.min_x;
  selection->cluster_max_x = cluster.max_x;
  selection->cluster_min_y = cluster.min_y;
  selection->cluster_max_y = cluster.max_y;
  const float cluster_center_y = 0.5f * (cluster.min_y + cluster.max_y);
  const float cluster_width = gs_max(0.0f, cluster.max_y - cluster.min_y);
  const bool centered_cluster =
    manual_direction == 0 &&
    fabsf(cluster_center_y) <= 0.50f * input->front_trigger_base_half_width &&
    cluster_width <= config->centered_entry_max_width;
  const bool same_tracked_cluster =
    selector->centered_entry_active &&
    selector->tracking_active &&
    hypotf(
      cluster.centroid.x - selector->tracked_centroid.x,
      cluster.centroid.y - selector->tracked_centroid.y) <=
      config->cluster_track_tolerance;
  if (same_tracked_cluster && manual_direction == 0) {
    selection->base_direction = selector->tracked_direction;
  }

  dwa_geometric_obstacle_edge_t preselected_edge =
    dwa_geometric_obstacle_edge_default();
  int preselected_direction = 0;
  bool has_preselected_edge = false;
  if (selection->base_direction == 0) {
    if (manual_direction != 0) {
      preselected_direction = manual_direction;
      has_preselected_edge = gs_edge_for_direction(
        input, config, &cluster, preselected_direction, &preselected_edge);
      if (!has_preselected_edge) {
        gs_reset_geometry_target(selector);
        selection->reason = preselected_edge.reason;
        dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
        return;
      }
    } else {
      dwa_geometric_obstacle_edge_t left_edge = dwa_geometric_obstacle_edge_default();
      dwa_geometric_obstacle_edge_t right_edge = dwa_geometric_obstacle_edge_default();
      const bool left_valid = gs_edge_for_direction(input, config, &cluster, 1, &left_edge);
      const bool right_valid = gs_edge_for_direction(input, config, &cluster, -1, &right_edge);
      if (!left_valid && !right_valid) {
        gs_reset_geometry_target(selector);
        selection->reason = left_edge.reason;
        dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
        return;
      }

      const float left_angle = left_valid ? fabsf(atan2f(
        left_edge.point.y - input->left_laser.y,
        left_edge.point.x - input->left_laser.x)) :
        INFINITY;
      const float right_angle = right_valid ? fabsf(atan2f(
        right_edge.point.y - input->right_laser.y,
        right_edge.point.x - input->right_laser.x)) :
        INFINITY;
      if (centered_cluster &&
        gs_min(left_angle, right_angle) < config->small_target_heading)
      {
        gs_reset_geometry_target(selector);
        selection->reason = "raw_dwa_wait_direction_center";
        dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
        return;
      }

      const bool choose_left = left_valid && (!right_valid ||
        left_edge.radar_distance < right_edge.radar_distance);
      preselected_direction = choose_left ? 1 : -1;
      preselected_edge = choose_left ? left_edge : right_edge;
      has_preselected_edge = true;
    }
    selection->base_direction = preselected_direction;
  }
  if (selection->base_direction == 0) {
    gs_reset_geometry_target(selector);
    selection->reason = "no_base_direction";
    dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
    return;
  }
  /* 同向手柄不改变大方向，只有明确反向输入才切换候选集合。 */
  selection->manual_reverse = selection->raw_base_direction != 0 &&
    manual_direction != 0 &&
    manual_direction != selection->base_direction;
  selection->requested_direction = selection->manual_reverse ?
    manual_direction : selection->base_direction;

  /* 手柄明确反向时，目标边缘点和候选轨迹必须同步切换到新的绕行侧。 */
  selection->radar_point = selection->requested_direction > 0 ?
    input->left_laser : input->right_laser;
  selection->obstacle_from_left_scan = selection->requested_direction > 0;

  dwa_geometric_obstacle_edge_t edge;
  if (has_preselected_edge && preselected_direction == selection->requested_direction) {
    edge = preselected_edge;
  } else {
    edge = dwa_select_cluster_edge_for_direction(
      &cluster, &selection->radar_point, selection->requested_direction,
      input->front_trigger_start_x, config);
  }
  selection->obstacle_segment_size = edge.segment_size;
  selection->edge_candidate_count = edge.candidate_count;
  selection->obstacle_segment_start = edge.segment_start;
  selection->obstacle_segment_end = edge.segment_end;
  selection->obstacle_edge_index = edge.edge_index;
  selection->edge_radar_distance = edge.radar_distance;
  selection->edge_front_clearance = edge.front_clearance;
  if (!edge.valid) {
    gs_reset_geometry_target(selector);
    selection->reason = edge.reason;
    dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
    return;
  }
  selection->obstacle_point = edge.point;
  selection->obstacle_edge_valid = true;
  const float obstacle_clearance =
    gs_max(0.0f, selection->obstacle_point.x - input->clearance_front_edge);
  const float adaptive_lookahead = dwa_clamp_value(
    0.75f * obstacle_clearance,
    config->minimum_lookahead_distance,
    config->lookahead_distance);
  selection->lookahead_distance = adaptive_lookahead;

  const float edge_x = selection->obstacle_point.x - selection->radar_point.x;
  const float edge_y = selection->obstacle_point.y - selection->radar_point.y;
  const float raw_unsigned_target = fabsf(atan2f(edge_y, edge_x));
  if (raw_unsigned_target < config->minimum_target_heading) {
    gs_reset_geometry_target(selector);
    selection->reason = "target_heading_too_small";
    dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
    return;
  }

  const float compensation_clearance = gs_min(obstacle_clearance, edge.front_clearance);
  const bool near_small_angle =
    isfinite(compensation_clearance) &&
    compensation_clearance <= config->near_small_angle_distance &&
    raw_unsigned_target < config->small_target_heading &&
    config->near_target_heading_compensation > 0.0f;
  const bool centered_small_angle_case =
    centered_cluster &&
    !selection->manual_reverse &&
    raw_unsigned_target < config->small_target_heading;
  const bool continue_centered_entry =
    selector->centered_entry_active &&
    selector->tracking_active &&
    selection->requested_direction == selector->tracked_direction &&
    hypotf(
      cluster.centroid.x - selector->tracked_centroid.x,
      cluster.centroid.y - selector->tracked_centroid.y) <=
      config->cluster_track_tolerance;
  const bool raw_entry_started =
    fabsf(plan->selected.w) >= config->raw_override_yawrate &&
    dwa_turn_direction(plan->selected.w, input->direction_epsilon) ==
    selection->requested_direction;
  const bool start_centered_entry =
    centered_small_angle_case && raw_entry_started;
  const bool use_centered_entry_handoff =
    start_centered_entry || continue_centered_entry;
  selection->raw_target_heading = selection->requested_direction * raw_unsigned_target;
  selection->target_heading_compensated = near_small_angle;
  selection->target_heading_compensation = near_small_angle ?
    selection->requested_direction * config->near_target_heading_compensation : 0.0f;
  const float unfiltered_target_heading = dwa_normalize_angle(
    selection->raw_target_heading + selection->target_heading_compensation);
  if (!selector->filtered_target_valid ||
    selector->filtered_direction != selection->requested_direction)
  {
    selector->filtered_target_heading = unfiltered_target_heading;
    selector->filtered_target_valid = true;
    selector->filtered_direction = selection->requested_direction;
    selector->filtered_target_compensated = near_small_angle;
  } else if (selector->filtered_target_compensated != near_small_angle) {
    selector->filtered_target_heading = unfiltered_target_heading;
    selector->filtered_target_compensated = near_small_angle;
  } else {
    const float delta = dwa_normalize_angle(
      unfiltered_target_heading - selector->filtered_target_heading);
    selector->filtered_target_heading = dwa_normalize_angle(
      selector->filtered_target_heading + config->target_filter_alpha * delta);
  }
  selection->target_heading = selector->filtered_target_heading;
  selection->geometry_valid = true;

  float raw_heading = 0.0f;
  dwa_point_t raw_lookahead_point;
  raw_lookahead_point.x = 0.0f;
  raw_lookahead_point.y = 0.0f;
  const bool raw_heading_valid = dwa_geometric_heading_at_arc_distance(
    &plan->selected_trajectory, &selection->reference_origin, adaptive_lookahead,
    &raw_heading, &raw_lookahead_point);
  if (raw_heading_valid) {
    selection->raw_heading = raw_heading;
    selection->raw_heading_error = fabsf(dwa_normalize_angle(
      raw_heading - selection->target_heading));
  }
  /* 二次选择只允许在原始DWA弦方向附近微调，避免几何目标把转弯放大成急转。 */
  const float allowed_heading_growth = near_small_angle ?
    config->near_target_heading_compensation + config->maximum_heading_growth :
    config->maximum_heading_growth;
  const float maximum_candidate_heading = raw_heading_valid ?
    gs_min(DWA_PI, fabsf(raw_heading) + allowed_heading_growth) :
    DWA_PI;

  /* 9.24 性能修复：ranked 为进程级持久缓冲，容量跨周期复用。 */
  static gs_ranked_candidate_t *ranked = NULL;
  static size_t ranked_capacity = 0;
  size_t ranked_size = 0;
  const float raw_clearance = plan->selected_trajectory.min_clearance;
  const float required_clearance = gs_max(
    config->minimum_candidate_clearance,
    isfinite(raw_clearance) ?
    raw_clearance - config->maximum_clearance_loss :
    config->minimum_candidate_clearance);

  for (size_t index = 0; index < plan->trajectory_count; ++index) {
    const dwa_trajectory_t *trajectory = &plan->trajectories[index];
    if (!trajectory->valid || trajectory->command.v <= 1e-6f ||
      trajectory->min_clearance < required_clearance ||
      dwa_turn_direction(trajectory->command.w, input->direction_epsilon) !=
      selection->requested_direction)
    {
      continue;
    }

    float heading = 0.0f;
    dwa_point_t lookahead_point;
    lookahead_point.x = 0.0f;
    lookahead_point.y = 0.0f;
    if (!dwa_geometric_heading_at_arc_distance(
        trajectory, &selection->reference_origin, adaptive_lookahead,
        &heading, &lookahead_point))
    {
      continue;
    }
    if (raw_heading_valid && fabsf(heading) > maximum_candidate_heading) {
      continue;
    }

    if (use_centered_entry_handoff) {
      /* trajectory.states.back()：generate_trajectory 保证 states >= 1。 */
      const dwa_point_t final_reference = dwa_geometric_reference_point(
        &trajectory->states[trajectory->state_count - 1],
        &selection->reference_origin);
      if (final_reference.x + 1e-12f < cluster.max_x) {
        continue;
      }
    }

    gs_ranked_candidate_t candidate;
    candidate.index = index;
    candidate.heading = heading;
    candidate.lookahead_point = lookahead_point;
    candidate.heading_error = fabsf(dwa_normalize_angle(
      heading - selection->target_heading));
    if (candidate.heading_error < selection->heading_error) {
      selection->selected_heading = candidate.heading;
      selection->selected_lookahead_point = candidate.lookahead_point;
      selection->heading_error = candidate.heading_error;
    }
    if (candidate.heading_error > config->maximum_heading_error) {
      continue;
    }
    candidate.speed_change = fabsf(trajectory->command.v - plan->selected.v);
    candidate.original_cost = trajectory->total_cost;
    if (ranked_size == ranked_capacity) {
      ranked_capacity = ranked_capacity > 0 ? ranked_capacity * 2 : 16;
      ranked = (gs_ranked_candidate_t *)realloc(
        ranked, ranked_capacity * sizeof(gs_ranked_candidate_t));
    }
    ranked[ranked_size++] = candidate;
  }

  selection->eligible_count = (int)ranked_size;
  if (ranked_size == 0) {
    gs_reset_handoff(selector);
    selection->reason = isfinite(selection->heading_error) ?
      "no_candidate_within_heading_gate" : "no_candidate_reaches_lookahead";
    dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
    return;
  }

  /* std::sort(ranked.begin(), ranked.end(), comp) —— libstdc++ 复刻。 */
  gs_std_sort(ranked, ranked_size, config);

  {
    const gs_ranked_candidate_t *candidate = &ranked[0];
    const dwa_trajectory_t *trajectory = &plan->trajectories[candidate->index];
    if (centered_small_angle_case && !use_centered_entry_handoff) {
      gs_reset_handoff(selector);
      selection->reason = "raw_dwa_wait_turn_center";
      dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
      return;
    }
    if (selector->centered_entry_active && !continue_centered_entry &&
      !start_centered_entry)
    {
      gs_reset_handoff(selector);
      selection->reason = "raw_dwa_center_cluster_lost";
      dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
      return;
    }
    if (use_centered_entry_handoff) {
      selector->centered_entry_active = true;
      gs_update_handoff(selector, &cluster, selection->requested_direction,
        selection->target_heading);
      selection->stable_cycles = selector->stable_cycles;
      selection->tracking_active = selector->tracking_active;
      const bool raw_same_direction =
        dwa_turn_direction(plan->selected.w, input->direction_epsilon) ==
        selection->requested_direction;
      const bool heading_agrees =
        raw_heading_valid &&
        selection->raw_heading_error <= config->handoff_heading_agree;
      if (!selector->handoff_committed) {
        selector->handoff_committed =
          selector->stable_cycles >= config->handoff_stable_cycles &&
          raw_same_direction && heading_agrees;
      }
      selection->handoff_ready = selector->handoff_committed;
      if (!selection->handoff_ready) {
        selection->reason = raw_heading_valid ? "raw_dwa_entry_center" :
          "raw_heading_unavailable";
        dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
        return;
      }
    } else {
      gs_reset_handoff(selector);
      selection->handoff_ready = true;
    }
    selection->applied = true;
    selection->reason = "selected";
    selection->selected_index = candidate->index;
    selection->selected = trajectory->command;
    dwa_trajectory_copy(&selection->selected_trajectory, trajectory);
    selection->selected_heading = candidate->heading;
    selection->selected_lookahead_point = candidate->lookahead_point;
    selection->heading_error = candidate->heading_error;
  }
  dwa_geometric_fused_obstacle_cluster_destroy(&cluster);
}
