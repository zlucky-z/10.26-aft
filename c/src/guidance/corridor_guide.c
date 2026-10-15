/*
 * 模块功能：通道检测与跟踪实现。
 * 直译自 native/guidance/corridor_types.hpp + native/dwa_corridor_guide.hpp
 * （逐行对应，不改逻辑/默认值/执行顺序）。
 * std::map<pair<int,int>, vector<size_t>>（空间桶，只按 key 查找，从不遍历）
 * → 开放寻址哈希表（桶内保持插入顺序，与 C++ vector 一致）。
 */
#include <stdlib.h>
#include <string.h>

#include "corridor_guide.h"
#include "../common/dwa_geometry.h"

static inline float cg_min(float a, float b) { return a < b ? a : b; }
static inline float cg_max(float a, float b) { return a > b ? a : b; }

/* ---------------- corridor_types ---------------- */

dwa_corridor_guide_config_t dwa_corridor_guide_config_default(void)
{
  dwa_corridor_guide_config_t config;
  config.enabled = true;
  config.search_start_x = 0.25f;
  config.search_end_x = 1.80f;
  config.minimum_width_margin = 0.04f;
  config.maximum_width = 1.30f;
  config.search_half_width = 1.20f;
  config.approach_start_x = 0.25f;
  config.cluster_gap_distance = 0.14f;
  config.cluster_min_points = 3;
  config.gate_max_depth_difference = 0.50f;
  config.gate_blocking_clearance = 0.10f;
  config.gate_endpoint_exclusion = 0.12f;
  config.gate_dedup_distance = 0.16f;
  config.gate_projection_length = 0.75f;
  config.explicit_intent_ratio = 0.15f;
  config.maximum_heading = 60.0f * DWA_PI / 180.0f;
  config.intent_lateral_range = 0.70f;
  config.stable_cycles = 3;
  config.lost_hold_cycles = 4;
  config.track_lateral_tolerance = 0.35f;
  config.track_heading_tolerance = 25.0f * DWA_PI / 180.0f;
  config.path_filter_alpha = 0.35f;
  config.trajectory_lookahead = 1.20f;
  config.lateral_error_weight = 2.00f;
  config.heading_error_weight = 0.70f;
  config.progress_weight = 0.35f;
  return config;
}

bool dwa_validate_dwa_corridor_guide_config(
  const dwa_corridor_guide_config_t *config,
  const char **reason)
{
  /* 原代码为一个长 || 链；失败原因固定一条。 */
  const bool invalid =
    !isfinite(config->search_start_x) || config->search_start_x < 0.0f ||
    !isfinite(config->search_end_x) || config->search_end_x <= config->search_start_x ||
    !isfinite(config->minimum_width_margin) || config->minimum_width_margin < 0.0f ||
    !isfinite(config->maximum_width) || config->maximum_width <= 0.0f ||
    !isfinite(config->search_half_width) || config->search_half_width <= 0.0f ||
    !isfinite(config->approach_start_x) || config->approach_start_x < 0.0f ||
    config->approach_start_x >= config->search_end_x ||
    !isfinite(config->cluster_gap_distance) || config->cluster_gap_distance <= 0.0f ||
    config->cluster_min_points < 2 ||
    !isfinite(config->gate_max_depth_difference) ||
    config->gate_max_depth_difference <= 0.0f ||
    !isfinite(config->gate_blocking_clearance) ||
    config->gate_blocking_clearance < 0.0f ||
    !isfinite(config->gate_endpoint_exclusion) ||
    config->gate_endpoint_exclusion < 0.0f ||
    !isfinite(config->gate_dedup_distance) || config->gate_dedup_distance <= 0.0f ||
    !isfinite(config->gate_projection_length) ||
    config->gate_projection_length <= 0.0f ||
    !isfinite(config->explicit_intent_ratio) || config->explicit_intent_ratio < 0.0f ||
    config->explicit_intent_ratio > 1.0f ||
    !isfinite(config->maximum_heading) || config->maximum_heading <= 0.0f ||
    config->maximum_heading >= 0.5f * DWA_PI ||
    !isfinite(config->intent_lateral_range) || config->intent_lateral_range < 0.0f ||
    config->stable_cycles < 1 ||
    config->lost_hold_cycles < 0 ||
    !isfinite(config->track_lateral_tolerance) ||
    config->track_lateral_tolerance <= 0.0f ||
    !isfinite(config->track_heading_tolerance) ||
    config->track_heading_tolerance <= 0.0f ||
    !isfinite(config->path_filter_alpha) || config->path_filter_alpha <= 0.0f ||
    config->path_filter_alpha > 1.0f ||
    !isfinite(config->trajectory_lookahead) ||
    config->trajectory_lookahead <= 0.0f ||
    !isfinite(config->lateral_error_weight) || config->lateral_error_weight < 0.0f ||
    !isfinite(config->heading_error_weight) || config->heading_error_weight < 0.0f ||
    !isfinite(config->progress_weight) || config->progress_weight < 0.0f;
  if (invalid) {
    if (reason != NULL) *reason = "invalid DWA corridor guide configuration";
    return false;
  }
  return true;
}

/* ---------------- DwaCorridorGuidance 动态数组 ---------------- */

static void cg_slices_reserve(
  dwa_corridor_guidance_t *guidance, size_t capacity)
{
  if (guidance->slices_capacity < capacity) {
    size_t next = guidance->slices_capacity > 0 ? guidance->slices_capacity : 8;
    while (next < capacity) next *= 2;
    dwa_corridor_slice_t *grown =
      (dwa_corridor_slice_t *)realloc(guidance->slices, next * sizeof(*grown));
    guidance->slices = grown;
    guidance->slices_capacity = next;
  }
}

static void cg_slices_push_back(
  dwa_corridor_guidance_t *guidance, const dwa_corridor_slice_t *slice)
{
  cg_slices_reserve(guidance, guidance->slices_size + 1);
  guidance->slices[guidance->slices_size++] = *slice;
}

static void cg_gates_reserve(
  dwa_corridor_guidance_t *guidance, size_t capacity)
{
  if (guidance->candidate_gates_capacity < capacity) {
    size_t next = guidance->candidate_gates_capacity > 0 ?
      guidance->candidate_gates_capacity : 8;
    while (next < capacity) next *= 2;
    dwa_corridor_slice_t *grown =
      (dwa_corridor_slice_t *)realloc(guidance->candidate_gates, next * sizeof(*grown));
    guidance->candidate_gates = grown;
    guidance->candidate_gates_capacity = next;
  }
}

static void cg_gates_push_back(
  dwa_corridor_guidance_t *guidance, const dwa_corridor_slice_t *gate)
{
  cg_gates_reserve(guidance, guidance->candidate_gates_size + 1);
  guidance->candidate_gates[guidance->candidate_gates_size++] = *gate;
}

void dwa_corridor_guidance_reset(dwa_corridor_guidance_t *guidance)
{
  guidance->detected = false;
  guidance->active = false;
  guidance->held = false;
  guidance->stable_cycles = 0;
  guidance->lost_cycles = 0;
  guidance->slice_count = 0;
  guidance->candidate_count = 0;
  guidance->cluster_count = 0;
  guidance->raw_gate_count = 0;
  guidance->blocked_gate_count = 0;
  guidance->reject_width = 0;
  guidance->reject_heading = 0;
  guidance->reject_depth = 0;
  guidance->reject_blocked = 0;
  guidance->entry_x = INFINITY;
  guidance->entry_y = 0.0f;
  guidance->entry_distance = INFINITY;
  guidance->minimum_width = 0.0f;
  guidance->target_y = 0.0f;
  guidance->target_heading = 0.0f;
  guidance->slices_size = 0;            /* vector 清空但保留容量 */
  guidance->candidate_gates_size = 0;
  guidance->reason = "disabled";        /* 成员初始化器 reason = "disabled" */
}

void dwa_corridor_guidance_destroy(dwa_corridor_guidance_t *guidance)
{
  free(guidance->slices);
  guidance->slices = NULL;
  guidance->slices_size = 0;
  guidance->slices_capacity = 0;
  free(guidance->candidate_gates);
  guidance->candidate_gates = NULL;
  guidance->candidate_gates_size = 0;
  guidance->candidate_gates_capacity = 0;
}

void dwa_corridor_guidance_copy(
  dwa_corridor_guidance_t *dst, const dwa_corridor_guidance_t *src)
{
  cg_slices_reserve(dst, src->slices_size);
  if (src->slices_size > 0) {
    memcpy(dst->slices, src->slices, src->slices_size * sizeof(dwa_corridor_slice_t));
  }
  dst->slices_size = src->slices_size;
  cg_gates_reserve(dst, src->candidate_gates_size);
  if (src->candidate_gates_size > 0) {
    memcpy(dst->candidate_gates, src->candidate_gates,
      src->candidate_gates_size * sizeof(dwa_corridor_slice_t));
  }
  dst->candidate_gates_size = src->candidate_gates_size;
  dst->detected = src->detected;
  dst->active = src->active;
  dst->held = src->held;
  dst->stable_cycles = src->stable_cycles;
  dst->lost_cycles = src->lost_cycles;
  dst->slice_count = src->slice_count;
  dst->candidate_count = src->candidate_count;
  dst->cluster_count = src->cluster_count;
  dst->raw_gate_count = src->raw_gate_count;
  dst->blocked_gate_count = src->blocked_gate_count;
  dst->reject_width = src->reject_width;
  dst->reject_heading = src->reject_heading;
  dst->reject_depth = src->reject_depth;
  dst->reject_blocked = src->reject_blocked;
  dst->entry_x = src->entry_x;
  dst->entry_y = src->entry_y;
  dst->entry_distance = src->entry_distance;
  dst->minimum_width = src->minimum_width;
  dst->target_y = src->target_y;
  dst->target_heading = src->target_heading;
  dst->reason = src->reason;
}

/* ---------------- 空间桶哈希（std::map 等价物） ---------------- */

typedef struct {
  bool used;
  int bx;
  int by;
  size_t *items;
  size_t count;
  size_t capacity;
} cg_bucket_t;

typedef struct {
  cg_bucket_t *slots;
  size_t slot_count;   /* 2 的幂 */
  size_t used_count;
} cg_bucket_map_t;

static void cg_bucket_map_init(cg_bucket_map_t *map)
{
  map->slot_count = 16;
  map->used_count = 0;
  map->slots = (cg_bucket_t *)calloc(map->slot_count, sizeof(cg_bucket_t));
}

static void cg_bucket_map_destroy(cg_bucket_map_t *map)
{
  for (size_t i = 0; i < map->slot_count; ++i) {
    free(map->slots[i].items);
  }
  free(map->slots);
  map->slots = NULL;
  map->slot_count = 0;
  map->used_count = 0;
}

static size_t cg_bucket_hash(int bx, int by, size_t slot_count)
{
  unsigned int h = (unsigned int)bx * 73856093u ^ (unsigned int)by * 19349663u;
  return (size_t)(h & (unsigned int)(slot_count - 1));
}

static cg_bucket_t *cg_bucket_map_find(
  const cg_bucket_map_t *map, int bx, int by)
{
  size_t slot = cg_bucket_hash(bx, by, map->slot_count);
  for (size_t probe = 0; probe < map->slot_count; ++probe) {
    cg_bucket_t *bucket = &map->slots[(slot + probe) & (map->slot_count - 1)];
    if (!bucket->used) return NULL;
    if (bucket->bx == bx && bucket->by == by) return bucket;
  }
  return NULL;
}

static void cg_bucket_push(cg_bucket_t *bucket, size_t index)
{
  if (bucket->count == bucket->capacity) {
    size_t next = bucket->capacity > 0 ? bucket->capacity * 2 : 4;
    bucket->items = (size_t *)realloc(bucket->items, next * sizeof(size_t));
    bucket->capacity = next;
  }
  bucket->items[bucket->count++] = index;
}

static void cg_bucket_map_grow(cg_bucket_map_t *map)
{
  const size_t old_count = map->slot_count;
  cg_bucket_t *old_slots = map->slots;
  map->slot_count = old_count * 2;
  map->slots = (cg_bucket_t *)calloc(map->slot_count, sizeof(cg_bucket_t));
  for (size_t i = 0; i < old_count; ++i) {
    if (old_slots[i].used) {
      size_t slot = cg_bucket_hash(old_slots[i].bx, old_slots[i].by, map->slot_count);
      while (map->slots[slot].used) slot = (slot + 1) & (map->slot_count - 1);
      map->slots[slot] = old_slots[i];
    }
  }
  free(old_slots);
}

static cg_bucket_t *cg_bucket_map_insert(cg_bucket_map_t *map, int bx, int by)
{
  if (map->used_count * 10 >= map->slot_count * 7) {
    cg_bucket_map_grow(map);
  }
  size_t slot = cg_bucket_hash(bx, by, map->slot_count);
  for (size_t probe = 0; probe < map->slot_count; ++probe) {
    cg_bucket_t *bucket = &map->slots[(slot + probe) & (map->slot_count - 1)];
    if (!bucket->used) {
      bucket->used = true;
      bucket->bx = bx;
      bucket->by = by;
      bucket->items = NULL;
      bucket->count = 0;
      bucket->capacity = 0;
      ++map->used_count;
      return bucket;
    }
    if (bucket->bx == bx && bucket->by == by) return bucket;
  }
  return NULL; /* 不会到达（装载因子 < 1） */
}

/* ---------------- 聚类 ---------------- */

typedef struct {
  dwa_point_t *points;
  size_t count;
  size_t capacity;
  dwa_point_t min_angle_point;
  dwa_point_t max_angle_point;
  dwa_point_t min_y_point;
  dwa_point_t max_y_point;
  float min_angle;
  float max_angle;
  float min_y;
  float max_y;
  float sum_y;
} cg_obstacle_cluster_t;

static void cg_cluster_init(cg_obstacle_cluster_t *cluster)
{
  cluster->points = NULL;
  cluster->count = 0;
  cluster->capacity = 0;
  cluster->min_angle_point.x = 0.0f; cluster->min_angle_point.y = 0.0f;
  cluster->max_angle_point.x = 0.0f; cluster->max_angle_point.y = 0.0f;
  cluster->min_y_point.x = 0.0f; cluster->min_y_point.y = 0.0f;
  cluster->max_y_point.x = 0.0f; cluster->max_y_point.y = 0.0f;
  cluster->min_angle = INFINITY;
  cluster->max_angle = -INFINITY;
  cluster->min_y = INFINITY;
  cluster->max_y = -INFINITY;
  cluster->sum_y = 0.0f;
}

static void cg_cluster_destroy(cg_obstacle_cluster_t *cluster)
{
  free(cluster->points);
  cluster->points = NULL;
  cluster->count = 0;
  cluster->capacity = 0;
}

static void cg_cluster_push(cg_obstacle_cluster_t *cluster, const dwa_point_t *point)
{
  if (cluster->count == cluster->capacity) {
    size_t next = cluster->capacity > 0 ? cluster->capacity * 2 : 8;
    cluster->points = (dwa_point_t *)realloc(cluster->points, next * sizeof(dwa_point_t));
    cluster->capacity = next;
  }
  cluster->points[cluster->count++] = *point;
}

/* center_y() 直译。 */
static float cg_cluster_center_y(const cg_obstacle_cluster_t *cluster)
{
  return cluster->count == 0 ? 0.0f : cluster->sum_y / (float)cluster->count;
}

/* ---------------- 排序辅助（带上下文的稳定归并排序） ---------------- */

/* inner_edge_candidates 的候选点比较器。 */
static int cg_edge_point_cmp(const void *lhs, const void *rhs)
{
  const dwa_point_t *left = (const dwa_point_t *)lhs;
  const dwa_point_t *right = (const dwa_point_t *)rhs;
  if (fabsf(left->x - right->x) > 1e-6f) return left->x < right->x ? -1 : 1;
  if (fabsf(left->y) < fabsf(right->y)) return -1;
  if (fabsf(left->y) > fabsf(right->y)) return 1;
  return 0;
}

typedef struct {
  dwa_corridor_guidance_t guidance;   /* 持有 slices/candidate_gates 数组 */
  dwa_corridor_slice_t gate;
  float direction_error;
  bool direction_mismatch;
} cg_ranked_corridor_t;

/* RankedCorridor 比较器直译（捕获 explicit_direction 作为参数）。 */
static bool cg_ranked_less(
  const cg_ranked_corridor_t *left,
  const cg_ranked_corridor_t *right,
  bool explicit_direction)
{
  if (explicit_direction && left->direction_mismatch != right->direction_mismatch) {
    return !left->direction_mismatch;
  }
  if (explicit_direction &&
    fabsf(left->direction_error - right->direction_error) > 1e-9f) {
    return left->direction_error < right->direction_error;
  }
  if (fabsf(left->guidance.entry_distance - right->guidance.entry_distance) > 1e-9f) {
    return left->guidance.entry_distance < right->guidance.entry_distance;
  }
  if (fabsf(left->guidance.minimum_width - right->guidance.minimum_width) > 1e-9f) {
    return left->guidance.minimum_width > right->guidance.minimum_width;
  }
  return left->guidance.target_y < right->guidance.target_y;
}

/* 稳定归并排序（等价于 std::sort，仅在比较器视为相等时顺序可能不同）。 */
static void cg_ranked_merge_sort(
  cg_ranked_corridor_t *array, cg_ranked_corridor_t *tmp,
  size_t begin, size_t end, bool explicit_direction)
{
  if (end - begin <= 1) return;
  const size_t mid = begin + (end - begin) / 2;
  cg_ranked_merge_sort(array, tmp, begin, mid, explicit_direction);
  cg_ranked_merge_sort(array, tmp, mid, end, explicit_direction);
  size_t left = begin;
  size_t right = mid;
  size_t out = begin;
  while (left < mid && right < end) {
    if (cg_ranked_less(&array[right], &array[left], explicit_direction)) {
      tmp[out++] = array[right++];
    } else {
      tmp[out++] = array[left++];
    }
  }
  while (left < mid) tmp[out++] = array[left++];
  while (right < end) tmp[out++] = array[right++];
  memcpy(array + begin, tmp + begin, (end - begin) * sizeof(cg_ranked_corridor_t));
}

/* ---------------- path_y_at / path_heading_at ---------------- */

static float cg_path_y_at(const dwa_corridor_guidance_t *path, float x)
{
  if (path->slices_size == 0) return 0.0f;
  const dwa_point_t *first = &path->slices[0].center;
  if (x <= first->x) {
    return first->x > 1e-6f ? first->y * cg_max(0.0f, x) / first->x : first->y;
  }
  for (size_t index = 1; index < path->slices_size; ++index) {
    const dwa_point_t *left = &path->slices[index - 1].center;
    const dwa_point_t *right = &path->slices[index].center;
    if (x > right->x) continue;
    const float alpha = (x - left->x) / cg_max(right->x - left->x, 1e-6f);
    return left->y + alpha * (right->y - left->y);
  }
  if (path->slices_size == 1) return first->y;
  const dwa_point_t *left = &path->slices[path->slices_size - 2].center;
  const dwa_point_t *right = &path->slices[path->slices_size - 1].center;
  const float slope = (right->y - left->y) / cg_max(right->x - left->x, 1e-6f);
  return right->y + slope * (x - right->x);
}

static float cg_path_heading_at(const dwa_corridor_guidance_t *path, float x)
{
  if (path->slices_size == 0) return 0.0f;
  const dwa_point_t *first = &path->slices[0].center;
  if (x <= first->x) return atan2f(first->y, cg_max(first->x, 1e-6f));
  for (size_t index = 1; index < path->slices_size; ++index) {
    const dwa_point_t *left = &path->slices[index - 1].center;
    const dwa_point_t *right = &path->slices[index].center;
    if (x <= right->x) {
      return atan2f(right->y - left->y, cg_max(right->x - left->x, 1e-6f));
    }
  }
  if (path->slices_size == 1) return atan2f(first->y, cg_max(first->x, 1e-6f));
  const dwa_point_t *left = &path->slices[path->slices_size - 2].center;
  const dwa_point_t *right = &path->slices[path->slices_size - 1].center;
  return atan2f(right->y - left->y, cg_max(right->x - left->x, 1e-6f));
}

/* ---------------- inner_edge_candidates（lambda 直译） ---------------- */

typedef struct {
  dwa_point_t *data;
  size_t size;
  size_t capacity;
} cg_point_array_t;

static void cg_point_array_push(cg_point_array_t *array, const dwa_point_t *point)
{
  if (array->size == array->capacity) {
    size_t next = array->capacity > 0 ? array->capacity * 2 : 16;
    array->data = (dwa_point_t *)realloc(array->data, next * sizeof(dwa_point_t));
    array->capacity = next;
  }
  array->data[array->size++] = *point;
}

static void cg_point_array_destroy(cg_point_array_t *array)
{
  free(array->data);
  array->data = NULL;
  array->size = 0;
  array->capacity = 0;
}

static void cg_inner_edge_candidates(
  const cg_obstacle_cluster_t *cluster, bool lower_edge, float edge_band,
  cg_point_array_t *out)
{
  out->size = 0;
  const float boundary_y = lower_edge ? cluster->min_y : cluster->max_y;
  for (size_t i = 0; i < cluster->count; ++i) {
    const dwa_point_t *point = &cluster->points[i];
    if (lower_edge) {
      if (point->y <= boundary_y + edge_band) cg_point_array_push(out, point);
    } else if (point->y >= boundary_y - edge_band) {
      cg_point_array_push(out, point);
    }
  }
  if (out->size == 0) {
    cg_point_array_push(
      out, lower_edge ? &cluster->min_y_point : &cluster->max_y_point);
  }
  qsort(out->data, out->size, sizeof(dwa_point_t), cg_edge_point_cmp);

  /* 去重（hypot <= 0.03），最多保留 24 个。 */
  cg_point_array_t unique;
  unique.data = NULL;
  unique.size = 0;
  unique.capacity = 0;
  const size_t reserve_hint = cg_min(out->size, (size_t)24U);
  if (reserve_hint > 0) {
    unique.capacity = reserve_hint;
    unique.data = (dwa_point_t *)malloc(reserve_hint * sizeof(dwa_point_t));
  }
  for (size_t i = 0; i < out->size; ++i) {
    const dwa_point_t *candidate = &out->data[i];
    bool duplicate = false;
    for (size_t j = 0; j < unique.size; ++j) {
      if (hypotf(candidate->x - unique.data[j].x, candidate->y - unique.data[j].y) <= 0.03f) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    cg_point_array_push(&unique, candidate);
    if (unique.size >= 24U) break;
  }
  /* 返回 unique：换回 out 缓冲区。 */
  free(out->data);
  out->data = unique.data;
  out->size = unique.size;
  out->capacity = unique.capacity;
}

/* ---------------- detect（const 成员函数直译） ---------------- */

static void cg_detect(
  const dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_input_t *input,
  dwa_corridor_guidance_t *result)
{
  const dwa_corridor_guide_config_t *config = &guide->config;
  dwa_corridor_guidance_reset(result);
  result->reason = "no_corridor";
  if (input->points == NULL || input->points_count == 0) {
    result->reason = "no_points";
    return;
  }

  /* 通道最小宽度直接使用完整碰撞包络宽度，保持与 DWA 安全判定一致。 */
  const float body_width =
    2.0f * dwa_footprint_half_width_extent(&input->footprint, true);
  const float minimum_width = body_width + config->minimum_width_margin;
  if (minimum_width >= config->maximum_width) {
    result->reason = "invalid_width_range";
    return;
  }

  const float intent_ratio = fabsf(input->max_yawrate) > 1e-6f ?
    dwa_clamp_value(input->intent.w / input->max_yawrate, -1.0f, 1.0f) : 0.0f;
  const float intent_target_y = intent_ratio * config->intent_lateral_range;
  const bool explicit_direction =
    fabsf(intent_ratio) >= config->explicit_intent_ratio;

  cg_point_array_t roi_points;
  roi_points.data = NULL;
  roi_points.size = 0;
  roi_points.capacity = 0;
  for (size_t i = 0; i < input->points_count; ++i) {
    const dwa_point_t *point = &input->points[i];
    if (!isfinite(point->x) || !isfinite(point->y) ||
      point->x < config->search_start_x || point->x > config->search_end_x ||
      fabsf(point->y) > config->search_half_width)
    {
      continue;
    }
    cg_point_array_push(&roi_points, point);
  }
  if (roi_points.size == 0) {
    result->reason = "no_roi_points";
    return;
  }

  /* 用空间桶做欧氏聚类，确保左右边界来自不同的真实障碍簇。 */
  const float gap_squared =
    config->cluster_gap_distance * config->cluster_gap_distance;
  cg_bucket_map_t buckets;
  cg_bucket_map_init(&buckets);
  for (size_t index = 0; index < roi_points.size; ++index) {
    const int bx = (int)floorf(roi_points.data[index].x / config->cluster_gap_distance);
    const int by = (int)floorf(roi_points.data[index].y / config->cluster_gap_distance);
    cg_bucket_t *bucket = cg_bucket_map_insert(&buckets, bx, by);
    cg_bucket_push(bucket, index);
  }

  char *visited = (char *)calloc(roi_points.size, 1);
  cg_obstacle_cluster_t *clusters = NULL;
  size_t clusters_size = 0;
  size_t clusters_capacity = 0;
  size_t *queue = NULL;
  size_t queue_capacity = 0;
  for (size_t start = 0; start < roi_points.size; ++start) {
    if (visited[start]) continue;
    cg_obstacle_cluster_t cluster;
    cg_cluster_init(&cluster);
    /* std::vector<size_t> queue{start}; */
    size_t queue_size = 0;
    if (queue_capacity == 0) {
      queue_capacity = 16;
      queue = (size_t *)realloc(queue, queue_capacity * sizeof(size_t));
    }
    queue[queue_size++] = start;
    visited[start] = 1;
    for (size_t cursor = 0; cursor < queue_size; ++cursor) {
      const size_t index = queue[cursor];
      const dwa_point_t *point = &roi_points.data[index];
      cg_cluster_push(&cluster, point);
      cluster.sum_y += point->y;
      const float angle = atan2f(point->y, point->x);
      if (angle < cluster.min_angle) {
        cluster.min_angle = angle;
        cluster.min_angle_point = *point;
      }
      if (angle > cluster.max_angle) {
        cluster.max_angle = angle;
        cluster.max_angle_point = *point;
      }
      if (point->y < cluster.min_y) {
        cluster.min_y = point->y;
        cluster.min_y_point = *point;
      }
      if (point->y > cluster.max_y) {
        cluster.max_y = point->y;
        cluster.max_y_point = *point;
      }
      const int center_bx = (int)floorf(point->x / config->cluster_gap_distance);
      const int center_by = (int)floorf(point->y / config->cluster_gap_distance);
      for (int offset_x = -1; offset_x <= 1; ++offset_x) {
        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
          const cg_bucket_t *bucket = cg_bucket_map_find(
            &buckets, center_bx + offset_x, center_by + offset_y);
          if (bucket == NULL) continue;
          for (size_t k = 0; k < bucket->count; ++k) {
            const size_t next = bucket->items[k];
            if (visited[next]) continue;
            const float dx = roi_points.data[next].x - point->x;
            const float dy = roi_points.data[next].y - point->y;
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
    if ((int)cluster.count >= config->cluster_min_points) {
      if (clusters_size == clusters_capacity) {
        clusters_capacity = clusters_capacity > 0 ? clusters_capacity * 2 : 8;
        clusters = (cg_obstacle_cluster_t *)realloc(
          clusters, clusters_capacity * sizeof(cg_obstacle_cluster_t));
      }
      clusters[clusters_size++] = cluster; /* 对应 std::move(cluster) */
    } else {
      cg_cluster_destroy(&cluster);
    }
  }
  free(queue);
  free(visited);
  cg_bucket_map_destroy(&buckets);
  result->cluster_count = (int)clusters_size;
  if (clusters_size < 2) {
    result->reason = "insufficient_obstacle_clusters";
    goto cleanup_clusters;
  }

  {
    const float edge_band = cg_max(
      0.04f, cg_min(0.16f, config->cluster_gap_distance));
    const float approach_x = config->approach_start_x;

    cg_ranked_corridor_t *ranked = NULL;
    size_t ranked_size = 0;
    size_t ranked_capacity = 0;
    cg_point_array_t upper_edges;
    cg_point_array_t lower_edges;
    upper_edges.data = NULL; upper_edges.size = 0; upper_edges.capacity = 0;
    lower_edges.data = NULL; lower_edges.size = 0; lower_edges.capacity = 0;

    for (size_t left_index = 0; left_index < clusters_size; ++left_index) {
      for (size_t right_index = left_index + 1; right_index < clusters_size;
        ++right_index)
      {
        const cg_obstacle_cluster_t *upper_cluster = &clusters[left_index];
        const cg_obstacle_cluster_t *lower_cluster = &clusters[right_index];
        if (cg_cluster_center_y(upper_cluster) < cg_cluster_center_y(lower_cluster)) {
          const cg_obstacle_cluster_t *swap = upper_cluster;
          upper_cluster = lower_cluster;
          lower_cluster = swap;
        }
        cg_inner_edge_candidates(upper_cluster, true, edge_band, &upper_edges);
        cg_inner_edge_candidates(lower_cluster, false, edge_band, &lower_edges);
        for (size_t ui = 0; ui < upper_edges.size; ++ui) {
          for (size_t li = 0; li < lower_edges.size; ++li) {
            const dwa_point_t *upper = &upper_edges.data[ui];
            const dwa_point_t *lower = &lower_edges.data[li];
            if (upper->y <= lower->y) {
              continue;
            }
            const float edge_dx = upper->x - lower->x;
            const float edge_dy = upper->y - lower->y;
            const float lateral_gap = edge_dy;
            const float gate_width = hypotf(edge_dx, edge_dy);
            /* 9.28 Fix J：门配对拒绝诊断——分段计数各几何约束拒绝次数。 */
            if (lateral_gap < minimum_width || gate_width > config->maximum_width ||
              gate_width <= 1e-6f)
            {
              ++result->reject_width;
              continue;
            }
            if (fabsf(upper->x - lower->x) > config->gate_max_depth_difference)
            {
              ++result->reject_depth;
              continue;
            }

            dwa_corridor_slice_t gate;
            gate.left_boundary = *upper;
            gate.right_boundary = *lower;
            gate.center.x = 0.5f * (upper->x + lower->x);
            gate.center.y = 0.5f * (upper->y + lower->y);
            gate.width = gate_width;
            if (gate.center.x < config->search_start_x ||
              gate.center.x > config->search_end_x) {
              continue;
            }

            bool blocked = false;
            const float segment_length_squared = edge_dx * edge_dx + edge_dy * edge_dy;
            for (size_t pi = 0; pi < roi_points.size; ++pi) {
              const dwa_point_t *point = &roi_points.data[pi];
              const float px = point->x - lower->x;
              const float py = point->y - lower->y;
              const float projection = (px * edge_dx + py * edge_dy) /
                segment_length_squared;
              const float endpoint_fraction = config->gate_endpoint_exclusion /
                cg_max(gate_width, 1e-6f);
              if (projection <= endpoint_fraction ||
                projection >= 1.0f - endpoint_fraction) {
                continue;
              }
              const float cross = px * edge_dy - py * edge_dx;
              if (fabsf(cross) / cg_max(gate_width, 1e-6f) <=
                config->gate_blocking_clearance)
              {
                blocked = true;
                break;
              }
            }
            ++result->raw_gate_count;
            if (blocked) {
              ++result->blocked_gate_count;
              ++result->reject_blocked;
              continue;
            }

            /* 门边的法向量指向车头前方，用它生成门后的短中心线。 */
            float normal_x = edge_dy / gate_width;
            float normal_y = -edge_dx / gate_width;
            if (normal_x < 0.0f) {
              normal_x = -normal_x;
              normal_y = -normal_y;
            }
            const float gate_heading = atan2f(normal_y, normal_x);
            /* 入口角度从base_link原点看门中心，避免近距离时被前参考点放大成急角。 */
            const float approach_heading = atan2f(
              gate.center.y, cg_max(gate.center.x, 1e-6f));
            if (fabsf(gate_heading) > config->maximum_heading ||
              fabsf(approach_heading) > config->maximum_heading)
            {
              ++result->reject_heading;
              continue;
            }

            dwa_corridor_guidance_t candidate;
            /* C++ 默认构造把 vector 初始化为空；栈上指针必须先置空，
             * dwa_corridor_guidance_reset 只清标量不动数组指针。 */
            candidate.slices = NULL;
            candidate.slices_size = 0;
            candidate.slices_capacity = 0;
            candidate.candidate_gates = NULL;
            candidate.candidate_gates_size = 0;
            candidate.candidate_gates_capacity = 0;
            dwa_corridor_guidance_reset(&candidate);
            dwa_corridor_slice_t slice;
            slice.center.x = approach_x;
            slice.center.y = 0.0f;
            slice.left_boundary.x = approach_x;
            slice.left_boundary.y = 0.5f * body_width;
            slice.right_boundary.x = approach_x;
            slice.right_boundary.y = -0.5f * body_width;
            slice.width = body_width;
            cg_slices_push_back(&candidate, &slice);
            cg_slices_push_back(&candidate, &gate);
            /* Fix O(9.29 方案A)：门法线投影按 |门航向| 线性衰减。
             * S 弯第一拐在通道口时，选中门的法线沿弯道指向（实测约
             * ±50°），固定 0.75m 投影会把 target 外推到弯道内侧的
             * 虚构点（020815/091554/013354 三日志入口段 ty -21~-37cm、
             * th 恒 -0.86rad 实锤），DWA 忠实执行 → 弧线穿墙。
             * |航向|≤10° 投影全额保留（直段行为零变化）；10°~30°
             * 线性衰减；≥30° 归零且不再压入第三段折线——target 退回
             * approach→门中心 两点直线的外推（= 车到门中心的实测
             * 方向，cg_path_y_at 末段斜率外推语义），复刻"通道内启动
             * 不撞"的逐门前进模式：车逐门前进、逐门再定目标。 */
            {
              const float fade_start = 10.0f * DWA_PI / 180.0f;
              const float fade_end = 30.0f * DWA_PI / 180.0f;
              const float abs_gate_heading = fabsf(gate_heading);
              float projection = config->gate_projection_length;
              if (abs_gate_heading >= fade_end) {
                projection = 0.0f;
              } else if (abs_gate_heading > fade_start) {
                projection *= (fade_end - abs_gate_heading) /
                  (fade_end - fade_start);
              }
              if (projection > 0.01f) {
                slice.center.x = gate.center.x + normal_x * projection;
                slice.center.y = gate.center.y + normal_y * projection;
                slice.left_boundary.x = upper->x + normal_x * projection;
                slice.left_boundary.y = upper->y + normal_y * projection;
                slice.right_boundary.x = lower->x + normal_x * projection;
                slice.right_boundary.y = lower->y + normal_y * projection;
                slice.width = gate_width;
                cg_slices_push_back(&candidate, &slice);
              }
            }
            candidate.detected = true;
            candidate.slice_count = (int)candidate.slices_size;
            candidate.entry_x = gate.center.x;
            candidate.entry_y = gate.center.y;
            candidate.entry_distance = hypotf(
              gate.center.x - approach_x, gate.center.y);
            candidate.minimum_width = gate_width;
            candidate.target_y = cg_path_y_at(&candidate, config->trajectory_lookahead);
            candidate.target_heading = cg_path_heading_at(
              &candidate, config->trajectory_lookahead);
            candidate.reason = "corridor_detected";
            cg_gates_push_back(&candidate, &gate);
            const float direction_error =
              fabsf(candidate.target_y - intent_target_y);
            const bool direction_mismatch = explicit_direction &&
              intent_ratio * candidate.target_y < -0.05f;
            /* ranked.push_back(RankedCorridor{std::move(candidate), ...}) */
            if (ranked_size == ranked_capacity) {
              ranked_capacity = ranked_capacity > 0 ? ranked_capacity * 2 : 16;
              ranked = (cg_ranked_corridor_t *)realloc(
                ranked, ranked_capacity * sizeof(cg_ranked_corridor_t));
            }
            ranked[ranked_size].guidance = candidate; /* 指针所有权转移 */
            ranked[ranked_size].gate = gate;
            ranked[ranked_size].direction_error = direction_error;
            ranked[ranked_size].direction_mismatch = direction_mismatch;
            ++ranked_size;
          }
        }
      }
    }
    cg_point_array_destroy(&upper_edges);
    cg_point_array_destroy(&lower_edges);

    if (ranked_size == 0) {
      result->reason = result->raw_gate_count > 0 ?
        "no_unblocked_corridor_gate" : "no_corridor_gate";
      goto cleanup_ranked;
    }

    /* 前推无明确方向时严格按真实入口距离选最近通道，避免固定切片偏向远处通道。 */
    {
      cg_ranked_corridor_t *tmp = (cg_ranked_corridor_t *)malloc(
        ranked_size * sizeof(cg_ranked_corridor_t));
      cg_ranked_merge_sort(ranked, tmp, 0, ranked_size, explicit_direction);
      free(tmp);
    }

    {
      cg_ranked_corridor_t *unique_ranked = NULL;
      size_t unique_size = 0;
      size_t unique_capacity = 0;
      for (size_t i = 0; i < ranked_size; ++i) {
        bool duplicate = false;
        for (size_t j = 0; j < unique_size; ++j) {
          if (hypotf(
              ranked[i].guidance.entry_x - unique_ranked[j].guidance.entry_x,
              ranked[i].guidance.entry_y - unique_ranked[j].guidance.entry_y) <=
              config->gate_dedup_distance &&
            fabsf(dwa_normalize_angle(
              ranked[i].guidance.target_heading -
              unique_ranked[j].guidance.target_heading)) <=
              config->track_heading_tolerance)
          {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) {
          if (unique_size == unique_capacity) {
            unique_capacity = unique_capacity > 0 ? unique_capacity * 2 : 8;
            unique_ranked = (cg_ranked_corridor_t *)realloc(
              unique_ranked, unique_capacity * sizeof(cg_ranked_corridor_t));
          }
          unique_ranked[unique_size++] = ranked[i]; /* move：所有权转移 */
          ranked[i].guidance.slices = NULL;
          ranked[i].guidance.slices_size = 0;
          ranked[i].guidance.slices_capacity = 0;
          ranked[i].guidance.candidate_gates = NULL;
          ranked[i].guidance.candidate_gates_size = 0;
          ranked[i].guidance.candidate_gates_capacity = 0;
        }
      }
      if (unique_size == 0) {
        result->reason = "no_unique_corridor_gate";
        free(unique_ranked);
        goto cleanup_ranked;
      }

      const int cluster_count = result->cluster_count;
      const int raw_gate_count = result->raw_gate_count;
      const int blocked_gate_count = result->blocked_gate_count;
      /* result = std::move(unique_ranked.front().guidance); */
      {
        dwa_corridor_guidance_t *front = &unique_ranked[0].guidance;
        free(result->slices);
        free(result->candidate_gates);
        *result = *front;
        front->slices = NULL;
        front->slices_size = 0;
        front->slices_capacity = 0;
        front->candidate_gates = NULL;
        front->candidate_gates_size = 0;
        front->candidate_gates_capacity = 0;
      }
      result->candidate_count = (int)unique_size;
      result->candidate_gates_size = 0; /* candidate_gates.clear() */
      for (size_t i = 0; i < unique_size; ++i) {
        if (unique_ranked[i].guidance.slices_size != 0) {
          cg_gates_push_back(result, &unique_ranked[i].guidance.slices[1]);
        }
      }
      for (size_t i = 0; i < unique_size; ++i) {
        dwa_corridor_guidance_destroy(&unique_ranked[i].guidance);
      }
      free(unique_ranked);
      result->cluster_count = cluster_count;
      result->raw_gate_count = raw_gate_count;
      result->blocked_gate_count = blocked_gate_count;
      result->reason = explicit_direction ?
        "corridor_selected_by_intent" : "corridor_selected_by_distance";
    }

cleanup_ranked:
    for (size_t i = 0; i < ranked_size; ++i) {
      dwa_corridor_guidance_destroy(&ranked[i].guidance);
    }
    free(ranked);
  }

cleanup_clusters:
  for (size_t i = 0; i < clusters_size; ++i) {
    cg_cluster_destroy(&clusters[i]);
  }
  free(clusters);
  cg_point_array_destroy(&roi_points);
}

/* ---------------- update_pending / similar / lock / blend ---------------- */

static bool cg_paths_are_similar(
  const dwa_corridor_guide_t *guide,
  const dwa_corridor_guidance_t *left,
  const dwa_corridor_guidance_t *right)
{
  if (left->slices_size < 2 || right->slices_size < 2) return false;
  const float left_y = cg_path_y_at(left, guide->config.trajectory_lookahead);
  const float right_y = cg_path_y_at(right, guide->config.trajectory_lookahead);
  return fabsf(left_y - right_y) <= guide->config.track_lateral_tolerance &&
    fabsf(dwa_normalize_angle(left->target_heading - right->target_heading)) <=
    guide->config.track_heading_tolerance;
}

static bool cg_should_lock_near_field(
  const dwa_corridor_guide_t *guide,
  const dwa_corridor_guidance_t *previous,
  const dwa_corridor_guidance_t *current)
{
  const float lock_distance = 1.20f;
  const float heading_jump_limit = 15.0f * DWA_PI / 180.0f;
  const float lateral_jump_limit = 0.18f;
  const float entry_retract_limit = 0.18f;
  const bool near_entry =
    previous->entry_distance <= lock_distance || current->entry_distance <= lock_distance;
  if (!near_entry) return false;

  const float previous_y = cg_path_y_at(previous, guide->config.trajectory_lookahead);
  const float current_y = cg_path_y_at(current, guide->config.trajectory_lookahead);
  const float heading_jump = fabsf(
    dwa_normalize_angle(previous->target_heading - current->target_heading));
  const float lateral_jump = fabsf(previous_y - current_y);
  const float entry_y_jump = fabsf(previous->entry_y - current->entry_y);
  const bool entry_retracted = current->entry_x + entry_retract_limit < previous->entry_x;
  const bool target_crossed_center = previous->target_y * current->target_y < -0.02f;

  return heading_jump > heading_jump_limit ||
    lateral_jump > lateral_jump_limit ||
    entry_y_jump > lateral_jump_limit ||
    entry_retracted ||
    target_crossed_center;
}

/* blend_path(previous, current, out)：out 必须与 previous/current 不重叠。
 * 先深拷贝 current 到 out，再逐切片滤波。 */
static void cg_blend_path(
  const dwa_corridor_guide_t *guide,
  const dwa_corridor_guidance_t *previous,
  const dwa_corridor_guidance_t *current,
  dwa_corridor_guidance_t *out)
{
  const dwa_corridor_guide_config_t *config = &guide->config;
  dwa_corridor_guidance_copy(out, current);
  for (size_t i = 0; i < out->slices_size; ++i) {
    dwa_corridor_slice_t *slice = &out->slices[i];
    const float previous_center = cg_path_y_at(previous, slice->center.x);
    const float delta = slice->center.y - previous_center;
    const float filtered_center = previous_center + config->path_filter_alpha * delta;
    const float center_shift = filtered_center - slice->center.y;
    slice->center.y = filtered_center;
    slice->left_boundary.y += center_shift;
    slice->right_boundary.y += center_shift;
  }
  const size_t entry_index = out->slices_size > 1 ? 1U : 0U;
  out->entry_x = out->slices[entry_index].center.x;
  out->entry_y = out->slices[entry_index].center.y;
  out->entry_distance = hypotf(
    out->entry_x - config->approach_start_x, out->entry_y);
  out->target_y = cg_path_y_at(out, config->trajectory_lookahead);
  out->target_heading = cg_path_heading_at(out, config->trajectory_lookahead);
}

static void cg_update_pending(
  const dwa_corridor_guide_t *guide,
  dwa_corridor_guide_t *self,
  const dwa_corridor_guidance_t *detection)
{
  if (!detection->detected) {
    self->pending_valid = false;
    self->pending_stable_cycles = 0;
    dwa_corridor_guidance_reset(&self->pending_path);
    return;
  }
  if (!self->pending_valid || !cg_paths_are_similar(guide, &self->pending_path, detection)) {
    self->pending_valid = true;
    self->pending_stable_cycles = 1;
    dwa_corridor_guidance_copy(&self->pending_path, detection);
    return;
  }
  ++self->pending_stable_cycles;
  /* pending_path_ = blend_path(pending_path_, detection)：previous 先备份。 */
  dwa_corridor_guidance_copy(&self->blend_scratch, &self->pending_path);
  cg_blend_path(guide, &self->blend_scratch, detection, &self->pending_path);
  self->pending_path.detected = true;
}

/* ---------------- 对外接口 ---------------- */

void dwa_corridor_guide_init(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_config_t *config)
{
  guide->config = *config;
  guide->pending_valid = false;
  guide->pending_stable_cycles = 0;
  guide->active = false;
  guide->lost_cycles = 0;
  guide->near_lock_active = false;
  guide->pending_path.slices = NULL;
  guide->pending_path.slices_size = 0;
  guide->pending_path.slices_capacity = 0;
  guide->pending_path.candidate_gates = NULL;
  guide->pending_path.candidate_gates_size = 0;
  guide->pending_path.candidate_gates_capacity = 0;
  dwa_corridor_guidance_reset(&guide->pending_path);
  guide->active_path.slices = NULL;
  guide->active_path.slices_size = 0;
  guide->active_path.slices_capacity = 0;
  guide->active_path.candidate_gates = NULL;
  guide->active_path.candidate_gates_size = 0;
  guide->active_path.candidate_gates_capacity = 0;
  dwa_corridor_guidance_reset(&guide->active_path);
  guide->last_guidance.slices = NULL;
  guide->last_guidance.slices_size = 0;
  guide->last_guidance.slices_capacity = 0;
  guide->last_guidance.candidate_gates = NULL;
  guide->last_guidance.candidate_gates_size = 0;
  guide->last_guidance.candidate_gates_capacity = 0;
  dwa_corridor_guidance_reset(&guide->last_guidance);
  guide->blend_scratch.slices = NULL;
  guide->blend_scratch.slices_size = 0;
  guide->blend_scratch.slices_capacity = 0;
  guide->blend_scratch.candidate_gates = NULL;
  guide->blend_scratch.candidate_gates_size = 0;
  guide->blend_scratch.candidate_gates_capacity = 0;
  dwa_corridor_guidance_reset(&guide->blend_scratch);
}

void dwa_corridor_guide_init_default(dwa_corridor_guide_t *guide)
{
  const dwa_corridor_guide_config_t config = dwa_corridor_guide_config_default();
  dwa_corridor_guide_init(guide, &config);
}

void dwa_corridor_guide_destroy(dwa_corridor_guide_t *guide)
{
  dwa_corridor_guidance_destroy(&guide->pending_path);
  dwa_corridor_guidance_destroy(&guide->active_path);
  dwa_corridor_guidance_destroy(&guide->last_guidance);
  dwa_corridor_guidance_destroy(&guide->blend_scratch);
}

bool dwa_corridor_guide_set_config(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_config_t *config,
  const char **reason)
{
  if (!dwa_validate_dwa_corridor_guide_config(config, reason)) return false;
  guide->config = *config;
  dwa_corridor_guide_reset(guide);
  return true;
}

void dwa_corridor_guide_reset(dwa_corridor_guide_t *guide)
{
  guide->pending_valid = false;
  guide->pending_stable_cycles = 0;
  guide->active = false;
  guide->lost_cycles = 0;
  guide->near_lock_active = false;
  dwa_corridor_guidance_reset(&guide->pending_path);
  dwa_corridor_guidance_reset(&guide->active_path);
  dwa_corridor_guidance_reset(&guide->last_guidance);
}

const dwa_corridor_guidance_t *dwa_corridor_guide_update(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_input_t *input)
{
  dwa_corridor_guidance_t *last_guidance = &guide->last_guidance;
  dwa_corridor_guidance_reset(last_guidance);
  guide->near_lock_active = false;
  if (!guide->config.enabled) {
    dwa_corridor_guide_reset(guide);
    last_guidance->reason = "disabled";
    return last_guidance;
  }
  if (!input->perception_fresh || input->points == NULL || !input->forward_intent) {
    dwa_corridor_guide_reset(guide);
    last_guidance->reason = input->perception_fresh ? "no_forward_intent" :
      "perception_stale";
    return last_guidance;
  }

  dwa_corridor_guidance_t detection;
  detection.slices = NULL;
  detection.slices_size = 0;
  detection.slices_capacity = 0;
  detection.candidate_gates = NULL;
  detection.candidate_gates_size = 0;
  detection.candidate_gates_capacity = 0;
  cg_detect(guide, input, &detection);
  if (!guide->active) {
    /* 连续多周期识别到同一条通道后才锁定，避免单帧点云抖动抢占控制权。 */
    cg_update_pending(guide, guide, &detection);
    if (guide->pending_valid && guide->pending_stable_cycles >= guide->config.stable_cycles) {
      guide->active = true;
      dwa_corridor_guidance_copy(&guide->active_path, &guide->pending_path);
      guide->active_path.active = true;
      guide->lost_cycles = 0;
    }
  } else if (detection.detected && cg_paths_are_similar(guide, &guide->active_path, &detection)) {
    if (cg_should_lock_near_field(guide, &guide->active_path, &detection)) {
      /* 近距离进入通道后保持同一条中心线，避免门边点跳动把目标线拉平或反向。 */
      guide->near_lock_active = true;
    } else {
      /* active_path_ = blend_path(active_path_, detection) */
      dwa_corridor_guidance_copy(&guide->blend_scratch, &guide->active_path);
      cg_blend_path(guide, &guide->blend_scratch, &detection, &guide->active_path);
      cg_update_pending(guide, guide, &detection);
    }
    guide->active_path.active = true;
    guide->lost_cycles = 0;
  } else {
    ++guide->lost_cycles;
    cg_update_pending(guide, guide, &detection);
    if (guide->lost_cycles > guide->config.lost_hold_cycles) {
      guide->active = false;
      dwa_corridor_guidance_reset(&guide->active_path);
    }
  }

  if (guide->active) {
    dwa_corridor_guidance_copy(last_guidance, &guide->active_path);
    last_guidance->detected = detection.detected;
    last_guidance->active = true;
    last_guidance->held = !detection.detected || guide->lost_cycles > 0;
    last_guidance->stable_cycles = guide->pending_stable_cycles;
    last_guidance->lost_cycles = guide->lost_cycles;
    if (guide->near_lock_active) {
      last_guidance->held = true;
      last_guidance->reason = "corridor_near_locked";
    } else {
      last_guidance->reason = last_guidance->held ? "corridor_held" : "corridor_active";
    }
    last_guidance->cluster_count = detection.cluster_count;
    last_guidance->raw_gate_count = detection.raw_gate_count;
    last_guidance->blocked_gate_count = detection.blocked_gate_count;
    last_guidance->candidate_count = detection.candidate_count;
    /* last_guidance_.candidate_gates = detection.candidate_gates; */
    last_guidance->candidate_gates_size = 0;
    for (size_t i = 0; i < detection.candidate_gates_size; ++i) {
      cg_gates_push_back(last_guidance, &detection.candidate_gates[i]);
    }
    dwa_corridor_guidance_destroy(&detection);
    return last_guidance;
  }

  dwa_corridor_guidance_copy(last_guidance, &detection);
  last_guidance->detected = detection.detected;
  last_guidance->active = false;
  last_guidance->stable_cycles = guide->pending_stable_cycles;
  last_guidance->lost_cycles = guide->lost_cycles;
  if (detection.detected) last_guidance->reason = "corridor_stabilizing";
  else if (last_guidance->reason == NULL) last_guidance->reason = "no_corridor";
  dwa_corridor_guidance_destroy(&detection);
  return last_guidance;
}

const dwa_corridor_guidance_t *dwa_corridor_guide_guidance(
  const dwa_corridor_guide_t *guide)
{
  return &guide->last_guidance;
}
