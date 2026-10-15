/*
 * 模块功能：通道引导的数据契约、配置结构和参数校验 + 通道检测与跟踪控制器。
 * 直译自 native/guidance/corridor_types.hpp 与 native/dwa_corridor_guide.hpp
 * （逐行对应，不改逻辑/默认值/执行顺序）。
 * std::vector<DwaCorridorSlice> → 动态数组（count/capacity，push_back 语义保留）；
 * std::string reason → const char*（固定字面量，NULL == 空串）。
 */
#ifndef DWA_CORRIDOR_GUIDE_H
#define DWA_CORRIDOR_GUIDE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "../common/dwa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- corridor_types.hpp ---- */

typedef struct {
  bool enabled;
  float search_start_x;
  float search_end_x;
  float minimum_width_margin;
  float maximum_width;
  float search_half_width;
  float approach_start_x;
  float cluster_gap_distance;
  int cluster_min_points;
  float gate_max_depth_difference;
  float gate_blocking_clearance;
  float gate_endpoint_exclusion;
  float gate_dedup_distance;
  float gate_projection_length;
  float explicit_intent_ratio;
  float maximum_heading;
  float intent_lateral_range;
  int stable_cycles;
  int lost_hold_cycles;
  float track_lateral_tolerance;
  float track_heading_tolerance;
  float path_filter_alpha;
  float trajectory_lookahead;
  float lateral_error_weight;
  float heading_error_weight;
  float progress_weight;
} dwa_corridor_guide_config_t;

/* DwaCorridorGuideConfig{} 成员默认值直译。 */
dwa_corridor_guide_config_t dwa_corridor_guide_config_default(void);

/* validate_dwa_corridor_guide_config(config, reason) 直译；reason 可为 NULL。 */
bool dwa_validate_dwa_corridor_guide_config(
  const dwa_corridor_guide_config_t *config,
  const char **reason);

typedef struct {
  dwa_point_t center;
  dwa_point_t left_boundary;
  dwa_point_t right_boundary;
  float width;
} dwa_corridor_slice_t;

/* DwaCorridorGuidance：slices/candidate_gates 为复用容量的动态数组。 */
typedef struct {
  /* 检测结果、锁定状态和稳定性计数。 */
  bool detected;
  bool active;
  bool held;
  int stable_cycles;
  int lost_cycles;
  int slice_count;
  int candidate_count;
  int cluster_count;
  int raw_gate_count;
  int blocked_gate_count;
  /* 9.28 Fix J 诊断：门配对拒绝原因分布（每周期累计）。*/
  int reject_width;      /* lateral_gap < min 或 gate_width > max */
  int reject_heading;    /* 门航向超 max_heading_deg */
  int reject_depth;      /* 左右端点深度差超 gate_max_depth_difference */
  int reject_blocked;    /* 门内有点距边缘 < gate_blocking_clearance */
  float entry_x;
  float entry_y;
  float entry_distance;
  float minimum_width;
  float target_y;
  float target_heading;
  dwa_corridor_slice_t *slices;       /* vector<DwaCorridorSlice> */
  size_t slices_size;
  size_t slices_capacity;
  dwa_corridor_slice_t *candidate_gates;
  size_t candidate_gates_size;
  size_t candidate_gates_capacity;
  const char *reason;                 /* NULL == 空串 */
} dwa_corridor_guidance_t;

/* DwaCorridorGuidance{} 默认构造语义（数组容量保留，仅清空内容）。 */
void dwa_corridor_guidance_reset(dwa_corridor_guidance_t *guidance);
/* 释放动态数组。 */
void dwa_corridor_guidance_destroy(dwa_corridor_guidance_t *guidance);
/* 深拷贝（数组缓冲区按 dst 已有容量复用）。 */
void dwa_corridor_guidance_copy(
  dwa_corridor_guidance_t *dst, const dwa_corridor_guidance_t *src);

typedef struct {
  /* 当前周期经过筛选的车体坐标系障碍点。 */
  const dwa_point_t *points;
  size_t points_count;
  /* 用于计算通道最小宽度的完整车体包络。 */
  dwa_footprint_t footprint;
  /* 控制器当前的前进/转向意图。 */
  dwa_velocity_t intent;
  /* 障碍点输入是否在允许时限内更新。 */
  bool perception_fresh;
  /* 当前是否存在有效的前进意图。 */
  bool forward_intent;
  /* DWA最大角速度，用于把角速度意图换算为横向目标。 */
  float max_yawrate;
} dwa_corridor_guide_input_t;

/* DwaCorridorGuideInput{} 默认值直译。 */
static inline dwa_corridor_guide_input_t dwa_corridor_guide_input_default(void)
{
  dwa_corridor_guide_input_t input;
  input.points = NULL;
  input.points_count = 0;
  input.footprint = dwa_footprint_default();
  input.intent.v = 0.0f;
  input.intent.w = 0.0f;
  input.perception_fresh = false;
  input.forward_intent = false;
  input.max_yawrate = 1.0f;
  return input;
}

/* ---- dwa_corridor_guide.hpp：DwaCorridorGuide 类 ---- */

typedef struct {
  dwa_corridor_guide_config_t config;
  bool pending_valid;
  int pending_stable_cycles;
  bool active;
  int lost_cycles;
  bool near_lock_active;
  dwa_corridor_guidance_t pending_path;
  dwa_corridor_guidance_t blend_scratch;
  dwa_corridor_guidance_t active_path;
  dwa_corridor_guidance_t last_guidance;
} dwa_corridor_guide_t;

/* 构造：config_ = config；内部三条 guidance 为默认构造（含动态数组初始化）。 */
void dwa_corridor_guide_init(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_config_t *config);
void dwa_corridor_guide_init_default(dwa_corridor_guide_t *guide);
void dwa_corridor_guide_destroy(dwa_corridor_guide_t *guide);

/* set_config(config, reason) 直译。 */
bool dwa_corridor_guide_set_config(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_config_t *config,
  const char **reason);

/* reset() 直译。 */
void dwa_corridor_guide_reset(dwa_corridor_guide_t *guide);

/* update(input) 直译；返回内部 last_guidance_ 的只读指针。 */
const dwa_corridor_guidance_t *dwa_corridor_guide_update(
  dwa_corridor_guide_t *guide,
  const dwa_corridor_guide_input_t *input);

/* guidance() 直译。 */
const dwa_corridor_guidance_t *dwa_corridor_guide_guidance(
  const dwa_corridor_guide_t *guide);

#ifdef __cplusplus
}
#endif

#endif /* DWA_CORRIDOR_GUIDE_H */
