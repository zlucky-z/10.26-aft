/*
 * 模块功能：几何二次选轨迹模块：依据最近前方障碍边缘和轨迹弦方向优化DWA选择。
 * 直译自 native/dwa_geometric_selector.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * std::string reason → const char*；vector<DwaPoint> → 动态数组；
 * DwaGeometricSelection::selected_trajectory 复用容量（dwa_trajectory_copy）。
 */
#ifndef DWA_GEOMETRIC_SELECTOR_H
#define DWA_GEOMETRIC_SELECTOR_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "../common/dwa_types.h"
#include "../planning/dwa_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool enabled;
  float lookahead_distance;
  float minimum_lookahead_distance;
  float minimum_obstacle_forward_offset;
  float cluster_gap_distance;
  float cluster_forward_window;
  float edge_front_band;
  float cluster_track_tolerance;
  float centered_entry_max_width;
  int handoff_stable_cycles;
  float handoff_heading_agree;
  float minimum_candidate_clearance;
  float maximum_clearance_loss;
  float raw_override_yawrate;
  float minimum_target_heading;
  float maximum_heading_error;
  float manual_override_yawrate;
  float target_filter_alpha;
  float heading_tie_tolerance;
  float near_small_angle_distance;
  float small_target_heading;
  float near_target_heading_compensation;
  float maximum_heading_growth;
} dwa_geometric_selector_config_t;

/* DwaGeometricSelectorConfig{} 成员默认值直译。 */
dwa_geometric_selector_config_t dwa_geometric_selector_config_default(void);

/* validate_dwa_geometric_selector_config(config, reason) 直译。 */
bool dwa_validate_dwa_geometric_selector_config(
  const dwa_geometric_selector_config_t *config,
  const char **reason);

typedef struct {
  bool valid;
  dwa_point_t point;
  float range;
} dwa_geometric_scan_point_t;

/* DwaGeometricScanPoint{} 默认值直译。 */
static inline dwa_geometric_scan_point_t dwa_geometric_scan_point_default(void)
{
  dwa_geometric_scan_point_t value;
  value.valid = false;
  value.point.x = 0.0f;
  value.point.y = 0.0f;
  value.range = INFINITY;
  return value;
}

typedef struct {
  const dwa_geometric_scan_point_t *left_scan_points;
  size_t left_scan_points_count;
  const dwa_geometric_scan_point_t *right_scan_points;
  size_t right_scan_points_count;
  dwa_point_t left_laser;
  dwa_point_t right_laser;
  float front_trigger_start_x;
  float clearance_front_edge;
  float front_trigger_base_half_width;
  float front_trigger_expand_angle;
  dwa_velocity_t intent;
  int base_direction;
  float direction_epsilon;
} dwa_geometric_selector_input_t;

/* DwaGeometricSelectorInput{} 默认值直译。 */
static inline dwa_geometric_selector_input_t dwa_geometric_selector_input_default(void)
{
  dwa_geometric_selector_input_t input;
  input.left_scan_points = NULL;
  input.left_scan_points_count = 0;
  input.right_scan_points = NULL;
  input.right_scan_points_count = 0;
  input.left_laser.x = 0.0f; input.left_laser.y = 0.0f;
  input.right_laser.x = 0.0f; input.right_laser.y = 0.0f;
  input.front_trigger_start_x = 0.0f;
  input.clearance_front_edge = 0.0f;
  input.front_trigger_base_half_width = 0.0f;
  input.front_trigger_expand_angle = 0.0f;
  input.intent.v = 0.0f;
  input.intent.w = 0.0f;
  input.base_direction = 0;
  input.direction_epsilon = 0.02f;
  return input;
}

typedef struct {
  bool applied;
  bool geometry_valid;
  bool obstacle_edge_valid;
  bool cluster_valid;
  bool handoff_ready;
  bool tracking_active;
  bool manual_reverse;
  int raw_base_direction;
  int base_direction;
  int requested_direction;
  int eligible_count;
  int obstacle_segment_size;
  int edge_candidate_count;
  int obstacle_segment_start;
  int obstacle_segment_end;
  int obstacle_edge_index;
  bool obstacle_from_left_scan;
  int cluster_point_count;
  int stable_cycles;
  size_t selected_index;   /* 默认 SIZE_MAX */
  dwa_point_t obstacle_point;
  dwa_point_t cluster_centroid;
  dwa_point_t radar_point;
  dwa_point_t reference_origin;
  dwa_point_t selected_lookahead_point;
  dwa_velocity_t selected;
  dwa_trajectory_t selected_trajectory;  /* 深拷贝，容量复用 */
  float raw_target_heading;
  float target_heading;
  float target_heading_compensation;
  float selected_heading;
  float heading_error;
  float edge_radar_distance;
  float edge_front_clearance;
  float cluster_min_x;
  float cluster_max_x;
  float cluster_min_y;
  float cluster_max_y;
  float raw_heading;
  float raw_heading_error;
  float lookahead_distance;
  bool target_heading_compensated;
  const char *reason;      /* NULL == 空串 */
} dwa_geometric_selection_t;

/* DwaGeometricSelection{} 默认构造语义（selected_trajectory 容量保留）。 */
void dwa_geometric_selection_reset(dwa_geometric_selection_t *selection);
void dwa_geometric_selection_init(dwa_geometric_selection_t *selection);
void dwa_geometric_selection_destroy(dwa_geometric_selection_t *selection);

typedef struct {
  bool valid;
  dwa_point_t point;
  int segment_size;
  int candidate_count;
  int segment_start;
  int segment_end;
  int edge_index;
  float radar_distance;
  float front_clearance;
  const char *reason;
} dwa_geometric_obstacle_edge_t;

/* DwaGeometricObstacleEdge{} 默认值直译。 */
static inline dwa_geometric_obstacle_edge_t dwa_geometric_obstacle_edge_default(void)
{
  dwa_geometric_obstacle_edge_t edge;
  edge.valid = false;
  edge.point.x = 0.0f;
  edge.point.y = 0.0f;
  edge.segment_size = 0;
  edge.candidate_count = 0;
  edge.segment_start = -1;
  edge.segment_end = -1;
  edge.edge_index = -1;
  edge.radar_distance = INFINITY;
  edge.front_clearance = INFINITY;
  edge.reason = "edge_not_evaluated";
  return edge;
}

/* finite_geometric_point(point) 直译。 */
static inline bool dwa_finite_geometric_point(const dwa_point_t *point)
{
  return isfinite(point->x) && isfinite(point->y);
}

typedef struct {
  bool valid;
  dwa_point_t *points;    /* vector<DwaPoint> */
  size_t points_size;
  size_t points_capacity;
  dwa_point_t centroid;
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float nearest_front_x;
  float front_clearance;
} dwa_geometric_fused_obstacle_cluster_t;

/* DwaGeometricFusedObstacleCluster{} 默认值直译（数组为空）。 */
void dwa_geometric_fused_obstacle_cluster_init(
  dwa_geometric_fused_obstacle_cluster_t *cluster);
void dwa_geometric_fused_obstacle_cluster_destroy(
  dwa_geometric_fused_obstacle_cluster_t *cluster);

/* update_geometric_cluster_bounds(cluster, point) 直译。 */
void dwa_update_geometric_cluster_bounds(
  dwa_geometric_fused_obstacle_cluster_t *cluster,
  const dwa_point_t *point);

/* geometric_front_trigger_half_width_at(x, start, base, angle) 直译。 */
float dwa_geometric_front_trigger_half_width_at(
  float x,
  float front_trigger_start_x,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle);

/* point_blocks_front_trigger(point, start, base, angle) 直译。 */
bool dwa_point_blocks_front_trigger(
  const dwa_point_t *point,
  float front_trigger_start_x,
  float front_trigger_base_half_width,
  float front_trigger_expand_angle);

/*
 * build_nearest_fused_front_cluster(left_scan, right_scan, ..., config,
 *                                    selected_cluster, reason) 直译。
 * selected_cluster 为输出（对应 C++ 引用出参）。
 */
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
  const char **reason);

/* select_cluster_edge_for_direction(cluster, radar_point, direction, start_x, config) 直译。 */
dwa_geometric_obstacle_edge_t dwa_select_cluster_edge_for_direction(
  const dwa_geometric_fused_obstacle_cluster_t *cluster,
  const dwa_point_t *radar_point,
  int direction,
  float front_trigger_start_x,
  const dwa_geometric_selector_config_t *config);

/* geometric_reference_point(state, reference) 直译。 */
dwa_point_t dwa_geometric_reference_point(
  const dwa_state_t *state,
  const dwa_point_t *reference);

/*
 * geometric_heading_at_arc_distance(trajectory, reference_origin,
 *                                    lookahead_distance, &heading,
 *                                    lookahead_point) 直译；
 * lookahead_point 可为 NULL（对应 C++ 默认参数 nullptr）。
 */
bool dwa_geometric_heading_at_arc_distance(
  const dwa_trajectory_t *trajectory,
  const dwa_point_t *reference_origin,
  float lookahead_distance,
  float *heading,
  dwa_point_t *lookahead_point);

/* ---- DwaGeometricTrajectorySelector 类 ---- */

typedef struct {
  dwa_geometric_selector_config_t config;
  bool filtered_target_valid;
  int filtered_direction;
  float filtered_target_heading;
  bool filtered_target_compensated;
  bool tracking_active;
  bool handoff_committed;
  bool centered_entry_active;
  int tracked_direction;
  int stable_cycles;
  dwa_point_t tracked_centroid;
  float tracked_target_heading;
} dwa_geometric_trajectory_selector_t;

void dwa_geometric_selector_init(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_geometric_selector_config_t *config);
void dwa_geometric_selector_init_default(dwa_geometric_trajectory_selector_t *selector);

bool dwa_geometric_selector_set_config(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_geometric_selector_config_t *config,
  const char **reason);

void dwa_geometric_selector_reset(dwa_geometric_trajectory_selector_t *selector);

/* reset_geometry_target() 直译（tick 无有效轨迹时调用）。 */
void dwa_geometric_selector_reset_geometry_target(
  dwa_geometric_trajectory_selector_t *selector);

/*
 * select(plan, input) 直译；结果写入 *selection（由调用方持有，
 * 每次 select 开头按默认构造语义重置标量字段，selected_trajectory 容量复用）。
 */
void dwa_geometric_selector_select(
  dwa_geometric_trajectory_selector_t *selector,
  const dwa_plan_result_t *plan,
  const dwa_geometric_selector_input_t *input,
  dwa_geometric_selection_t *selection);

#ifdef __cplusplus
}
#endif

#endif /* DWA_GEOMETRIC_SELECTOR_H */
