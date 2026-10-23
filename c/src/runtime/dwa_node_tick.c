/*
 * 模块功能：DWA 周期主流程：按固定周期读取状态、处理感知、速度融合、
 * 规划、保护并发布控制。
 * 直译自 native/nodes/impl/node_tick.inc（逐行对应，不改逻辑/默认值/执行顺序）。
 * P6；全 float 规范。
 *
 * 直译决定：
 * - C++ RAII（tick_timing_guard / TickTimingScope）→ 显式 enter/leave +
 *   goto tick_exit 统一收尾（单线程 tick，active_tick_timing_ 语义等价）。
 * - publish_footprint / publish_obstacle_points / publish_occupancy_grid /
 *   publish_fused_occupied_cells / publish_geometric_selection /
 *   publish_candidates / publish_selected / publish_empty_selected 为 RViz
 *   可视化发布，数据面 no-op（kTrajectoryPublish 计时段随之不记账）。
 * - evaluate_command_trajectory 返回值轨迹 → 局部 dwa_trajectory_t +
 *   destroy（C++ 返回值语义）。
 */
#include "dwa_node_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "../hal/hal_log.h"
#include "../hal/hal_time.h"
#include "../common/dwa_geometry.h"
#include "../safety/stationary_turn_speed_limiter.h"

#define NODE_PI ((float)3.14159265358979323846)

/* ---------- 日志节流 key ---------- */

static const char *const kKeyFusedCloud = "dnt:fused_cloud";
static const char *const kKeySideWall = "dnt:side_wall";
static const char *const kKeyNearGear = "dnt:near_gear";
static const char *const kKeyHardStopLatch = "dnt:hard_stop_latch";
static const char *const kKeyCloudStale = "dnt:cloud_stale";
static const char *const kKeyNoTakeover = "dnt:no_takeover";
static const char *const kKeyGeoApplied = "dnt:geo_applied";
static const char *const kKeyGeoFallback = "dnt:geo_fallback";
static const char *const kKeyHandGuardSafe = "dnt:hand_guard_safe";
static const char *const kKeyHandGuardUnsafe = "dnt:hand_guard_unsafe";
static const char *const kKeyFinal = "dnt:final";
static const char *const kKeyManualTurnPenalty = "dnt:manual_turn_penalty"; /* AC1(10.10) 方案A */
static const char *const kKeyIntentFreeze = "dnt:intent_freeze";           /* AC2(10.10) 方案C */

/* 记录 DWA 无有效轨迹事件；连续多个 tick 只计一次，便于和电机桥兜底脱困比较。 */
static void dnt_record_no_valid_dwa_event(
  dwa_node_t *node, bool active, float front_clearance,
  float left_clearance, float right_clearance, int valid_count)
{
  if (!active) return;
  if (node->no_valid_escape_last_tick == 0 ||
      node->no_valid_escape_last_tick + 1 != node->tick_count) {
    ++node->no_valid_escape_event_count;
    hal_log(HAL_LOG_WARN,
      "脱困统计 source=dwa_no_valid event=%llu valid=%d "
      "front_clear=%.3f side_clear(L=%.3f,R=%.3f)",
      (unsigned long long)node->no_valid_escape_event_count,
      valid_count, front_clearance, left_clearance, right_clearance);
  }
  node->no_valid_escape_last_tick = node->tick_count;
}

/* RViz 可视化发布 no-op（数据面无 RViz）。 */
static void publish_footprint_stub(const dwa_node_t *node)
{
  (void)node;
}

static void publish_obstacle_points_stub(
  const dwa_node_t *node, const dwa_point_buf_t *obstacles)
{
  (void)node;
  (void)obstacles;
}

static void publish_occupancy_grid_stub(
  const dwa_node_t *node, const dwa_occupancy_grid_t *grid)
{
  (void)node;
  (void)grid;
}

static void publish_fused_occupied_cells_stub(
  const dwa_node_t *node, const dwa_occupancy_grid_t *grid)
{
  (void)node;
  (void)grid;
}

static void publish_geometric_selection_stub(
  const dwa_node_t *node, const dwa_geometric_selection_t *selection)
{
  (void)node;
  (void)selection;
}

static void publish_candidates_stub(
  const dwa_node_t *node, const dwa_plan_result_t *result)
{
  (void)node;
  (void)result;
}

static void publish_selected_stub(
  const dwa_node_t *node, const dwa_trajectory_t *trajectory, int valid_count)
{
  (void)node;
  (void)trajectory;
  (void)valid_count;
}

static void publish_empty_selected_stub(const dwa_node_t *node)
{
  (void)node;
}

/* publish_dynamic_velocity_diagnostics（RViz Marker；no-op）。 */
static void publish_dynamic_velocity_diagnostics_stub(
  const dwa_node_t *node,
  const dwa_dynamic_velocity_decision_t *decision,
  float front_clearance,
  float nearest_radar_distance)
{
  (void)node;
  (void)decision;
  (void)front_clearance;
  (void)nearest_radar_distance;
}

/* ---------- tick 主循环 ---------- */

void dwa_node_tick(dwa_node_t *node)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const dwa_dynamic_velocity_config_t *dc = &node->dynamic_config;
  dwa_tick_timing_t tick_timing;
  dwa_occupancy_grid_t *occupancy_grid = &node->occupancy_grid_workspace;
  dwa_point_buf_t *obstacles = &node->obstacles_workspace;
  dwa_point_buf_t *side_safety_points = &node->side_safety_points_workspace;
  dwa_geometric_scan_buf_t *left_geometric_scan_points =
    &node->left_geometric_scan_points_workspace;
  dwa_geometric_scan_buf_t *right_geometric_scan_points =
    &node->right_geometric_scan_points_workspace;
  bool fused_cloud_fresh = false;
  planner_side_clearance_diagnostics_t side_clearance_diag =
    planner_side_clearance_diagnostics_default();
  planner_side_hard_stop_circle_diagnostics_t side_hard_stop_circle_diag =
    planner_side_hard_stop_circle_diagnostics_default();
  planner_front_no_turn_stop_diagnostics_t front_no_turn_stop_diag =
    planner_front_no_turn_stop_diagnostics_default();
  planner_obstacle_diagnostics_t obstacle_diag;
  planner_grid_diagnostics_t grid_diag;
  dwa_velocity_t intent;
  dwa_dynamic_velocity_input_t dynamic_input = dwa_dynamic_velocity_input_default();
  planner_motor_feedback_snapshot_t motor_feedback;
  dwa_corridor_guide_input_t corridor_input = dwa_corridor_guide_input_default();
  const dwa_corridor_guidance_t *corridor_guidance;
  bool corridor_reference_active;
  dwa_side_wall_hold_input_t side_wall_hold_input =
    dwa_side_wall_hold_input_default();
  dwa_side_wall_hold_result_t side_wall_hold;
  dwa_velocity_t hand_intent;
  dwa_dynamic_velocity_decision_t raw_decision;
  dwa_dynamic_velocity_decision_t decision;
  bool hard_stop_condition;
  bool moving_dwa_takeover;
  bool static_dwa_takeover;
  bool forward_corridor_clear;
  float runtime_max_velocity;
  float moving_prebrake_y_down_rate;
  float planning_hold_target;
  float stationary_turn_ring_nearest = (float)INFINITY;
  dwa_twist_t output;
  bool odom_fresh;
  dwa_velocity_t current;
  const char *current_velocity_source;
  dwa_config_t planning_config;
  dwa_plan_result_t *result = &node->dwa_plan_workspace;
  float plan_ms = 0.0f;
  dwa_velocity_t raw_dwa_selected;
  int raw_dwa_direction;
  dwa_geometric_selection_t geometric_selection;
  bool hand_command_requested = false;
  const char *hand_command_source = "none";
  float hand_angular = 0.0f;
  bool first_moving_dwa_plan;
  bool output_hall_speed_control_active;
  bool final_command_published;
  bool hard_stop_latched = false; /* C++ hard_stop_latched_ 的本周期局部镜像 */

  /* geometric_selection 默认构造（C++ 声明即构造；tick_exit 统一 destroy）。 */
  dwa_geometric_selection_init(&geometric_selection);
  dwa_tick_timing_init(&tick_timing);
  ++node->tick_count;
  /* P9 链路诊断：每周期清零；各发布路径沿途打标签（robot_main 无条件
   * 打印 [DWA链路] 行，定位 raw DWA 与最终输出之间哪条策略改写指令）。 */
  node->diag_raw_dwa_v = 0.0f;
  node->diag_raw_dwa_w = 0.0f;
  node->diag_selected_v = 0.0f;
  node->diag_selected_w = 0.0f;
  node->diag_planning_cap = 0.0f;
  node->diag_runtime_max = 0.0f;
  node->diag_center_v = 0.0f;
  node->diag_center_w = 0.0f;
  node->diag_takeover = 0;
  node->diag_mode = -1;
  node->diag_valid_count = 0;
  strncpy(node->diag_path, "early_exit", sizeof(node->diag_path) - 1);
  node->diag_path[sizeof(node->diag_path) - 1] = '\0';
  node->stationary_turn_ring_speed_limit_mps = (float)INFINITY;
  node->stationary_turn_ring_nearest_distance_m = (float)INFINITY;
  publish_footprint_stub(node);

  /* 把两个雷达输入合成到本周期需要的占用栅格里。 */
  obstacles->count = 0;
  side_safety_points->count = 0;
  left_geometric_scan_points->count = 0;
  right_geometric_scan_points->count = 0;
  {
    dwa_fused_base_point_t *fused_cloud_points = NULL;
    size_t fused_cloud_count = 0;
    double fused_cloud_time = 0.0;

    dwa_tick_timing_enter(&tick_timing, DWA_TICK_SEGMENT_POINT_CLOUD);
    dnp_reset_occupancy_grid(node, occupancy_grid);
    hal_mutex_lock(&node->scan_mutex);
    /* fused_cloud_points_ 拷贝到本地（vector 拷贝语义）：
     * 此处直接引用缓冲（单写者 tick + 互斥回调，读期间不会被释放，
     * 与 C++ 拷贝的差别仅为生命周期；数据内容逐位一致）。 */
    fused_cloud_points = node->fused_cloud_points;
    fused_cloud_count = node->fused_cloud_count;
    fused_cloud_time = node->fused_cloud_time;
    hal_mutex_unlock(&node->scan_mutex);
    fused_cloud_fresh =
      !dwa_node_stale(fused_cloud_time, p->scan_timeout_sec);
    if (fused_cloud_fresh) {
      size_t i;
      for (i = 0; i < fused_cloud_count; ++i) {
        const dwa_fused_base_point_t *sample = &fused_cloud_points[i];
        const dwa_point_t *point = &sample->point;
        if (!isfinite(point->x) || !isfinite(point->y)) continue;
        if (!sample->front_safety_only &&
            dwa_point_inside_footprint_self_filter(point->x, point->y,
              &node->config.footprint, 0.0f)) {
          continue;
        }
        if (!sample->front_safety_only &&
            !sample->side_safety_only &&
            p->rider_self_filter_enabled &&
            point->x >= p->rider_self_filter_start_x &&
            point->x <= p->rider_self_filter_front_x &&
            fabsf(point->y) <= p->rider_self_filter_half_width) {
          continue;
        }
        stationary_turn_ring_nearest = stsl_nearest_obstacle_distance(
          point, 1, stationary_turn_ring_nearest);
      }
    }
    node->body_masked_points_this_tick = 0;
    node->side_flank_points_this_tick = 0; /* 9.28 Fix D */
    node->front_unclipped_clearance_this_tick = (float)INFINITY; /* V4d(10.08)：每周期重置，由 dnp_project_base_cloud_to_grid 在截断前统计 */
    node->front_safety_candidates_this_tick = 0;
    node->front_safety_hits_this_tick = 0;
    node->local_foot_masked_points_this_tick = 0;
    node->left_angle_cropped_points_this_tick = 0;
    node->right_angle_cropped_points_this_tick = 0;
    if (fused_cloud_fresh) {
      dnp_project_base_cloud_to_grid(
        node, fused_cloud_points, fused_cloud_count, &side_clearance_diag,
        occupancy_grid, obstacles,
        left_geometric_scan_points, right_geometric_scan_points,
        &side_hard_stop_circle_diag, &front_no_turn_stop_diag,
        side_safety_points);
    }
    /* 融合节点输出已经是统一base_link点云；DWA保留所有有效点，不再二次去重。 */
    dnp_mark_fused_obstacles_in_grid(node, obstacles, occupancy_grid);
    hal_log_throttle(hal_steady_now_sec(), 1000, kKeyFusedCloud, HAL_LOG_INFO,
      "融合点云: fresh=%d points=%zu side=%zu DWA二次去重=关闭",
      fused_cloud_fresh ? 1 : 0, obstacles->count, side_safety_points->count);
    dnp_clear_current_footprint(node, occupancy_grid);
    dnp_inflate_occupied_cells(node, occupancy_grid);
    dnp_clear_current_footprint(node, occupancy_grid);
    publish_obstacle_points_stub(node, obstacles);
    publish_occupancy_grid_stub(node, occupancy_grid);
    publish_fused_occupied_cells_stub(node, occupancy_grid);
    obstacle_diag = dnp_diagnose_obstacles(node, obstacles);
    if (p->occupancy_grid_build_enabled) {
      grid_diag = dnp_diagnose_grid(occupancy_grid);
    }
    dwa_tick_timing_leave(&tick_timing);
  }

  /* 把控制器意图、霍尔速度和前方净距交给速度状态机。 */
  intent = dwa_node_read_intent(node);
  dynamic_input.now_sec = (float)dwa_node_now_sec();
  dynamic_input.forward_intent = intent.v > p->deadband_linear;
  motor_feedback = dwa_node_consume_motor_feedback(
    node, dynamic_input.forward_intent);
  {
    stsl_config_t ring_config;
    stsl_result_t ring_result;
    const float gear1_speed = p->gear_peak_speeds_count > 0 ?
      p->gear_peak_speeds[0] : p->max_velocity;
    ring_config.enabled = p->stationary_turn_ring_enabled;
    ring_config.controller_center_x = 100;
    ring_config.controller_center_y = 100;
    ring_config.controller_x_dead_low = p->controller_x_dead_low;
    ring_config.controller_x_dead_high = p->controller_x_dead_high;
    ring_config.controller_y_dead_low = p->controller_y_dead_low;
    ring_config.forward_side_angle_deg = p->stationary_turn_ring_forward_side_angle_deg;
    ring_config.outer_radius_m = p->stationary_turn_ring_outer_radius_m;
    ring_config.inner_radius_m = p->stationary_turn_ring_inner_radius_m;
    ring_config.gear1_speed_mps = gear1_speed;
    ring_config.inner_speed_ratio = p->stationary_turn_ring_inner_speed_ratio;
    ring_result = stsl_evaluate(&ring_config,
      motor_feedback.controller_x, motor_feedback.controller_y,
      motor_feedback.controller_axes_valid,
      fused_cloud_fresh, stationary_turn_ring_nearest);
    if (ring_result.speed_limited) {
      node->stationary_turn_ring_speed_limit_mps = ring_result.speed_limit_mps;
      node->stationary_turn_ring_nearest_distance_m =
        ring_result.nearest_obstacle_distance_m;
    }
  }
  dynamic_input.forward_start_rising = motor_feedback.forward_start_rising;
  dynamic_input.perception_fresh = fused_cloud_fresh;
  dynamic_input.hall_speed_fresh = motor_feedback.fresh;
  dynamic_input.actual_forward_speed = motor_feedback.forward_speed;
  dynamic_input.raw_gear = motor_feedback.raw_gear;
  dynamic_input.raw_gear_valid = motor_feedback.raw_gear_valid;
  /* V4d(10.08)：前向净距优先用未截断点云统计（统计与排除语义见
   * dnp_project_base_cloud_to_grid 的 V4d 注释）——障碍表的
   * nearest_front 被 obstacle_range(板端3.50) 钳在≈2.6m，F1d 的
   * bypass_near_zone_far_distance(3.80) 永远触发不了。两者排除语义
   * 一致（侧救援点/足迹自点/骑乘带点都排除）；无命中或点云不新鲜
   * 时两者同为 INFINITY，fallback 兜底不改旧行为。 */
  dynamic_input.front_clearance =
    isfinite(node->front_unclipped_clearance_this_tick) ?
    node->front_unclipped_clearance_this_tick :
    obstacle_diag.nearest_front;
  dynamic_input.gear_front_clearance = obstacle_diag.nearest_gear_front;
  dynamic_input.left_side_clearance = side_clearance_diag.left;
  dynamic_input.right_side_clearance = side_clearance_diag.right;
  /* 9.28 Fix D：侧净距诊断（含跨车身救援点，[DWA链路] 行打印） */
  node->diag_side_clear_left = side_clearance_diag.left;
  node->diag_side_clear_right = side_clearance_diag.right;
  dynamic_input.global_nearest_radar_distance =
    obstacle_diag.nearest_radar_distance;
  dynamic_input.sweep_clearance = (float)INFINITY;
  {
    size_t i;
    for (i = 0; i < side_safety_points->count; ++i) {
      const dwa_point_t *point = &side_safety_points->points[i];
      float sweep_distance;
      if (!isfinite(point->x) || !isfinite(point->y) || point->x < -0.10f) continue;
      /* 骑乘者腿部点不算扫掠障碍，否则会卡住速度恢复门和近障碍限档解除。 */
      if (dnp_point_inside_rider_self_filter(node, point->x, point->y)) continue;
      sweep_distance = hypotf(
        point->x - p->side_hard_stop_circle_center_x,
        point->y - p->side_hard_stop_circle_center_y);
      if (isfinite(sweep_distance) &&
        sweep_distance < dynamic_input.sweep_clearance)
      {
        dynamic_input.sweep_clearance = sweep_distance;
      }
    }
  }

  /* 独立近障碍物限档：以包含脚踏/脚尖的物理车体中心为圆心，
   * 只根据融合点云判断，不进入DWA、不修改轨迹。 */
  node->near_obstacle_gear_nearest_distance = (float)INFINITY;
  if (fused_cloud_fresh && dc->near_obstacle_gear_enabled) {
    size_t i;
    for (i = 0; i < obstacles->count; ++i) {
      const dwa_point_t *point = &obstacles->points[i];
      const float dx = point->x - dc->near_obstacle_gear_center_x;
      const float dy = point->y - dc->near_obstacle_gear_center_y;
      const float distance = hypotf(dx, dy);
      if (isfinite(distance) &&
        distance <= dc->near_obstacle_gear_release_radius &&
        distance < node->near_obstacle_gear_nearest_distance)
      {
        node->near_obstacle_gear_nearest_distance = distance;
      }
    }
    {
      const float threshold = node->near_obstacle_gear_latched ?
        dc->near_obstacle_gear_release_radius : dc->near_obstacle_gear_radius;
      const bool within_near_obstacle_radius =
        isfinite(node->near_obstacle_gear_nearest_distance) &&
        node->near_obstacle_gear_nearest_distance <= threshold;
      /* 走廊墙面可能落在1m圆内，但只要前方净距、左右侧净距和扫掠区
       * 都满足恢复门槛，就不应把“正常侧墙”误判成近障碍限档。 */
      const float restore_side_limit = dc->speed_restore_side_clearance > 0.0f ?
        dc->speed_restore_side_clearance : 0.0f;
      const float restore_sweep_limit = dc->speed_restore_sweep_clearance > 0.0f ?
        dc->speed_restore_sweep_clearance : 0.0f;
      const bool front_blocked =
        isfinite(dynamic_input.front_clearance) &&
        dynamic_input.front_clearance <= dc->moving_dwa_trigger_distance;
      const bool side_blocked =
        (isfinite(dynamic_input.left_side_clearance) &&
        dynamic_input.left_side_clearance <= restore_side_limit) ||
        (isfinite(dynamic_input.right_side_clearance) &&
        dynamic_input.right_side_clearance <= restore_side_limit);
      const bool sweep_blocked =
        isfinite(dynamic_input.sweep_clearance) &&
        dynamic_input.sweep_clearance <= restore_sweep_limit;
      const bool near_obstacle_safety_blocked =
        front_blocked || side_blocked || sweep_blocked;
      node->near_obstacle_gear_latched =
        within_near_obstacle_radius && near_obstacle_safety_blocked;
    }
  } else {
    node->near_obstacle_gear_latched = false;
  }
  dynamic_input.near_obstacle_gear_active = node->near_obstacle_gear_latched;

  /* 硬停止只按当前新鲜融合点云的当前周期判定，不做跨周期锁存。
   * 障碍物离开前方硬停止区域后，下一周期立即允许重新规划/运动；
   * 障碍物仍在前方时，当前周期仍由安全控制强制停止。 */
  hard_stop_condition = fused_cloud_fresh &&
    ((isfinite(obstacle_diag.nearest_front_radar_distance) &&
    obstacle_diag.nearest_front_radar_distance <= p->radar_contact_distance) ||
    (isfinite(dynamic_input.front_clearance) &&
    dynamic_input.front_clearance <= 0.0f));
  node->hard_stop_latched = hard_stop_condition;
  node->hard_stop_clear_cycles = 0;
  hard_stop_latched = node->hard_stop_latched;

  corridor_input.points = side_safety_points->points;
  corridor_input.points_count = side_safety_points->count;
  corridor_input.footprint = node->config.footprint;
  corridor_input.intent = intent;
  corridor_input.perception_fresh = dynamic_input.perception_fresh;
  corridor_input.forward_intent = dynamic_input.forward_intent;
  corridor_input.max_yawrate = node->config.max_yawrate;
  corridor_guidance = dwa_corridor_guide_update(&node->corridor_guide,
    &corridor_input);
  corridor_reference_active =
    corridor_guidance->active && dynamic_input.forward_intent;

  /* 前进时保护沿墙角速度；原地转向和倒车保持原有侧边路径。 */
  side_wall_hold_input.now_sec = dynamic_input.now_sec;
  side_wall_hold_input.dt_sec = node->config.control_period;
  side_wall_hold_input.forward = dynamic_input.forward_intent;
  side_wall_hold_input.perception_fresh = dynamic_input.perception_fresh;
  side_wall_hold_input.left_clearance = side_clearance_diag.left;
  side_wall_hold_input.right_clearance = side_clearance_diag.right;
  side_wall_hold_input.requested_yawrate = intent.w;
  side_wall_hold = dwa_side_wall_hold_controller_update(
    &node->side_wall_hold_controller, &side_wall_hold_input);
  hand_intent = intent;
  hand_intent.w = side_wall_hold.output_yawrate;
  if (side_wall_hold.active) {
    hal_log_throttle(hal_steady_now_sec(), 250, kKeySideWall, HAL_LOG_INFO,
      "side wall hold side=%s raw(L=%.3f,R=%.3f) "
      "used(L=%.3f,R=%.3f) pred(L=%.3f,R=%.3f) "
      "close(L=%.3f,R=%.3f)mps held=%d/%d "
      "latch=%d/%d active=%d/%d raw_w=%+.3f correction=%+.3f out_w=%+.3f",
      dwa_side_wall_hold_side_name(side_wall_hold.side),
      side_clearance_diag.left, side_clearance_diag.right,
      side_wall_hold.left_clearance, side_wall_hold.right_clearance,
      side_wall_hold.left_predicted_clearance,
      side_wall_hold.right_predicted_clearance,
      side_wall_hold.left_closing_rate, side_wall_hold.right_closing_rate,
      side_wall_hold.left_clearance_held ? 1 : 0,
      side_wall_hold.right_clearance_held ? 1 : 0,
      side_wall_hold.left_latched ? 1 : 0,
      side_wall_hold.right_latched ? 1 : 0,
      side_wall_hold.left_active ? 1 : 0,
      side_wall_hold.right_active ? 1 : 0,
      intent.w, side_wall_hold.correction_yawrate, hand_intent.w);
  }

  dynamic_input.restore_pose_stable =
    fabsf(intent.w) <= dc->speed_restore_max_yawrate &&
    fabsf(hand_intent.w) <= dc->speed_restore_max_yawrate;
  if (p->dynamic_velocity_fusion_enabled) {
    raw_decision = dwa_dynamic_velocity_fusion_update(
      &node->dynamic_fusion, &dynamic_input);
  } else {
    /* 9.27 A/B 实测开关：速度模块整体旁路（yaml dynamic_velocity_fusion_enabled: false）。
     * 前进意图+感知新鲜 ⇒ DWA 直接接管；速度上限恒为 forward_speed_limit，
     * 无预减速/净距锁存/比例减速；档位透传控制器原始档。硬停止锁存、
     * 倒车、无前进分量的原地转向仍走原有安全通道（takeover=false 时
     * 手柄路径不变）。 */
    raw_decision = dwa_dynamic_velocity_decision_default();
    raw_decision.mode = DYNAMIC_CONTROL_MOVING_DWA;
    /* F1d(9.27)：速度目标策略。【H1(10.09) 修订：空旷态档位峰值已废除，
     * MOVING_DWA 期间 target 恒 forward_speed_limit(0.70)，全档位避障
     * 等效 1 档——见下方 H1 注释块与 014339 定案。两态描述仅存档。】
     * - 避障区（前方净距 < far）：target = forward_speed_limit(0.70)，
     *   即实测验证正确的避障速度，恒定不压零；DWA 评分/绕行/硬停链
     *   照旧，桥在 0.70 上限内执行 DWA 决策。
     * - 空旷区（净距 > far+迟滞，或无障碍/净距无效）：target = 档位
     *   峰值 gear_peak_speeds[raw_gear]，恢复档位原生速度。【已废】
     * - 迟滞 0.30m 防边界净距噪声导致目标抖动（=反复加减速）。
     * 已撤销方案教训：F1b max(基线,实速) 是实速棘轮——接管切换瞬间
     * 原生档速冒头→target 跟着抬高→滑行回落，即加减速振荡（073950
     * 日志 1.07→0.90 滑行段实锤）；F1c 线性到零曲线在净距<2.2m 即
     * 压速、≤0.40m 归零，与 DWA 绕行抢速度，避障明显劣化。本方案
     * 两态均不依赖实速，棘轮与抢速度同时消除。 */
    {
      /* V4c(10.08)：近障锁存判定距离改用独立参数 bypass_near_zone_far_distance，
       * 与 moving_prebrake_profile_far_distance(距离比例减速 profile 消费方)
       * 解耦。H2(10.09) 板端收紧到 1.20：锁存=近障最后硬闸（前方1.2m内
       * 锁1档避障），更远的减速交给 DWA 评分层自行处理（判死视界2s）；
       * 3.80 旧值因大走廊边界抖动顿挫已废（014339 定案）。 */
      const float zone_far = (dc->bypass_near_zone_far_distance > 0.0f)
        ? dc->bypass_near_zone_far_distance
        : dc->moving_prebrake_profile_far_distance;
      const float zone_release = zone_far + 0.30f;
      const float clearance = dynamic_input.front_clearance;
      /* V4a(10.08)：释放防抖——latch 解除须连续 hold_cycles 个周期净距
       * 超 release。073233 日志实锤 fwd 单周期闪烁（1.0~1.4 与 2.5~7.6
       * 间跳变）会让 latch 瞬间释放→cap 回档位峰值→重新加速→再次断崖
       * （yo-yo 回冲，23821→23875→24055 链）。连续计数吸收闪烁；
       * 正常出通道仅延迟 hold_cycles×周期 恢复全速，无感知。
       * A3(10.09)：释放加侧净距门控（用户定案"通道内见缝隙不许
       * 加速"）——前方净空之外还须两侧任一侧净距>release_side_
       * distance(0.40)：1m 窄弯里视线穿过弯道开口时 fwd 真实跳
       * 3m+ 且可持续 2s（040707 实锤 4 次误释放全在通道内，加速
       * 到 1.18 后墙点复现→再锁→再刹），仅看前方必误释放；通道
       * 内两侧恒 0.15~0.4m 永不释放，恒 0.70 到底，驶出通道两侧
       * 先张开才恢复档位速度。侧净距 INF（该侧无观测点）视为
       * 张开。门控 0=禁用（退回只看前方的 A2 行为）。 */
      const int release_hold = dc->bypass_near_zone_release_hold_cycles;
      const float release_side = dc->bypass_near_zone_release_side_distance;
      if (isfinite(clearance)) {
        if (clearance < zone_far) {
          node->bypass_near_zone_latched = true;
          node->bypass_release_count = 0;
        } else if (clearance > zone_release) {
          /* 两侧任一侧净距>门控值才算"出通道"（INF=张开）。 */
          bool side_open = true;
          if (release_side > 0.0f) {
            const float sl = dynamic_input.left_side_clearance;
            const float sr = dynamic_input.right_side_clearance;
            side_open = (!isfinite(sl) || sl > release_side) ||
              (!isfinite(sr) || sr > release_side);
          }
          if (!side_open) {
            node->bypass_release_count = 0;
          } else {
            if (node->bypass_release_count < 255u) {
              node->bypass_release_count++;
            }
            if (node->bypass_release_count >= (unsigned)release_hold) {
              node->bypass_near_zone_latched = false;
            }
          }
        } else {
          /* 迟滞带内：非明确释放样本，清零连续计数（严格防抖）。 */
          node->bypass_release_count = 0;
        }
      } else {
        node->bypass_near_zone_latched = false;
        node->bypass_release_count = 0;
      }
      /* 先算档位峰值：两态空旷区抬升 + V4b 斜坡起坡钳位都需要它。 */
      float peak = 0.0f;
      bool peak_valid = false;
      if (dynamic_input.raw_gear_valid &&
        dynamic_input.raw_gear >= 1 &&
        dynamic_input.raw_gear <= DWA_FUSION_GEAR_COUNT)
      {
        const float v = dc->gear_peak_speeds[dynamic_input.raw_gear - 1];
        if (isfinite(v)) {
          peak = v;
          peak_valid = true;
        }
      }
      /* H2(10.09)：恢复两态、触发距离收紧到近障——用户定案
       * "前方1.2m内有障碍就进1档避障，前方净空后释放1档锁定，
       * 恢复控制器原有档位速度"。
       * - H1 恒 0.70 过于一刀切：空旷直道也被压到 1 档，全程慢。
       * - F1d 原版 3.80m 触发太远：大走廊 front 在 3.80/4.10 边界
       *   抖动→tgt 翻转 12 次→顿挫（014339 定案）。
       * - H2 折中：锁存触发=前净距<bypass_near_zone_far_distance
       *   （板端 1.20），释放=净距>1.50（+0.30 迟滞）且连续 3 周期
       *   （V4a 防抖）。锁存期 target=0.70（1 档避障），释放后
       *   target=档位峰值。
       * - 高速接近时 DWA 评分层不依赖锁存自行减速/转向（判死视界
       *   2s）；锁存是最后一道硬闸：入口由 V4b 斜坡（A1 一次性
       *   武装）从实速平滑接住，近距 0.40m 内 B1 线性归零。 */
      float target = dc->forward_speed_limit;
      if (!node->bypass_near_zone_latched && peak_valid && peak > target) {
        target = peak;
      }
      /* V4b(10.08)：断崖斜坡——进近障区瞬间 target 不瞬降
       * forward_speed_limit，而是从断崖时 hall 实速按实际可减速斜率
       * （默认 0.40 m/s²≈电机桥物理减速）降到 forward_speed_limit。
       * 根因：cap 骤降后电机桥实际减速仅 ~0.4 m/s²，瞬降期间
       * (14~18 周期) DWA 仍按 v<=0.7 假想速度评分，判死视界乐观
       * 4 倍→5 档直行到墙前才追赶式急转(w 1.28)撞对侧。
       * 斜坡指令与实际减速曲线同步：实际车速不会因此变快（仍受
       * 电机桥物理限制），只修正 DWA 采样/判死视界的认知。
       * hall 无效或实速本就 <= 0.70 时不激活，行为与原版一致。
       * 起点钳到档位峰值以内：维持两态不超档位约束的语义。
       * A1(10.09)：斜坡改一次性武装（released→latched 边沿后首次
       * 到达起坡一次，ramp_done 置位后锁存期内不再重挂）。V4b 原版
       * ramp 落地后下一 tick 只要 hall 实速 >floor 就从实速重新起坡：
       * target/cap/发布 v 全部跳回实速 → hall 闭环（目标=发布 v，
       * dwa_node_safety.c hall_speed_control 消息）误差归零永不下压
       * → 4 档进通道 fb 冲到档位峰值 1.41 起飞（103434 ln=2464-2506
       * 实锤，0.700→1.287 跳变即重挂）。锁存期间 target 钉死
       * forward_speed_limit，由 hall 闭环把实速压下来。 */
      if (node->bypass_near_zone_latched &&
        dc->bypass_near_zone_ramp_enabled)
      {
        const float floor_v = dc->forward_speed_limit;
        if (!node->bypass_near_zone_ramp_active &&
          !node->bypass_near_zone_ramp_done)
        {
          float start = floor_v;
          if (dynamic_input.hall_speed_fresh &&
            isfinite(dynamic_input.actual_forward_speed) &&
            dynamic_input.actual_forward_speed > floor_v)
          {
            start = dynamic_input.actual_forward_speed;
            if (peak_valid && start > peak) {
              start = peak;
            }
          }
          node->bypass_near_zone_ramp_target = start;
          node->bypass_near_zone_ramp_active = (start > floor_v);
          node->bypass_near_zone_ramp_done = true;
        }
        if (node->bypass_near_zone_ramp_active) {
          const float step = dc->bypass_near_zone_ramp_deceleration *
            node->config.control_period;
          float ramped = node->bypass_near_zone_ramp_target - step;
          if (ramped < floor_v) {
            ramped = floor_v;
          }
          node->bypass_near_zone_ramp_target = ramped;
          if (ramped > target) {
            target = ramped;
          }
          if (ramped <= floor_v) {
            node->bypass_near_zone_ramp_active = false;
          }
        }
      } else {
        node->bypass_near_zone_ramp_active = false;
        node->bypass_near_zone_ramp_done = false;
      }
      /* B1(10.09)：锁存态近距减速——前净距 < moving_prebrake_profile_
       * zero_distance(0.40m) 时 target 按 forward_speed_limit×净距/zero
       * 线性归零（净距≤0 → 0）。103434 末段实锤：F1d 锁存态无近距减速，
       * cap 恒 0.70 到底，净距 0.05m 仍 sel 0.73——配合零余量碰撞判定
       * （障碍格中心须进足迹矩形才判死）形成末段挤压。
       * 复用既有 prebrake profile 的 zero 距离语义（曲线现成，无新参数）；
       * 只在近距段生效：净距 ≥0.40m 两态行为不变。F1c 教训（2.2m 起
       * 压速与 DWA 绕行抢速度）不复发——起压点仅 0.40m 接触段。
       * 不依赖实速；净距无效（INF）时不介入。 */
      if (node->bypass_near_zone_latched) {
        const float zero_v = dc->moving_prebrake_profile_zero_distance;
        if (zero_v > 0.0f && isfinite(clearance) && clearance < zero_v) {
          float near_cap = dc->forward_speed_limit * (clearance / zero_v);
          if (near_cap < 0.0f) {
            near_cap = 0.0f;
          }
          if (near_cap < target) {
            target = near_cap;
          }
        }
      }
      raw_decision.target_max_velocity = target;
      raw_decision.held_forward_speed = target;
    }
    raw_decision.dwa_takeover = dynamic_input.forward_intent &&
      dynamic_input.perception_fresh && !hard_stop_latched;
    raw_decision.hall_speed_control_active = true;
    raw_decision.raw_gear = dynamic_input.raw_gear;
    raw_decision.raw_gear_valid = dynamic_input.raw_gear_valid;
    raw_decision.effective_gear = dynamic_input.raw_gear_valid ?
      dynamic_input.raw_gear : dc->fallback_gear;
    /* A3(10.09)：近障锁存=档位状态强制 1 档。用户定案："必须档位本身
     * 变 1 档——只限速到 0.7 而档位还是 5，转弯仍按 5 档差速/转向
     * 特性执行，必导致过弯问题。"锁存期档位输出=moving_dwa_gear_
     * level(1)（复用 moving_dwa_force_gear=true 配置），控制器按
     * 原厂 1 档协议执行：原生限速 0.70 + 1 档转弯特性；释放（前方
     * 净空 + A3 侧净距门控）后档位回手柄原始档、cap 回档位峰值，
     * 速度/档位两态完全对称。
     * 旧代码 raw_gear 透传=档位恒手柄档：040707 全程 effective=5、
     * "motor output gear" 切换日志 0 条实锤（fusion 的 force_gear
     * 覆盖在旁路架构下是死代码，9.27 记录已预告此坑）。 */
    if (node->bypass_near_zone_latched && dc->moving_dwa_force_gear) {
      const int forced_gear = dc->moving_dwa_gear_level;
      if (forced_gear >= 1 && forced_gear <= DWA_FUSION_GEAR_COUNT) {
        raw_decision.effective_gear = forced_gear;
        raw_decision.gear_closed_loop_active = true;
      }
    }
    /* The stationary-turn ring is a gear-state limiter, not only a linear
     * speed cap. Force the native controller gear so both translation and
     * steering use first-gear behavior in the ring. */
    if (isfinite(node->stationary_turn_ring_speed_limit_mps)) {
      raw_decision.effective_gear = 1;
      raw_decision.gear_closed_loop_active = true;
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyNearGear,
        HAL_LOG_INFO,
        "stationary-turn ring forces controller gear=1 nearest=%.3fm",
        node->stationary_turn_ring_nearest_distance_m);
    }
    raw_decision.gear = raw_decision.effective_gear;
  }
  node->diag_mode = (int)raw_decision.mode;
  node->diag_takeover = raw_decision.dwa_takeover ? 1 : 0;
  /* 原始档位只读；有效档位通过控制消息传给电机桥，让控制器按原厂档位执行。 */
  node->output_effective_gear = raw_decision.effective_gear;
  node->output_gear_closed_loop_active = raw_decision.gear_closed_loop_active;
  hal_log_throttle(hal_steady_now_sec(), 500, kKeyNearGear, HAL_LOG_INFO,
    "near-obstacle gear=%d nearest=%.3fm center=(%.2f,%.2f) radius=%.2f "
    "release=%.2f",
    raw_decision.near_obstacle_gear_active ? 1 : 0,
    node->near_obstacle_gear_nearest_distance,
    dc->near_obstacle_gear_center_x,
    dc->near_obstacle_gear_center_y,
    dc->near_obstacle_gear_radius,
    dc->near_obstacle_gear_release_radius);
  dns_log_gear_closed_loop_state(node, &raw_decision, &dynamic_input, &intent);

  if (dns_handle_manual_bypass(
      node, &intent, &hand_intent, &side_wall_hold, &raw_decision,
      dynamic_input.perception_fresh, hard_stop_latched,
      dynamic_input.front_clearance,
      obstacle_diag.nearest_front_radar_distance,
      dynamic_input.global_nearest_radar_distance, &side_clearance_diag,
      &side_hard_stop_circle_diag, &front_no_turn_stop_diag,
      &motor_feedback, side_safety_points))
  {
    strncpy(node->diag_path, "manual_bypass", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }

  decision = raw_decision;
  if (hard_stop_latched && intent.v > p->deadband_linear) {
    /* handle_manual_bypass 通常在上面返回；此守卫防止后续任何重构让
     * 正向 DWA 轨迹穿过锁存。 */
    hal_log_throttle(hal_steady_now_sec(), 250, kKeyHardStopLatch,
      HAL_LOG_ERROR,
      "前方硬停止锁存阻止DWA正向输出：front_clear=%.3f "
      "front_nearest_radar=%.3f global_nearest_radar=%.3f intent_v=%.3f",
      dynamic_input.front_clearance,
      obstacle_diag.nearest_front_radar_distance,
      dynamic_input.global_nearest_radar_distance,
      intent.v);
    strncpy(node->diag_path, "hard_stop", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }
  moving_dwa_takeover = decision.dwa_takeover &&
    decision.mode == DYNAMIC_CONTROL_MOVING_DWA;
  static_dwa_takeover = decision.dwa_takeover &&
    decision.mode == DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA;
  if (!moving_dwa_takeover) {
    node->moving_dwa_command_published = false;
  }
  if (!static_dwa_takeover) {
    dns_reset_static_dwa_command_tracking(node);
  }
  forward_corridor_clear = !decision.dwa_takeover &&
    dynamic_input.perception_fresh &&
    (!isfinite(dynamic_input.front_clearance) ||
    dynamic_input.front_clearance > dc->moving_dwa_trigger_distance);
  runtime_max_velocity = dns_update_runtime_velocity_limit(node, &decision);
  node->diag_runtime_max = runtime_max_velocity;
  moving_prebrake_y_down_rate =
    dns_update_moving_prebrake_y_down_rate(node, &dynamic_input, &decision);
  planning_hold_target = runtime_max_velocity;
  if (static_dwa_takeover) {
    planning_hold_target = dns_static_dwa_planning_hold_target(
      node, &dynamic_input, runtime_max_velocity);
  }
  publish_dynamic_velocity_diagnostics_stub(
    node, &decision, dynamic_input.front_clearance,
    obstacle_diag.nearest_radar_distance);
  if (decision.mode != DYNAMIC_CONTROL_HAND || node->tick_count % 10 == 0) {
    hal_log(HAL_LOG_INFO,
      "dwa tick=%llu mode=%s hall_raw(L=%.3f,R=%.3f) hall_v=%.3f fresh=%d "
      "S_DWA=%.3f S_gear=%.3f gear_comp=%.3f gear_t=%.3f target=%.3f "
      "brake_y=%.1f/s front_clear=%.2f@%.2f,%.2f "
      "nearest_radar=%.2f@%.2f,%.2f front_nearest_radar=%.2f "
      "side(L=%.2f,R=%.2f,min=%.2f pred=%.2f close=%.2fmps "
      "limit=%.2f raw_gear=%d effective_gear=%d target_gear=%d transition=%d "
      "gear_valid=%d closed=%d hold=%d/%d peak=%.2f cap=%d) "
      "intent(v=%.2f,w=%.2f) obstacles=%zu "
      "grid(free=%zu,occupied=%zu,unknown=%zu) "
      "inside_collision=%zu body_mask=%zu leg_mask=%zu "
      "angle_crop(L=%zu,R=%zu) fused_cloud_stale=%d",
      (unsigned long long)node->tick_count,
      dwa_dynamic_control_mode_name(decision.mode),
      motor_feedback.left_mps, motor_feedback.right_mps,
      motor_feedback.forward_speed,
      dynamic_input.hall_speed_fresh ? 1 : 0,
      decision.moving_prebrake_distance, decision.gear_trigger_distance,
      decision.gear_transition_compensation_distance,
      decision.gear_transition_remaining_sec, runtime_max_velocity,
      moving_prebrake_y_down_rate,
      obstacle_diag.nearest_front, obstacle_diag.nearest_front_point.x,
      obstacle_diag.nearest_front_point.y,
      obstacle_diag.nearest_gear_front,
      obstacle_diag.nearest_radar_distance, obstacle_diag.nearest_radar_point.x,
      obstacle_diag.nearest_radar_point.y,
      obstacle_diag.nearest_front_radar_distance,
      side_clearance_diag.left, side_clearance_diag.right,
      decision.side_clearance,
      decision.side_predicted_clearance, decision.side_closing_rate,
      decision.side_speed_limit, decision.raw_gear, decision.effective_gear,
      decision.gear_target, decision.gear_transition_active ? 1 : 0,
      decision.raw_gear_valid ? 1 : 0,
      decision.gear_closed_loop_active ? 1 : 0,
      decision.gear_restore_hold_count,
      dc->gear_restore_hold_cycles,
      decision.gear_peak_speed, decision.side_cap_active ? 1 : 0,
      intent.v, intent.w,
      obstacle_diag.total,
      grid_diag.free_cells, grid_diag.occupied_cells, grid_diag.unknown_cells,
      obstacle_diag.inside_current_footprint,
      node->body_masked_points_this_tick,
      node->local_foot_masked_points_this_tick,
      node->left_angle_cropped_points_this_tick,
      node->right_angle_cropped_points_this_tick,
      fused_cloud_fresh ? 0 : 1);
  }
  /* 控制器直控或速度状态处理的指令不进入后面的 DWA 分支。 */
  if (!dynamic_input.perception_fresh) {
    dwa_twist_t stop;
    dwa_dynamic_velocity_fusion_reset(&node->dynamic_fusion);
    dwa_side_wall_hold_controller_reset(&node->side_wall_hold_controller);
    dwa_node_reset_planning_control(node);
    node->forced_straight_active = false;
    node->last_output_linear = 0.0f;
    dwa_twist_reset(&stop);
    dns_publish_command_without_trajectory(node, &stop, false,
      DYNAMIC_CONTROL_HAND, false);
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyCloudStale, HAL_LOG_ERROR,
      "forward stopped: fused base cloud stale=%d",
      fused_cloud_fresh ? 0 : 1);
    strncpy(node->diag_path, "cloud_stale", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }
  if (dns_handle_direct_moving_control(
      node, &hand_intent, &intent, &dynamic_input, &decision,
      runtime_max_velocity, &motor_feedback, side_safety_points))
  {
    strncpy(node->diag_path, "direct_ctrl", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }
  if (!decision.dwa_takeover) {
    dwa_twist_t stop;
    node->last_output_linear = 0.0f;
    dwa_twist_reset(&stop);
    dns_publish_command_without_trajectory(node, &stop, false,
      DYNAMIC_CONTROL_HAND, false);
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyNoTakeover, HAL_LOG_ERROR,
      "forward stopped: state=%s reached planner without DWA takeover",
      dwa_dynamic_control_mode_name(decision.mode));
    strncpy(node->diag_path, "no_takeover", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }

  /* 首次播种目标一次；后续霍尔指令持续更新反馈环。 */
  first_moving_dwa_plan = moving_dwa_takeover &&
    !node->moving_dwa_command_published;
  /* 为本次 DWA 周期准备初始速度和运行时限。 */
  dwa_twist_reset(&output);
  odom_fresh = !dwa_node_stale(node->odom_time, p->odom_timeout_sec);
  current = intent;
  current_velocity_source = "intent";
  if (odom_fresh) {
    current = node->current_velocity;
    current_velocity_source = "odom";
  } else {
    /* 9.27 转弯窗口修复：板端无 odom 发布者（odom_fresh 恒 false），若
     * 窗口中心取 intent.w，直行时每周期重置为
     * ±(max_d_yawrate*control_period)=±0.32 rad/s，转向无法逐周期积累——
     * 正前障碍需要 |w|≈0.8~1.4 的候选永远采不到（032446 日志直行段
     * |w| max=0.383、p50=0 实锤，带转向意图段 |w| 可达 1.40 正常）。
     * 改用上周期发布指令的 w 回放作窗口中心（经典 DWA 闭环语义）：
     * 每周期在上一输出基础上继续 ±0.32，~0.3s 可达满幅；无障碍时评分
     * 自动向 intent.w 方向逐周期回落，不会持续漂移。last_published_cmd
     * 覆盖全部发布路径（HAND 直通/PREBRAKE 直控/DWA final），即车辆
     * 实际正在执行的指令。
     * 9.27 R1 窗心混合（S 弯实测 054326 反馈修复）：lastcmd 回放有转向
     * 动量锁存——顺窗心 147:0 逆窗心 0，初始平局选右后每周期锁右
     * （"一直避障右偏"）；且手柄大幅打左而左墙仍近时 DWA 可逆手柄
     * 选右，下圈又翻左，逐圈摆动（"乱转/反转"）。混合语义（对齐 9.20
     * 基线"驾驶员指挥方向、DWA 管幅度"）：手柄带转向意图（|intent.w|>
     * deadband_angular）窗心跟手柄——S 弯/带把避障方向由手柄即时指挥，
     * 不再锁死也不逆手柄；手柄直行（|intent.w|≤死区）保留 lastcmd
     * 回放——正前避障的转向积累能力不变。 */
    if (fabsf(intent.w) > p->deadband_angular) {
      current.w = intent.w;
      current_velocity_source = "intent_w";
    } else {
      current.w = node->last_published_cmd.angular_z;
      current_velocity_source = "lastcmd_w";
    }
  }
  if (dynamic_input.hall_speed_fresh) {
    current.v = dynamic_input.actual_forward_speed;
  }

  planning_config = node->config;
  planning_config.front_turn_bias_clearance_m = dynamic_input.front_clearance;
  planning_config.front_turn_bias_active = decision.dwa_takeover &&
    dynamic_input.perception_fresh && dynamic_input.forward_intent &&
    isfinite(dynamic_input.front_clearance) &&
    dynamic_input.left_side_clearance > p->front_turn_bias_side_clearance_m &&
    dynamic_input.right_side_clearance > p->front_turn_bias_side_clearance_m &&
    fabsf(intent.w) <= p->deadband_angular;
  node->diag_front_turn_bias_active =
    planning_config.front_turn_bias_active ? 1 : 0;
  node->diag_front_turn_bias_clearance = dynamic_input.front_clearance;
  /* 规划侧唯一的速度干预是 DWA 线速度上限。预减速/退出交接和全部
   * slew/霍尔/档位行为都归电机桥。
   * 9.27 用户指令"放开所有 DWA 速度限制"：采样上限不再叠加
   * moving_dwa_speed_limit（2.5km/h 避障档常量 0.6944）——该常量曾用于
   * 近距撞墙修复的第二道钳制，但电机桥地板删除后它成了唯一的满速钉子，
   * 妨碍 DWA 用更高速度候选评分（也压扁 v 采样多样性）。
   * F1b(9.27)：接管圈采样上限直接跟 runtime_max_velocity——原
   * min(config.max_velocity, runtime) 把 runtime 抬到 0.95（霍尔实速）
   * 后又压回 yaml 常量 0.70，三道 v 闸门里第一道就是它。速度模块恢复
   * 时 runtime=距离比例目标（<0.70 减速语义保持不变）。 */
  {
    float planning_cap = node->config.max_velocity;
    if (decision.dwa_takeover && runtime_max_velocity >= 0.0f) {
      planning_cap = runtime_max_velocity;
    }
    planning_config.max_velocity = planning_cap;
  }
  if (corridor_reference_active) {
    planning_config.use_path_cost = true;
    planning_config.path_reference.enabled = true;
    planning_config.path_reference.target_y = corridor_guidance->target_y;
    planning_config.path_reference.target_heading =
      corridor_guidance->target_heading;
    planning_config.path_reference.lookahead =
      node->corridor_guide_config.trajectory_lookahead;
    planning_config.path_reference.lateral_weight =
      node->corridor_guide_config.lateral_error_weight;
    planning_config.path_reference.heading_weight =
      node->corridor_guide_config.heading_error_weight;
    planning_config.path_reference.progress_weight =
      node->corridor_guide_config.progress_weight;
  } else {
    planning_config.path_reference.enabled = false;
  }

  /* 每周期重规划 DWA，仅保留当前左右方向锁。 */

  /* 生成原始 DWA 候选轨迹。 */
  {
    const double plan_started = hal_steady_now_sec();
    dwa_tick_timing_enter(&tick_timing, DWA_TICK_SEGMENT_DWA_PLANNING);
    /* F1e(9.27)：旁路模式下评分意图速度跟两态目标。
     * 081202 日志实锤：hall 消息 target 传的是发布 v
     * （dwa_node_safety.c publish_hall_speed_control 用 output->linear_x），
     * 桥跟踪的就是 DWA 选中的速度；而评分 to_goal 目标点锚在
     * intent.v×predict_time——意图被 read_intent 钳到 0.693，更快的
     * 轨迹全部"过冲"被罚，空旷时 DWA 永远选 0.69，决策层 1.20 根本
     * 到不了桥。修复：评分意图速度改用两态 target（近障 0.70=已验证
     * 行为；空旷=档位峰值→目标点挪远→快速轨迹胜出→发布 v 上升→
     * 桥自然加速）。仅改评分副本，intent 本体不动（manual_bypass/
     * 接管判定等消费方不受影响）。 */
    dwa_velocity_t plan_intent = intent;
    /* 9.29 Fix U（对齐同事 ltw_dwa 实车验证接线）：SWH 修正进规划意图。
     * 原生基线语义里 SWH 修正只进手柄直通路径（hand_intent），DWA
     * 接管期评分意图 plan_intent=原始 intent——净距<25cm 锁存后的
     * 外纠修正量（±0.03~0.12 rad/s，斜率缓升）被整段丢弃（062033
     * 日志实锤：R=0.05 贴墙 2s、sel w=0 满速）。接线后 to_goal 目标点
     * （goal_lateral ∝ plan_intent.w）随修正量向离墙侧偏移，离墙轨迹
     * 在评分中真正变便宜。SWH 未激活时 output_yawrate=requested（恒
     * 等价直通），宽走廊零影响；双侧危险钳 0 语义保留（同事同款）。
     * 通道指引用原始 intent 更新（colleague 同序，指引不被 SWH 改写）。
     * 速度通道零触碰（SWH 全程只输出角速度）。 */
    plan_intent.w = hand_intent.w;
    if (!p->dynamic_velocity_fusion_enabled) {
      plan_intent.v = raw_decision.target_max_velocity;
      /* F1f(9.27)：评分前瞻随两态目标放大，解除 2.0m 目标点钳制。
       * 082953 日志实锤根因：评分目标点 goal_distance =
       * min(intent_lookahead, intent.v×predict_time)（dwa_scoring.c:202），
       * 板端 predict_time=intent_lookahead=2.00——3/4/5 档 reachable
       * (2.4~3.34m) 全被钳到 2.0m，最优速度恒 = goal/predict_time
       * = 2.0/2.0 = 1.0m/s（3.6km/h），与档位无关（实测 5 档仍 3.6，
       * sel v 钉 1.02~1.06、窗心实速 0.9~1.1 互锁不涨）。修复：仅
       * 旁路模式把本周期评分副本的 intent_lookahead 抬到 reachable，
       * 目标点回到档位峰值射程，最优速度=两态目标（5 档 3.34m→1.67）。
       * 近障 reachable(0.70×2=1.40)<2.00 不触发，避障评分逐字节不变；
       * goal_ratio=reachable/lookahead 抬后恒 1.0，横向牵引几何不变。
       * 只改本周期 planning_config 副本，规划层与 yaml 基线不动（对拍
       * 不受影响；intent_lookahead 全库仅 dwa_scoring.c:203/205 消费）。 */
      {
        const float reachable =
          plan_intent.v * planning_config.predict_time;
        if (isfinite(reachable) &&
          reachable > planning_config.intent_lookahead)
        {
          planning_config.intent_lookahead = reachable;
        }
      }
    }
    /* AC1(10.10) 方案A：近障锁存+强打杆期评分层低v罚。003347 定案：
     * 窄缝塌缩段 74% raw_v=sel_v=0 双零是评分自选（刀1 弧端点评分下
     * 低速贴弧候选 to_goal 更便宜），旁路 target 恒 0.70 无任何压制
     * （A5 押错层实证，勿再碰旁路）。塌缩链：选低 v→发布 0→窗心 v
     * 逐拍下滑→候选池被加速度窗压死→想恢复也爬不出，且 v≈0 差速
     * 转向推不动（045814/051608 同症状）。修复：锁存+强打杆
     * （|intent.w|>=门槛，raw 手柄意图非 SWH 改写值）+前推时，临时
     * 打开速度代价（仅本周期 planning_config 副本，yaml 基线/对拍不
     * 动），speed_cost=max(0, v_high-v) 经候选间归一化 → v=0 候选吃
     * 满罚、贴窗顶候选零罚——评分从"原地贴弧最优"翻回"贴窗顶推进
     * 最优"，车保持推进、差速有力、窗口自锁环被打破。松杆/直行/
     * 非锁存逐字节不变（use_speed_cost 基线 false）。 */
    if (!p->dynamic_velocity_fusion_enabled &&
      node->bypass_near_zone_latched &&
      p->scoring_manual_turn_speed_gain > 0.0f &&
      p->scoring_manual_turn_min_yawrate > 0.0f &&
      fabsf(intent.w) >= p->scoring_manual_turn_min_yawrate &&
      intent.v > 0.05f)
    {
      planning_config.use_speed_cost = true;
      planning_config.speed_cost_gain = p->scoring_manual_turn_speed_gain;
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyManualTurnPenalty,
        HAL_LOG_INFO,
        "scoring manual-turn speed penalty: intent_w=%.3f intent_v=%.3f gain=%.2f",
        intent.w, intent.v, p->scoring_manual_turn_speed_gain);
    }
    dwa_plan_dwa_into(
      obstacles->points, obstacles->count, &current, &plan_intent,
      &planning_config, result, occupancy_grid,
      &node->dwa_spatial_index_workspace, &node->dwa_blocking_index_workspace);
    plan_ms = (hal_steady_now_sec() - plan_started) * 1e3;
    dwa_tick_timing_leave(&tick_timing);
  }
  (void)plan_ms; /* Timing remains available through the tick timing collector. */
  (void)planning_hold_target; /* Retained for timing/diagnostic parity. */
  raw_dwa_selected = result->selected;
  dnt_record_no_valid_dwa_event(
    node,
    decision.dwa_takeover && dynamic_input.perception_fresh &&
      dynamic_input.forward_intent && result->valid_count <= 0,
    dynamic_input.front_clearance, dynamic_input.left_side_clearance,
    dynamic_input.right_side_clearance, result->valid_count);
  /* AC2(10.10) 方案C诊断：意图方向冻结触发标记（区别于评分层自选 0
   * ——raw/sel 双零但 valid>0 且 reject_reason=意图方向判死）。 */
  if (!result->selected_trajectory.valid &&
    result->selected_trajectory.reject_reason != NULL &&
    strcmp(result->selected_trajectory.reject_reason,
      "intent_direction_all_vetoed") == 0)
  {
    hal_log_throttle(hal_steady_now_sec(), 500, kKeyIntentFreeze,
      HAL_LOG_INFO,
      "dwa intent-direction freeze: intent_w=%.3f valid=%d",
      intent.w, result->valid_count);
  }
  /* P9 链路诊断：raw = 评分选中的原始 DWA 输出；sel 先置 raw，
   * 几何选择器改写时再更新。 */
  node->diag_raw_dwa_v = raw_dwa_selected.v;
  node->diag_raw_dwa_w = raw_dwa_selected.w;
  node->diag_selected_v = raw_dwa_selected.v;
  node->diag_selected_w = raw_dwa_selected.w;
  node->diag_planning_cap = planning_config.max_velocity;
  node->diag_center_v = current.v;
  node->diag_center_w = current.w;
  node->diag_valid_count = result->valid_count;
  strncpy(node->diag_path, "planned", sizeof(node->diag_path) - 1);
  node->diag_path[sizeof(node->diag_path) - 1] = '\0';
  raw_dwa_direction = dwa_turn_direction(
    raw_dwa_selected.w, p->deadband_angular);
  if (decision.dwa_takeover && result->selected_trajectory.valid)
  {
    dwa_geometric_selector_input_t selector_input =
      dwa_geometric_selector_input_default();
    selector_input.left_scan_points = left_geometric_scan_points->items;
    selector_input.left_scan_points_count = left_geometric_scan_points->count;
    selector_input.right_scan_points = right_geometric_scan_points->items;
    selector_input.right_scan_points_count = right_geometric_scan_points->count;
    selector_input.left_laser.x = node->left_mount.x;
    selector_input.left_laser.y = node->left_mount.y;
    selector_input.right_laser.x = node->right_mount.x;
    selector_input.right_laser.y = node->right_mount.y;
    selector_input.front_trigger_start_x = p->front_trigger_start_x;
    selector_input.clearance_front_edge = dns_collision_front_edge(node);
    selector_input.front_trigger_base_half_width =
      0.5f * p->front_trigger_base_width;
    selector_input.front_trigger_expand_angle =
      p->front_trigger_expand_angle_deg * NODE_PI / 180.0f;
    selector_input.intent = intent;
    /* 只从原始 DWA 评分方向中选择；不复用历史锁或脱困状态。 */
    selector_input.base_direction = raw_dwa_direction;
    selector_input.direction_epsilon = p->deadband_angular;
    {
      dwa_tick_timing_enter(&tick_timing, DWA_TICK_SEGMENT_DWA_PLANNING);
      dwa_geometric_selector_select(
        &node->geometric_selector, result, &selector_input,
        &geometric_selection);
      dwa_tick_timing_leave(&tick_timing);
    }
    if (geometric_selection.applied) {
      const dwa_velocity_t base_selected = result->selected;
      result->selected = geometric_selection.selected;
      dwa_trajectory_copy(&result->selected_trajectory,
        &geometric_selection.selected_trajectory);
      node->diag_selected_v = result->selected.v;
      node->diag_selected_w = result->selected.w;
      hal_log_throttle(hal_steady_now_sec(), 200, kKeyGeoApplied,
        HAL_LOG_INFO,
        "dwa_geometric selected raw_dir=%+d requested_dir=%+d manual_reverse=%d "
        "cluster=(n=%d center=%.3f,%.3f bounds=[%.3f,%.3f]x[%.3f,%.3f]) "
        "edge_E=(%.3f,%.3f) radar=(%.3f,%.3f) side=%s "
        "segment=[%d,%d] edge_index=%d segment_size=%d edge_candidates=%d "
        "edge_range=%.3f front_clearance=%.3f raw_target=%.1fdeg "
        "target=%.1fdeg path=%.1fdeg error=%.1fdeg raw_path=%.1fdeg "
        "raw_error=%.1fdeg handoff=%d stable=%d comp=%d comp_delta=%.1fdeg "
        "base(v=%.3f,w=%+.3f) selected(v=%.3f,w=%+.3f) eligible=%d",
        geometric_selection.raw_base_direction,
        geometric_selection.requested_direction,
        geometric_selection.manual_reverse ? 1 : 0,
        geometric_selection.cluster_point_count,
        geometric_selection.cluster_centroid.x,
        geometric_selection.cluster_centroid.y,
        geometric_selection.cluster_min_x,
        geometric_selection.cluster_max_x,
        geometric_selection.cluster_min_y,
        geometric_selection.cluster_max_y,
        geometric_selection.obstacle_point.x,
        geometric_selection.obstacle_point.y,
        geometric_selection.radar_point.x,
        geometric_selection.radar_point.y,
        geometric_selection.obstacle_from_left_scan ? "LEFT" : "RIGHT",
        geometric_selection.obstacle_segment_start,
        geometric_selection.obstacle_segment_end,
        geometric_selection.obstacle_edge_index,
        geometric_selection.obstacle_segment_size,
        geometric_selection.edge_candidate_count,
        geometric_selection.edge_radar_distance,
        geometric_selection.edge_front_clearance,
        geometric_selection.raw_target_heading * 180.0f / NODE_PI,
        geometric_selection.target_heading * 180.0f / NODE_PI,
        geometric_selection.selected_heading * 180.0f / NODE_PI,
        geometric_selection.heading_error * 180.0f / NODE_PI,
        geometric_selection.raw_heading * 180.0f / NODE_PI,
        geometric_selection.raw_heading_error * 180.0f / NODE_PI,
        geometric_selection.handoff_ready ? 1 : 0,
        geometric_selection.stable_cycles,
        geometric_selection.target_heading_compensated ? 1 : 0,
        geometric_selection.target_heading_compensation * 180.0f / NODE_PI,
        base_selected.v, base_selected.w,
        result->selected.v, result->selected.w,
        geometric_selection.eligible_count);
    } else {
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyGeoFallback,
        HAL_LOG_INFO,
        "dwa_geometric fallback reason=%s raw_dir=%+d requested_dir=%+d "
        "geometry=%d eligible=%d cluster=(n=%d center=%.3f,%.3f "
        "bounds=[%.3f,%.3f]x[%.3f,%.3f]) edge_E=(%.3f,%.3f) side=%s "
        "segment=[%d,%d] edge_index=%d segment_size=%d edge_candidates=%d "
        "edge_range=%.3f front_clearance=%.3f raw_target=%.1fdeg "
        "target=%.1fdeg nearest_path=%.1fdeg error=%.1fdeg raw_path=%.1fdeg "
        "raw_error=%.1fdeg handoff=%d stable=%d comp=%d comp_delta=%.1fdeg",
        geometric_selection.reason ? geometric_selection.reason : "",
        geometric_selection.raw_base_direction,
        geometric_selection.requested_direction,
        geometric_selection.geometry_valid ? 1 : 0,
        geometric_selection.eligible_count,
        geometric_selection.cluster_point_count,
        geometric_selection.cluster_centroid.x,
        geometric_selection.cluster_centroid.y,
        geometric_selection.cluster_min_x,
        geometric_selection.cluster_max_x,
        geometric_selection.cluster_min_y,
        geometric_selection.cluster_max_y,
        geometric_selection.obstacle_point.x,
        geometric_selection.obstacle_point.y,
        geometric_selection.obstacle_from_left_scan ? "LEFT" : "RIGHT",
        geometric_selection.obstacle_segment_start,
        geometric_selection.obstacle_segment_end,
        geometric_selection.obstacle_edge_index,
        geometric_selection.obstacle_segment_size,
        geometric_selection.edge_candidate_count,
        geometric_selection.edge_radar_distance,
        geometric_selection.edge_front_clearance,
        geometric_selection.raw_target_heading * 180.0f / NODE_PI,
        geometric_selection.target_heading * 180.0f / NODE_PI,
        geometric_selection.selected_heading * 180.0f / NODE_PI,
        geometric_selection.heading_error * 180.0f / NODE_PI,
        geometric_selection.raw_heading * 180.0f / NODE_PI,
        geometric_selection.raw_heading_error * 180.0f / NODE_PI,
        geometric_selection.handoff_ready ? 1 : 0,
        geometric_selection.stable_cycles,
        geometric_selection.target_heading_compensated ? 1 : 0,
        geometric_selection.target_heading_compensation * 180.0f / NODE_PI);
    }
  } else {
    dwa_geometric_selector_reset_geometry_target(&node->geometric_selector);
    geometric_selection.reason = decision.dwa_takeover ?
      "no_valid_dwa_trajectory" : "dwa_inactive";
  }
  publish_geometric_selection_stub(node, &geometric_selection);

  /* DWA 不接管时，验证直控轨迹是否安全。 */
  if (!decision.dwa_takeover && forward_corridor_clear) {
    hand_command_requested = true;
    hand_command_source = "corridor_clear";
    hand_angular = hand_intent.w;
  } else if (!decision.dwa_takeover &&
    dwa_node_pure_forward_intent(node, &hand_intent) &&
    dwa_node_find_valid_straight_trajectory(result) != NULL)
  {
    hand_command_requested = true;
    hand_command_source = "valid_straight";
    /* 刀2(10.10)：防抖锥直通化。开=非接管 valid_straight 路径手动转向
     * 直通原始手柄 w（与 corridor_clear 同款），不再按锥角比例压缩；
     * 关=恢复旧防抖锥。安全闸不变：下方 hand_trajectory 仍以全
     * predict_time 视界校验 (v, raw_w) 组合，判死即交回 DWA。 */
    hand_angular = node->params_snapshot.forced_straight_manual_passthrough ?
      hand_intent.w :
      dwa_node_manual_yawrate_for_straight_override(node, &hand_intent);
  }

  if (hand_command_requested) {
    /* 电机桥拥有全部速度 slew/交接；手柄指令保持原始值。
     * 9.27 近距撞墙修复：hand_guard 放行时线速度必须过距离限速钳制
     * min(手柄意图, runtime_max_velocity)。之前此处发布原始 hand_linear，
     * 而轨迹评估用"过滤后障碍表"判安全——墙心带落入骑乘过滤盲区时
     * 直行轨迹被误判 valid，原始满速被直接放行（实测 fwd 3.8→1.0m
     * 全程 v=66.5 无减速、42.1 限速平台后反弹回 52-62）。速度模块
     * 只限速不加速：手柄推得比限速小时按手柄走。 */
    const float hand_linear_raw = hand_intent.v > 0.0f ? hand_intent.v : 0.0f;
    const float hand_linear = hand_linear_raw < runtime_max_velocity ?
      hand_linear_raw : runtime_max_velocity;
    dwa_trajectory_t hand_trajectory;
    const dwa_velocity_t hand_command = { hand_linear, hand_angular };
    dwa_trajectory_init(&hand_trajectory);
    {
      dwa_tick_timing_enter(&tick_timing, DWA_TICK_SEGMENT_DWA_PLANNING);
      /* Fix V 豁免：手柄直通指令的碰撞校验是安全放行闸（valid 决定
       * 直通/接管），必须保持全 predict_time 视界判死，不参与判死
       * 视距解耦——只有 DWA 候选评分才解耦。 */
      dwa_config_t hand_eval_config = planning_config;
      hand_eval_config.collision_veto_enabled = false;
      dwa_evaluate_command_trajectory_into(
        &hand_command, obstacles->points, obstacles->count, &hand_intent,
        runtime_max_velocity, &hand_eval_config, &hand_trajectory,
        occupancy_grid, &node->dwa_spatial_index_workspace,
        &node->dwa_blocking_index_workspace);
      dwa_tick_timing_leave(&tick_timing);
    }
    if (hand_trajectory.valid) {
      bool hand_command_published;
      node->forced_straight_active = true;
      node->last_output_linear = hand_linear;
      output.linear_x = hand_linear;
      output.angular_z = hand_angular;
      strncpy(node->diag_path, "hand_safe", sizeof(node->diag_path) - 1);
      node->diag_path[sizeof(node->diag_path) - 1] = '\0';
      hand_command_published = dns_publish_motion_command_safely(
        node, &output, false, decision.mode, &motor_feedback,
        dynamic_input.perception_fresh, side_safety_points, &intent,
        "safe_hand_override", &side_clearance_diag);
      publish_candidates_stub(node, result);
      if (hand_command_published) {
        publish_selected_stub(node, &hand_trajectory, result->valid_count);
      } else {
        publish_empty_selected_stub(node);
      }
      dns_log_trajectory_scores(node, result, &tick_timing);
      hal_log_throttle(hal_steady_now_sec(), 500, kKeyHandGuardSafe,
        HAL_LOG_INFO,
        "dwa hand_guard safe source=%s d_front=%.2f out(v=%.3f,w=%.3f) "
        "clearance=%.2f",
        hand_command_source, obstacle_diag.nearest_front,
        output.linear_x, output.angular_z, hand_trajectory.min_clearance);
      dwa_trajectory_destroy(&hand_trajectory);
      goto tick_exit;
    }

    hal_log_throttle(hal_steady_now_sec(), 300, kKeyHandGuardUnsafe,
      HAL_LOG_WARN,
      "dwa hand_guard unsafe source=%s cmd(v=%.3f,w=%.3f) reject=%s; "
      "DWA retake selected(v=%.3f,w=%.3f) valid=%d",
      hand_command_source, hand_linear, hand_angular,
      hand_trajectory.reject_reason ? hand_trajectory.reject_reason : "",
      result->selected.v, result->selected.w,
      result->selected_trajectory.valid ? 1 : 0);
    dwa_trajectory_destroy(&hand_trajectory);
  }

  /* 前方未被拦截时，发布选中的 DWA 轨迹。
   * DWA 只负责输出候选轨迹速度上限；速度平滑和交接由电机桥负责。
   * 9.27 用户指令"放开所有 DWA 速度限制"：输出钳制不再叠加
   * moving_dwa_speed_limit 常量 0.6944（与采样上限同步删除，理由见上）。
   * F1b(9.27)：只保留 runtime_max_velocity 作满速上限——原逻辑
   * runtime > config.max_velocity(0.70) 时回退 0.70，是三道 v 闸门的
   * 第三道；现在 runtime 可达霍尔实速（3档≈0.95），只保留负值兜底。 */
  {
    const float capped = result->selected.v > 0.0f ? result->selected.v : 0.0f;
    float limit = runtime_max_velocity;
    if (limit < 0.0f) {
      limit = node->config.max_velocity;
    }
    output.linear_x = capped < limit ? capped : limit;
  }
  output.angular_z = result->selected.w;
  strncpy(node->diag_path, "final", sizeof(node->diag_path) - 1);
  node->diag_path[sizeof(node->diag_path) - 1] = '\0';
  if (node->forced_straight_active && !decision.dwa_takeover) {
    output.linear_x = dwa_node_slew_output_linear(node, output.linear_x);
    if (fabsf(output.linear_x - result->selected.v) <= 1e-6f) {
      node->forced_straight_active = false;
    }
  } else {
    node->forced_straight_active = false;
    node->last_output_linear = output.linear_x;
  }
  /* DWA 提供带上限的原始指令；电机桥拥有全部线速度交接、霍尔反馈和
   * slew。角速度原样通过。 */
  output_hall_speed_control_active = true;
  final_command_published = dns_publish_motion_command_safely(
    node, &output, output_hall_speed_control_active, decision.mode,
    &motor_feedback, dynamic_input.perception_fresh, side_safety_points,
    &intent, "dwa_final", &side_clearance_diag);
  if (!final_command_published) {
    if (moving_dwa_takeover) node->moving_dwa_command_published = true;
    publish_candidates_stub(node, result);
    publish_empty_selected_stub(node);
    strncpy(node->diag_path, "pub_fail", sizeof(node->diag_path) - 1);
    node->diag_path[sizeof(node->diag_path) - 1] = '\0';
    goto tick_exit;
  }
  if (moving_dwa_takeover) {
    node->moving_dwa_command_published = true;
    if (result->selected_trajectory.valid && output.linear_x > 1e-6f) {
      node->last_valid_dwa_linear = output.linear_x;
    }
    if (first_moving_dwa_plan) {
      hal_log(HAL_LOG_WARN,
        "moving DWA handoff: Hall speed control released; executing "
        "selected(v=%.3f,w=%.3f)",
        output.linear_x, output.angular_z);
    }
  } else if (static_dwa_takeover) {
    dns_note_static_dwa_command_published(node, output.linear_x);
  }
  publish_candidates_stub(node, result);
  publish_selected_stub(node, &result->selected_trajectory, result->valid_count);
  dns_log_trajectory_scores(node, result, &tick_timing);
  hal_log_throttle(hal_steady_now_sec(), 500, kKeyFinal, HAL_LOG_INFO,
    "dwa mode=%s d_front=%.2f Vdwa=%.3f hall_v=%.3f current_v=%.3f(%s) "
    "intent(v=%.2f,w=%.2f) out(v=%.2f,w=%.2f) obstacles=%zu valid=%d "
    "clearance=%.2f cost=%.2f",
    dwa_dynamic_control_mode_name(decision.mode),
    dynamic_input.front_clearance, runtime_max_velocity,
    motor_feedback.forward_speed, current.v, current_velocity_source,
    intent.v, intent.w,
    result->selected.v, result->selected.w, obstacles->count,
    result->valid_count,
    result->selected_trajectory.min_clearance,
    result->selected_trajectory.total_cost);

tick_exit:
  dwa_geometric_selection_destroy(&geometric_selection);
  dns_log_tick_timing(node, &tick_timing);
}
