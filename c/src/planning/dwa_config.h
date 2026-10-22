/*
 * 模块功能：DWA规划配置和计划结果的数据结构定义。
 * 直译自 native/planning/dwa_config.hpp。
 * 说明：DwaAngularProfile 只作为 C++ 侧工作区，不参与规划结果，未移植（见 TRANSLATION_NOTES）。
 */
#ifndef DWA_PLANNING_CONFIG_H
#define DWA_PLANNING_CONFIG_H

#include <stdbool.h>

#include "../common/dwa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float max_velocity;
  float min_velocity;
  float max_yawrate;
  float min_yawrate;
  float max_acceleration;
  float max_deceleration;
  /* G1(10.09)：采样窗口下界专用减速度。>0 时 v_low = current.v -
   * 本值×control_period（仅影响候选窗口下界）；<=0 退回 max_deceleration
   * （原行为）。判死视界 T_veto（dwa_scoring.c）与输出斜率限制
   * （dwa_node.c/dwa_node_safety.c）仍用 max_deceleration，不受影响。
   * 根因：窗口下界与判死视界共用 max_deceleration，拐角断崖时窗口
   * [0.55,0.70] 候选全死→fallback(0,0)→实速崩 0→窗口锚定实速塌缩
   * [0,0.10]→sel 钉死 0.10 爬行（摇摇车，005931 日志 44 次贴停）。 */
  float sampling_deceleration_window;
  float max_d_yawrate;
  float predict_time;
  float sim_dt;
  float control_period;
  float slow_velocity_threshold;
  int velocity_samples;
  int yawrate_samples;
  int planning_worker_count;
  float obstacle_range;
  float obstacle_cost_free_margin;
  /* Forward-only yaw-ranking nudge; runtime fields are populated each tick. */
  bool front_turn_bias_active;
  float front_turn_bias_clearance_m;
  float front_turn_bias_start_m;
  float front_turn_bias_full_m;
  float front_turn_bias_side_clearance_m;
  float intent_lookahead;
  float intent_yaw_to_lateral;
  /* 10.08 Fix V2→V3c：碰撞判死视距解耦 + 净距渐进罚（双视界）。
   * 判死视界 T_veto 内碰撞=刹车来不及→否决；净距视界=min(horizon_m,
   * 刹车视距弧长+0.30m)——低速短弧（通道内修正舵候选不被对侧墙满罚
   * 钳死）、高速以 horizon_m 封顶（不系统性压高速），弧长内记真实
   * 净距不判死（渐进转弯引导）；两视界都过期才整体跳过。仅作用于
   * DWA 候选评分；手柄直通校验与前向净距探针（prebrake/硬停数据源）
   * 保持全视界。 */
  bool collision_veto_enabled;
  float collision_veto_reaction_sec;
  float collision_veto_min_sec;
  float collision_veto_max_sec;
  float clearance_horizon_m;
  /* Extra clearance along the toe leading edge only. */
  float toe_collision_margin_m;
  float obstacle_cost_gain;
  float to_goal_cost_gain;
  float speed_cost_gain;
  float path_cost_gain;
  bool use_speed_cost;
  bool use_path_cost;
  /* AC2(10.10) 方案C：意图方向冻结门槛。|intent.w| >= 本值 且 有效
   * 候选中无任何同号 w 候选（意图方向弧全部判死）时，输出冻结 (0,0)
   * （与 no_valid_trajectory 同路径），不再选反向/原地任意方向候选。
   * 003347 实锤：窄缝里强打杆(|iw|>=0.5) 1529 行中 6% 输出与意图
   * 反向（推左 -1.10 输出右 +0.10 持续 10+ 拍）——"不受控"体感来源。
   * <=0 禁用（原行为）。方案A（低v罚）由 tick 层临时打开
   * use_speed_cost 实现，无新增配置字段。 */
  float intent_freeze_min_yawrate;
  dwa_path_reference_t path_reference;
  float trajectory_reference_x;
  float trajectory_reference_y;
  bool unknown_is_obstacle;
  dwa_footprint_t footprint;
} dwa_config_t;

/* 返回 DwaConfig 成员默认值。 */
static inline dwa_config_t dwa_config_default(void)
{
  dwa_config_t config;
  config.max_velocity = 1.00f;
  config.min_velocity = 0.00f;
  config.max_yawrate = 1.00f;
  config.min_yawrate = 0.05f;
  config.max_acceleration = 0.50f;
  config.max_deceleration = 2.00f;
  config.sampling_deceleration_window = 0.0f; /* G1：<=0 退回 max_deceleration */
  config.max_d_yawrate = 3.20f;
  config.predict_time = 3.00f;
  config.sim_dt = 0.10f;
  config.control_period = 0.10f;
  config.slow_velocity_threshold = 0.10f;
  config.velocity_samples = 3;
  config.yawrate_samples = 20;
  config.planning_worker_count = 1;
  config.obstacle_range = 2.50f;
  config.obstacle_cost_free_margin = 0.40f;
  config.front_turn_bias_active = false;
  config.front_turn_bias_clearance_m = INFINITY;
  config.front_turn_bias_start_m = 0.40f;
  config.front_turn_bias_full_m = 0.30f;
  config.front_turn_bias_side_clearance_m = 0.60f;
  config.collision_veto_enabled = false;
  config.collision_veto_reaction_sec = 0.30f;
  config.collision_veto_min_sec = 0.50f;
  config.collision_veto_max_sec = 2.00f;
  config.clearance_horizon_m = 1.60f;
  config.toe_collision_margin_m = 0.0f;
  config.intent_lookahead = 2.00f;
  config.intent_yaw_to_lateral = 1.00f;
  config.obstacle_cost_gain = 1.00f;
  config.to_goal_cost_gain = 0.80f;
  config.speed_cost_gain = 0.40f;
  config.path_cost_gain = 0.40f;
  config.use_speed_cost = false;
  config.use_path_cost = false;
  config.intent_freeze_min_yawrate = 0.0f; /* AC2：<=0 禁用 */
  config.path_reference = dwa_path_reference_default();
  config.trajectory_reference_x = 0.297f;
  config.trajectory_reference_y = 0.0f;
  config.unknown_is_obstacle = false;
  config.footprint = dwa_footprint_default();
  return config;
}

/*
 * DwaPlanResult：trajectories 为复用容量的动态数组。
 * angular_profiles / candidate_profile_indices / candidate_yawrates 为
 * C++ 侧诊断工作区（不参与规划结果），未移植；阶段耗时字段保留为 0。
 */
typedef struct {
  dwa_velocity_t selected;
  dwa_trajectory_t selected_trajectory;
  dwa_trajectory_t *trajectories;
  size_t trajectory_count;
  size_t trajectory_capacity;
  int valid_count;
  float spatial_index_ms;
  float trajectory_eval_ms;
  float normalize_ms;
  float select_ms;
} dwa_plan_result_t;

void dwa_plan_result_init(dwa_plan_result_t *result);
void dwa_plan_result_destroy(dwa_plan_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* DWA_PLANNING_CONFIG_H */
