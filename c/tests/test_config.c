/*
 * P1 验收：加载 config/dwa_planner.yaml，输出参数表并与 C++ 版
 * （node_constructor 默认值 + yaml 覆盖 + clamp）对照。
 * 用法: test_config [yaml路径] [期望值表路径(可选)]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/config/dwa_params.h"

/* P4.5：DWA_PI 已 float 化；期望值计算保持 double π（C++ 基线路径）。 */
static const double TEST_PI = 3.14159265358979323846;

static int failures = 0;

static void expect_double(const char *name, double actual, double expected)
{
  /* P4.5：参数字段已 float 化，期望值经同样 float 舍入后比较（1 ulp 容差）。 */
  const double expected_f = (double)(float)expected;
  if (!(actual == expected_f || fabs(actual - expected_f) <= 1e-6)) {
    printf("FAIL %s: actual=%.17g expected(float)=%.17g\n", name, actual, expected_f);
    ++failures;
  }
}

static void expect_int(const char *name, long actual, long expected)
{
  if (actual != expected) {
    printf("FAIL %s: actual=%ld expected=%ld\n", name, actual, expected);
    ++failures;
  }
}

static void expect_str(const char *name, const char *actual, const char *expected)
{
  if (strcmp(actual, expected) != 0) {
    printf("FAIL %s: actual=%s expected=%s\n", name, actual, expected);
    ++failures;
  }
}

int main(int argc, char **argv)
{
  const char *yaml_path = argc > 1 ? argv[1] : "../../../config/dwa_planner.yaml";
  dwa_planner_params_t params;
  const int rc = dwa_planner_params_load(yaml_path, &params);
  if (rc != 0) {
    printf("FAIL cannot load yaml: %s\n", yaml_path);
    return 1;
  }
  printf("yaml=%s entries loaded OK\n", yaml_path);
  dwa_planner_params_dump(&params);

  /* 期望值 = yaml 覆盖后的 C++ 版取值（构造路径逐条核对）。 */
  expect_str("fused_cloud_topic", params.fused_cloud_topic, "/dwa/fused_scan_base");
  expect_str("output_topic", params.output_topic, "/dwa/cmd_vel_candidate");
  expect_str("frame_id", params.frame_id, "base_link");
  expect_double("left_laser_x", params.left_laser_x, 0.545);
  expect_double("left_laser_y", params.left_laser_y, 0.290);
  expect_double("left_laser_yaw", params.left_laser_yaw, -1.5707963267948966);
  expect_int("left_laser_angle_reversed", params.left_laser_angle_reversed, 0);
  expect_double("right_laser_y", params.right_laser_y, -0.290);
  expect_double("right_laser_yaw", params.right_laser_yaw, 1.5707963267948966);
  expect_double("footprint_front", params.footprint_front, 0.50);
  expect_double("footprint_back", params.footprint_back, 0.14);
  expect_double("footprint_half_width", params.footprint_half_width, 0.295);
  expect_double("footrest_start_x", params.footrest_start_x, 0.57);
  expect_double("footrest_front_x", params.footrest_front_x, 0.75);
  expect_double("footrest_half_width", params.footrest_half_width, 0.23);
  expect_double("toe_safety_front_x", params.toe_safety_front_x, 0.90);
  expect_double("toe_safety_half_width", params.toe_safety_half_width, 0.23);
  expect_double("footprint_padding", params.footprint_padding, 0.0);
  expect_double("self_filter_body_front_x", params.self_filter_body_front_x, 0.50);
  expect_double("footprint_self_filter_padding", params.footprint_self_filter_padding, 0.05);
  expect_int("local_foot_mask_enabled", params.local_foot_mask_enabled, 0);
  expect_double("left_local_foot_angle_min_deg", params.left_local_foot_angle_min_deg, 10.0);
  expect_double("right_local_foot_angle_max_deg", params.right_local_foot_angle_max_deg, -10.0);
  expect_double("left_planning_angle_min_deg", params.left_planning_angle_min_deg, 30.0);
  expect_double("right_planning_angle_max_deg", params.right_planning_angle_max_deg, -30.0);
  expect_double("trajectory_reference_x", params.trajectory_reference_x, 0.50);
  expect_double("grid_resolution", params.grid_resolution, 0.05);
  expect_double("grid_min_x", params.grid_min_x, -0.80);
  expect_double("grid_max_x", params.grid_max_x, 3.60);
  expect_double("grid_max_abs_y", params.grid_max_abs_y, 3.00);
  expect_int("grid_unknown_is_obstacle", params.grid_unknown_is_obstacle, 0);
  expect_double("max_velocity", params.max_velocity, 0.70);
  expect_double("min_velocity", params.min_velocity, 0.0);
  expect_double("max_reverse_velocity", params.max_reverse_velocity, 0.70);
  expect_double("max_yawrate", params.max_yawrate, 1.40);
  expect_double("min_yawrate", params.min_yawrate, 0.05);
  expect_double("max_acceleration", params.max_acceleration, 1.50); /* G2(10.09)：1.00→1.50 */
  expect_double("max_deceleration", params.max_deceleration, 1.50);
  /* G1(10.09)：yaml 新增采样窗口下界专用减速度 3.00（<=0 退回 max_deceleration） */
  expect_double("sampling_deceleration_window", params.sampling_deceleration_window, 3.00);
  /* 10.08 Fix Z：yaml 值 3.20→12.00（恢复基线舵权，单周期满舵） */
  expect_double("max_d_yawrate", params.max_d_yawrate, 12.00);
  expect_double("predict_time", params.predict_time, 2.00);
  expect_double("sim_dt", params.sim_dt, 0.10);
  expect_double("control_period", params.control_period, 0.10);
  expect_int("velocity_samples", params.velocity_samples, 4);
  expect_int("yawrate_samples", params.yawrate_samples, 27);
  expect_int("planning_worker_count", params.planning_worker_count, 4);
  /* 10.08 Fix Z：yaml 值 2.50→3.50（恢复基线视距） */
  expect_double("obstacle_range", params.obstacle_range, 3.50);
  expect_double("intent_lookahead", params.intent_lookahead, 2.00);
  /* A4b(10.09)：补偿强度 yaml 化（ltw 原值 0.40/0.50/0.08/0.35/0.85/0.35，
   * 首版过补后调参——宽走廊拖曳+通道转大，权限 w_scale 4.0→2.0） */
  expect_double("yaw_feedback_gain_follow", params.yaw_feedback_gain_follow, 0.30);
  expect_double("yaw_feedback_gain_kill", params.yaw_feedback_gain_kill, 0.50);
  expect_double("yaw_feedback_fight_margin", params.yaw_feedback_fight_margin, 0.08);
  expect_double("yaw_feedback_integ_max_frac", params.yaw_feedback_integ_max_frac, 0.15);
  expect_double("yaw_feedback_integ_release", params.yaw_feedback_integ_release, 0.70);
  expect_double("yaw_feedback_ref_w_max", params.yaw_feedback_ref_w_max, 0.70);
  /* 10.09 方案①：手柄豁免快率阈值（yaml 新键） */
  expect_double("yaw_feedback_manual_slew_min_theta", params.yaw_feedback_manual_slew_min_theta, 0.10);
  /* 10.09 刀1：语义改为意图跟随增益（目标点=意图弧端点），yaml 2.00→1.00 */
  expect_double("intent_yaw_to_lateral", params.intent_yaw_to_lateral, 1.00);
  /* 10.08 Fix V3b：判死/净距双视界参数（yaml 显式配置；净距视界改固定弧长） */
  expect_int("collision_veto_enabled", params.collision_veto_enabled, 0);
  expect_double("collision_veto_reaction_sec", params.collision_veto_reaction_sec, 0.30);
  expect_double("collision_veto_min_sec", params.collision_veto_min_sec, 0.50);
  expect_double("collision_veto_max_sec", params.collision_veto_max_sec, 2.00);
  expect_double("clearance_horizon_m", params.clearance_horizon_m, 1.60);
  expect_double("toe_collision_margin_m", params.toe_collision_margin_m, 0.00);
  expect_double("front_turn_bias_start_m", params.front_turn_bias_start_m, 0.40);
  expect_double("front_turn_bias_full_m", params.front_turn_bias_full_m, 0.30);
  expect_double("front_turn_bias_side_clearance_m", params.front_turn_bias_side_clearance_m, 0.60);
  expect_int("use_speed_cost", params.use_speed_cost, 0);
  expect_int("use_path_cost", params.use_path_cost, 0);
  /* 10.10 方案A+C：窄缝塌缩评分层修复（yaml 新键） */
  expect_double("scoring_manual_turn_min_yawrate", params.scoring_manual_turn_min_yawrate, 0.50);
  expect_double("scoring_manual_turn_speed_gain", params.scoring_manual_turn_speed_gain, 0.80);
  expect_double("intent_freeze_min_yawrate", params.intent_freeze_min_yawrate, 0.00);
  /* 10.10 刀2：防抖锥直通化（valid_straight 路径手动转向不再压缩） */
  expect_int("forced_straight_manual_passthrough", params.forced_straight_manual_passthrough, 1);
  expect_double("dwa_geometric_handoff_heading_agree", params.dwa_geometric_handoff_heading_agree, 12.0 * TEST_PI / 180.0);
  expect_double("dwa_geometric_small_target_heading", params.dwa_geometric_small_target_heading, 10.0 * TEST_PI / 180.0);
  expect_double("dwa_geometric_max_heading_growth", params.dwa_geometric_max_heading_growth, 8.0 * TEST_PI / 180.0);
  expect_double("gear_peak_speeds[0]", params.gear_peak_speeds[0], 0.70);
  expect_double("gear_peak_speeds[4]", params.gear_peak_speeds[4], 1.67);
  expect_int("gear_peak_speeds_count", params.gear_peak_speeds_count, 5);
  expect_double("moving_prebrake_base_distance", params.moving_prebrake_base_distance, 0.90);
  expect_double("moving_dwa_speed_limit", params.moving_dwa_speed_limit, 0.6944);
  expect_int("stationary_turn_ring_enabled", params.stationary_turn_ring_enabled, 1);
  expect_double("stationary_turn_ring_outer_radius_m", params.stationary_turn_ring_outer_radius_m, 1.00);
  expect_double("stationary_turn_ring_inner_radius_m", params.stationary_turn_ring_inner_radius_m, 0.91);
  expect_double("stationary_turn_ring_forward_side_angle_deg", params.stationary_turn_ring_forward_side_angle_deg, 10.0);
  expect_double("stationary_turn_ring_inner_speed_ratio", params.stationary_turn_ring_inner_speed_ratio, 0.50);
  /* 10.08 Fix V4：F1d 释放防抖 */
  expect_int("bypass_near_zone_release_hold_cycles", params.bypass_near_zone_release_hold_cycles, 3);
  /* 10.08 Fix V4c：F1d 近障锁存判定距离（独立参数，板端 3.80 提前减速） */
  expect_double("bypass_near_zone_far_distance", params.bypass_near_zone_far_distance, 1.20);
  expect_double("moving_prebrake_y_down_rate_min", params.moving_prebrake_y_down_rate_min, 30.0);
  expect_double("moving_prebrake_y_down_rate_max", params.moving_prebrake_y_down_rate_max, 80.0);
  expect_double("side_detection_angle_deg", params.side_detection_angle_deg, 20.0);
  expect_int("side_hard_stop_sweep_enabled", params.side_hard_stop_sweep_enabled, 0);
  expect_int("side_hard_stop_triangle_enabled", params.side_hard_stop_triangle_enabled, 1);
  expect_double("side_hard_stop_triangle_gap_m", params.side_hard_stop_triangle_gap_m, 0.20);
  expect_double("side_hard_stop_circle_radius", params.side_hard_stop_circle_radius, 0.50);
  expect_double("front_hard_stop_escape_linear_speed", params.front_hard_stop_escape_linear_speed, 0.00);
  expect_double("front_hard_stop_escape_yawrate", params.front_hard_stop_escape_yawrate, 0.50);
  expect_double("stop_escape_trigger_sec", params.stop_escape_trigger_sec, 0.00);
  expect_double("side_wall_hold_min_yawrate", params.side_wall_hold_min_yawrate, 0.04);
  expect_double("front_corridor_width", params.front_corridor_width, 0.59);
  expect_double("front_trigger_base_width", params.front_trigger_base_width, 0.59);
  expect_double("front_trigger_start_x", params.front_trigger_start_x, 0.50);
  expect_double("front_trigger_expand_angle_deg", params.front_trigger_expand_angle_deg, 5.0);
  expect_int("controller_x_dead_low", params.controller_x_dead_low, 89);
  expect_int("controller_y_dead_high", params.controller_y_dead_high, 111);
  expect_double("planning_min_x", params.planning_min_x, -0.15);
  expect_double("planning_max_abs_y", params.planning_max_abs_y, 2.50);
  expect_int("marker_stride", params.marker_stride, 6);
  expect_int("log_trajectory_scores", params.log_trajectory_scores, 0);
  expect_int("publish_debug_visualization", params.publish_debug_visualization, 0);
  expect_double("dwa_corridor_max_heading", params.dwa_corridor_max_heading, 60.0 * TEST_PI / 180.0);
  expect_double("dwa_corridor_track_heading_tolerance", params.dwa_corridor_track_heading_tolerance, 25.0 * TEST_PI / 180.0);

  /* 映射到 planning config 与 C++ config_ 一致性抽查 */
  dwa_config_t config;
  dwa_planner_params_to_config(&params, &config);
  expect_double("config.max_velocity", config.max_velocity, 0.70);
  expect_double("config.max_yawrate", config.max_yawrate, 1.40);
  expect_double("config.footprint.front", config.footprint.front, 0.50);
  expect_double("config.footprint.footrest_start", config.footprint.footrest_start, 0.57);
  expect_double("config.obstacle_range", config.obstacle_range, 3.50);
  /* 10.08 Fix V3b：config 拷贝抽查（净距视界固定弧长 1.60m） */
  expect_double("config.clearance_horizon_m", config.clearance_horizon_m, 1.60);
  expect_double("config.toe_collision_margin_m", config.toe_collision_margin_m, 0.00);
  expect_double("config.front_turn_bias_start_m", config.front_turn_bias_start_m, 0.40);
  expect_double("config.front_turn_bias_full_m", config.front_turn_bias_full_m, 0.30);
  expect_double("config.front_turn_bias_side_clearance_m", config.front_turn_bias_side_clearance_m, 0.60);
  expect_int("config.collision_veto_enabled", config.collision_veto_enabled, 0);
  expect_int("config.unknown_is_obstacle", config.unknown_is_obstacle, 0);

  /* 默认值路径（yaml 缺失时）：与 C++ declare 默认一致。 */
  dwa_planner_params_t defaults;
  dwa_planner_params_defaults(&defaults);
  expect_double("default max_velocity", defaults.max_velocity, 1.00);
  expect_double("default trajectory_reference_x", defaults.trajectory_reference_x, 0.297);
  expect_double("default footprint_front", defaults.footprint_front, 0.34);
  expect_double("default footprint_half_width", defaults.footprint_half_width, 0.295);
  expect_double("default toe_collision_margin_m", defaults.toe_collision_margin_m, 0.00);
  expect_double("default intent_freeze_min_yawrate", defaults.intent_freeze_min_yawrate, 0.00);
  expect_int("default grid_unknown_is_obstacle", defaults.grid_unknown_is_obstacle, 1);
  expect_double("default front_trigger_start_x", defaults.front_trigger_start_x, 0.48);
  expect_double("default front_corridor_width", defaults.front_corridor_width, 0.59);
  expect_double("default stop_escape_trigger_sec", defaults.stop_escape_trigger_sec, 1.0);
  expect_double("default front_hard_stop_escape_yawrate", defaults.front_hard_stop_escape_yawrate, 0.30);
  expect_int("yaml near_obstacle_yawrate_limit_enabled", params.near_obstacle_yawrate_limit_enabled, 0);
  expect_double("default near_obstacle_max_yawrate", defaults.near_obstacle_max_yawrate, 0.70);
  expect_int("default near_obstacle_yawrate_limit_enabled", defaults.near_obstacle_yawrate_limit_enabled, 1);
  expect_int("default local_foot_mask_enabled", defaults.local_foot_mask_enabled, 1);

  if (failures == 0) {
    printf("test_config: ALL PASS\n");
    return 0;
  }
  printf("test_config: %d FAILURES\n", failures);
  return 1;
}
