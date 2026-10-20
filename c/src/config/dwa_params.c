/* 默认值 + yaml 覆盖 + constructor 同款 clamp，见 dwa_params.h。 */
#include "dwa_params.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dwa_geometry.h"

static void copy_str(char *dst, const char *src)
{
  strncpy(dst, src, DWA_PARAM_STRING_MAX - 1);
  dst[DWA_PARAM_STRING_MAX - 1] = '\0';
}

void dwa_planner_params_defaults(dwa_planner_params_t *p)
{
  memset(p, 0, sizeof(*p));

  /* 话题：declare_parameter<std::string> 默认值 */
  copy_str(p->fused_cloud_topic, "/dwa/fused_scan_base");
  copy_str(p->cmd_vel_raw_topic, "/cmd_vel_raw");
  copy_str(p->motor_feedback_topic, "/dwa/motor_feedback");
  copy_str(p->hall_speed_control_topic, "/dwa/hall_speed_control");
  copy_str(p->odom_topic, "/odom");
  copy_str(p->output_topic, "/dwa/cmd_vel_candidate");
  copy_str(p->frame_id, "base_link");

  /* 雷达安装：constructor 默认（left 0.48/0.25/0.0，右对称） */
  p->left_laser_x = 0.48f;  p->left_laser_y = 0.25f;  p->left_laser_yaw = 0.0f;
  p->left_laser_angle_reversed = false;
  p->left_laser_angle_offset_deg = 0.0f;
  p->right_laser_x = 0.48f; p->right_laser_y = -0.25f; p->right_laser_yaw = 0.0f;
  p->right_laser_angle_reversed = false;
  p->right_laser_angle_offset_deg = 0.0f;

  /* 车体包络：DwaFootprint 成员默认 + constructor 无覆盖 */
  p->footprint_front = 0.34f; p->footprint_back = 0.32f; p->footprint_half_width = 0.315f;
  p->footrest_start_x = 0.0f; p->footrest_front_x = 0.0f; p->footrest_half_width = 0.0f;
  p->toe_safety_front_x = 0.0f; p->toe_safety_half_width = 0.0f;
  p->footprint_padding = 0.0f;
  p->self_filter_body_front_x = 0.48f;      /* node_members 默认 */
  p->footprint_self_filter_padding = 0.02f; /* node_members 默认 */

  /* 骑乘者身体区自点过滤：P9 板端新增，C++ 生产版无此参数。
   * Fix H(9.28)：默认关闭，对齐 9.11-ok 基线（基线无骑乘带，
   * 膝/脚点作为前方障碍容忍=保守减速）。实测四轮（F3 墙闸→
   * Fix E x 跨度判据→Fix F 分侧闸→Fix G 墙保持）均未解决窄通道
   * 蹭墙，根因即此带与窄墙同区域：窄墙 |y|≤0.35 整段落在带内，
   * 点云闪烁导致墙时隐时现。跨车身补充观测已在 robot_main 恢复
   * 直通（Fix H），无需再靠"簇形状猜墙"。 */
  p->rider_self_filter_enabled = false;
  p->rider_self_filter_start_x = 0.76f;  /* 碰撞前缘(脚踏0.75+余量)，覆盖脚尖 */
  p->rider_self_filter_front_x = 1.35f; /* 膝盖最前位置（实测约1.11，留余量） */
  p->rider_self_filter_half_width = 0.35f;
  /* F3(9.27) 宽簇墙闸：带内点按 y 排序聚类，存在横向宽度 ≥ preserve_width
   * 的连续簇（墙/大面积障碍）时本周期停用骑乘过滤——防 F2(a) 盲区带复发
   * （宽墙整面被删→满速冲墙→硬停循环）。窄簇（膝盖/脚尖）仍正常过滤。 */
  p->rider_filter_cluster_gap = 0.15f;   /* y 排序聚类最大点间间隙，单位m */
  p->rider_filter_preserve_width = 0.40f; /* 判定"墙"的最小簇横向宽度，单位m */

  /* 速度模块总开关（A/B 实测用）：默认开=原速度融合状态机；
   * yaml 置 false 后速度模块整体旁路，前进时 DWA 直接接管、恒满速。 */
  p->dynamic_velocity_fusion_enabled = true;

  /* 脚部屏蔽：node_members 默认 */
  p->local_foot_mask_enabled = true;
  p->local_foot_mask_max_range = 0.40f;
  p->left_local_foot_angle_min_deg = 25.0f; p->left_local_foot_angle_max_deg = 90.0f;
  p->right_local_foot_angle_min_deg = -90.0f; p->right_local_foot_angle_max_deg = -25.0f;

  /* 规划扇区：node_members 默认 */
  p->left_planning_angle_min_deg = -90.0f; p->left_planning_angle_max_deg = 60.0f;
  p->right_planning_angle_min_deg = -60.0f; p->right_planning_angle_max_deg = 90.0f;

  /* 轨迹参考点：DwaConfig 默认 */
  p->trajectory_reference_x = 0.297f;
  p->trajectory_reference_y = 0.0f;

  /* 栅格：node_members 默认 */
  p->grid_resolution = 0.05f;
  p->grid_min_x = -0.80f; p->grid_max_x = 3.60f; p->grid_max_abs_y = 3.00f;
  p->grid_inflation_radius = 0.0f;
  p->grid_unknown_is_obstacle = true; /* constructor 硬编码 true */
  p->occupancy_grid_build_enabled = false; /* 9.24: 默认停用建图（规划已不查询栅格） */

  /* DWA：DwaConfig 默认（max_velocity 特例见下） */
  p->max_velocity = 1.00f;    /* DwaConfig.max_velocity 默认；declare 使用该默认 */
  p->min_velocity = 0.00f;
  p->max_reverse_velocity = 0.70f;
  p->max_yawrate = 1.00f;
  p->min_yawrate = 0.05f;
  p->max_acceleration = 0.50f;
  p->max_deceleration = 2.00f;
  p->sampling_deceleration_window = 0.0f; /* G1：<=0 退回 max_deceleration */
  p->max_d_yawrate = 3.20f;
  p->predict_time = 3.00f;
  p->sim_dt = 0.10f;
  p->control_period = 0.10f;
  p->slow_velocity_threshold = 0.10f;
  p->velocity_samples = 3;
  p->yawrate_samples = 20;
  p->planning_worker_count = 1;
  p->obstacle_range = 2.50f;
  /* A4(10.09)：IMU 角速度闭环补偿默认开（板端 yaml 显式控制） */
  p->yaw_feedback_enabled = true;
  p->imu_gz_sign = 1.0f;
  /* A4b(10.09)：补偿强度首版过补后的新默认（与 yaw_feedback.c 同值，
   * yaml 缺键/负值时兜底）。宽走廊拖曳=泄漏慢+权限大；通道过转=
   * 积分上限 0.49 远超通道指令 ~0.14。 */
  p->yaw_feedback_gain_follow = 0.30f;
  p->yaw_feedback_gain_kill = 0.50f;
  p->yaw_feedback_fight_margin = 0.08f;
  p->yaw_feedback_integ_max_frac = 0.15f;
  p->yaw_feedback_integ_release = 0.70f;
  p->yaw_feedback_ref_w_max = 0.70f;
  p->yaw_feedback_manual_slew_min_theta = 0.10f;
  p->obstacle_cost_free_margin = 0.40f;
  p->front_turn_bias_start_m = 0.40f;
  p->front_turn_bias_full_m = 0.30f;
  p->front_turn_bias_side_clearance_m = 0.60f;
  p->intent_lookahead = 2.00f;
  p->intent_yaw_to_lateral = 1.00f;
  p->collision_veto_enabled = false;
  p->collision_veto_reaction_sec = 0.30f;
  p->collision_veto_min_sec = 0.50f;
  p->collision_veto_max_sec = 2.00f;
  p->clearance_horizon_m = 1.60f;
  p->toe_collision_margin_m = 0.10f;
  p->obstacle_cost_gain = 1.00f;
  p->to_goal_cost_gain = 0.80f;
  p->speed_cost_gain = 0.40f;
  p->path_cost_gain = 0.40f;
  p->use_speed_cost = false;
  p->use_path_cost = false;
  /* AC1/AC2(10.10)：方案A+C 默认关（板端 yaml 显式开启），代码默认
   * 0=禁用保证对拍/单测行为不变。 */
  p->scoring_manual_turn_min_yawrate = 0.0f;
  p->scoring_manual_turn_speed_gain = 0.0f;
  p->intent_freeze_min_yawrate = 0.0f;

  /* 几何二次选择：DwaGeometricSelectorConfig 默认 */
  p->dwa_geometric_selector_enabled = true;
  p->dwa_geometric_lookahead_distance = 0.50f;
  p->dwa_geometric_min_lookahead_distance = 0.15f;
  p->dwa_geometric_min_forward_offset = 0.05f;
  p->dwa_geometric_cluster_gap_distance = 0.18f;
  p->dwa_geometric_cluster_forward_window = 0.40f;
  p->dwa_geometric_edge_front_band = 0.10f;
  p->dwa_geometric_cluster_track_tolerance = 0.25f;
  p->dwa_geometric_centered_entry_max_width = 0.45f;
  p->dwa_geometric_handoff_stable_cycles = 3;
  p->dwa_geometric_handoff_heading_agree = 12.0f * DWA_PI / 180.0f;
  p->dwa_geometric_raw_entry_yawrate = 0.30f;
  p->dwa_geometric_min_candidate_clearance = 0.05f;
  p->dwa_geometric_max_clearance_loss = 0.08f;
  p->dwa_geometric_min_target_heading = 0.0f;
  p->dwa_geometric_near_small_angle_distance = 0.50f;
  p->dwa_geometric_small_target_heading = 10.0f * DWA_PI / 180.0f;
  p->dwa_geometric_near_target_heading_compensation = 7.0f * DWA_PI / 180.0f;
  p->dwa_geometric_max_heading_growth = 8.0f * DWA_PI / 180.0f;
  p->dwa_geometric_max_error = 10.0f * DWA_PI / 180.0f;
  p->dwa_geometric_manual_override_yawrate = 0.0f;
  p->dwa_geometric_target_filter_alpha = 0.45f;
  p->dwa_geometric_heading_tie = 2.0f * DWA_PI / 180.0f;

  /* 超时/死区 */
  p->scan_timeout_sec = 0.60f;
  p->cmd_timeout_sec = 0.50f;
  p->odom_timeout_sec = 0.50f;
  p->deadband_linear = 0.02f;
  p->deadband_angular = 0.02f;
  p->forced_straight_yaw_tolerance = 0.05f;
  p->forced_straight_manual_cone_deg = 45.0f;
  p->forced_straight_manual_max_yawrate = 0.30f;
  p->forced_straight_manual_passthrough = true;

  /* 档位/动态速度：DynamicVelocityConfig 默认 */
  p->gear_peak_speeds_count = 5;
  p->gear_peak_speeds[0] = 0.70f; p->gear_peak_speeds[1] = 1.00f;
  p->gear_peak_speeds[2] = 1.20f; p->gear_peak_speeds[3] = 1.40f;
  p->gear_peak_speeds[4] = 1.67f;
  p->controller_gear_fallback = 1;
  p->gear_closed_loop_enabled = true;
  p->gear_restore_hysteresis = 0.10f;
  p->gear_force_level = 1;
  p->gear_force_forward_only = true;
  p->gear_front_trigger_extra_angle_deg = 10.0f;
  p->gear_restore_hold_cycles = 10;
  p->gear_transition_step_sec = 0.15f;
  p->gear_transition_safety_margin = 0.05f;
  p->gear_speed_transition_enabled = true;
  p->gear_speed_transition_duration_sec = 2.0f;

  p->moving_prebrake_base_distance = 0.70f;
  p->moving_prebrake_speed_ratio = 0.70f;
  p->moving_prebrake_target_speed = 0.40f;
  p->moving_prebrake_deceleration = 1.20f;
  p->moving_prebrake_profile_far_distance = 2.20f;
  p->moving_prebrake_profile_zero_distance = 0.40f;
  p->moving_prebrake_recovery_acceleration = 0.40f;
  p->bypass_near_zone_release_hold_cycles = 3;
  p->bypass_near_zone_ramp_enabled = true;
  p->bypass_near_zone_ramp_deceleration = 0.40f;
  p->bypass_near_zone_far_distance = 2.20f;
  /* A3(10.09)：释放侧净距门控（板端 yaml 显式覆盖 0.40）。 */
  p->bypass_near_zone_release_side_distance = 0.40f;
  p->moving_dwa_trigger_distance = 0.50f;
  p->moving_dwa_entry_speed_max = 0.45f;
  p->moving_release_hysteresis = 0.80f;
  p->moving_dwa_speed_limit = 0.42f;
  p->moving_dwa_yawrate_limit = 0.70f;
  p->moving_dwa_force_gear = true;
  p->moving_dwa_gear_level = 1;

  p->near_obstacle_gear_enabled = true;
  p->near_obstacle_gear_center_x = 0.38f;
  p->near_obstacle_gear_center_y = 0.0f;
  p->near_obstacle_gear_radius = 1.0f;
  p->near_obstacle_gear_release_radius = 1.10f;
  p->near_obstacle_gear_level = 1;
  p->stationary_turn_ring_enabled = true;
  p->stationary_turn_ring_outer_radius_m = 1.00f;
  p->stationary_turn_ring_inner_radius_m = 0.91f;
  p->stationary_turn_ring_forward_side_angle_deg = 10.0f;
  p->stationary_turn_ring_inner_speed_ratio = 0.50f;

  p->moving_prebrake_reaction_sec = 0.30f;
  p->moving_prebrake_safety_margin = 0.05f;
  p->moving_prebrake_y_down_rate_min = 60.0f;
  p->moving_prebrake_y_down_rate_max = 180.0f;
  p->moving_prebrake_y_rate_release = 60.0f;
  p->moving_hall_speed_filter_alpha = 0.35f;

  p->static_start_distance = 0.50f;
  p->static_vdwa_gain = 2.50f;
  p->static_vdwa_recalculate_sec = 0.10f;

  /* 侧边保护：node_members 默认 */
  p->side_detection_angle_deg = 20.0f;
  p->manual_turn_forward_min_angle_deg = 65.0f;
  p->manual_turn_forward_max_angle_deg = 115.0f;
  p->side_hard_stop_circle_enabled = true;
  p->side_hard_stop_circle_center_x = 0.51f;
  p->side_hard_stop_circle_center_y = 0.0f;
  p->side_hard_stop_circle_radius = 0.44f;
  p->side_hard_stop_reverse_escape_yawrate = 0.30f;
  p->side_hard_stop_wheel_track_m = 0.50f;
  p->side_hard_stop_sweep_prediction_sec = 0.45f;
  p->side_hard_stop_sweep_brake_sec = 0.25f;
  p->side_hard_stop_sweep_min_yawrate = 0.15f;

  p->front_hard_stop_escape_enabled = true;
  p->radar_contact_distance = 0.065f;
  p->hard_stop_release_clear_cycles = 3;
  p->front_hard_stop_escape_min_clearance = 0.10f;
  p->front_hard_stop_escape_linear_speed = 0.04f;
  p->front_hard_stop_escape_yawrate = 0.30f;
  p->stop_escape_trigger_sec = 1.0f;
  p->near_obstacle_yawrate_limit_enabled = true;
  p->near_obstacle_max_yawrate = 0.70f;

  p->dwa_velocity_ramp_enabled = true;

  /* 侧边动态限速：DynamicVelocityConfig 默认 */
  p->side_speed_trigger = 1.20f;
  p->side_speed_target = 1.20f;
  p->side_limit_distance = 0.30f;
  p->side_prediction_reaction_sec = 0.25f;
  p->side_prediction_deceleration = 1.50f;
  p->side_closing_rate_filter_alpha = 0.35f;
  p->side_closing_rate_max = 1.00f;
  p->side_cap_release_distance = 0.30f;
  p->side_cap_release_clear_sec = 0.30f;
  p->side_target_down_rate = 1.50f;
  p->side_target_up_rate = 0.20f;

  /* 侧墙保持：SideWallHoldConfig 默认 */
  p->side_wall_hold_enabled = true;
  p->side_wall_hold_distance = 0.20f;
  p->side_wall_hold_correction_distance = 0.20f;
  p->side_wall_hold_prediction_sec = 0.60f;
  p->side_wall_hold_invalid_hold_sec = 0.25f;
  p->side_wall_hold_forward_margin_deg = 5.0f;
  p->side_wall_hold_release_distance = 0.22f;
  p->side_wall_hold_kp = 0.80f;
  p->side_wall_hold_min_yawrate = 0.03f;
  p->side_wall_hold_max_yawrate = 0.12f;
  p->side_wall_hold_yaw_accel = 0.25f;
  p->side_wall_hold_release_sec = 0.30f;

  p->dynamic_release_margin = 0.0f;
  p->dynamic_release_clear_sec = 0.0f;
  p->speed_restore_side_clearance = 0.45f;
  p->speed_restore_sweep_clearance = 0.45f;
  p->speed_restore_stable_sec = 1.0f;
  p->speed_restore_max_yawrate = 0.12f;

  /* 前方触发区：constructor 的计算型默认（依赖雷达安装与半宽）。 */
  p->front_corridor_width = 2.0f * p->footprint_half_width;             /* 0.63 */
  p->front_trigger_start_x = 0.5f * (p->left_laser_x + p->right_laser_x); /* 0.48 */
  p->front_trigger_base_width = 2.0f * p->footprint_half_width;         /* 0.63 */
  p->front_trigger_expand_angle_deg = 5.0f;

  p->controller_x_dead_low = 89;  p->controller_x_dead_high = 111;
  p->controller_y_dead_low = 89;  p->controller_y_dead_high = 111;

  p->hall_feedback_timeout_sec = 0.25f;

  p->debug_nominal_speed = 0.0f;
  p->force_debug_intent = false;
  p->planning_min_x = -0.15f;
  p->planning_max_abs_y = 2.50f;
  p->marker_stride = 3;
  p->log_trajectory_scores = false;
  p->publish_debug_visualization = true; /* node_members 默认 true */
  p->trajectory_score_top_n = 10;
  p->trajectory_score_log_interval_ms = 1000;

  /* 通道引导：DwaCorridorGuideConfig 默认 */
  p->dwa_corridor_guide_enabled = true;
  p->dwa_corridor_search_start_x = 0.25f;
  p->dwa_corridor_search_end_x = 1.80f;
  p->dwa_corridor_min_width_margin = 0.04f;
  p->dwa_corridor_max_width = 1.30f;
  p->dwa_corridor_search_half_width = 1.20f;
  p->dwa_corridor_cluster_gap_distance = 0.14f;
  p->dwa_corridor_cluster_min_points = 3;
  p->dwa_corridor_gate_max_depth_difference = 0.50f;
  p->dwa_corridor_gate_blocking_clearance = 0.10f;
  p->dwa_corridor_gate_endpoint_exclusion = 0.12f;
  p->dwa_corridor_gate_dedup_distance = 0.16f;
  p->dwa_corridor_gate_projection_length = 0.75f;
  p->dwa_corridor_approach_start_x = 0.25f;
  p->dwa_corridor_explicit_intent_ratio = 0.15f;
  p->dwa_corridor_max_heading = 60.0f * DWA_PI / 180.0f;
  p->dwa_corridor_intent_lateral_range = 0.70f;
  p->dwa_corridor_stable_cycles = 3;
  p->dwa_corridor_lost_hold_cycles = 4;
  p->dwa_corridor_track_lateral_tolerance = 0.35f;
  p->dwa_corridor_track_heading_tolerance = 25.0f * DWA_PI / 180.0f;
  p->dwa_corridor_path_filter_alpha = 0.35f;
  p->dwa_corridor_trajectory_lookahead = 1.20f;
  p->dwa_corridor_lateral_weight = 2.00f;
  p->dwa_corridor_heading_weight = 0.70f;
  p->dwa_corridor_progress_weight = 0.35f;
}

int dwa_planner_params_load(const char *yaml_path, dwa_planner_params_t *p)
{
  dwa_planner_params_defaults(p);
  dwa_yaml_doc_t doc;
  const int loaded = dwa_yaml_load_file(yaml_path, "dwa_local_planner", &doc);
  if (loaded < 0) return -1;
  const dwa_yaml_doc_t *d = &doc;
  double v; /* A4b：补偿强度键临时值（>0 才覆盖默认） */

  /* ---- 话题 ---- */
  dwa_yaml_get_string(d, "fused_cloud_topic", p->fused_cloud_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "cmd_vel_raw_topic", p->cmd_vel_raw_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "motor_feedback_topic", p->motor_feedback_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "hall_speed_control_topic", p->hall_speed_control_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "odom_topic", p->odom_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "output_topic", p->output_topic, DWA_PARAM_STRING_MAX);
  dwa_yaml_get_string(d, "frame_id", p->frame_id, DWA_PARAM_STRING_MAX);

  /* ---- 雷达安装 ---- */
  p->left_laser_x = dwa_yaml_get_double(d, "left_laser_x", p->left_laser_x);
  p->left_laser_y = dwa_yaml_get_double(d, "left_laser_y", p->left_laser_y);
  p->left_laser_yaw = dwa_yaml_get_double(d, "left_laser_yaw", p->left_laser_yaw);
  p->left_laser_angle_reversed = dwa_yaml_get_bool(d, "left_laser_angle_reversed", p->left_laser_angle_reversed);
  p->left_laser_angle_offset_deg = dwa_yaml_get_double(d, "left_laser_angle_offset_deg", p->left_laser_angle_offset_deg);
  p->right_laser_x = dwa_yaml_get_double(d, "right_laser_x", p->right_laser_x);
  p->right_laser_y = dwa_yaml_get_double(d, "right_laser_y", p->right_laser_y);
  p->right_laser_yaw = dwa_yaml_get_double(d, "right_laser_yaw", p->right_laser_yaw);
  p->right_laser_angle_reversed = dwa_yaml_get_bool(d, "right_laser_angle_reversed", p->right_laser_angle_reversed);
  p->right_laser_angle_offset_deg = dwa_yaml_get_double(d, "right_laser_angle_offset_deg", p->right_laser_angle_offset_deg);

  /* ---- 车体包络 ---- */
  p->footprint_front = dwa_yaml_get_double(d, "footprint_front", p->footprint_front);
  p->footprint_back = dwa_yaml_get_double(d, "footprint_back", p->footprint_back);
  p->footprint_half_width = dwa_yaml_get_double(d, "footprint_half_width", p->footprint_half_width);
  p->footrest_start_x = dwa_yaml_get_double(d, "footrest_start_x", p->footrest_start_x);
  p->footrest_front_x = dwa_yaml_get_double(d, "footrest_front_x", p->footrest_front_x);
  p->footrest_half_width = dwa_yaml_get_double(d, "footrest_half_width", p->footrest_half_width);
  p->toe_safety_front_x = dwa_yaml_get_double(d, "toe_safety_front_x", p->toe_safety_front_x);
  p->toe_safety_half_width = dwa_yaml_get_double(d, "toe_safety_half_width", p->toe_safety_half_width);
  p->footprint_padding = dwa_yaml_get_double(d, "footprint_padding", p->footprint_padding);
  p->self_filter_body_front_x = dwa_yaml_get_double(d, "self_filter_body_front_x", p->self_filter_body_front_x);
  /* constructor: std::max(0.0, param) */
  p->footprint_self_filter_padding = dwa_max0(dwa_yaml_get_double(d, "footprint_self_filter_padding", p->footprint_self_filter_padding));
  /* 骑乘者身体区自点过滤（P9 新增） */
  p->rider_self_filter_enabled = dwa_yaml_get_bool(d, "rider_self_filter_enabled", p->rider_self_filter_enabled);
  p->rider_self_filter_start_x = dwa_yaml_get_double(d, "rider_self_filter_start_x", p->rider_self_filter_start_x);
  p->rider_self_filter_front_x = dwa_yaml_get_double(d, "rider_self_filter_front_x", p->rider_self_filter_front_x);
  p->rider_self_filter_half_width = dwa_yaml_get_double(d, "rider_self_filter_half_width", p->rider_self_filter_half_width);
  p->rider_filter_cluster_gap = dwa_yaml_get_double(d, "rider_filter_cluster_gap", p->rider_filter_cluster_gap);
  p->rider_filter_preserve_width = dwa_yaml_get_double(d, "rider_filter_preserve_width", p->rider_filter_preserve_width);
  /* 速度模块总开关（A/B 实测用） */
  p->dynamic_velocity_fusion_enabled = dwa_yaml_get_bool(d, "dynamic_velocity_fusion_enabled", p->dynamic_velocity_fusion_enabled);

  /* ---- 脚部屏蔽 ---- */
  p->local_foot_mask_enabled = dwa_yaml_get_bool(d, "local_foot_mask_enabled", p->local_foot_mask_enabled);
  p->local_foot_mask_max_range = dwa_max0(dwa_yaml_get_double(d, "local_foot_mask_max_range", p->local_foot_mask_max_range));
  p->left_local_foot_angle_min_deg = dwa_yaml_get_double(d, "left_local_foot_angle_min_deg", p->left_local_foot_angle_min_deg);
  p->left_local_foot_angle_max_deg = dwa_yaml_get_double(d, "left_local_foot_angle_max_deg", p->left_local_foot_angle_max_deg);
  p->right_local_foot_angle_min_deg = dwa_yaml_get_double(d, "right_local_foot_angle_min_deg", p->right_local_foot_angle_min_deg);
  p->right_local_foot_angle_max_deg = dwa_yaml_get_double(d, "right_local_foot_angle_max_deg", p->right_local_foot_angle_max_deg);

  /* ---- 规划扇区 ---- */
  p->left_planning_angle_min_deg = dwa_yaml_get_double(d, "left_planning_angle_min_deg", p->left_planning_angle_min_deg);
  p->left_planning_angle_max_deg = dwa_yaml_get_double(d, "left_planning_angle_max_deg", p->left_planning_angle_max_deg);
  p->right_planning_angle_min_deg = dwa_yaml_get_double(d, "right_planning_angle_min_deg", p->right_planning_angle_min_deg);
  p->right_planning_angle_max_deg = dwa_yaml_get_double(d, "right_planning_angle_max_deg", p->right_planning_angle_max_deg);

  p->trajectory_reference_x = dwa_yaml_get_double(d, "trajectory_reference_x", p->trajectory_reference_x);
  p->trajectory_reference_y = dwa_yaml_get_double(d, "trajectory_reference_y", p->trajectory_reference_y);

  /* ---- 栅格 ---- */
  p->grid_resolution = dwa_yaml_get_double(d, "grid_resolution", p->grid_resolution);
  p->grid_min_x = dwa_yaml_get_double(d, "grid_min_x", p->grid_min_x);
  p->grid_max_x = dwa_yaml_get_double(d, "grid_max_x", p->grid_max_x);
  p->grid_max_abs_y = dwa_yaml_get_double(d, "grid_max_abs_y", p->grid_max_abs_y);
  p->grid_inflation_radius = dwa_yaml_get_double(d, "grid_inflation_radius", p->grid_inflation_radius);
  p->grid_unknown_is_obstacle = dwa_yaml_get_bool(d, "grid_unknown_is_obstacle", p->grid_unknown_is_obstacle);
  p->occupancy_grid_build_enabled = dwa_yaml_get_bool(d, "occupancy_grid_build_enabled", p->occupancy_grid_build_enabled);

  /* ---- DWA ---- */
  p->max_velocity = dwa_yaml_get_double(d, "max_velocity", p->max_velocity);
  p->min_velocity = dwa_yaml_get_double(d, "min_velocity", p->min_velocity);
  p->max_reverse_velocity = dwa_yaml_get_double(d, "max_reverse_velocity", p->max_reverse_velocity);
  p->max_yawrate = dwa_yaml_get_double(d, "max_yawrate", p->max_yawrate);
  p->min_yawrate = dwa_yaml_get_double(d, "min_yawrate", p->min_yawrate);
  p->max_acceleration = dwa_yaml_get_double(d, "max_acceleration", p->max_acceleration);
  p->max_deceleration = dwa_yaml_get_double(d, "max_deceleration", p->max_deceleration);
  p->sampling_deceleration_window = dwa_yaml_get_double(d, "sampling_deceleration_window", p->sampling_deceleration_window);
  if (p->sampling_deceleration_window < 0.0f) p->sampling_deceleration_window = 0.0f;
  p->max_d_yawrate = dwa_yaml_get_double(d, "max_d_yawrate", p->max_d_yawrate);
  p->predict_time = dwa_yaml_get_double(d, "predict_time", p->predict_time);
  p->sim_dt = dwa_yaml_get_double(d, "sim_dt", p->sim_dt);
  p->control_period = dwa_yaml_get_double(d, "control_period", p->control_period);
  p->yaw_feedback_enabled = dwa_yaml_get_bool(d, "yaw_feedback_enabled", p->yaw_feedback_enabled);
  p->imu_gz_sign = (float)dwa_yaml_get_double(d, "imu_gz_sign",
    p->imu_gz_sign != 0.0f ? (double)p->imu_gz_sign : 1.0);
  /* A4b(10.09)：补偿强度 yaml 化（<=0 不覆盖默认，负值视为缺键） */
  v = dwa_yaml_get_double(d, "yaw_feedback_gain_follow", 0.0);
  if (v > 0.0) p->yaw_feedback_gain_follow = (float)v;
  v = dwa_yaml_get_double(d, "yaw_feedback_gain_kill", 0.0);
  if (v > 0.0) p->yaw_feedback_gain_kill = (float)v;
  v = dwa_yaml_get_double(d, "yaw_feedback_fight_margin", 0.0);
  if (v > 0.0) p->yaw_feedback_fight_margin = (float)v;
  v = dwa_yaml_get_double(d, "yaw_feedback_integ_max_frac", 0.0);
  if (v > 0.0) p->yaw_feedback_integ_max_frac = (float)v;
  v = dwa_yaml_get_double(d, "yaw_feedback_integ_release", 0.0);
  if (v > 0.0) p->yaw_feedback_integ_release = (float)v;
  v = dwa_yaml_get_double(d, "yaw_feedback_ref_w_max", 0.0);
  if (v >= 0.10) p->yaw_feedback_ref_w_max = (float)v; /* 防除零下限 */
  /* 手柄豁免阈值：>=0 才覆盖（0=显式禁用豁免，负值=无效用默认） */
  v = dwa_yaml_get_double(d, "yaw_feedback_manual_slew_min_theta", -1.0);
  if (v >= 0.0) p->yaw_feedback_manual_slew_min_theta = (float)v;
  p->slow_velocity_threshold = dwa_yaml_get_double(d, "slow_velocity_threshold", p->slow_velocity_threshold);
  p->velocity_samples = dwa_yaml_get_int(d, "velocity_samples", p->velocity_samples);
  p->yawrate_samples = dwa_yaml_get_int(d, "yawrate_samples", p->yawrate_samples);
  p->planning_worker_count = dwa_yaml_get_int(d, "planning_worker_count", p->planning_worker_count);
  if (p->planning_worker_count < 1) p->planning_worker_count = 1; /* std::max(1, param) */
  p->obstacle_range = dwa_yaml_get_double(d, "obstacle_range", p->obstacle_range);
  p->intent_lookahead = dwa_yaml_get_double(d, "intent_lookahead", p->intent_lookahead);
  p->intent_yaw_to_lateral = dwa_yaml_get_double(d, "intent_yaw_to_lateral", p->intent_yaw_to_lateral);
  p->obstacle_cost_free_margin = dwa_yaml_get_double(d, "obstacle_cost_free_margin", p->obstacle_cost_free_margin);
  p->front_turn_bias_start_m = dwa_yaml_get_double(d, "front_turn_bias_start_m", p->front_turn_bias_start_m);
  p->front_turn_bias_full_m = dwa_yaml_get_double(d, "front_turn_bias_full_m", p->front_turn_bias_full_m);
  p->front_turn_bias_side_clearance_m = dwa_yaml_get_double(d,
    "front_turn_bias_side_clearance_m", p->front_turn_bias_side_clearance_m);
  if (p->front_turn_bias_start_m <= p->front_turn_bias_full_m) {
    p->front_turn_bias_start_m = 0.40f;
    p->front_turn_bias_full_m = 0.30f;
  }
  if (p->front_turn_bias_side_clearance_m < 0.0f) {
    p->front_turn_bias_side_clearance_m = 0.60f;
  }
  p->collision_veto_enabled = dwa_yaml_get_bool(d, "collision_veto_enabled", p->collision_veto_enabled);
  p->collision_veto_reaction_sec = dwa_yaml_get_double(d, "collision_veto_reaction_sec", p->collision_veto_reaction_sec);
  p->collision_veto_min_sec = dwa_yaml_get_double(d, "collision_veto_min_sec", p->collision_veto_min_sec);
  p->collision_veto_max_sec = dwa_yaml_get_double(d, "collision_veto_max_sec", p->collision_veto_max_sec);
  p->clearance_horizon_m = dwa_yaml_get_double(d, "clearance_horizon_m", p->clearance_horizon_m);
  if (p->clearance_horizon_m < 0.0f) p->clearance_horizon_m = 0.0f;
  p->toe_collision_margin_m = dwa_yaml_get_double(d, "toe_collision_margin_m", p->toe_collision_margin_m);
  if (p->toe_collision_margin_m < 0.0f) p->toe_collision_margin_m = 0.0f;
  p->obstacle_cost_gain = dwa_yaml_get_double(d, "obstacle_cost_gain", p->obstacle_cost_gain);
  p->to_goal_cost_gain = dwa_yaml_get_double(d, "to_goal_cost_gain", p->to_goal_cost_gain);
  p->speed_cost_gain = dwa_yaml_get_double(d, "speed_cost_gain", p->speed_cost_gain);
  p->path_cost_gain = dwa_yaml_get_double(d, "path_cost_gain", p->path_cost_gain);
  p->use_speed_cost = dwa_yaml_get_bool(d, "use_speed_cost", p->use_speed_cost);
  p->use_path_cost = dwa_yaml_get_bool(d, "use_path_cost", p->use_path_cost);
  /* AC1/AC2(10.10)：方案A+C（yaml 新键，负值钳 0=禁用）。 */
  p->scoring_manual_turn_min_yawrate = dwa_yaml_get_double(d,
    "scoring_manual_turn_min_yawrate", p->scoring_manual_turn_min_yawrate);
  p->scoring_manual_turn_speed_gain = dwa_yaml_get_double(d,
    "scoring_manual_turn_speed_gain", p->scoring_manual_turn_speed_gain);
  p->intent_freeze_min_yawrate = dwa_yaml_get_double(d,
    "intent_freeze_min_yawrate", p->intent_freeze_min_yawrate);
  if (p->scoring_manual_turn_min_yawrate < 0.0f) p->scoring_manual_turn_min_yawrate = 0.0f;
  if (p->scoring_manual_turn_speed_gain < 0.0f) p->scoring_manual_turn_speed_gain = 0.0f;
  if (p->intent_freeze_min_yawrate < 0.0f) p->intent_freeze_min_yawrate = 0.0f;

  /* ---- 几何二次选择（deg → rad，与 constructor 同一转换表达式） ---- */
  p->dwa_geometric_selector_enabled = dwa_yaml_get_bool(d, "dwa_geometric_selector_enabled", p->dwa_geometric_selector_enabled);
  p->dwa_geometric_lookahead_distance = dwa_yaml_get_double(d, "dwa_geometric_lookahead_distance", p->dwa_geometric_lookahead_distance);
  p->dwa_geometric_min_lookahead_distance = dwa_yaml_get_double(d, "dwa_geometric_min_lookahead_distance", p->dwa_geometric_min_lookahead_distance);
  p->dwa_geometric_min_forward_offset = dwa_yaml_get_double(d, "dwa_geometric_min_forward_offset", p->dwa_geometric_min_forward_offset);
  p->dwa_geometric_cluster_gap_distance = dwa_yaml_get_double(d, "dwa_geometric_cluster_gap_distance", p->dwa_geometric_cluster_gap_distance);
  p->dwa_geometric_cluster_forward_window = dwa_yaml_get_double(d, "dwa_geometric_cluster_forward_window", p->dwa_geometric_cluster_forward_window);
  p->dwa_geometric_edge_front_band = dwa_yaml_get_double(d, "dwa_geometric_edge_front_band", p->dwa_geometric_edge_front_band);
  p->dwa_geometric_cluster_track_tolerance = dwa_yaml_get_double(d, "dwa_geometric_cluster_track_tolerance", p->dwa_geometric_cluster_track_tolerance);
  p->dwa_geometric_centered_entry_max_width = dwa_yaml_get_double(d, "dwa_geometric_centered_entry_max_width", p->dwa_geometric_centered_entry_max_width);
  p->dwa_geometric_handoff_stable_cycles = dwa_yaml_get_int(d, "dwa_geometric_handoff_stable_cycles", p->dwa_geometric_handoff_stable_cycles);
  p->dwa_geometric_handoff_heading_agree = dwa_yaml_get_double(d, "dwa_geometric_handoff_heading_agree_deg", p->dwa_geometric_handoff_heading_agree * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_geometric_raw_entry_yawrate = dwa_yaml_get_double(d, "dwa_geometric_raw_entry_yawrate", p->dwa_geometric_raw_entry_yawrate);
  p->dwa_geometric_min_candidate_clearance = dwa_yaml_get_double(d, "dwa_geometric_min_candidate_clearance", p->dwa_geometric_min_candidate_clearance);
  p->dwa_geometric_max_clearance_loss = dwa_yaml_get_double(d, "dwa_geometric_max_clearance_loss", p->dwa_geometric_max_clearance_loss);
  p->dwa_geometric_min_target_heading = dwa_yaml_get_double(d, "dwa_geometric_min_target_angle_deg", 0.0f) * DWA_PI / 180.0f;
  p->dwa_geometric_near_small_angle_distance = dwa_yaml_get_double(d, "dwa_geometric_near_small_angle_distance", p->dwa_geometric_near_small_angle_distance);
  p->dwa_geometric_small_target_heading = dwa_yaml_get_double(d, "dwa_geometric_small_target_angle_deg", p->dwa_geometric_small_target_heading * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_geometric_near_target_heading_compensation = dwa_yaml_get_double(d, "dwa_geometric_near_target_heading_compensation_deg", p->dwa_geometric_near_target_heading_compensation * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_geometric_max_heading_growth = dwa_yaml_get_double(d, "dwa_geometric_max_heading_growth_deg", p->dwa_geometric_max_heading_growth * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_geometric_max_error = dwa_yaml_get_double(d, "dwa_geometric_max_error_deg", p->dwa_geometric_max_error * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_geometric_manual_override_yawrate = dwa_yaml_get_double(d, "dwa_geometric_manual_override_yawrate", p->dwa_geometric_manual_override_yawrate);
  p->dwa_geometric_target_filter_alpha = dwa_yaml_get_double(d, "dwa_geometric_target_filter_alpha", p->dwa_geometric_target_filter_alpha);
  p->dwa_geometric_heading_tie = dwa_yaml_get_double(d, "dwa_geometric_heading_tie_deg", p->dwa_geometric_heading_tie * 180.0f / DWA_PI) * DWA_PI / 180.0f;

  /* ---- 超时/死区/直行覆盖（constructor clamp 同款） ---- */
  p->scan_timeout_sec = dwa_yaml_get_double(d, "scan_timeout_sec", p->scan_timeout_sec);
  p->cmd_timeout_sec = dwa_yaml_get_double(d, "cmd_timeout_sec", p->cmd_timeout_sec);
  p->odom_timeout_sec = dwa_yaml_get_double(d, "odom_timeout_sec", p->odom_timeout_sec);
  p->deadband_linear = dwa_yaml_get_double(d, "deadband_linear", p->deadband_linear);
  p->deadband_angular = dwa_yaml_get_double(d, "deadband_angular", p->deadband_angular);
  p->forced_straight_yaw_tolerance = dwa_max0(dwa_yaml_get_double(d, "forced_straight_yaw_tolerance", p->forced_straight_yaw_tolerance));
  p->forced_straight_manual_cone_deg = dwa_clamp_value(dwa_yaml_get_double(d, "forced_straight_manual_cone_deg", p->forced_straight_manual_cone_deg), 0.0f, 90.0f);
  p->forced_straight_manual_max_yawrate = dwa_clamp_value(dwa_yaml_get_double(d, "forced_straight_manual_max_yawrate", p->forced_straight_manual_max_yawrate), 0.0f, p->max_yawrate);
  p->forced_straight_manual_passthrough = dwa_yaml_get_bool(d, "forced_straight_manual_passthrough", p->forced_straight_manual_passthrough);

  /* ---- 档位/动态速度 ---- */
  /* yaml 数组解析层保持 double（I/O 边界），落盘到 float 字段时舍入。 */
  {
    double gear_vals[DWA_GEAR_PEAK_SPEEDS_MAX];
    const int gear_n = dwa_yaml_get_double_array(d, "gear_peak_speeds",
      gear_vals, DWA_GEAR_PEAK_SPEEDS_MAX);
    for (int gi = 0; gi < gear_n; ++gi) {
      p->gear_peak_speeds[gi] = gear_vals[gi];
    }
    p->gear_peak_speeds_count = gear_n;
  }
  p->controller_gear_fallback = dwa_yaml_get_int(d, "controller_gear_fallback", p->controller_gear_fallback);
  p->gear_closed_loop_enabled = dwa_yaml_get_bool(d, "gear_closed_loop_enabled", p->gear_closed_loop_enabled);
  p->gear_restore_hysteresis = dwa_yaml_get_double(d, "gear_restore_hysteresis", p->gear_restore_hysteresis);
  p->gear_force_level = dwa_yaml_get_int(d, "gear_force_level", p->gear_force_level);
  p->gear_force_forward_only = dwa_yaml_get_bool(d, "gear_force_forward_only", p->gear_force_forward_only);
  p->gear_front_trigger_extra_angle_deg = dwa_yaml_get_double(d, "gear_front_trigger_extra_angle_deg", p->gear_front_trigger_extra_angle_deg);
  p->gear_restore_hold_cycles = dwa_yaml_get_int(d, "gear_restore_hold_cycles", p->gear_restore_hold_cycles);
  p->gear_transition_step_sec = dwa_yaml_get_double(d, "gear_transition_step_sec", p->gear_transition_step_sec);
  p->gear_transition_safety_margin = dwa_max0(dwa_yaml_get_double(d, "gear_transition_safety_margin", p->gear_transition_safety_margin));
  p->gear_speed_transition_enabled = dwa_yaml_get_bool(d, "gear_speed_transition_enabled", p->gear_speed_transition_enabled);
  p->gear_speed_transition_duration_sec = dwa_yaml_get_double(d, "gear_speed_transition_duration_sec", p->gear_speed_transition_duration_sec);

  p->moving_prebrake_base_distance = dwa_yaml_get_double(d, "moving_prebrake_base_distance", p->moving_prebrake_base_distance);
  p->moving_prebrake_speed_ratio = dwa_yaml_get_double(d, "moving_prebrake_speed_ratio", p->moving_prebrake_speed_ratio);
  p->moving_prebrake_target_speed = dwa_yaml_get_double(d, "moving_prebrake_target_speed", p->moving_prebrake_target_speed);
  p->moving_prebrake_deceleration = dwa_yaml_get_double(d, "moving_prebrake_deceleration", p->moving_prebrake_deceleration);
  p->moving_prebrake_profile_far_distance = dwa_max0(dwa_yaml_get_double(d, "moving_prebrake_profile_far_distance", p->moving_prebrake_profile_far_distance));
  p->moving_prebrake_profile_zero_distance = dwa_max0(dwa_yaml_get_double(d, "moving_prebrake_profile_zero_distance", p->moving_prebrake_profile_zero_distance));
  p->moving_prebrake_recovery_acceleration = dwa_max0(dwa_yaml_get_double(d, "moving_prebrake_recovery_acceleration", p->moving_prebrake_recovery_acceleration));
  p->bypass_near_zone_release_hold_cycles = dwa_yaml_get_int(d, "bypass_near_zone_release_hold_cycles", p->bypass_near_zone_release_hold_cycles);
  if (p->bypass_near_zone_release_hold_cycles < 0) p->bypass_near_zone_release_hold_cycles = 0;
  p->bypass_near_zone_ramp_enabled = dwa_yaml_get_bool(d, "bypass_near_zone_ramp_enabled", p->bypass_near_zone_ramp_enabled);
  p->bypass_near_zone_ramp_deceleration = dwa_max0(dwa_yaml_get_double(d, "bypass_near_zone_ramp_deceleration", p->bypass_near_zone_ramp_deceleration));
  p->bypass_near_zone_far_distance = dwa_max0(dwa_yaml_get_double(d, "bypass_near_zone_far_distance", p->bypass_near_zone_far_distance));
  p->bypass_near_zone_release_side_distance = dwa_max0(dwa_yaml_get_double(d, "bypass_near_zone_release_side_distance", p->bypass_near_zone_release_side_distance));
  p->moving_dwa_speed_limit = dwa_yaml_get_double(d, "moving_dwa_speed_limit", p->moving_dwa_speed_limit);
  p->moving_dwa_yawrate_limit = dwa_yaml_get_double(d, "moving_dwa_yawrate_limit", p->moving_dwa_yawrate_limit);
  p->moving_dwa_force_gear = dwa_yaml_get_bool(d, "moving_dwa_force_gear", p->moving_dwa_force_gear);
  p->moving_dwa_gear_level = dwa_yaml_get_int(d, "moving_dwa_gear_level", p->moving_dwa_gear_level);
  p->near_obstacle_gear_enabled = dwa_yaml_get_bool(d, "near_obstacle_gear_enabled", p->near_obstacle_gear_enabled);
  p->near_obstacle_gear_center_x = dwa_yaml_get_double(d, "near_obstacle_gear_center_x", p->near_obstacle_gear_center_x);
  p->near_obstacle_gear_center_y = dwa_yaml_get_double(d, "near_obstacle_gear_center_y", p->near_obstacle_gear_center_y);
  p->near_obstacle_gear_radius = dwa_yaml_get_double(d, "near_obstacle_gear_radius", p->near_obstacle_gear_radius);
  p->near_obstacle_gear_release_radius = dwa_yaml_get_double(d, "near_obstacle_gear_release_radius", p->near_obstacle_gear_release_radius);
  p->near_obstacle_gear_level = dwa_yaml_get_int(d, "near_obstacle_gear_level", p->near_obstacle_gear_level);
  p->stationary_turn_ring_enabled = dwa_yaml_get_bool(d, "stationary_turn_ring_enabled", p->stationary_turn_ring_enabled);
  p->stationary_turn_ring_outer_radius_m = dwa_max0(dwa_yaml_get_double(d, "stationary_turn_ring_outer_radius_m", p->stationary_turn_ring_outer_radius_m));
  p->stationary_turn_ring_inner_radius_m = dwa_max0(dwa_yaml_get_double(d, "stationary_turn_ring_inner_radius_m", p->stationary_turn_ring_inner_radius_m));
  p->stationary_turn_ring_forward_side_angle_deg = dwa_clamp_value(dwa_yaml_get_double(d, "stationary_turn_ring_forward_side_angle_deg", p->stationary_turn_ring_forward_side_angle_deg), 0.0f, 45.0f);
  p->stationary_turn_ring_inner_speed_ratio = dwa_clamp_value(dwa_yaml_get_double(d, "stationary_turn_ring_inner_speed_ratio", p->stationary_turn_ring_inner_speed_ratio), 0.0f, 1.0f);
  p->moving_dwa_trigger_distance = dwa_yaml_get_double(d, "moving_dwa_trigger_distance", p->moving_dwa_trigger_distance);
  p->moving_dwa_entry_speed_max = dwa_yaml_get_double(d, "moving_dwa_entry_speed_max", p->moving_dwa_entry_speed_max);
  p->moving_release_hysteresis = dwa_max0(dwa_yaml_get_double(d, "moving_release_hysteresis", p->moving_release_hysteresis));
  p->moving_prebrake_reaction_sec = dwa_max0(dwa_yaml_get_double(d, "moving_prebrake_reaction_sec", p->moving_prebrake_reaction_sec));
  p->moving_prebrake_safety_margin = dwa_max0(dwa_yaml_get_double(d, "moving_prebrake_safety_margin", p->moving_prebrake_safety_margin));
  p->moving_prebrake_y_down_rate_min = fmaxf(1.0f, dwa_yaml_get_double(d, "moving_prebrake_y_down_rate_min", p->moving_prebrake_y_down_rate_min));
  p->moving_prebrake_y_down_rate_max = fmaxf(p->moving_prebrake_y_down_rate_min, dwa_yaml_get_double(d, "moving_prebrake_y_down_rate_max", p->moving_prebrake_y_down_rate_max));
  p->moving_prebrake_y_rate_release = fmaxf(1.0f, dwa_yaml_get_double(d, "moving_prebrake_y_rate_release", p->moving_prebrake_y_rate_release));
  p->moving_hall_speed_filter_alpha = dwa_clamp_value(dwa_yaml_get_double(d, "moving_hall_speed_filter_alpha", p->moving_hall_speed_filter_alpha), 0.0f, 1.0f);
  p->static_start_distance = dwa_yaml_get_double(d, "static_start_distance", p->static_start_distance);
  p->static_vdwa_gain = dwa_yaml_get_double(d, "static_vdwa_gain", p->static_vdwa_gain);
  p->dynamic_release_margin = dwa_yaml_get_double(d, "dynamic_release_margin", p->dynamic_release_margin);
  p->dynamic_release_clear_sec = dwa_yaml_get_double(d, "dynamic_release_clear_sec", p->dynamic_release_clear_sec);
  p->static_vdwa_recalculate_sec = dwa_yaml_get_double(d, "static_vdwa_recalculate_sec", p->static_vdwa_recalculate_sec);

  /* ---- 侧边保护（clamp 同款） ---- */
  p->side_detection_angle_deg = dwa_clamp_value(dwa_yaml_get_double(d, "side_detection_angle_deg", p->side_detection_angle_deg), 0.0f, 90.0f);
  p->manual_turn_forward_min_angle_deg = dwa_clamp_value(dwa_yaml_get_double(d, "manual_turn_forward_min_angle_deg", p->manual_turn_forward_min_angle_deg), 0.0f, 90.0f);
  p->manual_turn_forward_max_angle_deg = dwa_clamp_value(dwa_yaml_get_double(d, "manual_turn_forward_max_angle_deg", p->manual_turn_forward_max_angle_deg), 90.0f, 180.0f);
  p->side_hard_stop_circle_enabled = dwa_yaml_get_bool(d, "side_hard_stop_circle_enabled", p->side_hard_stop_circle_enabled);
  p->side_hard_stop_circle_center_x = dwa_yaml_get_double(d, "side_hard_stop_circle_center_x", p->side_hard_stop_circle_center_x);
  p->side_hard_stop_circle_center_y = dwa_yaml_get_double(d, "side_hard_stop_circle_center_y", p->side_hard_stop_circle_center_y);
  p->side_hard_stop_circle_radius = dwa_yaml_get_double(d, "side_hard_stop_circle_radius", p->side_hard_stop_circle_radius);
  p->side_hard_stop_reverse_escape_yawrate = dwa_yaml_get_double(d, "side_hard_stop_reverse_escape_yawrate", p->side_hard_stop_reverse_escape_yawrate);
  p->side_hard_stop_wheel_track_m = dwa_yaml_get_double(d, "side_hard_stop_wheel_track_m", p->side_hard_stop_wheel_track_m);
  p->side_hard_stop_sweep_prediction_sec = dwa_yaml_get_double(d, "side_hard_stop_sweep_prediction_sec", p->side_hard_stop_sweep_prediction_sec);
  p->side_hard_stop_sweep_brake_sec = dwa_yaml_get_double(d, "side_hard_stop_sweep_brake_sec", p->side_hard_stop_sweep_brake_sec);
  p->side_hard_stop_sweep_min_yawrate = dwa_yaml_get_double(d, "side_hard_stop_sweep_min_yawrate", p->side_hard_stop_sweep_min_yawrate);

  p->front_hard_stop_escape_enabled = dwa_yaml_get_bool(d, "front_hard_stop_escape_enabled", p->front_hard_stop_escape_enabled);
  p->radar_contact_distance = dwa_yaml_get_double(d, "radar_contact_distance", p->radar_contact_distance);
  p->hard_stop_release_clear_cycles = dwa_yaml_get_int(d, "hard_stop_release_clear_cycles", p->hard_stop_release_clear_cycles);
  p->front_hard_stop_escape_min_clearance = dwa_yaml_get_double(d, "front_hard_stop_escape_min_clearance", p->front_hard_stop_escape_min_clearance);
  p->front_hard_stop_escape_linear_speed = dwa_yaml_get_double(d, "front_hard_stop_escape_linear_speed", p->front_hard_stop_escape_linear_speed);
  p->front_hard_stop_escape_yawrate = dwa_yaml_get_double(d, "front_hard_stop_escape_yawrate", p->front_hard_stop_escape_yawrate);
  p->stop_escape_trigger_sec = dwa_yaml_get_double(d, "stop_escape_trigger_sec", p->stop_escape_trigger_sec);
  p->near_obstacle_yawrate_limit_enabled = dwa_yaml_get_bool(d, "near_obstacle_yawrate_limit_enabled", p->near_obstacle_yawrate_limit_enabled);

  p->dwa_velocity_ramp_enabled = dwa_yaml_get_bool(d, "dwa_velocity_ramp_enabled", p->dwa_velocity_ramp_enabled);

  /* ---- 侧边动态限速 ---- */
  p->side_speed_trigger = dwa_yaml_get_double(d, "side_speed_trigger", p->side_speed_trigger);
  p->side_speed_target = dwa_yaml_get_double(d, "side_speed_target", p->side_speed_target);
  p->side_limit_distance = dwa_yaml_get_double(d, "side_limit_distance", p->side_limit_distance);
  p->side_prediction_reaction_sec = dwa_yaml_get_double(d, "side_prediction_reaction_sec", p->side_prediction_reaction_sec);
  p->side_prediction_deceleration = dwa_yaml_get_double(d, "side_prediction_deceleration", p->side_prediction_deceleration);
  p->side_closing_rate_filter_alpha = dwa_yaml_get_double(d, "side_closing_rate_filter_alpha", p->side_closing_rate_filter_alpha);
  p->side_closing_rate_max = dwa_yaml_get_double(d, "side_closing_rate_max", p->side_closing_rate_max);
  p->side_cap_release_distance = dwa_yaml_get_double(d, "side_cap_release_distance", p->side_cap_release_distance);
  p->side_cap_release_clear_sec = dwa_yaml_get_double(d, "side_cap_release_clear_sec", p->side_cap_release_clear_sec);
  p->side_target_down_rate = dwa_yaml_get_double(d, "side_target_down_rate", p->side_target_down_rate);
  p->side_target_up_rate = dwa_yaml_get_double(d, "side_target_up_rate", p->side_target_up_rate);

  /* ---- 侧墙保持 ---- */
  p->side_wall_hold_enabled = dwa_yaml_get_bool(d, "side_wall_hold_enabled", p->side_wall_hold_enabled);
  p->side_wall_hold_distance = dwa_yaml_get_double(d, "side_wall_hold_distance", p->side_wall_hold_distance);
  p->side_wall_hold_correction_distance = dwa_yaml_get_double(d, "side_wall_hold_correction_distance", p->side_wall_hold_correction_distance);
  p->side_wall_hold_prediction_sec = dwa_yaml_get_double(d, "side_wall_hold_prediction_sec", p->side_wall_hold_prediction_sec);
  p->side_wall_hold_invalid_hold_sec = dwa_yaml_get_double(d, "side_wall_hold_invalid_hold_sec", p->side_wall_hold_invalid_hold_sec);
  p->side_wall_hold_forward_margin_deg = dwa_clamp_value(dwa_yaml_get_double(d, "side_wall_hold_forward_margin_deg", p->side_wall_hold_forward_margin_deg), 0.0f, 30.0f);
  p->side_wall_hold_release_distance = dwa_yaml_get_double(d, "side_wall_hold_release_distance", p->side_wall_hold_release_distance);
  p->side_wall_hold_kp = dwa_yaml_get_double(d, "side_wall_hold_kp", p->side_wall_hold_kp);
  p->side_wall_hold_min_yawrate = dwa_yaml_get_double(d, "side_wall_hold_min_yawrate", p->side_wall_hold_min_yawrate);
  p->side_wall_hold_max_yawrate = dwa_yaml_get_double(d, "side_wall_hold_max_yawrate", p->side_wall_hold_max_yawrate);
  p->side_wall_hold_yaw_accel = dwa_yaml_get_double(d, "side_wall_hold_yaw_accel", p->side_wall_hold_yaw_accel);
  p->side_wall_hold_release_sec = dwa_yaml_get_double(d, "side_wall_hold_release_sec", p->side_wall_hold_release_sec);

  p->dynamic_release_margin = dwa_yaml_get_double(d, "dynamic_release_margin", p->dynamic_release_margin);
  p->dynamic_release_clear_sec = dwa_yaml_get_double(d, "dynamic_release_clear_sec", p->dynamic_release_clear_sec);
  p->speed_restore_side_clearance = dwa_yaml_get_double(d, "speed_restore_side_clearance", p->speed_restore_side_clearance);
  p->speed_restore_sweep_clearance = dwa_yaml_get_double(d, "speed_restore_sweep_clearance", p->speed_restore_sweep_clearance);
  p->speed_restore_stable_sec = dwa_yaml_get_double(d, "speed_restore_stable_sec", p->speed_restore_stable_sec);
  p->speed_restore_max_yawrate = dwa_yaml_get_double(d, "speed_restore_max_yawrate", p->speed_restore_max_yawrate);

  /* ---- 前方触发区（默认为计算值，yaml 可覆盖） ---- */
  p->front_corridor_width = dwa_yaml_get_double(d, "front_corridor_width", 2.0f * p->footprint_half_width);
  p->front_trigger_start_x = dwa_yaml_get_double(d, "front_trigger_start_x", 0.5f * (p->left_laser_x + p->right_laser_x));
  p->front_trigger_base_width = dwa_yaml_get_double(d, "front_trigger_base_width", 2.0f * p->footprint_half_width);
  p->front_trigger_expand_angle_deg = dwa_yaml_get_double(d, "front_trigger_expand_angle_deg", p->front_trigger_expand_angle_deg);

  p->controller_x_dead_low = dwa_yaml_get_int(d, "controller_x_dead_low", p->controller_x_dead_low);
  p->controller_x_dead_high = dwa_yaml_get_int(d, "controller_x_dead_high", p->controller_x_dead_high);
  p->controller_y_dead_low = dwa_yaml_get_int(d, "controller_y_dead_low", p->controller_y_dead_low);
  p->controller_y_dead_high = dwa_yaml_get_int(d, "controller_y_dead_high", p->controller_y_dead_high);

  p->hall_feedback_timeout_sec = fmaxf(0.01f, dwa_yaml_get_double(d, "hall_feedback_timeout_sec", p->hall_feedback_timeout_sec));

  /* ---- 调试/点云裁剪 ---- */
  p->debug_nominal_speed = dwa_yaml_get_double(d, "debug_nominal_speed", p->debug_nominal_speed);
  p->force_debug_intent = dwa_yaml_get_bool(d, "force_debug_intent", p->force_debug_intent);
  p->planning_min_x = dwa_yaml_get_double(d, "planning_min_x", p->planning_min_x);
  p->planning_max_abs_y = dwa_yaml_get_double(d, "planning_max_abs_y", p->planning_max_abs_y);
  p->marker_stride = dwa_yaml_get_int(d, "marker_stride", p->marker_stride);
  if (p->marker_stride < 1) p->marker_stride = 1;
  p->log_trajectory_scores = dwa_yaml_get_bool(d, "log_trajectory_scores", p->log_trajectory_scores);
  p->publish_debug_visualization = dwa_yaml_get_bool(d, "publish_debug_visualization", p->publish_debug_visualization);
  p->trajectory_score_top_n = dwa_yaml_get_int(d, "trajectory_score_top_n", p->trajectory_score_top_n);
  if (p->trajectory_score_top_n < 1) p->trajectory_score_top_n = 1;
  p->trajectory_score_log_interval_ms = dwa_yaml_get_int(d, "trajectory_score_log_interval_ms", p->trajectory_score_log_interval_ms);
  if (p->trajectory_score_log_interval_ms < 100) p->trajectory_score_log_interval_ms = 100;

  /* ---- 通道引导 ---- */
  p->dwa_corridor_guide_enabled = dwa_yaml_get_bool(d, "dwa_corridor_guide_enabled", p->dwa_corridor_guide_enabled);
  p->dwa_corridor_search_start_x = dwa_yaml_get_double(d, "dwa_corridor_search_start_x", p->dwa_corridor_search_start_x);
  p->dwa_corridor_search_end_x = dwa_yaml_get_double(d, "dwa_corridor_search_end_x", p->dwa_corridor_search_end_x);
  p->dwa_corridor_min_width_margin = dwa_yaml_get_double(d, "dwa_corridor_min_width_margin", p->dwa_corridor_min_width_margin);
  p->dwa_corridor_max_width = dwa_yaml_get_double(d, "dwa_corridor_max_width", p->dwa_corridor_max_width);
  p->dwa_corridor_search_half_width = dwa_yaml_get_double(d, "dwa_corridor_search_half_width", p->dwa_corridor_search_half_width);
  p->dwa_corridor_cluster_gap_distance = dwa_yaml_get_double(d, "dwa_corridor_cluster_gap_distance", p->dwa_corridor_cluster_gap_distance);
  p->dwa_corridor_cluster_min_points = dwa_yaml_get_int(d, "dwa_corridor_cluster_min_points", p->dwa_corridor_cluster_min_points);
  p->dwa_corridor_gate_max_depth_difference = dwa_yaml_get_double(d, "dwa_corridor_gate_max_depth_difference", p->dwa_corridor_gate_max_depth_difference);
  p->dwa_corridor_gate_blocking_clearance = dwa_yaml_get_double(d, "dwa_corridor_gate_blocking_clearance", p->dwa_corridor_gate_blocking_clearance);
  p->dwa_corridor_gate_endpoint_exclusion = dwa_yaml_get_double(d, "dwa_corridor_gate_endpoint_exclusion", p->dwa_corridor_gate_endpoint_exclusion);
  p->dwa_corridor_gate_dedup_distance = dwa_yaml_get_double(d, "dwa_corridor_gate_dedup_distance", p->dwa_corridor_gate_dedup_distance);
  p->dwa_corridor_gate_projection_length = dwa_yaml_get_double(d, "dwa_corridor_gate_projection_length", p->dwa_corridor_gate_projection_length);
  p->dwa_corridor_approach_start_x = dwa_yaml_get_double(d, "dwa_corridor_approach_start_x", p->dwa_corridor_approach_start_x);
  p->dwa_corridor_explicit_intent_ratio = dwa_yaml_get_double(d, "dwa_corridor_explicit_intent_ratio", p->dwa_corridor_explicit_intent_ratio);
  p->dwa_corridor_max_heading = dwa_yaml_get_double(d, "dwa_corridor_max_heading_deg", p->dwa_corridor_max_heading * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_corridor_intent_lateral_range = dwa_yaml_get_double(d, "dwa_corridor_intent_lateral_range", p->dwa_corridor_intent_lateral_range);
  p->dwa_corridor_stable_cycles = dwa_yaml_get_int(d, "dwa_corridor_stable_cycles", p->dwa_corridor_stable_cycles);
  p->dwa_corridor_lost_hold_cycles = dwa_yaml_get_int(d, "dwa_corridor_lost_hold_cycles", p->dwa_corridor_lost_hold_cycles);
  p->dwa_corridor_track_lateral_tolerance = dwa_yaml_get_double(d, "dwa_corridor_track_lateral_tolerance", p->dwa_corridor_track_lateral_tolerance);
  p->dwa_corridor_track_heading_tolerance = dwa_yaml_get_double(d, "dwa_corridor_track_heading_tolerance_deg", p->dwa_corridor_track_heading_tolerance * 180.0f / DWA_PI) * DWA_PI / 180.0f;
  p->dwa_corridor_path_filter_alpha = dwa_yaml_get_double(d, "dwa_corridor_path_filter_alpha", p->dwa_corridor_path_filter_alpha);
  p->dwa_corridor_trajectory_lookahead = dwa_yaml_get_double(d, "dwa_corridor_trajectory_lookahead", p->dwa_corridor_trajectory_lookahead);
  p->dwa_corridor_lateral_weight = dwa_yaml_get_double(d, "dwa_corridor_lateral_weight", p->dwa_corridor_lateral_weight);
  p->dwa_corridor_heading_weight = dwa_yaml_get_double(d, "dwa_corridor_heading_weight", p->dwa_corridor_heading_weight);
  p->dwa_corridor_progress_weight = dwa_yaml_get_double(d, "dwa_corridor_progress_weight", p->dwa_corridor_progress_weight);

  return 0;
}

void dwa_planner_params_to_config(
  const dwa_planner_params_t *p, dwa_config_t *config)
{
  *config = dwa_config_default();
  config->max_velocity = p->max_velocity;
  config->min_velocity = p->min_velocity;
  config->max_yawrate = p->max_yawrate;
  config->min_yawrate = p->min_yawrate;
  config->max_acceleration = p->max_acceleration;
  config->max_deceleration = p->max_deceleration;
  config->sampling_deceleration_window = p->sampling_deceleration_window;
  config->max_d_yawrate = p->max_d_yawrate;
  config->predict_time = p->predict_time;
  config->sim_dt = p->sim_dt;
  config->control_period = p->control_period;
  config->slow_velocity_threshold = p->slow_velocity_threshold;
  config->velocity_samples = p->velocity_samples;
  config->yawrate_samples = p->yawrate_samples;
  config->planning_worker_count = p->planning_worker_count;
  config->obstacle_range = p->obstacle_range;
  config->intent_lookahead = p->intent_lookahead;
  config->intent_yaw_to_lateral = p->intent_yaw_to_lateral;
  config->obstacle_cost_free_margin = p->obstacle_cost_free_margin;
  config->front_turn_bias_start_m = p->front_turn_bias_start_m;
  config->front_turn_bias_full_m = p->front_turn_bias_full_m;
  config->front_turn_bias_side_clearance_m =
    p->front_turn_bias_side_clearance_m;
  config->collision_veto_enabled = p->collision_veto_enabled;
  config->collision_veto_reaction_sec = p->collision_veto_reaction_sec;
  config->collision_veto_min_sec = p->collision_veto_min_sec;
  config->collision_veto_max_sec = p->collision_veto_max_sec;
  config->clearance_horizon_m = p->clearance_horizon_m;
  config->toe_collision_margin_m = p->toe_collision_margin_m;
  config->obstacle_cost_gain = p->obstacle_cost_gain;
  config->to_goal_cost_gain = p->to_goal_cost_gain;
  config->speed_cost_gain = p->speed_cost_gain;
  config->path_cost_gain = p->path_cost_gain;
  config->use_speed_cost = p->use_speed_cost;
  config->use_path_cost = p->use_path_cost;
  config->intent_freeze_min_yawrate = p->intent_freeze_min_yawrate; /* AC2(10.10) */
  config->trajectory_reference_x = p->trajectory_reference_x;
  config->trajectory_reference_y = p->trajectory_reference_y;
  config->unknown_is_obstacle = p->grid_unknown_is_obstacle;
  config->footprint.front = p->footprint_front;
  config->footprint.back = p->footprint_back;
  config->footprint.half_width = p->footprint_half_width;
  config->footprint.padding = p->footprint_padding;
  config->footprint.footrest_start = p->footrest_start_x;
  config->footprint.footrest_front = p->footrest_front_x;
  config->footprint.footrest_half_width = p->footrest_half_width;
  config->footprint.toe_front = p->toe_safety_front_x;
  config->footprint.toe_half_width = p->toe_safety_half_width;
  config->path_reference = dwa_path_reference_default();
}

void dwa_planner_params_dump(const dwa_planner_params_t *p)
{
  printf("== dwa_planner params ==\n");
  printf("fused_cloud_topic=%s\n", p->fused_cloud_topic);
  printf("cmd_vel_raw_topic=%s\n", p->cmd_vel_raw_topic);
  printf("motor_feedback_topic=%s\n", p->motor_feedback_topic);
  printf("hall_speed_control_topic=%s\n", p->hall_speed_control_topic);
  printf("odom_topic=%s\n", p->odom_topic);
  printf("output_topic=%s\n", p->output_topic);
  printf("frame_id=%s\n", p->frame_id);
  printf("left_laser=(%.17g,%.17g,%.17g) reversed=%d offset_deg=%.17g\n",
    p->left_laser_x, p->left_laser_y, p->left_laser_yaw,
    p->left_laser_angle_reversed ? 1 : 0, p->left_laser_angle_offset_deg);
  printf("right_laser=(%.17g,%.17g,%.17g) reversed=%d offset_deg=%.17g\n",
    p->right_laser_x, p->right_laser_y, p->right_laser_yaw,
    p->right_laser_angle_reversed ? 1 : 0, p->right_laser_angle_offset_deg);
  printf("footprint front=%.17g back=%.17g half=%.17g pad=%.17g\n",
    p->footprint_front, p->footprint_back, p->footprint_half_width, p->footprint_padding);
  printf("footrest start=%.17g front=%.17g half=%.17g toe_front=%.17g toe_half=%.17g\n",
    p->footrest_start_x, p->footrest_front_x, p->footrest_half_width,
    p->toe_safety_front_x, p->toe_safety_half_width);
  printf("self_filter body_front=%.17g pad=%.17g\n",
    p->self_filter_body_front_x, p->footprint_self_filter_padding);
  printf("max_velocity=%.17g min_velocity=%.17g max_reverse=%.17g max_yawrate=%.17g min_yawrate=%.17g\n",
    p->max_velocity, p->min_velocity, p->max_reverse_velocity, p->max_yawrate, p->min_yawrate);
  printf("accel=%.17g decel=%.17g sampling_decel_window=%.17g d_yaw=%.17g predict=%.17g sim_dt=%.17g period=%.17g\n",
    p->max_acceleration, p->max_deceleration, p->sampling_deceleration_window, p->max_d_yawrate,
    p->predict_time, p->sim_dt, p->control_period);
  printf("slow_threshold=%.17g samples=%dx%d workers=%d obstacle_range=%.17g\n",
    p->slow_velocity_threshold, p->velocity_samples, p->yawrate_samples,
    p->planning_worker_count, p->obstacle_range);
  printf("intent lookahead=%.17g yaw_to_lateral=%.17g\n",
    p->intent_lookahead, p->intent_yaw_to_lateral);
  printf("yaw_fb enabled=%d gz_sign=%.17g gain_f=%.17g gain_k=%.17g fight=%.17g "
    "integ_frac=%.17g integ_release=%.17g ref_w_max=%.17g manual_slew_theta=%.17g\n",
    p->yaw_feedback_enabled ? 1 : 0, p->imu_gz_sign,
    p->yaw_feedback_gain_follow, p->yaw_feedback_gain_kill,
    p->yaw_feedback_fight_margin, p->yaw_feedback_integ_max_frac,
    p->yaw_feedback_integ_release, p->yaw_feedback_ref_w_max,
    p->yaw_feedback_manual_slew_min_theta);
  printf("gains obstacle=%.17g goal=%.17g speed=%.17g path=%.17g use_speed=%d use_path=%d\n",
    p->obstacle_cost_gain, p->to_goal_cost_gain, p->speed_cost_gain, p->path_cost_gain,
    p->use_speed_cost ? 1 : 0, p->use_path_cost ? 1 : 0);
  printf("manual_turn_penalty min_yaw=%.17g speed_gain=%.17g intent_freeze_yaw=%.17g\n",
    p->scoring_manual_turn_min_yawrate, p->scoring_manual_turn_speed_gain,
    p->intent_freeze_min_yawrate);
  printf("trajectory_ref=(%.17g,%.17g)\n", p->trajectory_reference_x, p->trajectory_reference_y);
  printf("grid res=%.17g x=[%.17g,%.17g] abs_y=%.17g inflation=%.17g unknown=%d\n",
    p->grid_resolution, p->grid_min_x, p->grid_max_x, p->grid_max_abs_y,
    p->grid_inflation_radius, p->grid_unknown_is_obstacle ? 1 : 0);
  printf("deadband linear=%.17g angular=%.17g\n", p->deadband_linear, p->deadband_angular);
  printf("front_trigger start_x=%.17g base_width=%.17g expand=%.17gdeg corridor_width=%.17g\n",
    p->front_trigger_start_x, p->front_trigger_base_width,
    p->front_trigger_expand_angle_deg, p->front_corridor_width);
  printf("gear_peak_speeds=[%d]", p->gear_peak_speeds_count);
  for (int i = 0; i < p->gear_peak_speeds_count; ++i) printf(" %.17g", p->gear_peak_speeds[i]);
  printf("\n");
  printf("stationary_turn_ring enabled=%d outer=%.17g inner=%.17g angle=%.17g inner_ratio=%.17g\n",
    p->stationary_turn_ring_enabled ? 1 : 0,
    p->stationary_turn_ring_outer_radius_m,
    p->stationary_turn_ring_inner_radius_m,
    p->stationary_turn_ring_forward_side_angle_deg,
    p->stationary_turn_ring_inner_speed_ratio);
}
