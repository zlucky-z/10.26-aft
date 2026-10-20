/*
 * 模块功能：DWA 节点全部运行参数的 C 结构与加载（默认值 + yaml 覆盖）。
 * 字段名与 config/dwa_planner.yaml 完全一致；
 * 默认值逐条照搬 native/nodes/impl/node_constructor.inc / node_members.inc
 * 以及各子模块配置结构体的成员默认值（见 c/docs/TRANSLATION_NOTES.md）。
 * constructor 中对个别参数施加的 clamp/max 在加载后同样施加。
 */
#ifndef DWA_PARAMS_H
#define DWA_PARAMS_H

#include <stdbool.h>

#include "dwa_yaml.h"
#include "../planning/dwa_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DWA_PARAM_STRING_MAX 128
#define DWA_GEAR_PEAK_SPEEDS_MAX 16

typedef struct {
  /* ---- 话题/坐标系（字符串） ---- */
  char fused_cloud_topic[DWA_PARAM_STRING_MAX];
  char cmd_vel_raw_topic[DWA_PARAM_STRING_MAX];
  char motor_feedback_topic[DWA_PARAM_STRING_MAX];
  char hall_speed_control_topic[DWA_PARAM_STRING_MAX];
  char odom_topic[DWA_PARAM_STRING_MAX];
  char output_topic[DWA_PARAM_STRING_MAX];
  char frame_id[DWA_PARAM_STRING_MAX];

  /* ---- 雷达安装 ---- */
  float left_laser_x, left_laser_y, left_laser_yaw;
  bool left_laser_angle_reversed;
  float left_laser_angle_offset_deg;
  float right_laser_x, right_laser_y, right_laser_yaw;
  bool right_laser_angle_reversed;
  float right_laser_angle_offset_deg;

  /* ---- 车体包络 ---- */
  float footprint_front, footprint_back, footprint_half_width;
  float footrest_start_x, footrest_front_x, footrest_half_width;
  float toe_safety_front_x, toe_safety_half_width;
  float footprint_padding;
  float self_filter_body_front_x, footprint_self_filter_padding;

  /* ---- 骑乘者身体区自点过滤（只作用于速度模块清距诊断，不改 DWA 障碍图） ---- */
  bool rider_self_filter_enabled;
  float rider_self_filter_start_x, rider_self_filter_front_x;
  float rider_self_filter_half_width;
  /* F3(9.27) 宽簇墙闸：带内存在横向宽簇（墙）时本周期停用过滤。 */
  float rider_filter_cluster_gap;      /* y 聚类最大点间间隙，单位m */
  float rider_filter_preserve_width;   /* 判定墙的最小簇横向宽度，单位m */

  /* ---- 速度模块总开关（A/B 实测用）----
   * true  = 原速度融合状态机（HAND/PREBRAKE/HOLD/SIDE_CAP/STATIC）。
   * false = 速度模块整体旁路：前进意图+感知新鲜 ⇒ DWA 直接接管，
   *         速度上限恒为 forward_speed_limit（无预减速/锁存/比例减速），
   *         档位透传原始档。硬停止/倒车/原地转向走原安全通道。 */
  bool dynamic_velocity_fusion_enabled;

  /* ---- 脚部角度屏蔽 ---- */
  bool local_foot_mask_enabled;
  float local_foot_mask_max_range;
  float left_local_foot_angle_min_deg, left_local_foot_angle_max_deg;
  float right_local_foot_angle_min_deg, right_local_foot_angle_max_deg;

  /* ---- 左右雷达规划扇区 ---- */
  float left_planning_angle_min_deg, left_planning_angle_max_deg;
  float right_planning_angle_min_deg, right_planning_angle_max_deg;

  /* ---- 轨迹参考点 ---- */
  float trajectory_reference_x, trajectory_reference_y;

  /* ---- 局部占用栅格 ---- */
  float grid_resolution, grid_min_x, grid_max_x, grid_max_abs_y;
  float grid_inflation_radius;
  /* 9.24: 轨迹评估已不再查询占用栅格（点云桶为唯一碰撞源），建图链
   * （射线清空/占据标记/footprint清除/诊断扫描）成为纯开销。开关关闭
   * 后仍保留栅格尺寸——障碍点表边界检查依赖 world_to_cell。 */
  bool occupancy_grid_build_enabled;
  bool grid_unknown_is_obstacle;

  /* ---- DWA 速度窗口/采样/评分 ---- */
  float max_velocity, min_velocity, max_reverse_velocity;
  float max_yawrate, min_yawrate;
  float max_acceleration, max_deceleration, max_d_yawrate;
  /* G1(10.09)：采样窗口下界专用减速度；<=0 退回 max_deceleration（原行为）。
   * 只影响 DWA 候选窗口 v_low，判死视界/输出斜率不动（见 dwa_config.h）。 */
  float sampling_deceleration_window;
  float predict_time, sim_dt, control_period, slow_velocity_threshold;
  int velocity_samples, yawrate_samples, planning_worker_count;
  float obstacle_range, intent_lookahead, intent_yaw_to_lateral;
  float obstacle_cost_free_margin;
  float front_turn_bias_start_m, front_turn_bias_full_m;
  float front_turn_bias_side_clearance_m;
  /* A4(10.09)：IMU 角速度闭环补偿（ltw 基线实车验证方案移植）。
   * DWA 接管期（MOVING_DWA）以 bmi270 gz 为反馈，对发布的角速度
   * 做 P 阻尼+小积分+斜率限幅，补偿 1 档强制后底板执行缺口
   * （045814/051608 实测：指令→实际等效比 0.78→0.57，通道弯道
   * selw 0.14 物理仅 ~0.06，转不过来）。IMU 不可用时自动旁路。 */
  bool yaw_feedback_enabled;
  float imu_gz_sign;          /* gz 符号约定：+1=左转 gz>0；装反改 -1 */
  /* A4b(10.09)：补偿强度 yaml 化（首版实测过补——宽走廊车身被纠偏
   * 拖曳、窄通道转稍微大。ltw 原标定基准是 1 档 0.35 权限的车，本
   * 项目 w_max=1.40 → w_scale=4.0 权限整体放大是根因之一）。
   * <=0 时沿用代码默认（yaw_feedback.c 同值），负值不生效。 */
  float yaw_feedback_gain_follow;   /* P 跟随增益（ltw 0.40→0.30） */
  float yaw_feedback_gain_kill;     /* P 压制增益（ltw 0.50） */
  float yaw_feedback_fight_margin;  /* 压制模式触发余量 rad/s（ltw 0.08） */
  float yaw_feedback_integ_max_frac;/* 积分上限=w_max×本值（ltw 0.35→0.15） */
  float yaw_feedback_integ_release; /* 无需求泄漏系数/拍（ltw 0.85→0.70） */
  float yaw_feedback_ref_w_max;     /* w_scale 分母（ltw 0.35→0.70，即 4.0→2.0） */
  float yaw_feedback_manual_slew_min_theta; /* 手柄豁免快率阈值 rad（10.09 方案①，0=禁用） */
  /* 10.08 Fix V：碰撞判死视距解耦（仅 DWA 候选评分；安全层探针不受影响） */
  bool collision_veto_enabled;
  float collision_veto_reaction_sec, collision_veto_min_sec, collision_veto_max_sec;
  /* 10.08 Fix V3c：净距统计视界——净距弧长 = min(本值, 刹车视距弧长
   * +0.30m)。低速端短弧保通道修正舵不被对侧墙满罚钳死（V3b 固定弧
   * 长致通道骑右墙）；高速端以本值封顶，不像 V3 时间视界拉到 2.28m
   * 压高速。弧长内的步记真实净距（足迹内点记 0 满罚）但不判死。 */
  float clearance_horizon_m;
  float toe_collision_margin_m;
  float obstacle_cost_gain, to_goal_cost_gain, speed_cost_gain, path_cost_gain;
  bool use_speed_cost, use_path_cost;
  /* AC1/AC2(10.10) 方案A+C：窄缝塌缩评分层修复（003347 定案：
   * 塌缩段 74% 评分自选 raw_v=sel_v=0、强打杆 31% 输出近零、6% 反向；
   * A5 证明旁路 target 恒 0.70 无压制——必须动评分层，禁碰旁路）。
   * 方案A：近障锁存+强打杆+前推时，tick 层临时打开 use_speed_cost
   * 并以专用增益参与评分（v=0 候选归一化后吃满罚、贴窗顶候选零罚），
   * 评分从"原地贴弧最优"翻回"贴窗顶推进最优"，保差速转向有力。
   * 方案C：意图方向候选全判死时输出冻结(0,0)（dwa_sampling.c），
   * 消除推左出右的反飘。三键 <=0 均禁用（原行为）。 */
  float scoring_manual_turn_min_yawrate; /* 方案A激活打杆门槛 rad/s */
  float scoring_manual_turn_speed_gain;  /* 方案A速度罚增益 */
  float intent_freeze_min_yawrate;       /* 方案C激活打杆门槛 rad/s */

  /* ---- 几何二次选择（deg 入参，rad 存储） ---- */
  bool dwa_geometric_selector_enabled;
  float dwa_geometric_lookahead_distance, dwa_geometric_min_lookahead_distance;
  float dwa_geometric_min_forward_offset, dwa_geometric_cluster_gap_distance;
  float dwa_geometric_cluster_forward_window, dwa_geometric_edge_front_band;
  float dwa_geometric_cluster_track_tolerance, dwa_geometric_centered_entry_max_width;
  int dwa_geometric_handoff_stable_cycles;
  float dwa_geometric_handoff_heading_agree;      /* rad */
  float dwa_geometric_raw_entry_yawrate;
  float dwa_geometric_min_candidate_clearance, dwa_geometric_max_clearance_loss;
  float dwa_geometric_min_target_heading;          /* rad */
  float dwa_geometric_near_small_angle_distance;
  float dwa_geometric_small_target_heading;        /* rad */
  float dwa_geometric_near_target_heading_compensation; /* rad */
  float dwa_geometric_max_heading_growth;          /* rad */
  float dwa_geometric_max_error;                   /* rad */
  float dwa_geometric_manual_override_yawrate;
  float dwa_geometric_target_filter_alpha;
  float dwa_geometric_heading_tie;                 /* rad */

  /* ---- 超时/死区/直行覆盖 ---- */
  float scan_timeout_sec, cmd_timeout_sec, odom_timeout_sec;
  float deadband_linear, deadband_angular;
  float forced_straight_yaw_tolerance;
  float forced_straight_manual_cone_deg, forced_straight_manual_max_yawrate;
  /* 刀2(10.10)：直行覆盖(valid_straight)路径下手动转向是否直通原始
   * 手柄角速度。true=不再按防抖锥压小转向（corridor_clear 路径同款
   * 直通）；false=恢复旧防抖锥（锥角比例×max_yawrate 压缩）。碰撞
   * 安全闸不受影响：hand_trajectory 全视界校验失败仍交回 DWA。 */
  bool forced_straight_manual_passthrough;

  /* ---- 档位/动态速度 ---- */
  float gear_peak_speeds[DWA_GEAR_PEAK_SPEEDS_MAX];
  int gear_peak_speeds_count;
  int controller_gear_fallback;
  bool gear_closed_loop_enabled;
  float gear_restore_hysteresis;
  int gear_force_level;
  bool gear_force_forward_only;
  float gear_front_trigger_extra_angle_deg;
  int gear_restore_hold_cycles;
  float gear_transition_step_sec, gear_transition_safety_margin;
  bool gear_speed_transition_enabled;
  float gear_speed_transition_duration_sec;

  float moving_prebrake_base_distance, moving_prebrake_speed_ratio;
  float moving_prebrake_target_speed, moving_prebrake_deceleration;
  /* 9.27 距离比例减速：净距>=far 满速，线性降到净距<=zero 目标 0；
   * 净距回升时按 recovery_acceleration 平滑恢复。 */
  float moving_prebrake_profile_far_distance;
  float moving_prebrake_profile_zero_distance;
  float moving_prebrake_recovery_acceleration;
  /* V4(10.08)：F1d 两态 target 的释放防抖与断崖斜坡，见
   * dynamic_velocity_fusion.h 同名字段注释。 */
  int bypass_near_zone_release_hold_cycles;
  bool bypass_near_zone_ramp_enabled;
  float bypass_near_zone_ramp_deceleration;
  /* V4c(10.08)：F1d 近障锁存判定距离（独立于 prebrake profile 的
   * moving_prebrake_profile_far_distance，避免双消费方互相牵连）。
   * 板端 3.80 提前减速，治 4/5 档减速跑道不足撞对侧。 */
  float bypass_near_zone_far_distance;
  /* A3(10.09)：1 档避障锁存的释放侧净距门控（m，0=禁用）。
   * 用户定案：锁存后在通道里不允许提速——1m 窄弯里视线穿过
   * 弯道开口时 fwd 真实跳 3m+（040707 实锤 4 次误释放全在通道
   * 内），只看前方必然"见缝隙就加速"。释放须同时满足：前方
   * 净空（>release 防抖）且 两侧任一侧净距>本值——通道内两侧
   * 恒 0.15~0.4m 永不释放，恒 0.70 到底；驶出通道两侧先张开才
   * 恢复档位速度。 */
  float bypass_near_zone_release_side_distance;
  float moving_dwa_trigger_distance, moving_dwa_entry_speed_max;
  float moving_release_hysteresis;
  float moving_dwa_speed_limit, moving_dwa_yawrate_limit;
  bool moving_dwa_force_gear;
  int moving_dwa_gear_level;

  bool near_obstacle_gear_enabled;
  float near_obstacle_gear_center_x, near_obstacle_gear_center_y;
  float near_obstacle_gear_radius, near_obstacle_gear_release_radius;
  int near_obstacle_gear_level;

  bool stationary_turn_ring_enabled;
  float stationary_turn_ring_outer_radius_m;
  float stationary_turn_ring_inner_radius_m;
  float stationary_turn_ring_forward_side_angle_deg;
  float stationary_turn_ring_inner_speed_ratio;

  float moving_prebrake_reaction_sec, moving_prebrake_safety_margin;
  float moving_prebrake_y_down_rate_min, moving_prebrake_y_down_rate_max;
  float moving_prebrake_y_rate_release;
  float moving_hall_speed_filter_alpha;

  float static_start_distance, static_vdwa_gain, static_vdwa_recalculate_sec;

  /* ---- 侧边保护 ---- */
  float side_detection_angle_deg;
  float manual_turn_forward_min_angle_deg, manual_turn_forward_max_angle_deg;
  bool side_hard_stop_circle_enabled;
  float side_hard_stop_circle_center_x, side_hard_stop_circle_center_y;
  float side_hard_stop_circle_radius;
  float side_hard_stop_reverse_escape_yawrate;
  float side_hard_stop_wheel_track_m;
  float side_hard_stop_sweep_prediction_sec, side_hard_stop_sweep_brake_sec;
  float side_hard_stop_sweep_min_yawrate;

  /* ---- 前方硬停/脱困 ---- */
  bool front_hard_stop_escape_enabled;
  float radar_contact_distance;
  int hard_stop_release_clear_cycles;
  float front_hard_stop_escape_min_clearance;
  float front_hard_stop_escape_linear_speed;
  float front_hard_stop_escape_yawrate;
  float stop_escape_trigger_sec;
  bool near_obstacle_yawrate_limit_enabled;
  float near_obstacle_max_yawrate;

  bool dwa_velocity_ramp_enabled;

  /* ---- 侧边动态限速 ---- */
  float side_speed_trigger, side_speed_target, side_limit_distance;
  float side_prediction_reaction_sec, side_prediction_deceleration;
  float side_closing_rate_filter_alpha, side_closing_rate_max;
  float side_cap_release_distance, side_cap_release_clear_sec;
  float side_target_down_rate, side_target_up_rate;

  /* ---- 侧墙保持 ---- */
  bool side_wall_hold_enabled;
  float side_wall_hold_distance, side_wall_hold_correction_distance;
  float side_wall_hold_prediction_sec, side_wall_hold_invalid_hold_sec;
  float side_wall_hold_forward_margin_deg;
  float side_wall_hold_release_distance, side_wall_hold_kp;
  float side_wall_hold_min_yawrate, side_wall_hold_max_yawrate;
  float side_wall_hold_yaw_accel, side_wall_hold_release_sec;

  /* ---- 动态解除/速度恢复 ---- */
  float dynamic_release_margin, dynamic_release_clear_sec;
  float speed_restore_side_clearance, speed_restore_sweep_clearance;
  float speed_restore_stable_sec, speed_restore_max_yawrate;

  /* ---- 前方触发区 ---- */
  float front_corridor_width, front_trigger_start_x, front_trigger_base_width;
  float front_trigger_expand_angle_deg;

  /* ---- 控制器死区 ---- */
  int controller_x_dead_low, controller_x_dead_high;
  int controller_y_dead_low, controller_y_dead_high;

  float hall_feedback_timeout_sec;

  /* ---- 调试/点云裁剪 ---- */
  float debug_nominal_speed;
  bool force_debug_intent;
  float planning_min_x, planning_max_abs_y;
  int marker_stride;
  bool log_trajectory_scores, publish_debug_visualization;
  int trajectory_score_top_n, trajectory_score_log_interval_ms;

  /* ---- 通道引导（deg 入参，rad 存储） ---- */
  bool dwa_corridor_guide_enabled;
  float dwa_corridor_search_start_x, dwa_corridor_search_end_x;
  float dwa_corridor_min_width_margin, dwa_corridor_max_width;
  float dwa_corridor_search_half_width;
  float dwa_corridor_cluster_gap_distance;
  int dwa_corridor_cluster_min_points;
  float dwa_corridor_gate_max_depth_difference;
  float dwa_corridor_gate_blocking_clearance, dwa_corridor_gate_endpoint_exclusion;
  float dwa_corridor_gate_dedup_distance, dwa_corridor_gate_projection_length;
  float dwa_corridor_approach_start_x, dwa_corridor_explicit_intent_ratio;
  float dwa_corridor_max_heading;                  /* rad */
  float dwa_corridor_intent_lateral_range;
  int dwa_corridor_stable_cycles, dwa_corridor_lost_hold_cycles;
  float dwa_corridor_track_lateral_tolerance;
  float dwa_corridor_track_heading_tolerance;      /* rad */
  float dwa_corridor_path_filter_alpha, dwa_corridor_trajectory_lookahead;
  float dwa_corridor_lateral_weight, dwa_corridor_heading_weight;
  float dwa_corridor_progress_weight;
} dwa_planner_params_t;

/* 全部默认值（与 C++ 版构造路径一致），不读 yaml。 */
void dwa_planner_params_defaults(dwa_planner_params_t *params);

/*
 * 默认值 + yaml 覆盖 + constructor 同款 clamp。
 * 返回 0 成功；yaml 文件读取失败返回 -1（此时 params 仍为默认值）。
 */
int dwa_planner_params_load(
  const char *yaml_path, dwa_planner_params_t *params);

/* 将参数映射进 planning 层 dwa_config_t（等价于 node_constructor 中对 config_ 的赋值）。 */
void dwa_planner_params_to_config(
  const dwa_planner_params_t *params, dwa_config_t *config);

/* 打印参数表（验收用）。 */
void dwa_planner_params_dump(const dwa_planner_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* DWA_PARAMS_H */
