/*
 * 模块功能：DWA 运行时节点数据面（node_members.inc 直译）与输入输出接口。
 * 对应 C++ 侧 DwaLocalPlannerNode 的全部可直译成员：工作区、模块实例、
 * 参数快照、控制状态；ROS 回调 → 输入注入函数；publish → 输出面。
 *
 * 分文件实现：
 *   dwa_node.c        —— init/destroy/输入注入/read_intent/motor_feedback
 *                        （node_input_state.inc 直译）
 *   dwa_node_tick.c   —— tick() 主循环（node_tick.inc 直译）
 *   dwa_node_safety.c —— 安全控制流（node_safety_control.inc 直译）
 *   dwa_node_perception.c —— 感知投影/掩码/诊断（node_perception_grid.inc +
 *                        node_perception_scan.inc 直译）
 *
 * P6 阶段；全 float 规范（P4.5）。
 */
#ifndef DWA_NODE_H
#define DWA_NODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../common/dwa_types.h"
#include "../config/dwa_params.h"
#include "../fusion/dynamic_velocity_fusion.h"
#include "../guidance/corridor_guide.h"
#include "../guidance/dwa_geometric_selector.h"
#include "../hal/hal_mutex.h"
#include "../perception/local_occupancy_grid.h"
#include "../planning/dwa_sampling.h"
#include "../runtime/planner_data_types.h"
#include "../safety/side_wall_hold_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 输出面（geometry_msgs::msg::Twist 的数据面） ---- */
typedef struct {
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
} dwa_twist_t;

static inline void dwa_twist_reset(dwa_twist_t *t)
{
  t->linear_x = 0.0f;
  t->linear_y = 0.0f;
  t->linear_z = 0.0f;
  t->angular_x = 0.0f;
  t->angular_y = 0.0f;
  t->angular_z = 0.0f;
}

/* ---- Hall 速度控制输出帧（std_msgs::msg::Float32MultiArray 数据面，8 元素） ---- */
typedef struct {
  /* Internal callback metadata; not serialized to the controller. */
  float data[8];
  int escape_request;
} dwa_hall_speed_control_msg_t;

/*
 * 控制输出 sink：publish_regular_command 的最终落点。
 * 对应 C++ 侧 cmd_pub_ + hall_speed_control_pub_ 两个话题发布；
 * P7 电机桥 / P8 影子模式在这里接入。
 */
typedef void (*dwa_node_cmd_sink_fn)(
  void *user_data,
  const dwa_twist_t *cmd,
  const dwa_hall_speed_control_msg_t *hall_control);

/* ---- FusedBasePoint（融合点云解码后的点） ---- */
typedef struct {
  dwa_point_t point;
  bool is_left;
  /* Compatibility metadata retained for the base-cloud wire format. The
   * production path now sends one canonical filtered cloud; no synthetic
   * side-rescue or front-only point stream is appended. */
  uint8_t side_safety_only;
  /* Compatibility metadata retained; production points use the canonical
   * cloud and are consumed by the normal geometric views. */
  uint8_t front_safety_only;
} dwa_fused_base_point_t;

/* ---- 动态点数组工作区（vector<DwaPoint> 直译） ---- */
typedef struct {
  dwa_point_t *points;
  size_t count;
  size_t capacity;
} dwa_point_buf_t;

void dwa_point_buf_destroy(dwa_point_buf_t *buf);

/* ---- 动态几何扫描点数组（vector<DwaGeometricScanPoint> 直译） ---- */
typedef struct {
  dwa_geometric_scan_point_t *items;
  size_t count;
  size_t capacity;
} dwa_geometric_scan_buf_t;

void dwa_geometric_scan_buf_destroy(dwa_geometric_scan_buf_t *buf);

/* ---- TickTiming（planner_runtime.hpp 直译，ns 精度计时降为 double 秒差） ---- */
typedef enum {
  DWA_TICK_SEGMENT_OTHER = 0,
  DWA_TICK_SEGMENT_POINT_CLOUD = 1,
  DWA_TICK_SEGMENT_DWA_PLANNING = 2,
  DWA_TICK_SEGMENT_TRAJECTORY_PUBLISH = 3
} dwa_tick_segment_t;

typedef struct {
  double started_sec;
  double last_transition_sec;
  dwa_tick_segment_t active;
  dwa_tick_segment_t stack[8];
  size_t stack_count;
  double total_ms;
  double pointcloud_ms;
  double dwa_ms;
  double trajectory_publish_ms;
  bool finished;
} dwa_tick_timing_t;

void dwa_tick_timing_init(dwa_tick_timing_t *timing);
void dwa_tick_timing_enter(dwa_tick_timing_t *timing, dwa_tick_segment_t segment);
void dwa_tick_timing_leave(dwa_tick_timing_t *timing);
void dwa_tick_timing_finish(dwa_tick_timing_t *timing);
double dwa_tick_timing_other_ms(const dwa_tick_timing_t *timing);

/*
 * DWA 运行时节点（node_members.inc 直译）。
 * 参数快照集中在 params（dwa_planner_params_t，P1 已含默认值与 yaml 覆盖）；
 * 运行时状态与工作区为本结构成员，命名与 C++ 一一对应。
 */
typedef struct {
  /* ---- 参数与配置（constructor 落定的最终值） ---- */
  const dwa_planner_params_t *params; /* 外部持有；init 时深拷贝到本地快照 */
  dwa_planner_params_t params_snapshot;
  planner_laser_mount_t left_mount;
  planner_laser_mount_t right_mount;
  dwa_config_t config;

  /* ---- 工作区 ---- */
  dwa_plan_result_t dwa_plan_workspace;
  dwa_obstacle_spatial_index_t dwa_spatial_index_workspace;
  dwa_grid_blocking_index_t dwa_blocking_index_workspace;
  dwa_occupancy_grid_t occupancy_grid_workspace;
  dwa_point_buf_t obstacles_workspace;
  dwa_point_buf_t side_safety_points_workspace;
  dwa_geometric_scan_buf_t left_geometric_scan_points_workspace;
  dwa_geometric_scan_buf_t right_geometric_scan_points_workspace;

  /* ---- 控制状态（node_members 直译，含默认值） ---- */
  float last_output_linear;
  float last_valid_dwa_linear;
  bool forced_straight_active;
  bool moving_dwa_command_published;
  bool hard_stop_latched;
  int hard_stop_clear_cycles;
  float hard_stop_escape_started_sec; /* NaN = 未激活 */
  int hard_stop_escape_eval_cycles;
  planner_manual_turn_direction_t hard_stop_escape_direction;
  uint64_t hard_stop_escape_last_active_tick;
  /* 脱困事件统计：按一次连续脱困过程计数，不按控制周期重复计数。 */
  uint64_t no_valid_escape_event_count;
  uint64_t no_valid_escape_last_tick;
  uint64_t hard_stop_escape_event_count;
  uint64_t hard_stop_escape_event_last_tick;
  bool escape_request_pending;
  /* 9.28 Fix I：方向锁定时刻（滞回窗口起点），NaN = 未锁定 */
  float hard_stop_escape_direction_latched_sec;
  bool near_obstacle_gear_latched;
  float near_obstacle_gear_nearest_distance;
  float stationary_turn_ring_speed_limit_mps;
  float stationary_turn_ring_nearest_distance_m;
  float moving_prebrake_y_down_rate;
  float runtime_velocity_limit;
  bool runtime_velocity_limit_initialized;
  int output_effective_gear;
  bool output_gear_closed_loop_active;
  float static_dwa_hall_target;
  bool static_dwa_hall_target_valid;
  uint64_t tick_count;
  size_t body_masked_points_this_tick;
  size_t side_flank_points_this_tick; /* 9.28 Fix D：本周期跨车身侧墙救援点数 */
  /* V4d(10.08)：本周期前方触发楔形区【未截断】最小前向净距——在
   * dnp_project_base_cloud_to_grid 的 obstacle_range 截断之前从融合
   * 点云统计。障碍表内的 nearest_front 被 obstacle_range(板端3.50) 钳在
   * ≈obstacle_range−车头前缘(0.90)≈2.6m，F1d 的
   * bypass_near_zone_far_distance(板端3.80) 永远够不着（093514 日志
   * 定案：通道远墙出感知圈→latch 释放回档位峰值→墙进圈急刹+大舵撞
   * 对侧）。INFINITY = 本周期前方楔形区无命中。仅供速度模块 F1d 与
   * [DWA链路] 日志，不进障碍表/栅格/评分。 */
  float front_unclipped_clearance_this_tick;
  size_t front_safety_candidates_this_tick;
  size_t front_safety_hits_this_tick;
  size_t local_foot_masked_points_this_tick;
  size_t left_angle_cropped_points_this_tick;
  size_t right_angle_cropped_points_this_tick;
  /* F3(9.27) 骑乘过滤宽簇墙闸诊断：带内点数 + 本周期是否检出宽簇
   * （墙/大面积障碍，此时停用骑乘过滤防止盲区带复发）。 */
  size_t rider_band_points_this_tick;
  bool rider_band_wall_gate_this_tick;
  bool bypass_near_zone_latched; /* F1d(9.27)：旁路速度两态迟滞锁存（避障区/空旷区） */
  /* V4a(10.08)：latch 释放防抖——连续超 release 阈值的周期计数；
   * 任何低于 release 的样本（含迟滞带）清零。 */
  unsigned char bypass_release_count;
  /* V4b(10.08)：断崖斜坡状态——进近障区时 target 从 hall 实速按
   * 实际减速度斜坡降到 forward_speed_limit，防 DWA 认知断崖。 */
  bool bypass_near_zone_ramp_active;
  float bypass_near_zone_ramp_target;
  /* A1(10.09)：斜坡一次性武装标志——锁存期内只在首次到达时起坡一次。
   * V4b 原版 ramp 落地后 ramp_active=false，下一 tick 只要 hall 实速
   * >floor 就从实速重新起坡：target/cap/发布 v 全部跳回实速 → hall
   * 闭环（目标=发布 v）误差归零永不下压 → 4 档进通道起飞（103434
   * ln=2464-2506：fb 0.92→1.41，target 0.700→1.287 跳变即重挂）。
   * 锁存期间 target 钉死 forward_speed_limit；释放时复位。 */
  bool bypass_near_zone_ramp_done;

  /* ---- Hall 反馈滤波状态（node_input_state） ---- */
  float last_wheel_left_mps;
  float last_wheel_right_mps;
  float filtered_actual_forward_speed;
  bool moving_hall_speed_filter_initialized;
  bool wheel_feedback_valid;
  double last_wheel_feedback_time;
  bool forward_start_rising_latched;
  int last_controller_x;
  bool controller_x_valid;
  int last_controller_y;
  bool controller_y_valid;
  double last_controller_axes_time;
  int raw_gear;
  bool raw_gear_valid;

  /* ---- 档位日志状态（node_members：log_gear_closed_loop_state 的记忆） ---- */
  bool gear_log_initialized;
  int last_logged_raw_gear;
  int last_logged_effective_gear;
  int last_logged_gear_target;
  bool last_logged_gear_closed_loop_active;
  bool last_logged_gear_transition_active;

  /* ---- 模块实例 ---- */
  dwa_dynamic_velocity_config_t dynamic_config;
  dwa_dynamic_velocity_fusion_t dynamic_fusion;
  dwa_side_wall_hold_config_t side_wall_hold_config;
  dwa_side_wall_hold_controller_t side_wall_hold_controller;
  dwa_corridor_guide_config_t corridor_guide_config;
  dwa_corridor_guide_t corridor_guide;
  dwa_geometric_selector_config_t geometric_selector_config;
  dwa_geometric_trajectory_selector_t geometric_selector;

  /* ---- 输入面（回调注入，mutex 保护） ---- */
  hal_mutex_t scan_mutex;
  dwa_fused_base_point_t *fused_cloud_points;
  size_t fused_cloud_count;
  size_t fused_cloud_capacity;
  double fused_cloud_time; /* 秒；0 = 从未收到 */
  hal_mutex_t cmd_mutex;
  dwa_twist_t last_cmd;
  double last_cmd_time;
  dwa_velocity_t current_velocity;
  double odom_time;
  hal_mutex_t motor_feedback_mutex;

  /* ---- 输出面（publish_motion_command_safely 的落点） ---- */
  dwa_twist_t last_published_cmd;
  uint64_t last_published_tick;
  bool hall_speed_control_active; /* 输出帧的 hall 速度控制标志（恒 true，保留语义） */

  /* ---- 调试开关 ---- */
  float debug_nominal_speed;
  bool force_debug_intent;

  /* ---- 诊断/日志开关（node_members 类内初始化器直译） ---- */
  bool publish_debug_visualization; /* RViz 可视化（C 数据面无 RViz，仅作日志门控） */
  bool log_trajectory_scores;
  int trajectory_score_top_n;
  int trajectory_score_log_interval_ms;
  float tick_timing_warn_ms;

  /* ---- 输出 sink（publish 的数据面落点） ---- */
  dwa_node_cmd_sink_fn cmd_sink;
  void *cmd_sink_user_data;

  /* ---- P9 链路诊断（robot_main 每周期打印；定位 raw DWA 与最终输出间
   *      哪条策略改写了指令。路径标签见 dwa_node_tick.c 各埋点） ---- */
  float diag_raw_dwa_v, diag_raw_dwa_w;   /* 规划原始选中（几何选择器前） */
  float diag_selected_v, diag_selected_w; /* 几何选择器后的选中 */
  float diag_planning_cap;                /* DWA 采样速度上限 */
  float diag_runtime_max;                 /* 距离比例目标 runtime_max_velocity */
  float diag_center_v, diag_center_w;     /* DWA 窗口中心 current */
  float diag_side_clear_left, diag_side_clear_right; /* 9.28 Fix D：侧净距（含跨车身救援点） */
  int diag_front_turn_bias_active;
  float diag_front_turn_bias_clearance;
  int diag_takeover;                      /* decision.dwa_takeover */
  int diag_mode;                          /* decision.mode 枚举值 */
  int diag_valid_count;                   /* 有效候选轨迹数 */
  char diag_path[24];                     /* 发布路径标签 */
} dwa_node_t;

/*
 * 初始化：深拷贝 params、按 constructor 语义组装各 config 并初始化模块实例。
 * 返回 0 成功；内存失败返回 -1（调用方须 dwa_node_destroy）。
 */
int dwa_node_init(dwa_node_t *node, const dwa_planner_params_t *params);
void dwa_node_destroy(dwa_node_t *node);

/* ---- 输入注入（ROS 回调直译；线程安全） ---- */

/* PointCloud2 已解码的融合点（fusion 节点输出；P7/P8 由真实数据源喂入）。 */
void dwa_node_on_fused_cloud(
  dwa_node_t *node, const dwa_fused_base_point_t *points, size_t count);

/* 控制器原始 Twist。 */
void dwa_node_on_cmd(dwa_node_t *node, const dwa_twist_t *cmd);

/* 电机反馈 Float32MultiArray：data[0]=x data[1]=y data[4/5]=轮速 data[6]=档位。 */
void dwa_node_on_motor_feedback(
  dwa_node_t *node, const float *data, size_t data_count);

/* 里程计速度。 */
void dwa_node_on_odom(dwa_node_t *node, const dwa_velocity_t *velocity);

/*
 * decode_base_cloud 直译（node_perception_grid.inc）：从融合节点 PointCloud2
 * 的原始字节缓冲解码 x/y/source 字段。data 布局由调用方按 fields 偏移给出；
 * 返回写入 out 的点数（<= max_out），输入不完整返回 0。
 */
size_t dwa_node_decode_base_cloud(
  const uint8_t *data,
  size_t data_size,
  size_t point_step,
  size_t x_offset,
  size_t y_offset,
  size_t source_offset,
  size_t point_count,
  dwa_fused_base_point_t *out,
  size_t max_out);

/* ---- 输出 sink：控制命令发布落点（NULL = 只记录到 last_published_cmd）。 ---- */
void dwa_node_set_cmd_sink(
  dwa_node_t *node, dwa_node_cmd_sink_fn sink, void *user_data);

/* ---- 输入状态（node_input_state.inc 直译） ---- */
dwa_velocity_t dwa_node_read_intent(const dwa_node_t *node);
planner_motor_feedback_snapshot_t dwa_node_motor_feedback_snapshot(
  const dwa_node_t *node);
planner_motor_feedback_snapshot_t dwa_node_consume_motor_feedback(
  dwa_node_t *node, bool forward_intent);
bool dwa_node_pure_forward_intent(const dwa_node_t *node,
  const dwa_velocity_t *intent);
float dwa_node_joystick_forward_angle_deg(const dwa_node_t *node,
  const dwa_velocity_t *intent);
float dwa_node_manual_yawrate_for_straight_override(const dwa_node_t *node,
  const dwa_velocity_t *intent);
const dwa_trajectory_t *dwa_node_find_valid_straight_trajectory(
  const dwa_plan_result_t *result);
float dwa_node_preview_slew_output_linear(const dwa_node_t *node, float target);
float dwa_node_slew_output_linear(dwa_node_t *node, float target);
void dwa_node_reset_planning_control(dwa_node_t *node);

/* ---- 主循环（node_tick.inc 直译；dwa_node_tick.c） ---- */
void dwa_node_tick(dwa_node_t *node);

/* now() 直译：数据时间戳用墙钟（rclcpp::Time 语义）。 */
double dwa_node_now_sec(void);

/* stale(time, timeout) 直译：从未收到（time<=0）视为过期。 */
bool dwa_node_stale(double data_time, float timeout_sec);

#ifdef __cplusplus
}
#endif

#endif /* DWA_NODE_H */
