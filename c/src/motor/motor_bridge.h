/*
 * 模块功能：电机桥节点（MotorXYBridge 直译）。
 * 对应 motor_bridge_cpp/native/motor_xy_cmd_vel_node.cpp：
 *   - ROS 订阅回调 → mb_on_* 直接注入接口（P8 由 IPC 层调用）
 *   - ROS 发布器   → 回调 sink（controller_cmd / feedback / wheel_yaw）
 *   - rclcpp wall timer → 主循环周期调用 mb_on_timer
 *   - steady_seconds() → 可注入时间源（默认进程起算单调钟，
 *     对齐 C++ steady_clock 语义）
 *
 * 保持 double 运算（P7 验收=串口帧字节一致，见 TRANSLATION_NOTES）。
 * 状态字符串用定长缓冲 + strcmp 直译 C++ std::string 语义（含 "_stop"
 * 后缀拼接），缓冲上限 MB_STATE_MAX=48（最长状态 30 + "_stop" 5）。
 */
#ifndef DWA_MOTOR_BRIDGE_H
#define DWA_MOTOR_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "motor_bridge_config.h"
#include "motor_control_profiles.h"
#include "motor_protocol.h"
#include "motor_serial.h"
#include "stop_escape_compensator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MB_STATE_MAX 48

/* —— 回调类型 —— */

/* controller_cmd_pub_（geometry_msgs/Twist） */
typedef void (*mb_controller_cmd_sink_t)(void *user, double linear_x,
  double angular_z);

/* feedback_pub_（std_msgs/Float32MultiArray：x,y,left_raw,right_raw,
 * left_mps,right_mps,gear） */
typedef void (*mb_feedback_sink_t)(void *user, const ms_sample_t *sample);

/* wheel_yaw_pub_（std_msgs/Float64） */
typedef void (*mb_wheel_yaw_sink_t)(void *user, double yaw_total);

/* 串口写帧观测钩子（对拍用；生产为 NULL，仅写串口） */
typedef void (*mb_frame_sink_t)(void *user, const mp_frame_t *frame);

/* 时间源（默认 = 进程起算 steady_clock 语义） */
typedef double (*mb_now_fn_t)(void *user);

/* sleep 源（默认 = 真实毫秒 sleep；对拍注入 no-op） */
typedef void (*mb_sleep_fn_t)(void *user, double milliseconds);

/* —— 注入块 ——
 * 对拍/测试在 mb_bridge_init 之前设置（b 须先零初始化）；
 * init 内部 memset 后恢复，保证构造期（启动停止帧串等）就走注入时钟。
 * 生产（bridge_main）保持全 NULL = 真实时钟/串口。 */
typedef struct {
  mb_controller_cmd_sink_t controller_cmd_sink;
  void *controller_cmd_sink_user;
  mb_feedback_sink_t feedback_sink;
  void *feedback_sink_user;
  mb_wheel_yaw_sink_t wheel_yaw_sink;
  void *wheel_yaw_sink_user;
  mb_frame_sink_t frame_sink;
  void *frame_sink_user;
  mb_now_fn_t now_fn;
  void *now_fn_user;
  mb_sleep_fn_t sleep_fn;
  void *sleep_fn_user;
} mb_injection_t;

/* —— ClosedLoopInfo / 干预斜坡信息（诊断结构直译） —— */
typedef struct {
  int active;
  double linear, angular;
  double actual_linear, actual_yaw_rate;
  double left_mps, right_mps;
  double linear_corr, angular_corr;
  double corrected_linear, corrected_angular;
} mb_closed_loop_info_t;

typedef struct {
  double angular;
  double ratio;
  int x;
} mb_turn_deadzone_info_t;

/* —— 逃逸点云视图（sensor_msgs/PointCloud2 镜像） —— */
typedef struct {
  char name[16]; /* "x" / "y" / ... */
  int datatype;  /* ROS PointField：FLOAT32=7, FLOAT64=8 */
  int offset;
} mb_cloud_field_t;

typedef struct {
  const uint8_t *data;
  size_t data_len;
  int width;
  int height;
  int point_step;
  int row_step;      /* <=0 时按 point_step*width（对齐 C++ 逻辑） */
  int big_endian;
  const mb_cloud_field_t *fields;
  size_t field_count;
} mb_cloud_view_t;

/* cmd_to_xy / apply_hall_speed_output / apply_linear_output_profile 结果 */
typedef struct {
  int x;
  int y;
  char state[MB_STATE_MAX];
} mb_cmd_result_t;

typedef struct {
  int x;
  int y;
  char state[MB_STATE_MAX];
} mb_hall_result_t;

typedef struct {
  int x;
  int y;
  char state[MB_STATE_MAX];
} mb_linear_result_t;

typedef struct {
  int should_write;
  char reason[MB_STATE_MAX];
} mb_write_decision_t;

/* —— 桥上下文（MotorXYBridge 成员全集） —— */
typedef struct mb_bridge_t {
  mb_config_t config;
  ms_serial_writer_t writer;
  se_stop_escape_compensator_t stop_escape;
  se_stop_escape_decision_t stop_escape_last_decision;
  double stop_escape_last_step_time;
  /* 电机桥兜底脱困事件统计：按 active 的上升沿计数。 */
  uint64_t stop_escape_event_count;
  int stop_escape_event_active;
  uint64_t tx_attempt_count;
  uint64_t tx_full_count;
  uint64_t tx_fail_count;
  long tx_last_result;
  int tx_last_errno;
  double tx_last_time;
  double last_chain_diag_time;
  double last_tx_error_log_time;
  /* 控制器有效反馈在线门控：启动后给予 1s 接收宽限，之后按有效帧刷新。 */
  double controller_feedback_last_valid_time;
  int controller_feedback_seen;
  int controller_feedback_online;
  /* Optional production interlock; individual sensor health is independent. */
  int sensor_interlock_enabled;
  int sensor_fault_last_mask;
  double last_valid_imu_time;
  int valid_imu_seen;
  double escape_imu_yaw_rate;
  double escape_imu_time;
  int escape_imu_valid;
  mc_hall_speed_controller_t hall_speed_controller;
  mc_moving_hold_controller_t moving_hold_controller;
  mc_prebrake_stop_controller_t prebrake_stop_controller;
  mc_linear_output_profile_t linear_output_profile;
  char linear_output_profile_kind[MB_STATE_MAX]; /* "idle" 起始 */

  double forward_speed_reference;
  double last_cmd_time;
  double last_linear;
  double last_angular;
  double last_raw_cmd_time;
  double last_raw_linear;
  double last_raw_angular;
  mp_frame_t last_frame;        /* 空 = len 0（C++ 空 vector） */
  mp_frame_t last_written_frame;
  double last_log_time;
  int was_motion;
  int intervention_motion_active;
  double safe_stop_hold_until;

  double last_wheel_feedback_time;
  int wheel_filter_initialized;
  double wheel_left_mps;
  double wheel_right_mps;
  double actual_linear;
  double actual_yaw_rate;
  double raw_yaw_rate;
  double wheel_yaw_total;
  double wheel_yaw_last_time;
  double wheel_yaw_last_rate;
  long wheel_yaw_accepted;
  long wheel_yaw_rejected;

  double closed_loop_angular_i;
  double closed_loop_last_time;
  mb_closed_loop_info_t closed_loop_info;
  int closed_loop_has_info;

  int intervention_slew_initialized;
  double intervention_slew_last_time;
  double intervention_slew_linear;
  double intervention_slew_angular;
  double intervention_slew_target_linear;
  double intervention_slew_target_angular;
  double intervention_slew_prev_linear;
  double intervention_slew_prev_angular;
  int intervention_slew_has_info;

  mb_turn_deadzone_info_t output_turn_deadzone_info;
  int output_turn_deadzone_has_info;

  int last_controller_x;
  int last_controller_y;
  double last_controller_sample_time;
  int controller_neutral_seen;

  int hall_speed_control_active;
  double hall_speed_control_target;
  double hall_speed_brake_target;
  double hall_speed_dynamic_down_rate;
  int hall_speed_control_mode;
  double last_hall_speed_control_time;
  int hard_stop_active;
  double last_hard_stop_time;
  int hall_speed_fault_wait_neutral;
  int hall_speed_release_active;

  double left_scan_time;
  double right_scan_time;
  double imu_time;
  int gyro_angle_byte;
  int controller_gear;
  int controller_gear_valid;
  int output_gear;
  int output_gear_valid;
  int output_gear_closed_loop_active;
  double last_output_gear_update_time;
  int horn_requested;
  double last_horn_time;
  double last_escape_cloud_warning_time;
  double last_raw_frame_log_time;
  mp_frame_t stop_frame;

  /* 发布器开关（对应 publisher shared_ptr 非空 = topic 非空） */
  int controller_cmd_pub;
  int feedback_pub;
  int wheel_yaw_pub;

  /* 输出 sink / 时钟 / sleep 注入（见 mb_injection_t 注释） */
  mb_injection_t inject;
} mb_bridge_t;

/*
 * 构造（MotorXYBridge ctor 直译）。先开串口（dry_run 跳过），
 * 再初始化控制器与状态；continuous 模式发启动停止帧串。
 * 前置条件：b 已零初始化（memset/calloc）；需要注入时钟/sink 时在调用
 * 前设置 b->inject（init 会保留）。失败返回 -1 并写 err（对应 C++ throw）。
 */
int mb_bridge_init(mb_bridge_t *b, const mb_config_t *config,
  char *err, size_t err_size);

/* 析构（~MotorXYBridge → stop_and_close 直译）。 */
void mb_bridge_destroy(mb_bridge_t *b);

/* —— 输入注入（ROS 订阅回调直译） —— */
void mb_on_cmd(mb_bridge_t *b, double linear_x, double angular_z);
void mb_on_hall_speed_control(mb_bridge_t *b, const float *data,
  size_t data_size);
void mb_on_escape_cloud(mb_bridge_t *b, const mb_cloud_view_t *cloud);
void mb_on_escape_imu_yawrate(mb_bridge_t *b, double yawrate, int valid);
void mb_bridge_enable_sensor_interlock(mb_bridge_t *b, int enabled);
void mb_on_left_scan(mb_bridge_t *b);
void mb_on_right_scan(mb_bridge_t *b);
void mb_on_imu(mb_bridge_t *b, double ax, double ay, double az);
void mb_on_horn(mb_bridge_t *b, int horn);

/* 反馈样本处理（publish_sample 直译；串口读回样本或对拍注入）。 */
void mb_publish_sample(mb_bridge_t *b, const ms_sample_t *sample);

/* 串口轮询（poll_status 直译：read_samples → publish_sample）。 */
void mb_poll_status(mb_bridge_t *b);

/* —— 主循环节拍（on_timer 直译，wall timer 周期调用） —— */
void mb_on_timer(mb_bridge_t *b);

/* —— 输出 sink 注册 —— */
void mb_bridge_set_controller_cmd_sink(mb_bridge_t *b,
  mb_controller_cmd_sink_t sink, void *user);
void mb_bridge_set_feedback_sink(mb_bridge_t *b,
  mb_feedback_sink_t sink, void *user);
void mb_bridge_set_wheel_yaw_sink(mb_bridge_t *b,
  mb_wheel_yaw_sink_t sink, void *user);
void mb_bridge_set_frame_sink(mb_bridge_t *b,
  mb_frame_sink_t sink, void *user);

/* —— 时间 / sleep 注入（对拍用） —— */
void mb_bridge_set_time_source(mb_bridge_t *b, mb_now_fn_t fn, void *user);
void mb_bridge_set_sleep_source(mb_bridge_t *b, mb_sleep_fn_t fn, void *user);

/* —— 供主程序/对拍使用的内部工具（对应 public 方法） —— */
int mb_horn_active(mb_bridge_t *b, double now);
int mb_radar_status(mb_bridge_t *b, double now);
int mb_current_output_gear(mb_bridge_t *b);
double mb_forward_reference_now(mb_bridge_t *b);
/* A2(10.09)：Y 油门映射基准（档位峰值缩放，详见 .c 注释）。 */
double mb_y_scale_reference_now(mb_bridge_t *b);
mp_frame_t mb_build_output_frame(mb_bridge_t *b, int x, int y);
void mb_motion_to_xy(mb_bridge_t *b, double linear, double angular,
  int *out_x, int *out_y);
int mb_hall_speed_control_is_fresh(mb_bridge_t *b, double now);
int mb_dwa_command_can_cover_hall_control_gap(mb_bridge_t *b, double now);
int mb_raw_command_is_recent(mb_bridge_t *b, double now);
int mb_raw_command_is_motion(mb_bridge_t *b);
int mb_raw_forward_is_active(mb_bridge_t *b, double now);
void mb_raw_override_xy(mb_bridge_t *b, int *out_x, int *out_y);
int mb_raw_forward_cap_y(mb_bridge_t *b);
void mb_latch_hall_speed_fault(mb_bridge_t *b, const char *reason);
int mb_wheel_feedback_is_fresh(mb_bridge_t *b, double now);
int mb_wheel_motion_active(mb_bridge_t *b, double now);
void mb_reset_closed_loop_state(mb_bridge_t *b);
void mb_reset_intervention_slew_state(mb_bridge_t *b);
double mb_limit_step(double previous, double target, double max_step);
void mb_update_wheel_feedback(mb_bridge_t *b, const ms_sample_t *sample);
void mb_update_wheel_yaw_total(mb_bridge_t *b, double now);
double mb_controller_axis_delta(mb_bridge_t *b, int value, int center,
  int dead_low, int dead_high, int invert);
double mb_controller_turn_ratio(mb_bridge_t *b, int value);
void mb_controller_status_to_cmd(mb_bridge_t *b, const ms_sample_t *sample,
  double *out_linear_x, double *out_angular_z);
int mb_controller_sample_is_neutral(mb_bridge_t *b, const ms_sample_t *sample);
int mb_physical_controller_is_neutral(mb_bridge_t *b, double now);
void mb_clear_motion_state_for_controller_neutral(mb_bridge_t *b);
void mb_apply_closed_loop(mb_bridge_t *b, double linear, double angular,
  double now, double *out_linear, double *out_angular);
void mb_apply_intervention_slew(mb_bridge_t *b, double linear, double angular,
  double now, double *out_linear, double *out_angular);
mb_cmd_result_t mb_cmd_to_xy(mb_bridge_t *b, int apply_closed_loop_flag);
mb_hall_result_t mb_apply_hall_speed_output(mb_bridge_t *b, int normal_x,
  int normal_y, const char *state, double now);
void mb_sync_hall_speed_controller_to_output(mb_bridge_t *b, int output_y,
  const char *state, double now);
mb_linear_result_t mb_apply_linear_output_profile(mb_bridge_t *b, int x, int y,
  const char *state, double now);
mb_write_decision_t mb_should_write_intervention(mb_bridge_t *b, double now,
  const char *state, int is_idle_stop);
se_stop_escape_decision_t mb_update_stop_escape(mb_bridge_t *b, double now,
  int controller_neutral_stop, int normal_output_stopped);
int mb_raw_reverse_passthrough_xy(mb_bridge_t *b, double now,
  int *out_x, int *out_y, char out_state[MB_STATE_MAX]);
void mb_stop_and_close(mb_bridge_t *b);
void mb_send_stop_burst(mb_bridge_t *b, const char *reason, int repeats,
  double settle);
void mb_send_intervention_release_stop(mb_bridge_t *b, const char *reason);

/* 桥内时钟（steady_seconds 直译：默认进程起算；注入时直通）。 */
double mb_now(mb_bridge_t *b);

#ifdef __cplusplus
}
#endif

#endif /* DWA_MOTOR_BRIDGE_H */
