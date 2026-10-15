/*
 * 模块功能：4 个电机控制类 + 4 个判定函数。
 * 直译自 motor_bridge_cpp/native/motor_control_profiles.hpp
 * （namespace hu_m40_motor_profiles → mc_ 前缀）。保持 double 运算。
 * 控制器为纯数据结构 + 显式 init/reset 函数（C++ 构造函数语义）。
 */
#ifndef DWA_MOTOR_CONTROL_PROFILES_H
#define DWA_MOTOR_CONTROL_PROFILES_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* —— 判定函数 —— */

/* hall_control_should_follow_raw */
int mc_hall_control_should_follow_raw(
  int mode, int raw_recent, double raw_linear, double linear_deadband);

/* controller_neutral_stop_required */
int mc_controller_neutral_stop_required(
  double sample_time, double now, double sample_timeout,
  int controller_x, int controller_y, int dead_low, int dead_high);

/* linear_output_hard_stop_required（state 为 C++ 状态字符串） */
int mc_linear_output_hard_stop_required(
  int x, int y, const char *state, int raw_forward_active);

/* linear_output_uses_protective_rates */
int mc_linear_output_uses_protective_rates(int mode, int control_fresh);

/* —— HallSpeedController —— */
/* last_info 用结构体保存（对齐 Python 的 dict） */
typedef struct {
  double target;
  double actual;
  double predicted;
  double rate;
  double error;
  double y;
} mc_hall_speed_info_t;

typedef struct {
  /* 构造参数（不变） */
  double gain;
  double max_down_rate;
  double max_up_rate;
  double deadband;
  double minimum_y;
  /* 状态 */
  double output_y;
  int initialized;
  double last_time;
  double last_actual;
  int has_last_actual;
  double last_actual_time;
  double actual_rate;
  mc_hall_speed_info_t last_info;
  int has_last_info;
} mc_hall_speed_controller_t;

void mc_hall_speed_controller_init(
  mc_hall_speed_controller_t *c,
  double gain, double max_down_rate, double max_up_rate,
  double deadband, double minimum_y);

void mc_hall_speed_controller_reset(mc_hall_speed_controller_t *c);

void mc_hall_speed_controller_start(
  mc_hall_speed_controller_t *c, double initial_y, double now);

int mc_hall_speed_controller_update(
  mc_hall_speed_controller_t *c,
  double target, double actual, double raw_cap_y, double initial_y,
  double now, double default_dt, double predict_sec);

/* —— MovingHoldController —— */
typedef struct {
  int emergency_stop;
  int controlled_y;
} mc_moving_hold_result_t;

typedef struct {
  mc_hall_speed_controller_t speed_controller;
  /* 构造参数（不变） */
  double predict_sec;
  double maximum_y;
  double emergency_speed;
  double release_margin;
  int emergency_active;
} mc_moving_hold_controller_t;

void mc_moving_hold_controller_init(
  mc_moving_hold_controller_t *c,
  double gain, double max_down_rate, double max_up_rate,
  double deadband, double minimum_y, double predict_sec,
  double maximum_y, double emergency_speed, double release_margin);

void mc_moving_hold_controller_reset(mc_moving_hold_controller_t *c);

int mc_moving_hold_controller_seed(
  mc_moving_hold_controller_t *c, double initial_y, double now);

mc_moving_hold_result_t mc_moving_hold_controller_update(
  mc_moving_hold_controller_t *c,
  double target, double actual, double raw_cap_y, double initial_y,
  double now, double default_dt);

/* —— HallPrebrakeStopController —— */
typedef struct {
  double target;
  double release_speed;
  double actual;
  double predicted;
  double deceleration;
  double hold_y;
  int released;
} mc_prebrake_info_t;

typedef struct {
  int released;
  int hold_y;
} mc_prebrake_result_t;

typedef struct {
  /* 构造参数（不变） */
  double predict_sec;
  double deadzone_y;
  double release_margin;
  /* 状态 */
  int active;
  int released;
  double hold_y;
  int has_last_speed;
  double last_speed;
  double last_time;
  double deceleration_rate;
  mc_prebrake_info_t last_info;
  int has_last_info;
} mc_prebrake_stop_controller_t;

void mc_prebrake_stop_controller_init(
  mc_prebrake_stop_controller_t *c,
  double predict_sec, double deadzone_y, double release_margin);

void mc_prebrake_stop_controller_reset(mc_prebrake_stop_controller_t *c);

void mc_prebrake_stop_controller_start(
  mc_prebrake_stop_controller_t *c,
  double target, double actual, double raw_cap_y, double target_command_y,
  double now);

mc_prebrake_result_t mc_prebrake_stop_controller_update(
  mc_prebrake_stop_controller_t *c,
  double target, double actual, double raw_cap_y, double target_command_y,
  double now, double default_dt);

/* —— LinearOutputProfile —— */
typedef struct {
  double target;
  double previous;
  double output;
  double delta;
  double dt;
  double rate_limit;
  int limited;
  int hard_stop;
} mc_linear_profile_info_t;

typedef struct {
  /* 构造参数（不变） */
  double minimum_y;
  /* 状态 */
  double output_y;
  int initialized;
  double last_time;
  mc_linear_profile_info_t last_info;
  int has_last_info;
} mc_linear_output_profile_t;

void mc_linear_output_profile_init(
  mc_linear_output_profile_t *c, double minimum_y);

void mc_linear_output_profile_reset(mc_linear_output_profile_t *c);

void mc_linear_output_profile_start(
  mc_linear_output_profile_t *c, double initial_y, double now);

int mc_linear_output_profile_hard_stop(
  mc_linear_output_profile_t *c, double now);

int mc_linear_output_profile_update(
  mc_linear_output_profile_t *c,
  double target_y, double now, double default_dt,
  double up_rate, double down_rate);

#ifdef __cplusplus
}
#endif

#endif /* DWA_MOTOR_CONTROL_PROFILES_H */
