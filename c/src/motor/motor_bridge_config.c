/*
 * 模块功能：motor_bridge_config.h 对应实现（Config/parse_config 直译）。
 */
#include "motor_bridge_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void copy_str(char *dst, size_t size, const char *src)
{
  strncpy(dst, src, size - 1);
  dst[size - 1] = '\0';
}

static int take_value(int argc, char **argv, int *index, char *value)
{
  if (*index + 1 >= argc) return 0;
  *index += 1;
  strcpy(value, argv[*index]);
  return 1;
}

/* parse_number<int>（stoi + consumed==size 校验） */
static int parse_int_value(const char *value, int *output)
{
  char *end = NULL;
  long v;
  if (value[0] == '\0') return 0;
  v = strtol(value, &end, 10);
  if (end == value || *end != '\0') return 0;
  *output = (int)v;
  return 1;
}

/* parse_number<double>（stod + consumed==size 校验） */
static int parse_double_value(const char *value, double *output)
{
  char *end = NULL;
  double v;
  if (value[0] == '\0') return 0;
  v = strtod(value, &end);
  if (end == value || *end != '\0') return 0;
  *output = v;
  return 1;
}

void mb_config_defaults(mb_config_t *c)
{
  memset(c, 0, sizeof(*c));
  copy_str(c->port, sizeof(c->port), "/dev/ttyS6");
  c->baud = 9600;
  c->protocol = MP_PROTOCOL_20;
  copy_str(c->protocol_name, sizeof(c->protocol_name), "2.0");
  copy_str(c->cmd_topic, sizeof(c->cmd_topic), "/cmd_vel");
  c->controller_cmd_topic[0] = '\0';
  c->feedback_topic[0] = '\0';
  c->wheel_yaw_topic[0] = '\0';
  copy_str(c->hall_speed_control_topic, sizeof(c->hall_speed_control_topic),
    "/dwa/hall_speed_control");
  copy_str(c->left_scan_topic, sizeof(c->left_scan_topic), "/scan_left_safety");
  copy_str(c->right_scan_topic, sizeof(c->right_scan_topic),
    "/scan_right_safety");
  copy_str(c->escape_cloud_topic, sizeof(c->escape_cloud_topic),
    "/dwa/fused_scan_base");
  c->escape_stop_hold_sec = 0.5;
  c->escape_stop_speed = 0.03;
  c->escape_forward_deadband = 0.02;
  c->escape_turn_yawrate = 0.5;
  c->escape_cloud_timeout = 0.60;
  c->escape_half_width = 0.27;
  c->radar_timeout = 0.5;
  copy_str(c->horn_topic, sizeof(c->horn_topic), "/wheelchair/horn_cmd");
  c->horn_timeout = 0.5;
  c->fallback_gear = 1;
  c->gear_peak_speeds_count = 0;
  copy_str(c->imu_topic, sizeof(c->imu_topic), "/imu/data_raw");
  copy_str(c->gyro_angle_source, sizeof(c->gyro_angle_source), "zero");
  c->max_forward_speed = 0.50;
  c->max_reverse_speed = 0.70;
  c->max_turn_speed = 0.60;
  c->forward_output_deadzone_y = 120;
  c->linear_deadband = 0.01;
  c->angular_deadband = 0.02;
  c->cmd_timeout = 0.5;
  c->interval_ms = 200;
  copy_str(c->output_mode, sizeof(c->output_mode), "continuous");
  c->intervention_linear_delta = 0.02;
  c->intervention_angular_delta = 0.015;
  c->intervention_release_stop_repeats = 3;
  c->safe_stop_hold_sec = 2.5;
  c->safe_stop_wheel_linear_threshold = 0.03;
  c->safe_stop_wheel_angular_threshold = 0.05;
  c->intervention_linear_slew_rate = 0.6;
  c->intervention_angular_slew_rate = 0.9;
  c->wheel_track_m = 0.56;
  c->wheel_feedback_timeout = 0.25;
  c->wheel_feedback_alpha = 0.35;
  c->hall_speed_control_timeout = 0.25;
  c->hall_speed_y_gain = 300.0;
  c->hall_speed_y_down_rate = 180.0;
  c->hall_speed_y_up_rate = 80.0;
  c->hall_moving_hold_y_up_rate = 20.0;
  c->hall_moving_hold_max_y = 150.0;
  c->hall_moving_hold_emergency_speed = 0.52;
  c->hall_moving_hold_release_margin = 0.04;
  c->hall_speed_deadband = 0.03;
  c->hall_speed_decel_predict_sec = 0.25;
  c->linear_profile_enabled = 0;
  c->linear_profile_comfort_up_rate = 15.0;
  c->linear_profile_comfort_down_rate = 15.0;
  c->linear_profile_protective_up_rate = 15.0;
  c->linear_profile_protective_down_rate = 60.0;
  c->wheel_yaw_max_dt = 0.15;
  copy_str(c->closed_loop_mode, sizeof(c->closed_loop_mode), "off");
  c->closed_loop_angular_kp = 0.22;
  c->closed_loop_angular_ki = 0.02;
  c->closed_loop_linear_kp = 0.0;
  c->closed_loop_max_angular_correction = 0.04;
  c->closed_loop_max_linear_correction = 0.0;
  c->pre_stop_repeats = 5;
  c->pre_stop_settle = 0.1;
  c->activate_stop_repeats = 0;
  c->activate_stop_settle = 0.05;
  c->stop_repeats = 5;
  c->stop_on_close = 1;
  copy_str(c->idle_stop_mode, sizeof(c->idle_stop_mode), "continuous");
  c->axis_lock = 0;
  c->invert_linear = 0;
  c->invert_angular = 0;
  c->controller_center_x = MP_CENTER;
  c->controller_center_y = MP_CENTER;
  c->controller_dead_low = 89;
  c->controller_dead_high = 111;
  c->output_turn_dead_low = 89;
  c->output_turn_dead_high = 111;
  c->controller_axis_lock = 0;
  c->controller_invert_x = 0;
  c->controller_invert_y = 0;
  c->controller_require_neutral = 1;
  c->feedback_verbose = 0;
  c->rx_debug = 0;
  c->dry_run = 0;
  c->verbose = 0;
}

static void set_protocol(mb_config_t *c, const char *value)
{
  copy_str(c->protocol_name, sizeof(c->protocol_name), value);
  if (strcmp(value, "2.0") == 0) c->protocol = MP_PROTOCOL_20;
  else if (strcmp(value, "2.1") == 0) c->protocol = MP_PROTOCOL_21;
  else if (strcmp(value, "2.3") == 0) c->protocol = MP_PROTOCOL_23;
  else c->protocol = MP_PROTOCOL_GEAR_HORN;
}

int mb_config_parse(int argc, char **argv, mb_config_t *c,
  char *err, size_t err_size)
{
  int index;
  char value[256];
  mb_config_defaults(c);
  err[0] = '\0';

  for (index = 1; index < argc; ++index) {
    const char *key = argv[index];
    if (strcmp(key, "--ros-args") == 0) break;
    if (strcmp(key, "--linear-profile-enabled") == 0) { c->linear_profile_enabled = 1; continue; }
    if (strcmp(key, "--no-linear-profile-enabled") == 0) { c->linear_profile_enabled = 0; continue; }
    if (strcmp(key, "--controller-require-neutral") == 0) { c->controller_require_neutral = 1; continue; }
    if (strcmp(key, "--no-controller-require-neutral") == 0) { c->controller_require_neutral = 0; continue; }
    if (strcmp(key, "--axis-lock") == 0) { c->axis_lock = 1; continue; }
    if (strcmp(key, "--invert-linear") == 0) { c->invert_linear = 1; continue; }
    if (strcmp(key, "--invert-angular") == 0) { c->invert_angular = 1; continue; }
    if (strcmp(key, "--controller-axis-lock") == 0) { c->controller_axis_lock = 1; continue; }
    if (strcmp(key, "--controller-invert-x") == 0) { c->controller_invert_x = 1; continue; }
    if (strcmp(key, "--controller-invert-y") == 0) { c->controller_invert_y = 1; continue; }
    if (strcmp(key, "--feedback-verbose") == 0) { c->feedback_verbose = 1; continue; }
    if (strcmp(key, "--rx-debug") == 0) { c->rx_debug = 1; continue; }
    if (strcmp(key, "--dry-run") == 0) { c->dry_run = 1; continue; }
    if (strcmp(key, "--verbose") == 0) { c->verbose = 1; continue; }
    if (strcmp(key, "--stop-on-close") == 0) { c->stop_on_close = 1; continue; }
    if (strcmp(key, "--no-stop-on-close") == 0) { c->stop_on_close = 0; continue; }

    if (!take_value(argc, argv, &index, value)) continue;
    if (strcmp(key, "--port") == 0) copy_str(c->port, sizeof(c->port), value);
    else if (strcmp(key, "--baud") == 0) parse_int_value(value, &c->baud);
    else if (strcmp(key, "--protocol") == 0) {
      if (strcmp(value, "2.0") != 0 && strcmp(value, "2.1") != 0 &&
          strcmp(value, "2.3") != 0 && strcmp(value, "gear_horn") != 0) {
        snprintf(err, err_size,
          "protocol must be 2.0, 2.1, 2.3, or gear_horn");
        return -1;
      }
      set_protocol(c, value);
    }
    else if (strcmp(key, "--cmd-topic") == 0) copy_str(c->cmd_topic, sizeof(c->cmd_topic), value);
    else if (strcmp(key, "--controller-cmd-topic") == 0) copy_str(c->controller_cmd_topic, sizeof(c->controller_cmd_topic), value);
    else if (strcmp(key, "--feedback-topic") == 0) copy_str(c->feedback_topic, sizeof(c->feedback_topic), value);
    else if (strcmp(key, "--wheel-yaw-topic") == 0) copy_str(c->wheel_yaw_topic, sizeof(c->wheel_yaw_topic), value);
    else if (strcmp(key, "--hall-speed-control-topic") == 0) copy_str(c->hall_speed_control_topic, sizeof(c->hall_speed_control_topic), value);
    else if (strcmp(key, "--left-scan-topic") == 0) copy_str(c->left_scan_topic, sizeof(c->left_scan_topic), value);
    else if (strcmp(key, "--right-scan-topic") == 0) copy_str(c->right_scan_topic, sizeof(c->right_scan_topic), value);
    else if (strcmp(key, "--escape-cloud-topic") == 0) copy_str(c->escape_cloud_topic, sizeof(c->escape_cloud_topic), value);
    else if (strcmp(key, "--escape-stop-hold-sec") == 0) parse_double_value(value, &c->escape_stop_hold_sec);
    else if (strcmp(key, "--escape-stop-speed") == 0) parse_double_value(value, &c->escape_stop_speed);
    else if (strcmp(key, "--escape-forward-deadband") == 0) parse_double_value(value, &c->escape_forward_deadband);
    else if (strcmp(key, "--escape-turn-yawrate") == 0) parse_double_value(value, &c->escape_turn_yawrate);
    else if (strcmp(key, "--escape-cloud-timeout") == 0) parse_double_value(value, &c->escape_cloud_timeout);
    else if (strcmp(key, "--escape-half-width") == 0) parse_double_value(value, &c->escape_half_width);
    else if (strcmp(key, "--radar-timeout") == 0) parse_double_value(value, &c->radar_timeout);
    else if (strcmp(key, "--horn-topic") == 0) copy_str(c->horn_topic, sizeof(c->horn_topic), value);
    else if (strcmp(key, "--horn-timeout") == 0) parse_double_value(value, &c->horn_timeout);
    else if (strcmp(key, "--fallback-gear") == 0) parse_int_value(value, &c->fallback_gear);
    else if (strcmp(key, "--gear-peak-speeds") == 0) {
      char s[256];
      char *pos;
      int total_count = 0;
      c->gear_peak_speeds_count = 0;
      copy_str(s, sizeof(s), value);
      pos = s;
      for (;;) {
        char *comma = strchr(pos, ',');
        if (comma != NULL) {
          *comma = '\0';
          total_count += 1;
          if (c->gear_peak_speeds_count < MB_GEAR_PEAK_SPEEDS_MAX) {
            c->gear_peak_speeds[c->gear_peak_speeds_count++] = strtod(pos, NULL);
          }
          pos = comma + 1;
        } else {
          if (*pos != '\0') {
            total_count += 1;
            if (c->gear_peak_speeds_count < MB_GEAR_PEAK_SPEEDS_MAX) {
              c->gear_peak_speeds[c->gear_peak_speeds_count++] = strtod(pos, NULL);
            }
          }
          break;
        }
      }
      /* C++ vector 无上限；用全量计数保住 >5 校验语义 */
      if (total_count > MB_GEAR_PEAK_SPEEDS_MAX) {
        c->gear_peak_speeds_count = total_count;
      }
    }
    else if (strcmp(key, "--imu-topic") == 0) copy_str(c->imu_topic, sizeof(c->imu_topic), value);
    else if (strcmp(key, "--gyro-angle-source") == 0) copy_str(c->gyro_angle_source, sizeof(c->gyro_angle_source), value);
    else if (strcmp(key, "--max-forward-speed") == 0) parse_double_value(value, &c->max_forward_speed);
    else if (strcmp(key, "--max-reverse-speed") == 0) parse_double_value(value, &c->max_reverse_speed);
    else if (strcmp(key, "--max-turn-speed") == 0) parse_double_value(value, &c->max_turn_speed);
    else if (strcmp(key, "--forward-output-deadzone-y") == 0) parse_int_value(value, &c->forward_output_deadzone_y);
    else if (strcmp(key, "--linear-deadband") == 0) parse_double_value(value, &c->linear_deadband);
    else if (strcmp(key, "--angular-deadband") == 0) parse_double_value(value, &c->angular_deadband);
    else if (strcmp(key, "--cmd-timeout") == 0) parse_double_value(value, &c->cmd_timeout);
    else if (strcmp(key, "--interval-ms") == 0) parse_int_value(value, &c->interval_ms);
    else if (strcmp(key, "--output-mode") == 0) copy_str(c->output_mode, sizeof(c->output_mode), value);
    else if (strcmp(key, "--intervention-linear-delta") == 0) parse_double_value(value, &c->intervention_linear_delta);
    else if (strcmp(key, "--intervention-angular-delta") == 0) parse_double_value(value, &c->intervention_angular_delta);
    else if (strcmp(key, "--intervention-release-stop-repeats") == 0) parse_int_value(value, &c->intervention_release_stop_repeats);
    else if (strcmp(key, "--safe-stop-hold-sec") == 0) parse_double_value(value, &c->safe_stop_hold_sec);
    else if (strcmp(key, "--safe-stop-wheel-linear-threshold") == 0) parse_double_value(value, &c->safe_stop_wheel_linear_threshold);
    else if (strcmp(key, "--safe-stop-wheel-angular-threshold") == 0) parse_double_value(value, &c->safe_stop_wheel_angular_threshold);
    else if (strcmp(key, "--intervention-linear-slew-rate") == 0) parse_double_value(value, &c->intervention_linear_slew_rate);
    else if (strcmp(key, "--intervention-angular-slew-rate") == 0) parse_double_value(value, &c->intervention_angular_slew_rate);
    else if (strcmp(key, "--wheel-track-m") == 0) parse_double_value(value, &c->wheel_track_m);
    else if (strcmp(key, "--wheel-feedback-timeout") == 0) parse_double_value(value, &c->wheel_feedback_timeout);
    else if (strcmp(key, "--wheel-feedback-alpha") == 0) parse_double_value(value, &c->wheel_feedback_alpha);
    else if (strcmp(key, "--hall-speed-control-timeout") == 0) parse_double_value(value, &c->hall_speed_control_timeout);
    else if (strcmp(key, "--hall-speed-y-gain") == 0) parse_double_value(value, &c->hall_speed_y_gain);
    else if (strcmp(key, "--hall-speed-y-down-rate") == 0) parse_double_value(value, &c->hall_speed_y_down_rate);
    else if (strcmp(key, "--hall-speed-y-up-rate") == 0) parse_double_value(value, &c->hall_speed_y_up_rate);
    else if (strcmp(key, "--hall-moving-hold-y-up-rate") == 0) parse_double_value(value, &c->hall_moving_hold_y_up_rate);
    else if (strcmp(key, "--hall-moving-hold-max-y") == 0) parse_double_value(value, &c->hall_moving_hold_max_y);
    else if (strcmp(key, "--hall-moving-hold-emergency-speed") == 0) parse_double_value(value, &c->hall_moving_hold_emergency_speed);
    else if (strcmp(key, "--hall-moving-hold-release-margin") == 0) parse_double_value(value, &c->hall_moving_hold_release_margin);
    else if (strcmp(key, "--hall-speed-deadband") == 0) parse_double_value(value, &c->hall_speed_deadband);
    else if (strcmp(key, "--hall-speed-decel-predict-sec") == 0) parse_double_value(value, &c->hall_speed_decel_predict_sec);
    else if (strcmp(key, "--linear-profile-comfort-up-rate") == 0) parse_double_value(value, &c->linear_profile_comfort_up_rate);
    else if (strcmp(key, "--linear-profile-comfort-down-rate") == 0) parse_double_value(value, &c->linear_profile_comfort_down_rate);
    else if (strcmp(key, "--linear-profile-protective-up-rate") == 0) parse_double_value(value, &c->linear_profile_protective_up_rate);
    else if (strcmp(key, "--linear-profile-protective-down-rate") == 0) parse_double_value(value, &c->linear_profile_protective_down_rate);
    else if (strcmp(key, "--wheel-yaw-max-dt") == 0) parse_double_value(value, &c->wheel_yaw_max_dt);
    else if (strcmp(key, "--closed-loop-mode") == 0) copy_str(c->closed_loop_mode, sizeof(c->closed_loop_mode), value);
    else if (strcmp(key, "--closed-loop-angular-kp") == 0) parse_double_value(value, &c->closed_loop_angular_kp);
    else if (strcmp(key, "--closed-loop-angular-ki") == 0) parse_double_value(value, &c->closed_loop_angular_ki);
    else if (strcmp(key, "--closed-loop-linear-kp") == 0) parse_double_value(value, &c->closed_loop_linear_kp);
    else if (strcmp(key, "--closed-loop-max-angular-correction") == 0) parse_double_value(value, &c->closed_loop_max_angular_correction);
    else if (strcmp(key, "--closed-loop-max-linear-correction") == 0) parse_double_value(value, &c->closed_loop_max_linear_correction);
    else if (strcmp(key, "--pre-stop-repeats") == 0) parse_int_value(value, &c->pre_stop_repeats);
    else if (strcmp(key, "--pre-stop-settle") == 0) parse_double_value(value, &c->pre_stop_settle);
    else if (strcmp(key, "--activate-stop-repeats") == 0) parse_int_value(value, &c->activate_stop_repeats);
    else if (strcmp(key, "--activate-stop-settle") == 0) parse_double_value(value, &c->activate_stop_settle);
    else if (strcmp(key, "--stop-repeats") == 0) parse_int_value(value, &c->stop_repeats);
    else if (strcmp(key, "--idle-stop-mode") == 0) copy_str(c->idle_stop_mode, sizeof(c->idle_stop_mode), value);
    else if (strcmp(key, "--controller-center-x") == 0) parse_int_value(value, &c->controller_center_x);
    else if (strcmp(key, "--controller-center-y") == 0) parse_int_value(value, &c->controller_center_y);
    else if (strcmp(key, "--controller-dead-low") == 0) parse_int_value(value, &c->controller_dead_low);
    else if (strcmp(key, "--controller-dead-high") == 0) parse_int_value(value, &c->controller_dead_high);
    else if (strcmp(key, "--output-turn-dead-low") == 0) parse_int_value(value, &c->output_turn_dead_low);
    else if (strcmp(key, "--output-turn-dead-high") == 0) parse_int_value(value, &c->output_turn_dead_high);
  }

  /* —— 数值校验（对齐 Python parse_args 的 parser.error 校验）—— */
  if (c->interval_ms <= 0) { snprintf(err, err_size, "--interval-ms must be positive"); return -1; }
  if (c->radar_timeout <= 0) { snprintf(err, err_size, "--radar-timeout must be positive"); return -1; }
  if (c->escape_stop_hold_sec < 0.0) { snprintf(err, err_size, "--escape-stop-hold-sec must be >= 0"); return -1; }
  if (c->escape_stop_speed < 0.0) { snprintf(err, err_size, "--escape-stop-speed must be >= 0"); return -1; }
  if (c->escape_forward_deadband < 0.0) { snprintf(err, err_size, "--escape-forward-deadband must be >= 0"); return -1; }
  if (c->escape_turn_yawrate < 0.0) { snprintf(err, err_size, "--escape-turn-yawrate must be >= 0"); return -1; }
  if (c->escape_cloud_timeout <= 0.0) { snprintf(err, err_size, "--escape-cloud-timeout must be positive"); return -1; }
  if (c->escape_half_width < 0.0) { snprintf(err, err_size, "--escape-half-width must be >= 0"); return -1; }
  if (c->horn_timeout <= 0) { snprintf(err, err_size, "--horn-timeout must be positive"); return -1; }
  if (!mp_valid_gear(c->fallback_gear)) { snprintf(err, err_size, "--fallback-gear must be in 1..5"); return -1; }
  /* 先查条数（避免越界），再查数值（对齐 C++ 校验消息顺序：正性检查在前、
   * 条数检查在后；两者均为致命错误，先后不影响可观察行为） */
  if (c->gear_peak_speeds_count > 5) { snprintf(err, err_size, "--gear-peak-speeds supports at most 5 entries"); return -1; }
  {
    int i;
    for (i = 0; i < c->gear_peak_speeds_count; ++i) {
      if (!(c->gear_peak_speeds[i] > 0.0)) {
        snprintf(err, err_size, "--gear-peak-speeds entries must be positive");
        return -1;
      }
    }
  }
  if (c->cmd_timeout <= 0) { snprintf(err, err_size, "--cmd-timeout must be positive"); return -1; }
  if (c->wheel_track_m <= 0) { snprintf(err, err_size, "--wheel-track-m must be positive"); return -1; }
  if (c->wheel_feedback_timeout <= 0) { snprintf(err, err_size, "--wheel-feedback-timeout must be positive"); return -1; }
  if (c->hall_speed_control_timeout <= 0) { snprintf(err, err_size, "--hall-speed-control-timeout must be positive"); return -1; }
  if (c->hall_speed_y_gain <= 0 || c->hall_speed_y_down_rate <= 0 || c->hall_speed_y_up_rate <= 0) {
    snprintf(err, err_size, "--hall-speed-y gain/down-rate/up-rate must be positive");
    return -1;
  }
  if (c->hall_moving_hold_y_up_rate <= 0) { snprintf(err, err_size, "--hall-moving-hold-y-up-rate must be positive"); return -1; }
  if (!(c->forward_output_deadzone_y < c->hall_moving_hold_max_y && c->hall_moving_hold_max_y <= 200)) {
    snprintf(err, err_size, "--hall-moving-hold-max-y must be above the forward deadzone and <= 200");
    return -1;
  }
  if (c->hall_moving_hold_emergency_speed <= 0) { snprintf(err, err_size, "--hall-moving-hold-emergency-speed must be positive"); return -1; }
  if (c->hall_moving_hold_release_margin < 0) { snprintf(err, err_size, "--hall-moving-hold-release-margin must be >= 0"); return -1; }
  if (c->hall_speed_deadband < 0) { snprintf(err, err_size, "--hall-speed-deadband must be >= 0"); return -1; }
  if (c->hall_speed_decel_predict_sec < 0) { snprintf(err, err_size, "--hall-speed-decel-predict-sec must be >= 0"); return -1; }
  if (c->linear_profile_comfort_up_rate <= 0 || c->linear_profile_comfort_down_rate <= 0 ||
      c->linear_profile_protective_up_rate <= 0 || c->linear_profile_protective_down_rate <= 0) {
    snprintf(err, err_size, "--linear-profile rates must be positive");
    return -1;
  }
  if (c->wheel_yaw_max_dt <= 0) { snprintf(err, err_size, "--wheel-yaw-max-dt must be positive"); return -1; }
  if (!(0.0 <= c->wheel_feedback_alpha && c->wheel_feedback_alpha <= 1.0)) {
    snprintf(err, err_size, "--wheel-feedback-alpha must be in 0..1");
    return -1;
  }
  if (strcmp(c->output_mode, "intervention-only") == 0 && c->controller_cmd_topic[0] == '\0') {
    snprintf(err, err_size, "--output-mode intervention-only requires --controller-cmd-topic");
    return -1;
  }
  if (!(0 <= c->controller_center_x && c->controller_center_x <= 200 &&
        0 <= c->controller_center_y && c->controller_center_y <= 200)) {
    snprintf(err, err_size, "--controller-center-x/y must be in 0..200");
    return -1;
  }
  if (!(0 <= c->controller_dead_low && c->controller_dead_low <= c->controller_dead_high &&
        c->controller_dead_high <= 200)) {
    snprintf(err, err_size, "--controller-dead-low/high must satisfy 0 <= low <= high <= 200");
    return -1;
  }
  if (!(0 <= c->output_turn_dead_low && c->output_turn_dead_low <= MP_CENTER &&
        MP_CENTER <= c->output_turn_dead_high && c->output_turn_dead_high <= 200)) {
    snprintf(err, err_size, "--output-turn-dead-low/high must satisfy 0 <= low <= 100 <= high <= 200");
    return -1;
  }
  return 0;
}
