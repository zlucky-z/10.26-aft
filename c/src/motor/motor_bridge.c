/*
 * 模块功能：motor_bridge.h 对应实现（motor_xy_cmd_vel_node.cpp 直译）。
 *
 * 直译注意（与 C++ 原版逐一对应）：
 *   - steady_seconds() → mb_now()（默认进程起算单调钟，可注入）
 *   - writer_.write() → mb_writer_write()（串口 + frame_sink 观测钩）
 *   - std::this_thread::sleep_for → mb_sleep_ms()（可注入 no-op 供对拍）
 *   - 状态字符串 → char[MB_STATE_MAX] + strcmp/拼接直译
 *   - sync_hall_speed_controller_to_output 在原版中定义但无调用点
 *     （死代码），此处直译保留、同样不接线。
 */
#include "motor_bridge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../hal/hal_log.h"
#include "../hal/hal_time.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

/* M_PI 值（math.h 在严格 ISO C 下可能不暴露 M_PI，自给同值字面量） */
#define MB_PI_D 3.14159265358979323846
#define MB_IMU_TIMEOUT_SEC 1.0

static int mb_sensor_fault_mask(mb_bridge_t *b, double now)
{
  int mask = 0;
  if (b->left_scan_time <= 0.0 ||
      now - b->left_scan_time > b->config.radar_timeout) {
    mask |= MP_SENSOR_STATUS_LEFT_RADAR_FAULT;
  }
  if (b->right_scan_time <= 0.0 ||
      now - b->right_scan_time > b->config.radar_timeout) {
    mask |= MP_SENSOR_STATUS_RIGHT_RADAR_FAULT;
  }
  if (!b->valid_imu_seen || b->last_valid_imu_time <= 0.0 ||
      now - b->last_valid_imu_time > MB_IMU_TIMEOUT_SEC) {
    mask |= MP_SENSOR_STATUS_IMU_FAULT;
  }
  return mask;
}

/* —— 小工具 —— */

static void state_set(char dst[MB_STATE_MAX], const char *src)
{
  size_t n = strlen(src);
  if (n >= MB_STATE_MAX) n = MB_STATE_MAX - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

/* s.size() >= 5 && s.substr(s.size()-5) == "_stop" */
static int state_ends_with_stop(const char *s)
{
  size_t n = strlen(s);
  return n >= 5 && strcmp(s + n - 5, "_stop") == 0;
}

static void state_append_stop(char s[MB_STATE_MAX])
{
  if (!state_ends_with_stop(s)) {
    strncat(s, "_stop", MB_STATE_MAX - strlen(s) - 1);
  }
}

double mb_now(mb_bridge_t *b)
{
  if (b->inject.now_fn != NULL) return b->inject.now_fn(b->inject.now_fn_user);
  {
    static double start = -1.0;
    const double now = hal_steady_now_sec();
    if (start < 0.0) {
      start = now;
      return 0.0;
    }
    return now - start;
  }
}

static void mb_sleep_ms(mb_bridge_t *b, double milliseconds)
{
  if (b->inject.sleep_fn != NULL) {
    b->inject.sleep_fn(b->inject.sleep_fn_user, milliseconds);
    return;
  }
  if (milliseconds <= 0.0) return;
#ifdef _WIN32
  Sleep((DWORD)(milliseconds + 0.5));
#else
  {
    struct timespec ts;
    ts.tv_sec = (time_t)(milliseconds / 1000.0);
    ts.tv_nsec = (long)((milliseconds - (double)ts.tv_sec * 1000.0) * 1e6);
    nanosleep(&ts, NULL);
  }
#endif
}

static int mb_controller_feedback_is_online(mb_bridge_t *b, double now)
{
  const double age = now - b->controller_feedback_last_valid_time;
  const int online = age < 1.0;
  if (online != b->controller_feedback_online) {
    b->controller_feedback_online = online;
    hal_log(online ? HAL_LOG_INFO : HAL_LOG_WARN,
      "controller_feedback_link %s age=%.3fs valid_frames_seen=%d; serial TX %s",
      online ? "ONLINE" : "OFFLINE", age, b->controller_feedback_seen,
      online ? "resumed" : "paused");
  }
  return online;
}

/* writer_.write() 直译 + 观测钩子；所有发帧路径统一受反馈在线状态门控。 */
static long mb_writer_write(mb_bridge_t *b, const mp_frame_t *frame)
{
  int write_errno = 0;
  const double now = mb_now(b);
  long written;
  if (!mb_controller_feedback_is_online(b, now)) {
    ++b->tx_attempt_count;
    b->tx_last_result = -3; /* 反馈离线，软件主动抑制发送 */
    b->tx_last_errno = 0;
    b->tx_last_time = now;
    return -3;
  }
  written = ms_serial_writer_write(&b->writer, frame, &write_errno);
  ++b->tx_attempt_count;
  b->tx_last_result = written;
  b->tx_last_errno = write_errno;
  b->tx_last_time = now;
  if (written == (long)frame->len) {
    ++b->tx_full_count;
  } else if (written != -2) {
    ++b->tx_fail_count;
    if (now - b->last_tx_error_log_time >= 1.0 ||
        b->last_tx_error_log_time == 0.0) {
      char hex[MP_FRAME_HEX_MAX];
      mp_frame_hex(frame, hex);
      hal_log(HAL_LOG_ERROR,
        "motor_tx_error bytes=%ld expected=%zu errno=%d attempts=%llu failures=%llu frame=%s",
        written, frame->len, write_errno,
        (unsigned long long)b->tx_attempt_count,
        (unsigned long long)b->tx_fail_count, hex);
      b->last_tx_error_log_time = now;
    }
  }
  if (b->inject.frame_sink != NULL) {
    b->inject.frame_sink(b->inject.frame_sink_user, frame);
  }
  return written;
}

/* read_float：大端 FLOAT32/FLOAT64 → double（__builtin_bswap 语义直译） */
static int mb_read_float(const uint8_t *p, int size, int big_endian,
  double *out)
{
  if (size == 4) {
    uint32_t u;
    float f;
    if (big_endian) {
      u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
          ((uint32_t)p[2] << 8) | (uint32_t)p[3];
    } else {
      memcpy(&u, p, 4);
    }
    memcpy(&f, &u, 4);
    *out = (double)f;
    return 1;
  }
  if (size == 8) {
    uint64_t u;
    double d;
    if (big_endian) {
      u = ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
          ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
          ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
          ((uint64_t)p[6] << 8) | (uint64_t)p[7];
    } else {
      memcpy(&u, p, 8);
    }
    memcpy(&d, &u, 8);
    *out = d;
    return 1;
  }
  return 0;
}

/* 逃逸点云暂存（C++ vector<pair> 语义；单线程假设同 ROS 单执行器） */
static se_point_t *g_escape_points = NULL;
static size_t g_escape_points_cap = 0;

static void mb_reset_escape_yaw_control(mb_bridge_t *b);

static se_point_t *escape_points_reserve(size_t count)
{
  if (count > g_escape_points_cap) {
    se_point_t *p = (se_point_t *)realloc(g_escape_points,
      count * sizeof(se_point_t));
    if (p == NULL) return NULL;
    g_escape_points = p;
    g_escape_points_cap = count;
  }
  return g_escape_points;
}

/* —— 构造 / 析构 —— */

int mb_bridge_init(mb_bridge_t *b, const mb_config_t *config_in,
  char *err, size_t err_size)
{
  char serial_err[256];
  const mb_injection_t inject = b->inject; /* caller 预设注入（须先零初始化） */
  /*
   * 别名防护（P9 板端实测踩坑）：config_in 允许指向 b 内部（如
   * &b->config）。memset 会先清零整个 b，若直接引用 config_in，之后
   * 所有 config->xxx 读取（含 ms_serial_writer_open 的 port）拿到的
   * 都是零值——port 变成空串，open("") 返回 ENOENT，表现为"串口打不开
   * 但配置打印正确"。先做本地快照，再统一从快照读取。
   */
  const mb_config_t config_saved = *config_in;
  const mb_config_t *config = &config_saved;
  memset(b, 0, sizeof(*b));
  b->inject = inject;
  b->config = config_saved;
  err[0] = '\0';

  /* 成员初始化列表顺序：writer_ 最先（串口打开失败 → throw） */
  if (ms_serial_writer_open(&b->writer, config->port, config->baud,
        config->dry_run, serial_err, sizeof(serial_err)) != 0) {
    snprintf(err, err_size, "%s", serial_err);
    return -1;
  }
  b->controller_feedback_last_valid_time = mb_now(b);
  b->controller_feedback_online = 1; /* 首秒宽限，避免初始化阶段误判离线 */
  b->sensor_fault_last_mask = -1;
  se_stop_escape_compensator_init(&b->stop_escape,
    config->escape_stop_hold_sec, config->escape_stop_speed,
    config->escape_turn_yawrate, config->escape_cloud_timeout,
    config->escape_half_width);
  b->stop_escape.max_duration_sec = config->escape_max_duration_sec;
  mc_hall_speed_controller_init(&b->hall_speed_controller,
    config->hall_speed_y_gain, config->hall_speed_y_down_rate,
    config->hall_speed_y_up_rate, config->hall_speed_deadband,
    (double)(MP_CENTER + 1));
  mc_moving_hold_controller_init(&b->moving_hold_controller,
    config->hall_speed_y_gain, config->hall_speed_y_down_rate,
    config->hall_moving_hold_y_up_rate, config->hall_speed_deadband,
    (double)(MP_CENTER + 1), config->hall_speed_decel_predict_sec,
    config->hall_moving_hold_max_y, config->hall_moving_hold_emergency_speed,
    config->hall_moving_hold_release_margin);
  mc_prebrake_stop_controller_init(&b->prebrake_stop_controller,
    config->hall_speed_decel_predict_sec,
    (double)config->forward_output_deadzone_y,
    config->hall_moving_hold_release_margin);
  mc_linear_output_profile_init(&b->linear_output_profile,
    (double)(MP_CENTER + 1));

  /* 类内默认成员初始化（未在 ctor 体赋值的字段） */
  b->linear_output_profile_kind[0] = '\0';
  state_set(b->linear_output_profile_kind, "idle");
  b->forward_speed_reference = 0.50;
  b->last_frame.len = 0;
  b->last_written_frame.len = 0;
  b->was_motion = 0;
  b->intervention_motion_active = 0;
  b->safe_stop_hold_until = 0.0;
  b->wheel_filter_initialized = 0;
  b->wheel_left_mps = 0.0;
  b->wheel_right_mps = 0.0;
  b->actual_linear = 0.0;
  b->actual_yaw_rate = 0.0;
  b->raw_yaw_rate = 0.0;
  b->wheel_yaw_total = 0.0;
  b->wheel_yaw_accepted = 0;
  b->wheel_yaw_rejected = 0;
  b->closed_loop_angular_i = 0.0;
  b->closed_loop_last_time = 0.0;
  b->closed_loop_has_info = 0;
  b->intervention_slew_initialized = 0;
  b->last_controller_x = MP_CENTER;
  b->last_controller_y = MP_CENTER;
  b->controller_neutral_seen = 1;
  b->hall_speed_control_active = 0;
  b->hall_speed_control_target = 0.0;
  b->hall_speed_brake_target = 0.40;
  b->hall_speed_dynamic_down_rate = 60.0;
  b->hall_speed_control_mode = 0;
  b->last_hall_speed_control_time = 0.0;
  b->hard_stop_active = 0;
  b->last_hard_stop_time = 0.0;
  b->hall_speed_fault_wait_neutral = 0;
  b->hall_speed_release_active = 0;
  b->left_scan_time = 0.0;
  b->right_scan_time = 0.0;
  b->imu_time = 0.0;
  b->gyro_angle_byte = 0;
  b->controller_gear = 1;
  b->controller_gear_valid = 0;
  b->output_gear = 1;
  b->output_gear_valid = 0;
  b->output_gear_closed_loop_active = 0;
  b->last_output_gear_update_time = 0.0;
  b->horn_requested = 0;
  b->last_horn_time = 0.0;
  b->last_escape_cloud_warning_time = 0.0;
  b->last_raw_frame_log_time = 0.0;
  b->stop_frame.len = 0;
  b->stop_escape_last_decision.active = 0;
  b->stop_escape_last_decision.command_enabled = 0;
  b->stop_escape_last_decision.yawrate = 0.0;
  b->stop_escape_last_decision.direction = SE_DIRECTION_NONE;
  b->stop_escape_last_decision.left_clearance = INFINITY;
  b->stop_escape_last_decision.right_clearance = INFINITY;
  b->stop_escape_last_decision.reason = "";
  b->stop_escape_event_count = 0;
  b->stop_escape_event_active = 0;
  b->dwa_escape_request = 0;
  b->dwa_escape_request_time = 0.0;
  b->escape_imu_yaw_rate = 0.0;
  b->escape_imu_time = 0.0;
  b->escape_imu_valid = 0;
  b->escape_yaw_integral = 0.0;
  b->escape_yaw_output = 0.0;
  b->escape_yaw_direction = SE_DIRECTION_NONE;
  b->escape_yaw_diag_target = 0.0;
  b->escape_yaw_diag_actual = 0.0;
  b->escape_yaw_diag_output = 0.0;
  b->escape_yaw_last_update_time = 0.0;
  b->escape_last_diag_log_time = 0.0;

  /* 发布器开关（topic 非空 = publisher 存在） */
  b->controller_cmd_pub = (config->controller_cmd_topic[0] != '\0');
  b->feedback_pub = (config->feedback_topic[0] != '\0');
  b->wheel_yaw_pub = (config->wheel_yaw_topic[0] != '\0');

  /* ctor 体 */
  b->forward_speed_reference = config->max_forward_speed;
  b->controller_neutral_seen = !config->controller_require_neutral;
  b->controller_gear = config->fallback_gear;
  b->output_gear = config->fallback_gear;
  b->stop_frame = mb_build_output_frame(b, MP_CENTER, MP_CENTER);

  hal_log(HAL_LOG_INFO,
    "motor XY bridge started: mode=%s port=%s baud=%d cmd_topic=%s timeout=%.2fs protocol=%s",
    config->dry_run ? "dry-run" : "serial", config->port, config->baud,
    config->cmd_topic, config->cmd_timeout, config->protocol_name);
  {
    char hex[MP_FRAME_HEX_MAX];
    mp_frame_hex(&b->stop_frame, hex);
    hal_log(HAL_LOG_INFO, "stop frame: %s", hex);
  }
  if (strcmp(config->output_mode, "continuous") == 0) {
    mb_send_stop_burst(b, "startup", -1, -1.0);
  }
  return 0;
}

void mb_bridge_destroy(mb_bridge_t *b)
{
  mb_stop_and_close(b);
}

/* —— 输入注入（回调直译） —— */

void mb_on_cmd(mb_bridge_t *b, double linear_x, double angular_z)
{
  b->last_cmd_time = mb_now(b);
  b->last_linear = linear_x;
  b->last_angular = angular_z;
}

void mb_on_hall_speed_control(mb_bridge_t *b, const float *data,
  size_t data_size)
{
  int active;
  double target;
  int mode;
  double brake_target;
  double dynamic_down_rate;
  int effective_gear = -1;
  int gear_closed_loop_active;
  int hard_stop;
  double now;

  if (data_size < 3) {
    hal_log(HAL_LOG_WARN, "ignore malformed hall speed control message");
    return;
  }
  active = (data[0] >= 0.5f);
  target = (double)data[1];
  mode = (int)mp_py_round((double)data[2]);
  brake_target = data_size >= 4 ? (double)data[3] : b->hall_speed_brake_target;
  dynamic_down_rate = data_size >= 4 + 1 ? (double)data[4]
    : b->config.linear_profile_protective_down_rate;
  if (data_size >= 6 && isfinite((double)data[5])) {
    const int candidate = (int)mp_py_round((double)data[5]);
    if (mp_valid_gear(candidate)) effective_gear = candidate;
  }
  gear_closed_loop_active =
    (data_size >= 7 && data[6] >= 0.5f);
  hard_stop = (data_size >= 8 && data[7] >= 0.5f);
  if (!isfinite(target) || target < 0.0) {
    hal_log(HAL_LOG_WARN, "ignore invalid hall speed target");
    return;
  }
  if (!isfinite(brake_target) || brake_target <= 0.0) {
    hal_log(HAL_LOG_WARN, "ignore invalid Hall prebrake target");
    return;
  }
  if (!isfinite(dynamic_down_rate) || dynamic_down_rate <= 0.0) {
    hal_log(HAL_LOG_WARN, "ignore invalid Hall dynamic down rate");
    return;
  }
  now = mb_now(b);
  if (hard_stop) {
    b->hard_stop_active = 0;
    b->last_hard_stop_time = now;
    b->hall_speed_control_active = 0;
    b->hall_speed_control_target = 0.0;
    b->hall_speed_control_mode = mode;
    b->hall_speed_release_active = 0;
    mc_hall_speed_controller_reset(&b->hall_speed_controller);
    mc_moving_hold_controller_reset(&b->moving_hold_controller);
    mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    mc_linear_output_profile_reset(&b->linear_output_profile);
    mb_reset_closed_loop_state(b);
    mb_reset_intervention_slew_state(b);
    b->last_hall_speed_control_time = now;
    return;
  }
  if (b->hard_stop_active) {
    b->hard_stop_active = 0;
  }
  {
    const int previous_mode = b->hall_speed_control_mode;
    if (b->hall_speed_control_active && !active) {
      /* Keep the single Hall output state when leaving prebrake/DWA.  The
       * release path must continue from the current Y instead of resetting
       * to CENTER and creating a second low-speed dip. */
      b->hall_speed_release_active = (b->last_raw_linear > 0.0);
      mc_moving_hold_controller_reset(&b->moving_hold_controller);
      mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    } else if (active && !b->hall_speed_control_active) {
      b->hall_speed_release_active = 0;
      /* Do not reset HallSpeedController here: entering a protected phase is
       * a target change, not a new motion session.  This preserves
       * continuity. */
      mc_moving_hold_controller_reset(&b->moving_hold_controller);
      mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    } else if (mode == MP_HALL_MODE_MOVING_PREBRAKE && previous_mode != mode) {
      mc_moving_hold_controller_reset(&b->moving_hold_controller);
      mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    } else if (mode == MP_HALL_MODE_MOVING_HOLD && previous_mode != mode) {
      mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    } else if ((previous_mode == MP_HALL_MODE_MOVING_PREBRAKE ||
                previous_mode == MP_HALL_MODE_MOVING_HOLD) &&
               (mode != MP_HALL_MODE_MOVING_PREBRAKE &&
                mode != MP_HALL_MODE_MOVING_HOLD)) {
      b->hall_speed_release_active = 0;
      mc_moving_hold_controller_reset(&b->moving_hold_controller);
      mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
    }
  }
  b->hall_speed_control_active = active;
  /* 9.27 用户指令"放开所有 DWA 速度限制"：删除 MOVING_DWA 的正向线速度
   * 地板（原 target = max(target, brake_target=0.6944)）。该地板把规划器
   * 选中的慢速避障候选（实测近障选 0.10~0.54）全部托回满速 2.5km/h，
   * 转弯半径 v/w 被强行拉大，近障"转不过来"。现在 DWA 指令 v 原样执行；
   * 零/纯转向指令仍保持零（不把原地转向变成前进）。回滚=恢复下方
   * 注释掉的 floor 块。 */
  /* if (mode == MP_HALL_MODE_MOVING_DWA &&
   *     target > b->config.linear_deadband) {
   *   const double bt = brake_target;
   *   target = (target > bt) ? target : bt;
   * } */
  b->hall_speed_control_target = target;
  b->hall_speed_brake_target = brake_target;
  {
    const double lo = b->config.linear_profile_protective_down_rate;
    const double hi_max =
      b->config.linear_profile_protective_down_rate > b->config.hall_speed_y_down_rate
      ? b->config.linear_profile_protective_down_rate
      : b->config.hall_speed_y_down_rate;
    b->hall_speed_dynamic_down_rate = mp_clamp_double(dynamic_down_rate, lo, hi_max);
  }
  b->hall_speed_control_mode = mode;
  if (effective_gear >= 1) {
    const int gear_changed = (!b->output_gear_valid ||
                              b->output_gear != effective_gear ||
                              b->output_gear_closed_loop_active != gear_closed_loop_active);
    b->output_gear = effective_gear;
    b->output_gear_valid = 1;
    b->output_gear_closed_loop_active = gear_closed_loop_active;
    b->last_output_gear_update_time = now;
    if (gear_changed) {
      hal_log(HAL_LOG_INFO, "motor output gear: raw=%d effective=%d source=%s",
        b->controller_gear, b->output_gear,
        gear_closed_loop_active ? "front_force_1" : "follow_controller");
    }
  }
  b->last_hall_speed_control_time = now;
}

void mb_on_escape_cloud(mb_bridge_t *b, const mb_cloud_view_t *cloud)
{
  const double now = mb_now(b);
  int x_field = -1, y_field = -1;
  int x_datatype = -1, y_datatype = -1;
  int x_offset = 0, y_offset = 0;
  size_t fi;
  int x_size, y_size;
  int point_step, width, height, row_step;
  size_t row, col;
  se_point_t *points;

  for (fi = 0; fi < cloud->field_count; ++fi) {
    if (strcmp(cloud->fields[fi].name, "x") == 0) {
      x_field = 1;
      x_datatype = cloud->fields[fi].datatype;
      x_offset = cloud->fields[fi].offset;
    }
    if (strcmp(cloud->fields[fi].name, "y") == 0) {
      y_field = 1;
      y_datatype = cloud->fields[fi].datatype;
      y_offset = cloud->fields[fi].offset;
    }
  }
  if (x_field < 0 || y_field < 0) {
    if (now - b->last_escape_cloud_warning_time >= 2.0) {
      hal_log(HAL_LOG_WARN, "stop escape ignored fused cloud without x/y fields");
      b->last_escape_cloud_warning_time = now;
    }
    return;
  }
  /* ROS PointField: FLOAT32=7, FLOAT64=8 */
  x_size = x_datatype == 7 ? 4 : (x_datatype == 8 ? 8 : -1);
  y_size = y_datatype == 7 ? 4 : (y_datatype == 8 ? 8 : -1);
  point_step = cloud->point_step;
  width = cloud->width;
  height = cloud->height;
  row_step = (cloud->row_step > 0) ? cloud->row_step : point_step * width;
  if (x_size < 0 || y_size < 0 || point_step <= 0 || width <= 0 || height <= 0 ||
      row_step < point_step * width || x_offset + x_size > point_step ||
      y_offset + y_size > point_step) {
    if (now - b->last_escape_cloud_warning_time >= 2.0) {
      hal_log(HAL_LOG_WARN, "stop escape ignored malformed fused cloud layout");
      b->last_escape_cloud_warning_time = now;
    }
    return;
  }
  points = escape_points_reserve((size_t)width * (size_t)height);
  if (points == NULL) return;
  {
    size_t count = 0;
    const uint8_t *data = cloud->data;
    const size_t data_len = cloud->data_len;
    for (row = 0; row < (size_t)height; ++row) {
      const size_t row_base = row * (size_t)row_step;
      for (col = 0; col < (size_t)width; ++col) {
        const size_t base = row_base + col * (size_t)point_step;
        double x = 0.0, y = 0.0;
        if (base + (size_t)point_step > data_len) break;
        if (!mb_read_float(data + base + x_offset, x_size, cloud->big_endian, &x)) continue;
        if (!mb_read_float(data + base + y_offset, y_size, cloud->big_endian, &y)) continue;
        points[count].x = x;
        points[count].y = y;
        count += 1;
      }
    }
    se_stop_escape_compensator_update_cloud(&b->stop_escape, points, count, now);
  }
}

void mb_on_escape_request(mb_bridge_t *b, int requested)
{
  b->dwa_escape_request = requested ? 1 : 0;
  b->dwa_escape_request_time = requested ? mb_now(b) : 0.0;
}

void mb_on_escape_imu_yawrate(mb_bridge_t *b, double yawrate, int valid)
{
  const double now = mb_now(b);
  b->escape_imu_yaw_rate = isfinite(yawrate) ? yawrate : 0.0;
  b->escape_imu_valid = valid && isfinite(yawrate) ? 1 : 0;
  b->escape_imu_time = now;
  if (b->escape_imu_valid) {
    b->last_valid_imu_time = now;
    b->valid_imu_seen = 1;
  }
}

void mb_bridge_enable_sensor_interlock(mb_bridge_t *b, int enabled)
{
  b->sensor_interlock_enabled = enabled ? 1 : 0;
  b->sensor_fault_last_mask = -1;
  hal_log(HAL_LOG_WARN, "sensor_interlock enabled=%d imu_timeout=%.2fs "
    "radar_timeout=%.2fs status_bits=(left:0x01,right:0x02,imu:0x04)",
    b->sensor_interlock_enabled, MB_IMU_TIMEOUT_SEC, b->config.radar_timeout);
}

void mb_on_left_scan(mb_bridge_t *b)
{
  b->left_scan_time = mb_now(b);
}

void mb_on_right_scan(mb_bridge_t *b)
{
  b->right_scan_time = mb_now(b);
}

void mb_on_imu(mb_bridge_t *b, double ax, double ay, double az)
{
  b->imu_time = mb_now(b);
  if (strcmp(b->config.gyro_angle_source, "imu_pitch") != 0) {
    b->gyro_angle_byte = 0;
    return;
  }
  {
    const double pitch_deg =
      atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / MB_PI_D;
    b->gyro_angle_byte =
      (int)mp_py_round(mp_clamp_double(pitch_deg, -127.0, 127.0)) & 0xFF;
  }
}

void mb_on_horn(mb_bridge_t *b, int horn)
{
  b->horn_requested = horn;
  b->last_horn_time = mb_now(b);
}

/* —— 辅助判定 —— */

int mb_horn_active(mb_bridge_t *b, double now)
{
  return b->horn_requested && b->last_horn_time > 0.0 &&
    now - b->last_horn_time <= b->config.horn_timeout;
}

int mb_radar_status(mb_bridge_t *b, double now)
{
  int status;
  if (b->config.protocol != MP_PROTOCOL_21 &&
      b->config.protocol != MP_PROTOCOL_GEAR_HORN &&
      b->config.protocol != MP_PROTOCOL_23) {
    return 0;
  }
  status = 0;
  if (b->left_scan_time <= 0.0 ||
      now - b->left_scan_time > b->config.radar_timeout) {
    status |= MP_SENSOR_STATUS_LEFT_RADAR_FAULT;
  }
  if (b->right_scan_time <= 0.0 ||
      now - b->right_scan_time > b->config.radar_timeout) {
    status |= MP_SENSOR_STATUS_RIGHT_RADAR_FAULT;
  }
  return status;
}

int mb_current_output_gear(mb_bridge_t *b)
{
  const double now = mb_now(b);
  if (b->output_gear_valid && b->last_output_gear_update_time > 0.0 &&
      now - b->last_output_gear_update_time <=
        b->config.hall_speed_control_timeout) {
    return b->output_gear;
  }
  if (b->controller_gear_valid) return b->controller_gear;
  return b->config.fallback_gear;
}

double mb_forward_reference_now(mb_bridge_t *b)
{
  if (b->config.gear_peak_speeds_count == 0) return b->forward_speed_reference;
  {
    const int gear = mb_current_output_gear(b);
    if (gear >= 1 && gear <= b->config.gear_peak_speeds_count) {
      const double v = b->config.gear_peak_speeds[gear - 1];
      return v < b->forward_speed_reference ? v : b->forward_speed_reference;
    }
  }
  return b->forward_speed_reference;
}

/* A2(10.09)：Y 油门映射基准——按当前档位峰值缩放，专供
 * mb_motion_to_xy 前进分支。与 mb_forward_reference_now 的
 * min(峰值,0.70) 语义刻意分离：后者继续服务手柄反向映射与
 * ordinary-forward 钳位（intent 链零改动）。
 *
 * 根因（014339/022757 日志定案）：固定基准 0.70 下任何指令
 * ≥0.56m/s 都映射 Y=200 满油门，车固件对 Y=200 的响应是当前
 * 档位峰值——"限速 0.70"在 ≥2 档从未被物理执行，全靠 hall
 * 积分器（40Y/s）事后下拉，期间满油门冲刺=4/5 档撞对侧与
 * fb±0.3 追摆振荡的机制根源。
 *
 * 改后：sel=0.70@5档 → Y≈162（0.70/1.67×80+120），基础 Y
 * 落在平衡点附近，hall 积分器只做小误差修剪；释放态
 * sel=1.67 → Y=200 满油门不受影响（空旷直道仍跑满档速）。
 * 无档位反馈（--gear-peak-speeds 未配置或档位无效）时退回
 * forward_speed_reference，行为与原版逐字节一致。 */
double mb_y_scale_reference_now(mb_bridge_t *b)
{
  if (b->config.gear_peak_speeds_count > 0) {
    const int gear = mb_current_output_gear(b);
    if (gear >= 1 && gear <= b->config.gear_peak_speeds_count) {
      const double v = b->config.gear_peak_speeds[gear - 1];
      if (v > 1e-6) return v;
    }
  }
  return b->forward_speed_reference;
}

mp_frame_t mb_build_output_frame(mb_bridge_t *b, int x, int y)
{
  const double now = mb_now(b);
  int sensor_status = mb_radar_status(b, now);
  if (b->sensor_interlock_enabled &&
      (mb_sensor_fault_mask(b, now) & MP_SENSOR_STATUS_IMU_FAULT)) {
    sensor_status |= MP_SENSOR_STATUS_IMU_FAULT;
  }
  return mp_build_frame(x, y, sensor_status, b->gyro_angle_byte,
    mb_current_output_gear(b), mb_horn_active(b, now), b->config.protocol);
}

void mb_motion_to_xy(mb_bridge_t *b, double linear, double angular,
  int *out_x, int *out_y)
{
  /* A2(10.09)：前进分支改用档位峰值基准（详见
   * mb_y_scale_reference_now 注释）；倒退分支保留固定基准——
   * 手柄反向映射用 max_reverse_speed(0.70) 生成 raw_linear，
   * 两边基准保持一致才不削弱倒车油门。 */
  const double forward_reference =
    b->forward_speed_reference > 1e-6 ? b->forward_speed_reference : 1e-6;
  const double y_ref_now = mb_y_scale_reference_now(b);
  const double y_scale_reference = y_ref_now > 1e-6 ? y_ref_now : 1e-6;
  double y;
  double x;
  double turn_ratio;
  int clamp_x;

  if (linear > 0.0) {
    const double deadzone_y = mp_clamp_double(
      (double)b->config.forward_output_deadzone_y, (double)MP_CENTER, 199.0);
    y = deadzone_y + 100.0 * linear / y_scale_reference;
    y = (y > deadzone_y + 1.0) ? y : deadzone_y + 1.0;
  } else {
    y = (linear < 0.0)
      ? (double)MP_CENTER + 100.0 * linear / forward_reference
      : (double)MP_CENTER;
  }

  b->output_turn_deadzone_has_info = 0;
  {
    const double max_turn =
      b->config.max_turn_speed > 1e-6 ? b->config.max_turn_speed : 1e-6;
    turn_ratio = mp_clamp_double(fabs(angular) / max_turn, 0.0, 1.0);
  }
  if (fabs(angular) >= b->config.angular_deadband) {
    if (angular > 0.0) {
      x = (double)b->config.output_turn_dead_high +
        turn_ratio * (200.0 - (double)b->config.output_turn_dead_high);
    } else {
      x = (double)b->config.output_turn_dead_low -
        turn_ratio * (double)b->config.output_turn_dead_low;
    }
    b->output_turn_deadzone_info.angular = angular;
    b->output_turn_deadzone_info.ratio = turn_ratio;
    b->output_turn_deadzone_info.x = mp_clamp_coord(x);
    b->output_turn_deadzone_has_info = 1;
    clamp_x = mp_clamp_coord(x);
  } else {
    x = (double)MP_CENTER;
    clamp_x = MP_CENTER;
  }
  *out_x = clamp_x;
  *out_y = mp_clamp_coord(y);
}

int mb_hall_speed_control_is_fresh(mb_bridge_t *b, double now)
{
  return b->last_hall_speed_control_time > 0.0 &&
    now - b->last_hall_speed_control_time <=
      b->config.hall_speed_control_timeout;
}

int mb_dwa_command_can_cover_hall_control_gap(mb_bridge_t *b, double now)
{
  return (b->hall_speed_control_mode == MP_HALL_MODE_MOVING_DWA ||
          b->hall_speed_control_mode == MP_HALL_MODE_STATIC_DYNAMIC_DWA) &&
    b->last_cmd_time > 0.0 && now - b->last_cmd_time <= b->config.cmd_timeout;
}

int mb_raw_command_is_recent(mb_bridge_t *b, double now)
{
  return b->last_raw_cmd_time > 0.0 &&
    now - b->last_raw_cmd_time <= b->config.cmd_timeout;
}

int mb_raw_command_is_motion(mb_bridge_t *b)
{
  return fabs(b->last_raw_linear) >= b->config.linear_deadband ||
    fabs(b->last_raw_angular) >= b->config.angular_deadband;
}

int mb_raw_forward_is_active(mb_bridge_t *b, double now)
{
  return mb_raw_command_is_recent(b, now) &&
    b->last_raw_linear > b->config.linear_deadband &&
    b->last_controller_y > b->config.controller_dead_high;
}

void mb_raw_override_xy(mb_bridge_t *b, int *out_x, int *out_y)
{
  mb_motion_to_xy(b, b->last_raw_linear, b->last_raw_angular, out_x, out_y);
}

int mb_raw_forward_cap_y(mb_bridge_t *b)
{
  int x, y;
  mb_raw_override_xy(b, &x, &y);
  return y;
}

void mb_latch_hall_speed_fault(mb_bridge_t *b, const char *reason)
{
  if (!b->hall_speed_fault_wait_neutral) {
    hal_log(HAL_LOG_ERROR,
      "Hall speed control stopped: %s; waiting for controller neutral", reason);
  }
  b->hall_speed_fault_wait_neutral = 1;
  b->hall_speed_release_active = 0;
  mc_hall_speed_controller_reset(&b->hall_speed_controller);
  mc_moving_hold_controller_reset(&b->moving_hold_controller);
  mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
}

int mb_wheel_feedback_is_fresh(mb_bridge_t *b, double now)
{
  return b->last_wheel_feedback_time > 0.0 &&
    now - b->last_wheel_feedback_time <= b->config.wheel_feedback_timeout;
}

int mb_wheel_motion_active(mb_bridge_t *b, double now)
{
  if (!mb_wheel_feedback_is_fresh(b, now)) return 0;
  return fabs(b->actual_linear) >= b->config.safe_stop_wheel_linear_threshold ||
    fabs(b->actual_yaw_rate) >= b->config.safe_stop_wheel_angular_threshold;
}

void mb_reset_closed_loop_state(mb_bridge_t *b)
{
  b->closed_loop_angular_i = 0.0;
  b->closed_loop_last_time = 0.0;
}

void mb_reset_intervention_slew_state(mb_bridge_t *b)
{
  b->intervention_slew_initialized = 0;
  b->intervention_slew_last_time = 0.0;
  b->intervention_slew_has_info = 0;
}

double mb_limit_step(double previous, double target, double max_step)
{
  return previous + mp_clamp_double(target - previous, -max_step, max_step);
}

void mb_update_wheel_feedback(mb_bridge_t *b, const ms_sample_t *sample)
{
  const double alpha = b->config.wheel_feedback_alpha;
  const double left_mps = sample->left_mps;
  const double right_mps = sample->right_mps;
  if (!b->wheel_filter_initialized) {
    b->wheel_left_mps = left_mps;
    b->wheel_right_mps = right_mps;
    b->wheel_filter_initialized = 1;
  } else {
    b->wheel_left_mps = alpha * left_mps + (1.0 - alpha) * b->wheel_left_mps;
    b->wheel_right_mps = alpha * right_mps + (1.0 - alpha) * b->wheel_right_mps;
  }
  /* sample->time is system CLOCK_MONOTONIC; bridge timers use process-relative mb_now().
   * Stamp receipt in the bridge clock domain to keep freshness comparisons valid. */
  b->last_wheel_feedback_time = mb_now(b);
  b->actual_linear = 0.5 * (b->wheel_left_mps + b->wheel_right_mps);
  b->actual_yaw_rate = (b->wheel_right_mps - b->wheel_left_mps) /
    b->config.wheel_track_m;
  b->raw_yaw_rate = (right_mps - left_mps) / b->config.wheel_track_m;
}

void mb_update_wheel_yaw_total(mb_bridge_t *b, double now)
{
  if (!mb_wheel_feedback_is_fresh(b, now)) {
    b->wheel_yaw_last_time = 0.0;
    b->wheel_yaw_last_rate = b->raw_yaw_rate;
    return;
  }
  if (b->wheel_yaw_last_time <= 0.0) {
    b->wheel_yaw_last_time = now;
    b->wheel_yaw_last_rate = b->raw_yaw_rate;
  } else {
    const double dt = now - b->wheel_yaw_last_time;
    b->wheel_yaw_last_time = now;
    if (dt > 0.0 && dt <= b->config.wheel_yaw_max_dt) {
      b->wheel_yaw_total +=
        0.5 * (b->wheel_yaw_last_rate + b->raw_yaw_rate) * dt;
      b->wheel_yaw_accepted += 1;
    } else {
      b->wheel_yaw_rejected += 1;
    }
    b->wheel_yaw_last_rate = b->raw_yaw_rate;
  }
  if (b->wheel_yaw_pub && b->inject.wheel_yaw_sink != NULL) {
    b->inject.wheel_yaw_sink(b->inject.wheel_yaw_sink_user, b->wheel_yaw_total);
  }
}

double mb_controller_axis_delta(mb_bridge_t *b, int value, int center,
  int dead_low, int dead_high, int invert)
{
  double delta;
  (void)b; /* C++ 成员函数未用 this（纯函数语义） */
  if (dead_low <= value && value <= dead_high) return 0.0;
  delta = (double)(value - center);
  if (invert) delta = -delta;
  return delta;
}

double mb_controller_turn_ratio(mb_bridge_t *b, int value)
{
  double ratio;
  if (b->config.controller_dead_low <= value &&
      value <= b->config.controller_dead_high) {
    return 0.0;
  }
  if (value < b->config.controller_dead_low) {
    const double low = (double)b->config.controller_dead_low;
    ratio = -(low - (double)value) / (low > 1.0 ? low : 1.0);
  } else {
    const double high = (double)b->config.controller_dead_high;
    const double span = 200.0 - high;
    ratio = ((double)value - high) / (span > 1.0 ? span : 1.0);
  }
  if (b->config.controller_invert_x) ratio = -ratio;
  return mp_clamp_double(ratio, -1.0, 1.0);
}

void mb_controller_status_to_cmd(mb_bridge_t *b, const ms_sample_t *sample,
  double *out_linear_x, double *out_angular_z)
{
  double linear_x = 0.0;
  double angular_z = 0.0;
  const double dy = mb_controller_axis_delta(b, sample->y,
    b->config.controller_center_y, b->config.controller_dead_low,
    b->config.controller_dead_high, b->config.controller_invert_y);
  if (dy > 0.0) {
    const double span = 200.0 - (double)b->config.controller_center_y;
    linear_x = mb_forward_reference_now(b) * dy / (span > 1.0 ? span : 1.0);
  } else if (dy < 0.0) {
    const double center = (double)b->config.controller_center_y;
    linear_x = b->config.max_reverse_speed * dy / (center > 1.0 ? center : 1.0);
  }
  angular_z = b->config.max_turn_speed * mb_controller_turn_ratio(b, sample->x);
  if (b->config.controller_axis_lock) {
    const double fwd_ref = mb_forward_reference_now(b);
    const double denom_base =
      (fwd_ref > b->config.max_reverse_speed ? fwd_ref : b->config.max_reverse_speed);
    const double denom = denom_base > 1e-6 ? denom_base : 1e-6;
    const double linear_ratio = fabs(linear_x) / denom;
    const double angular_denom =
      b->config.max_turn_speed > 1e-6 ? b->config.max_turn_speed : 1e-6;
    const double angular_ratio = fabs(angular_z) / angular_denom;
    if (angular_ratio > linear_ratio) linear_x = 0.0;
    else angular_z = 0.0;
  }
  *out_linear_x = linear_x;
  *out_angular_z = angular_z;
}

int mb_controller_sample_is_neutral(mb_bridge_t *b, const ms_sample_t *sample)
{
  return b->config.controller_dead_low <= sample->x &&
    sample->x <= b->config.controller_dead_high &&
    b->config.controller_dead_low <= sample->y &&
    sample->y <= b->config.controller_dead_high;
}

int mb_physical_controller_is_neutral(mb_bridge_t *b, double now)
{
  return mc_controller_neutral_stop_required(
    b->last_controller_sample_time, now, b->config.cmd_timeout,
    b->last_controller_x, b->last_controller_y,
    b->config.controller_dead_low, b->config.controller_dead_high);
}

void mb_clear_motion_state_for_controller_neutral(mb_bridge_t *b)
{
  b->last_linear = 0.0;
  b->last_angular = 0.0;
  b->last_raw_linear = 0.0;
  b->last_raw_angular = 0.0;
  b->hall_speed_control_active = 0;
  b->hall_speed_control_target = 0.0;
  b->hall_speed_control_mode = 0;
  b->hall_speed_release_active = 0;
  b->hall_speed_fault_wait_neutral = 0;
  b->last_hard_stop_time = 0.0;
  b->output_gear_closed_loop_active = 0;
  b->last_output_gear_update_time = 0.0;
  b->output_gear = b->controller_gear_valid
    ? b->controller_gear : b->config.fallback_gear;
  b->output_gear_valid = b->controller_gear_valid;
  mc_hall_speed_controller_reset(&b->hall_speed_controller);
  mc_moving_hold_controller_reset(&b->moving_hold_controller);
  mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
  mb_reset_closed_loop_state(b);
  mb_reset_intervention_slew_state(b);
  se_stop_escape_compensator_reset(&b->stop_escape);
  b->dwa_escape_request = 0;
  b->dwa_escape_request_time = 0.0;
  mb_reset_escape_yaw_control(b);
  b->stop_escape_last_decision.active = 0;
  b->stop_escape_last_decision.command_enabled = 0;
  b->stop_escape_last_decision.yawrate = 0.0;
  b->stop_escape_last_decision.direction = SE_DIRECTION_NONE;
  b->stop_escape_last_decision.left_clearance = INFINITY;
  b->stop_escape_last_decision.right_clearance = INFINITY;
  b->stop_escape_last_decision.reason = "";
  b->stop_escape_event_active = 0;
}

void mb_publish_sample(mb_bridge_t *b, const ms_sample_t *sample)
{
  const int is_status = (sample->kind == MS_SAMPLE_KIND_STATUS);
  const int is_drive_or_status =
    (sample->kind == MS_SAMPLE_KIND_DRIVE || is_status);

  /* 只有串口解析器校验并产出的有效 drive/status 帧才刷新在线计时；
   * 原始字节、坏 CRC、未知帧不会让控制器被误判为在线。 */
  if (is_drive_or_status) {
    b->controller_feedback_last_valid_time = mb_now(b);
    b->controller_feedback_seen = 1;
    (void)mb_controller_feedback_is_online(b,
      b->controller_feedback_last_valid_time);
  }

  if (is_status) {
    mb_update_wheel_feedback(b, sample);
    if (sample->gear_valid) {
      if (!b->controller_gear_valid || sample->gear != b->controller_gear) {
        hal_log(HAL_LOG_INFO, "controller gear feedback changed: gear=%d",
          sample->gear);
      }
      b->controller_gear = sample->gear;
      b->controller_gear_valid = 1;
      if (!b->output_gear_valid ||
          mb_now(b) - b->last_output_gear_update_time >
            b->config.hall_speed_control_timeout) {
        b->output_gear = sample->gear;
        b->output_gear_valid = 1;
        b->output_gear_closed_loop_active = 0;
      }
    }
    if (sample->x >= 0 && sample->y >= 0) {
      b->last_controller_x = sample->x;
      b->last_controller_y = sample->y;
      if (b->hall_speed_fault_wait_neutral &&
          mb_controller_sample_is_neutral(b, sample)) {
        b->hall_speed_fault_wait_neutral = 0;
        mc_hall_speed_controller_reset(&b->hall_speed_controller);
        hal_log(HAL_LOG_WARN,
          "Hall speed feedback recovered; controller neutral accepted");
      }
    }
  }

  if (is_status && b->feedback_pub && b->inject.feedback_sink != NULL) {
    b->inject.feedback_sink(b->inject.feedback_sink_user, sample);
  }

  {
    const int has_controller_xy = (sample->x >= 0 && sample->y >= 0);
    if (is_drive_or_status && has_controller_xy) {
      b->last_controller_x = sample->x;
      b->last_controller_y = sample->y;
      b->last_controller_sample_time = mb_now(b);
    }
    if (is_drive_or_status && has_controller_xy && b->controller_cmd_pub &&
        b->inject.controller_cmd_sink != NULL) {
      int publish_controller_cmd = 1;
      if (!b->controller_neutral_seen) {
        if (mb_controller_sample_is_neutral(b, sample)) {
          b->controller_neutral_seen = 1;
        } else {
          publish_controller_cmd = 0;
        }
      }
      if (publish_controller_cmd) {
        double linear_x, angular_z;
        mb_controller_status_to_cmd(b, sample, &linear_x, &angular_z);
        b->last_raw_cmd_time = mb_now(b);
        b->last_raw_linear = linear_x;
        b->last_raw_angular = angular_z;
        b->inject.controller_cmd_sink(b->inject.controller_cmd_sink_user, linear_x, angular_z);
      }
    }
  }

  {
    const double now = mb_now(b);
    if (is_drive_or_status && now - b->last_raw_frame_log_time >= 0.20) {
      b->last_raw_frame_log_time = now;
      /* Diagnostic only: distinguish the controller's parsed Y from the
       * final Y emitted after DWA/Hall speed control. */
      hal_log(HAL_LOG_INFO,
        "controller_raw: kind=%s x=%d y=%d raw_linear=%.3f raw_angular=%.3f gear=%d",
        sample->kind == MS_SAMPLE_KIND_DRIVE ? "drive" : "status",
        sample->x, sample->y, b->last_raw_linear, b->last_raw_angular,
        b->controller_gear_valid ? b->controller_gear : 0);
    }
  }
}

void mb_poll_status(mb_bridge_t *b)
{
  ms_sample_t samples[MS_SAMPLE_MAX];
  const int n = ms_serial_writer_read_samples(&b->writer, samples);
  int i;
  for (i = 0; i < n; ++i) {
    mb_publish_sample(b, &samples[i]);
  }
}

/* —— 闭环与干预斜坡 —— */

void mb_apply_closed_loop(mb_bridge_t *b, double linear, double angular,
  double now, double *out_linear, double *out_angular)
{
  double dt;
  double angular_error;
  double angular_corr;
  double linear_corr = 0.0;
  double corrected_linear;
  double corrected_angular;

  b->closed_loop_has_info = 0;
  if (strcmp(b->config.closed_loop_mode, "intervention") != 0) {
    mb_reset_closed_loop_state(b);
    *out_linear = linear;
    *out_angular = angular;
    return;
  }
  if (!mb_wheel_feedback_is_fresh(b, now)) {
    mb_reset_closed_loop_state(b);
    *out_linear = linear;
    *out_angular = angular;
    return;
  }
  if (b->closed_loop_last_time > 0.0) {
    dt = mp_clamp_double(now - b->closed_loop_last_time, 0.0, 0.25);
  } else {
    dt = (double)b->config.interval_ms / 1000.0;
  }
  b->closed_loop_last_time = now;

  angular_error = angular - b->actual_yaw_rate;
  if (fabs(angular) < b->config.angular_deadband) {
    b->closed_loop_angular_i = 0.0;
  } else {
    b->closed_loop_angular_i += angular_error * dt;
    if (b->config.closed_loop_angular_ki > 0.0) {
      const double max_i = b->config.closed_loop_max_angular_correction /
        b->config.closed_loop_angular_ki;
      b->closed_loop_angular_i =
        mp_clamp_double(b->closed_loop_angular_i, -max_i, max_i);
    }
  }
  angular_corr = b->config.closed_loop_angular_kp * angular_error +
    b->config.closed_loop_angular_ki * b->closed_loop_angular_i;
  angular_corr = mp_clamp_double(angular_corr,
    -b->config.closed_loop_max_angular_correction,
    b->config.closed_loop_max_angular_correction);

  if (fabs(linear) >= b->config.linear_deadband) {
    const double linear_error = linear - b->actual_linear;
    linear_corr = b->config.closed_loop_linear_kp * linear_error;
    linear_corr = mp_clamp_double(linear_corr,
      -b->config.closed_loop_max_linear_correction,
      b->config.closed_loop_max_linear_correction);
  }

  corrected_linear = linear + linear_corr;
  corrected_angular = angular + angular_corr;
  corrected_linear = mp_clamp_double(corrected_linear,
    -b->config.max_reverse_speed, mb_forward_reference_now(b));
  corrected_angular = mp_clamp_double(corrected_angular,
    -b->config.max_turn_speed, b->config.max_turn_speed);
  if (linear > 0.0) corrected_linear = corrected_linear > 0.0 ? corrected_linear : 0.0;
  else if (linear < 0.0) corrected_linear = corrected_linear < 0.0 ? corrected_linear : 0.0;
  else corrected_linear = 0.0;

  b->closed_loop_info.active = 1;
  b->closed_loop_info.linear = linear;
  b->closed_loop_info.angular = angular;
  b->closed_loop_info.actual_linear = b->actual_linear;
  b->closed_loop_info.actual_yaw_rate = b->actual_yaw_rate;
  b->closed_loop_info.left_mps = b->wheel_left_mps;
  b->closed_loop_info.right_mps = b->wheel_right_mps;
  b->closed_loop_info.linear_corr = linear_corr;
  b->closed_loop_info.angular_corr = angular_corr;
  b->closed_loop_info.corrected_linear = corrected_linear;
  b->closed_loop_info.corrected_angular = corrected_angular;
  b->closed_loop_has_info = 1;
  *out_linear = corrected_linear;
  *out_angular = corrected_angular;
}

void mb_apply_intervention_slew(mb_bridge_t *b, double linear, double angular,
  double now, double *out_linear, double *out_angular)
{
  const double linear_rate = b->config.intervention_linear_slew_rate;
  const double angular_rate = b->config.intervention_angular_slew_rate;
  double dt;
  double previous_linear;
  double previous_angular;
  double slewed_linear;
  double slewed_angular;

  b->intervention_slew_has_info = 0;
  if (linear_rate <= 0.0 && angular_rate <= 0.0) {
    mb_reset_intervention_slew_state(b);
    *out_linear = linear;
    *out_angular = angular;
    return;
  }
  if (!b->intervention_slew_initialized) {
    b->intervention_slew_linear = b->last_raw_linear;
    b->intervention_slew_angular = b->last_raw_angular;
    b->intervention_slew_last_time = now;
    b->intervention_slew_initialized = 1;
  }
  dt = mp_clamp_double(now - b->intervention_slew_last_time, 0.0, 0.25);
  if (dt <= 0.0) dt = (double)b->config.interval_ms / 1000.0;
  previous_linear = b->intervention_slew_linear;
  previous_angular = b->intervention_slew_angular;

  slewed_linear = linear;
  if (linear_rate > 0.0 && fabs(linear) > fabs(previous_linear)) {
    slewed_linear = mb_limit_step(previous_linear, linear, linear_rate * dt);
  }
  slewed_angular = angular;
  if (angular_rate > 0.0) {
    slewed_angular = mb_limit_step(previous_angular, angular, angular_rate * dt);
  }
  b->intervention_slew_linear = slewed_linear;
  b->intervention_slew_angular = slewed_angular;
  b->intervention_slew_last_time = now;
  b->intervention_slew_target_linear = linear;
  b->intervention_slew_target_angular = angular;
  b->intervention_slew_prev_linear = previous_linear;
  b->intervention_slew_prev_angular = previous_angular;
  b->intervention_slew_has_info = 1;
  *out_linear = slewed_linear;
  *out_angular = slewed_angular;
}

/* —— cmd_to_xy —— */

mb_cmd_result_t mb_cmd_to_xy(mb_bridge_t *b, int apply_closed_loop_flag)
{
  mb_cmd_result_t result;
  const double now = mb_now(b);
  double linear, angular;
  int x, y;

  if (now - b->last_cmd_time > b->config.cmd_timeout) {
    result.x = MP_CENTER;
    result.y = MP_CENTER;
    state_set(result.state, "timeout_stop");
    return result;
  }
  linear = b->last_linear;
  angular = b->last_angular;
  if (b->config.invert_linear) linear = -linear;
  if (b->config.invert_angular) angular = -angular;
  if (fabs(linear) < b->config.linear_deadband) linear = 0.0;
  if (fabs(angular) < b->config.angular_deadband) angular = 0.0;

  if (b->config.axis_lock) {
    const double fwd_ref = mb_forward_reference_now(b);
    const double denom_base =
      (fwd_ref > b->config.max_reverse_speed ? fwd_ref : b->config.max_reverse_speed);
    const double denom = denom_base > 1e-6 ? denom_base : 1e-6;
    const double linear_ratio = fabs(linear) / denom;
    const double angular_denom =
      b->config.max_turn_speed > 1e-6 ? b->config.max_turn_speed : 1e-6;
    const double angular_ratio = fabs(angular) / angular_denom;
    if (angular_ratio > linear_ratio) linear = 0.0;
    else angular = 0.0;
  }

  if (apply_closed_loop_flag) {
    mb_apply_closed_loop(b, linear, angular, now, &linear, &angular);
    mb_apply_intervention_slew(b, linear, angular, now, &linear, &angular);
  }

  mb_motion_to_xy(b, linear, angular, &x, &y);
  result.x = x;
  result.y = y;
  state_set(result.state, "active");
  return result;
}

/* —— apply_hall_speed_output —— */

mb_hall_result_t mb_hall_result(int x, int y, const char *state)
{
  mb_hall_result_t r;
  r.x = x;
  r.y = y;
  state_set(r.state, state);
  return r;
}

mb_hall_result_t mb_apply_hall_speed_output(mb_bridge_t *b, int normal_x,
  int normal_y, const char *state, double now)
{
  const double default_dt = (double)b->config.interval_ms / 1000.0;

  if (b->hall_speed_fault_wait_neutral) {
    return mb_hall_result(MP_CENTER, MP_CENTER, "hall_fault_stop");
  }

  if (b->hall_speed_control_active) {
    int raw_cap_y;
    int initial_y;
    int dwa_owns_linear;
    int controlled_y;

    if (!mb_hall_speed_control_is_fresh(b, now) &&
        !mb_dwa_command_can_cover_hall_control_gap(b, now)) {
      mb_latch_hall_speed_fault(b, "control topic timeout");
      return mb_hall_result(MP_CENTER, MP_CENTER, "hall_control_timeout_stop");
    }
    if (!mb_wheel_feedback_is_fresh(b, now)) {
      mb_latch_hall_speed_fault(b, "wheel feedback timeout");
      return mb_hall_result(MP_CENTER, MP_CENTER, "hall_feedback_timeout_stop");
    }
    {
      const int raw_forward_active = mb_raw_forward_is_active(b, now);
      if (!raw_forward_active && mc_hall_control_should_follow_raw(
            b->hall_speed_control_mode, mb_raw_command_is_recent(b, now),
            b->last_raw_linear, b->config.linear_deadband)) {
        int raw_x, raw_y;
        mc_hall_speed_controller_reset(&b->hall_speed_controller);
        mc_moving_hold_controller_reset(&b->moving_hold_controller);
        mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
        mb_raw_override_xy(b, &raw_x, &raw_y);
        return mb_hall_result(raw_x, raw_y, "controller_override");
      }
    }

    dwa_owns_linear = (b->hall_speed_control_mode == MP_HALL_MODE_MOVING_DWA ||
                       b->hall_speed_control_mode == MP_HALL_MODE_STATIC_DYNAMIC_DWA);
    raw_cap_y = dwa_owns_linear ? 200 : mb_raw_forward_cap_y(b);
    initial_y = normal_y < raw_cap_y ? normal_y : raw_cap_y;

    /* The planner command is the safety upper bound.  When DWA has already
     * selected zero forward velocity, do not keep chasing the stale Hall
     * target (typically 0.4167 m/s from pre-brake/DWA handoff).  The board
     * logs showed hall_speed_control continuing to raise Y after DWA had
     * published v=0, which delayed braking until the obstacle was already
     * too close.  Keep angular output unchanged and let HallSpeedController
     * take the immediate zero-target stop path. */
    {
      const int dwa_zero_linear_command = dwa_owns_linear &&
        fabs(b->last_linear) <= b->config.linear_deadband;
      if (dwa_zero_linear_command) {
        controlled_y = mc_hall_speed_controller_update(
          &b->hall_speed_controller, 0.0, b->actual_linear, (double)raw_cap_y,
          (double)initial_y, now, default_dt,
          b->config.hall_speed_decel_predict_sec);
        mc_moving_hold_controller_reset(&b->moving_hold_controller);
        mc_prebrake_stop_controller_reset(&b->prebrake_stop_controller);
        return mb_hall_result(normal_x, controlled_y, "hall_dwa_zero_brake");
      }
    }
    {
      const int prebrake = (b->hall_speed_control_mode == MP_HALL_MODE_MOVING_PREBRAKE);
      const int moving_hold = (b->hall_speed_control_mode == MP_HALL_MODE_MOVING_HOLD);

      if (prebrake) {
        int released;
        /* HallSpeedController is the sole ordinary speed controller. Do not
         * pre-clamp Y with HallPrebrakeStopController: its hold_y handoff can
         * create the observed one-cycle point-brake jump. Start from the
         * current command and let the Hall rate limit move continuously. */
        controlled_y = mc_hall_speed_controller_update(
          &b->hall_speed_controller, b->hall_speed_brake_target,
          b->actual_linear, (double)raw_cap_y, (double)initial_y, now,
          default_dt, b->config.hall_speed_decel_predict_sec);
        released = b->actual_linear <=
          b->hall_speed_brake_target + b->config.hall_moving_hold_release_margin;
        if (!released) {
          mc_moving_hold_controller_reset(&b->moving_hold_controller);
          return mb_hall_result(normal_x, controlled_y, "hall_prebrake_decelerate");
        }
        /* Keep the same HallSpeedController output through the prebrake ->
         * hold handoff.  Do not seed a second controller with its own slow
         * rise rate, which was the source of the 0.3~0.8 km/h plateau. */
        mc_moving_hold_controller_reset(&b->moving_hold_controller);
        return mb_hall_result(normal_x, controlled_y, "hall_prebrake_release");
      }

      if (moving_hold) {
        /* Retain MovingHoldController only as the emergency-stop safety
         * monitor.  Its controlled_y is deliberately not used for ordinary
         * motion; HallSpeedController is the sole linear-speed path. */
        const mc_moving_hold_result_t mr = mc_moving_hold_controller_update(
          &b->moving_hold_controller, b->hall_speed_control_target,
          b->actual_linear, (double)raw_cap_y, (double)initial_y, now,
          default_dt);
        if (mr.emergency_stop) {
          return mb_hall_result(MP_CENTER, MP_CENTER,
            "hall_moving_hold_emergency_stop");
        }
        controlled_y = mc_hall_speed_controller_update(
          &b->hall_speed_controller, b->hall_speed_control_target,
          b->actual_linear, (double)raw_cap_y, (double)initial_y, now,
          default_dt, b->config.hall_speed_decel_predict_sec);
        return mb_hall_result(normal_x, controlled_y, "hall_moving_hold");
      }

      controlled_y = mc_hall_speed_controller_update(
        &b->hall_speed_controller, b->hall_speed_control_target,
        b->actual_linear, (double)raw_cap_y, (double)initial_y, now,
        default_dt,
        (b->hall_speed_control_mode == MP_HALL_MODE_SIDE_CAP)
          ? b->config.hall_speed_decel_predict_sec : 0.0);
      return mb_hall_result(normal_x, controlled_y, "hall_speed_control");
    }
  }

  if (b->hall_speed_release_active) {
    if (!mb_raw_forward_is_active(b, now)) {
      int raw_x, raw_y;
      b->hall_speed_release_active = 0;
      mc_hall_speed_controller_reset(&b->hall_speed_controller);
      mb_raw_override_xy(b, &raw_x, &raw_y);
      return mb_hall_result(raw_x, raw_y, "controller_override");
    }
    {
      const int cap = mb_raw_forward_cap_y(b);
      const int target_y = normal_y < cap ? normal_y : cap;
      double dt;
      double max_step;
      double current_y;
      if (!b->hall_speed_controller.initialized) {
        mc_hall_speed_controller_start(&b->hall_speed_controller,
          (double)target_y, now);
      }
      {
        const double previous_time = b->hall_speed_controller.last_time;
        dt = (previous_time <= 0.0) ? (double)b->config.interval_ms / 1000.0
          : mp_clamp_double(now - previous_time, 0.001, 0.25);
      }
      b->hall_speed_controller.last_time = now;
      max_step = b->config.hall_speed_y_up_rate * dt;
      current_y = b->hall_speed_controller.output_y;
      current_y += mp_clamp_double((double)target_y - current_y,
        -max_step, max_step);
      b->hall_speed_controller.output_y = current_y;
      if (fabs((double)target_y - current_y) <= 0.5) {
        b->hall_speed_release_active = 0;
        /* Preserve the current output for the global ordinary-forward path.
         * Resetting here restarts the next cycle at Y=100 and causes a false
         * drop to the very-low-speed region. */
        b->hall_speed_controller.output_y = (double)target_y;
        return mb_hall_result(normal_x, target_y, "hall_release_complete");
      }
      return mb_hall_result(normal_x, mp_clamp_coord(current_y),
        "hall_speed_release");
    }
  }

  /* HallSpeedController is global for ordinary forward motion, not only for
   * pre-brake/DWA states.  This keeps open-field acceleration on the same
   * slew-limited path as obstacle-mode transitions.  Safety stop paths above
   * remain independent and angular output is intentionally untouched. */
  {
    const int ordinary_forward =
      mb_raw_forward_is_active(b, now) && normal_y > MP_CENTER;
    if (ordinary_forward) {
      const int raw_cap_y = mb_raw_forward_cap_y(b);
      /* Start from center on the first ordinary-forward command so
       * open-field acceleration is slew-limited too. Once started, retain
       * the Hall controller state. */
      const int initial_y = b->hall_speed_controller.initialized
        ? (normal_y < raw_cap_y ? normal_y : raw_cap_y) : MP_CENTER;
      /* Use the candidate command's native m/s value as the Hall target.
       * Reconstructing m/s from normal_y is lossy because motion_to_xy()
       * saturates the protocol Y coordinate at 200 (for example 0.70 m/s
       * maps to Y=220 before clamping).  Using last_linear_ preserves the
       * real hand/DWA command and prevents the recovery path from being
       * pulled down to the false ~1.7 km/h plateau. */
      const double pos_linear =
        b->last_linear > 0.0 ? b->last_linear : 0.0;
      const double fwd_ref = mb_forward_reference_now(b);
      const double target_speed_mps =
        mp_clamp_double(pos_linear, 0.0, fwd_ref);
      const double actual_speed_mps = mb_wheel_feedback_is_fresh(b, now)
        ? mp_clamp_double(b->actual_linear, 0.0, fwd_ref) : 0.0;
      const int controlled_y = mc_hall_speed_controller_update(
        &b->hall_speed_controller, target_speed_mps, actual_speed_mps,
        (double)raw_cap_y, (double)initial_y, now,
        (double)b->config.interval_ms / 1000.0, 0.0);
      return mb_hall_result(normal_x, controlled_y, "hall_global_speed_control");
    }
  }

  /* Do not hold stale forward slew state while the controller is neutral,
   * reversing, or turning in place. */
  if (!mb_raw_forward_is_active(b, now) || normal_y <= MP_CENTER) {
    mc_hall_speed_controller_reset(&b->hall_speed_controller);
  }
  return mb_hall_result(normal_x, normal_y, state);
}

void mb_sync_hall_speed_controller_to_output(mb_bridge_t *b, int output_y,
  const char *state, double now)
{
  /* 原版定义未调用（死代码），直译保留。 */
  if (output_y <= MP_CENTER) return;
  if (b->hall_speed_control_active) {
    if (b->hall_speed_control_mode == MP_HALL_MODE_MOVING_HOLD ||
        strcmp(state, "hall_prebrake_release") == 0) {
      mc_hall_speed_controller_start(&b->moving_hold_controller.speed_controller,
        (double)output_y, now);
    } else {
      mc_hall_speed_controller_start(&b->hall_speed_controller,
        (double)output_y, now);
    }
  } else if (b->hall_speed_release_active) {
    mc_hall_speed_controller_start(&b->hall_speed_controller,
      (double)output_y, now);
  }
}

mb_linear_result_t mb_linear_result_make(int x, int y, const char *state)
{
  mb_linear_result_t r;
  r.x = x;
  r.y = y;
  state_set(r.state, state);
  return r;
}

mb_linear_result_t mb_apply_linear_output_profile(mb_bridge_t *b, int x, int y,
  const char *state, double now)
{
  (void)now;
  /* HallSpeedController is the sole ordinary linear-speed controller.
   * Keep only the independent hard-stop safety check here; do not apply a
   * second comfort/protective Y slew after Hall output. */
  {
    const int raw_forward_active = mb_raw_forward_is_active(b, mb_now(b));
    if (mc_linear_output_hard_stop_required(x, y, state, raw_forward_active)) {
      char s[MB_STATE_MAX];
      mc_linear_output_profile_hard_stop(&b->linear_output_profile, mb_now(b));
      state_set(b->linear_output_profile_kind, "hard_stop");
      state_set(s, state);
      state_append_stop(s);
      return mb_linear_result_make(MP_CENTER, MP_CENTER, s);
    }
  }
  mc_linear_output_profile_reset(&b->linear_output_profile);
  state_set(b->linear_output_profile_kind, "bypassed_hall_primary");
  return mb_linear_result_make(x, y, state);
}

mb_write_decision_t mb_should_write_intervention(mb_bridge_t *b, double now,
  const char *state, int is_idle_stop)
{
  mb_write_decision_t d;

  if (strcmp(state, "stop_escape") == 0) {
    d.should_write = 1;
    state_set(d.reason, "stop_escape");
    return d;
  }
  if (strcmp(b->config.output_mode, "continuous") == 0) {
    d.should_write = 1;
    state_set(d.reason, "continuous");
    return d;
  }
  if (is_idle_stop) {
    const int hold_source_active =
      (strcmp(state, "timeout_stop") == 0 || mb_raw_command_is_motion(b) ||
       b->intervention_motion_active || mb_wheel_motion_active(b, now));
    if (hold_source_active) {
      b->safe_stop_hold_until =
        (b->safe_stop_hold_until > now + b->config.safe_stop_hold_sec)
        ? b->safe_stop_hold_until
        : now + b->config.safe_stop_hold_sec;
      d.should_write = 1;
      state_set(d.reason, "safe_stop_hold");
      return d;
    }
    if (now < b->safe_stop_hold_until) {
      d.should_write = 1;
      state_set(d.reason, "safe_stop_hold");
      return d;
    }
    b->safe_stop_hold_until = 0.0;
  }
  if (!mb_raw_command_is_recent(b, now)) {
    d.should_write = 0;
    state_set(d.reason, "no_raw");
    return d;
  }
  if (!mb_raw_command_is_motion(b)) {
    d.should_write = 0;
    state_set(d.reason, "raw_idle");
    return d;
  }
  if (strcmp(state, "timeout_stop") == 0) {
    d.should_write = 1;
    state_set(d.reason, "avoidance_timeout_stop");
    return d;
  }
  {
    const double linear_delta = fabs(b->last_linear - b->last_raw_linear);
    const double angular_delta = fabs(b->last_angular - b->last_raw_angular);
    if (linear_delta >= b->config.intervention_linear_delta ||
        angular_delta >= b->config.intervention_angular_delta) {
      d.should_write = 1;
      state_set(d.reason, "intervene");
      return d;
    }
  }
  d.should_write = 0;
  state_set(d.reason, "pass_through");
  return d;
}

static int mb_physical_forward_intent(mb_bridge_t *b, double now)
{
  double dy;
  if (b->last_controller_sample_time <= 0.0 ||
      now - b->last_controller_sample_time > b->config.cmd_timeout) {
    return 0;
  }
  dy = mb_controller_axis_delta(b, b->last_controller_y,
    b->config.controller_center_y, b->config.controller_dead_low,
    b->config.controller_dead_high, b->config.controller_invert_y);
  return dy > 0.0;
}

static void mb_reset_escape_yaw_control(mb_bridge_t *b)
{
  b->escape_yaw_integral = 0.0;
  b->escape_yaw_output = 0.0;
  b->escape_yaw_direction = SE_DIRECTION_NONE;
  b->escape_yaw_diag_target = 0.0;
  b->escape_yaw_diag_actual = 0.0;
  b->escape_yaw_diag_output = 0.0;
  b->escape_yaw_last_update_time = 0.0;
}

static double mb_escape_yaw_command(mb_bridge_t *b,
  se_direction_t direction, double now)
{
  const int imu_fresh = b->escape_imu_valid && b->escape_imu_time > 0.0 &&
    now >= b->escape_imu_time && now - b->escape_imu_time <= 0.25;
  const double sign = direction == SE_DIRECTION_LEFT ? 1.0 : -1.0;
  const double target = sign * fabs(b->config.escape_turn_yawrate);
  double output = target;
  double dt = b->escape_yaw_last_update_time > 0.0 ?
    now - b->escape_yaw_last_update_time : 0.0;

  if (b->escape_yaw_direction != direction) {
    mb_reset_escape_yaw_control(b);
    b->escape_yaw_direction = direction;
    dt = 0.0;
  }
  if (dt < 0.0 || dt > 0.10) dt = 0.0;
  b->escape_yaw_last_update_time = now;
  b->escape_yaw_diag_target = target;
  b->escape_yaw_diag_actual = imu_fresh ? b->escape_imu_yaw_rate : 0.0;

  if (imu_fresh) {
    const double error = target - b->escape_imu_yaw_rate;
    const double kp = 0.80;
    const double ki = 0.35;
    const double integral_limit = 0.25;
    const double output_limit = 0.90;
    double wanted;
    b->escape_yaw_integral += ki * error * dt;
    b->escape_yaw_integral = mp_clamp_double(
      b->escape_yaw_integral, -integral_limit, integral_limit);
    wanted = target + kp * error + b->escape_yaw_integral;
    wanted = mp_clamp_double(wanted, -output_limit, output_limit);
    /* Bound output slew so newly accumulated compensation cannot release as a jerk. */
    if (dt > 0.0 && b->escape_yaw_output != 0.0) {
      const double max_step = 2.0 * dt;
      output = b->escape_yaw_output +
        mp_clamp_double(wanted - b->escape_yaw_output, -max_step, max_step);
    } else {
      output = wanted;
    }
  } else {
    /* Invalid/stale IMU clears compensation immediately; retain only the safe base turn. */
    b->escape_yaw_integral = 0.0;
    output = target;
  }
  b->escape_yaw_output = output;
  b->escape_yaw_diag_output = output;
  if (now - b->escape_last_diag_log_time >= 0.20) {
    hal_log(HAL_LOG_INFO,
      "escape_yaw direction=%s imu_valid=%d target=%+.3f actual=%+.3f "
      "integral=%+.3f output=%+.3f",
      direction == SE_DIRECTION_LEFT ? "LEFT" : "RIGHT",
      imu_fresh, target, b->escape_yaw_diag_actual,
      b->escape_yaw_integral, output);
    b->escape_last_diag_log_time = now;
  }
  return output;
}

se_stop_escape_decision_t mb_update_stop_escape(mb_bridge_t *b, double now,
  int controller_neutral_stop, int normal_output_stopped)
{
  const int physical_forward = mb_physical_forward_intent(b, now);
  const double controller_turn = mb_controller_turn_ratio(b,
    b->last_controller_x);
  const se_direction_t manual_turn_direction = controller_turn > 0.0
    ? SE_DIRECTION_LEFT : (controller_turn < 0.0
      ? SE_DIRECTION_RIGHT : SE_DIRECTION_NONE);
  const int dwa_request_fresh = b->dwa_escape_request &&
    b->dwa_escape_request_time > 0.0 &&
    now >= b->dwa_escape_request_time &&
    now - b->dwa_escape_request_time <= 0.25;
  const int wheel_feedback_fresh = mb_wheel_feedback_is_fresh(b, now);
  const int vehicle_stopped = wheel_feedback_fresh &&
    fabs(b->actual_linear) <= b->config.escape_stop_speed;
  const int positive_dwa_command = b->last_cmd_time > 0.0 &&
    now >= b->last_cmd_time &&
    now - b->last_cmd_time <= b->config.cmd_timeout &&
    b->last_linear > b->config.linear_deadband;
  const int positive_forward_motion = wheel_feedback_fresh &&
    b->actual_linear > b->config.escape_stop_speed;
  /* Keep the maneuver latched from physical controller intent. The DWA
   * request/fallback trigger can flicker as outputs and feedback change; that
   * trigger is for event attribution, not the ongoing escape-enable signal.
   * Initial activation still requires stopped wheels and stopped normal output
   * inside the compensator, so this cannot take over ordinary movement. */
  b->stop_escape_last_decision = se_stop_escape_compensator_update_with_turn(
    &b->stop_escape, now, controller_neutral_stop, physical_forward,
    vehicle_stopped, normal_output_stopped, positive_dwa_command,
    positive_forward_motion, manual_turn_direction);
  if (b->stop_escape_last_decision.active &&
      !b->stop_escape_event_active) {
    ++b->stop_escape_event_count;
    hal_log(HAL_LOG_WARN,
      "脱困统计 source=unified_escape event=%llu trigger=%s direction=%s "
      "side_clear(L=%.3f,R=%.3f)",
      (unsigned long long)b->stop_escape_event_count,
      dwa_request_fresh ? "dwa_request" : "motor_bridge_stall",
      b->stop_escape_last_decision.direction == SE_DIRECTION_LEFT ?
        "LEFT" : "RIGHT",
      b->stop_escape_last_decision.left_clearance,
      b->stop_escape_last_decision.right_clearance);
  }
  b->stop_escape_event_active =
    b->stop_escape_last_decision.active ? 1 : 0;
  if (b->stop_escape_last_decision.active &&
      b->stop_escape_last_decision.command_enabled) {
    b->stop_escape_last_decision.yawrate = mb_escape_yaw_command(
      b, b->stop_escape_last_decision.direction, now);
  } else if (b->stop_escape_last_decision.active) {
    /* Pause compensator while retaining the escape latch. */
    mb_reset_escape_yaw_control(b);
    b->stop_escape_last_decision.yawrate = 0.0;
  } else {
    mb_reset_escape_yaw_control(b);
  }
  return b->stop_escape_last_decision;
}

/* —— on_timer —— */

void mb_on_timer(mb_bridge_t *b)
{
  int x, y;
  char state[MB_STATE_MAX];
  mp_frame_t frame;
  int is_idle_stop, is_motion;
  int should_write;
  char write_reason[MB_STATE_MAX];
  int controller_neutral_stop;
  int normal_output_stopped;
  se_stop_escape_decision_t stop_escape_decision;
  const double now = mb_now(b); /* C++ 在 on_timer 顶部捕获一次并贯穿使用 */

  mb_poll_status(b);
  (void)mb_controller_feedback_is_online(b, now);
  {
    mb_update_wheel_yaw_total(b, now);
    controller_neutral_stop = mb_physical_controller_is_neutral(b, now);

    if (controller_neutral_stop) {
      mb_clear_motion_state_for_controller_neutral(b);
      x = MP_CENTER;
      y = MP_CENTER;
      state_set(state, "controller_neutral_stop");
    } else if (b->hard_stop_active) {
      x = MP_CENTER;
      y = MP_CENTER;
      state_set(state, "hard_stop_latched");
    } else {
      /* raw_reverse_passthrough_xy */
      int reverse_x = 0, reverse_y = 0;
      char reverse_state[MB_STATE_MAX];
      const int has_reverse =
        mb_raw_reverse_passthrough_xy(b, now, &reverse_x, &reverse_y,
          reverse_state);
      if (has_reverse) {
        x = reverse_x;
        y = reverse_y;
        state_set(state, reverse_state);
      } else {
        const mb_cmd_result_t cr = mb_cmd_to_xy(b, 0);
        const mb_hall_result_t hr =
          mb_apply_hall_speed_output(b, cr.x, cr.y, cr.state, now);
        x = hr.x;
        y = hr.y;
        state_set(state, hr.state);
      }
    }
  }

  normal_output_stopped = (x == MP_CENTER && y == MP_CENTER) ||
    state_ends_with_stop(state);
  {
    const int sensor_fault_mask = b->sensor_interlock_enabled
      ? mb_sensor_fault_mask(b, now) : 0;
    if (sensor_fault_mask != b->sensor_fault_last_mask) {
      hal_log(sensor_fault_mask ? HAL_LOG_ERROR : HAL_LOG_INFO,
        "sensor_interlock fault_mask=0x%02X left_radar=%d right_radar=%d "
        "imu=%d action=%s", sensor_fault_mask,
        (sensor_fault_mask & MP_SENSOR_STATUS_LEFT_RADAR_FAULT) != 0,
        (sensor_fault_mask & MP_SENSOR_STATUS_RIGHT_RADAR_FAULT) != 0,
        (sensor_fault_mask & MP_SENSOR_STATUS_IMU_FAULT) != 0,
        sensor_fault_mask ? "FORCE_STOP" : "RELEASE");
      b->sensor_fault_last_mask = sensor_fault_mask;
    }
    if (sensor_fault_mask != 0) {
      /* A stale sensor must not let the DWA escape controller or manual
       * bypass emit motion. Clear escape state so recovery cannot replay it. */
      se_stop_escape_compensator_reset(&b->stop_escape);
      b->dwa_escape_request = 0;
      b->dwa_escape_request_time = 0.0;
      b->stop_escape_event_active = 0;
      b->stop_escape_last_decision.active = 0;
      b->stop_escape_last_decision.command_enabled = 0;
      b->stop_escape_last_decision.yawrate = 0.0;
      b->stop_escape_last_decision.direction = SE_DIRECTION_NONE;
      b->stop_escape_last_decision.reason = "sensor_fault";
      stop_escape_decision = b->stop_escape_last_decision;
      x = MP_CENTER;
      y = MP_CENTER;
      state_set(state, "sensor_fault_stop");
    } else {
      stop_escape_decision = mb_update_stop_escape(b, now,
        controller_neutral_stop, normal_output_stopped);
    }
  }
  if (stop_escape_decision.active && stop_escape_decision.yield_to_dwa) {
    /* Trial handoff: preserve this timer's normal DWA output. The escape
     * module retains its event/direction and will resume if motion fails. */
    state_set(state, "stop_escape_handoff");
  } else if (stop_escape_decision.active && stop_escape_decision.command_enabled) {
    double escape_angular = stop_escape_decision.yawrate;
    int ex, ey;
    if (b->config.invert_angular) escape_angular = -escape_angular;
    mb_motion_to_xy(b, 0.0, escape_angular, &ex, &ey);
    x = ex;
    y = ey;
    state_set(state, "stop_escape");
  } else if (stop_escape_decision.active) {
    /* Retain the latch but fail safe while sensing/clearance is insufficient. */
    x = MP_CENTER;
    y = MP_CENTER;
    state_set(state, "stop_escape_paused");
  }

  frame = mb_build_output_frame(b, x, y);
  is_idle_stop = (x == MP_CENTER && y == MP_CENTER);
  is_motion = !is_idle_stop;
  if (strcmp(b->config.output_mode, "continuous") == 0 && is_motion &&
      !b->was_motion && b->config.activate_stop_repeats > 0) {
    mb_send_stop_burst(b, "activate", b->config.activate_stop_repeats,
      b->config.activate_stop_settle);
  }

  if (controller_neutral_stop) {
    should_write = 1;
    state_set(write_reason, "controller_neutral_hard_stop");
  } else if (stop_escape_decision.active) {
    should_write = 1;
    state_set(write_reason, "stop_escape");
  } else if (b->hard_stop_active) {
    should_write = 1;
    state_set(write_reason, "hard_stop_latched");
  } else {
    const mb_write_decision_t sw =
      mb_should_write_intervention(b, now, state, is_idle_stop);
    should_write = sw.should_write;
    state_set(write_reason, sw.reason);
  }

  if (!should_write &&
      strcmp(b->config.output_mode, "intervention-only") == 0 &&
      b->intervention_motion_active &&
      (strcmp(write_reason, "raw_idle") == 0 ||
       strcmp(write_reason, "no_raw") == 0)) {
    mb_send_intervention_release_stop(b, write_reason);
    should_write = 0;
    state_append_stop(write_reason); /* write_reason += "_release_stop" */
  }

  if (should_write && strcmp(write_reason, "intervene") == 0) {
    const mb_cmd_result_t cr = mb_cmd_to_xy(b, 1);
    x = cr.x;
    y = cr.y;
    state_set(state, cr.state);
    frame = mb_build_output_frame(b, x, y);
    is_idle_stop = (x == MP_CENTER && y == MP_CENTER);
    is_motion = !is_idle_stop;
  } else {
    mb_reset_closed_loop_state(b);
    b->closed_loop_has_info = 0;
    mb_reset_intervention_slew_state(b);
  }

  if (should_write) {
    if (strcmp(state, "stop_escape") == 0) {
      mc_linear_output_profile_reset(&b->linear_output_profile);
      state_set(b->linear_output_profile_kind, "stop_escape_bypass");
    } else {
      const mb_linear_result_t lr =
        mb_apply_linear_output_profile(b, x, y, state, now);
      x = lr.x;
      y = lr.y;
      state_set(state, lr.state);
    }
    frame = mb_build_output_frame(b, x, y);
    is_idle_stop = (x == MP_CENTER && y == MP_CENTER);
    is_motion = !is_idle_stop;
  }

  if (should_write && strcmp(b->config.idle_stop_mode, "on-change") == 0 &&
      is_idle_stop && mp_frame_equal(&b->last_written_frame, &frame) &&
      !controller_neutral_stop) {
    should_write = 0;
    state_set(write_reason, "unchanged_idle");
  }

  if (b->sensor_interlock_enabled && mb_sensor_fault_mask(b, now) != 0) {
    x = MP_CENTER;
    y = MP_CENTER;
    frame = mb_build_output_frame(b, x, y);
    is_idle_stop = 1;
    is_motion = 0;
    should_write = 1;
    state_set(state, "sensor_fault_stop");
    state_set(write_reason, "sensor_fault_stop");
  }

  if (should_write) {
    mb_writer_write(b, &frame);
    b->last_written_frame = frame;
    b->intervention_motion_active = !is_idle_stop;
  }
  b->was_motion = is_motion;

  /* 端到端排障：每秒一条，记录输入门控、桥输出、实际 write() 返回值。
   * 低频采样避免测试日志持续刷写；写失败另有节流错误行。 */
  if (now - b->last_chain_diag_time >= 1.0 ||
      b->last_chain_diag_time == 0.0) {
    char hex[MP_FRAME_HEX_MAX];
    const double ctrl_age = b->last_controller_sample_time > 0.0
      ? now - b->last_controller_sample_time : -1.0;
    const double cmd_age = b->last_cmd_time > 0.0
      ? now - b->last_cmd_time : -1.0;
    const double raw_age = b->last_raw_cmd_time > 0.0
      ? now - b->last_raw_cmd_time : -1.0;
    const double hall_age = b->last_hall_speed_control_time > 0.0
      ? now - b->last_hall_speed_control_time : -1.0;
    const double wheel_age = b->last_wheel_feedback_time > 0.0
      ? now - b->last_wheel_feedback_time : -1.0;
    const int physical_forward = mb_physical_forward_intent(b, now);
    const int dwa_request_fresh = b->dwa_escape_request &&
      b->dwa_escape_request_time > 0.0 && now >= b->dwa_escape_request_time &&
      now - b->dwa_escape_request_time <= 0.25;
    int tx_age_ms = -1;
    mp_frame_hex(&frame, hex);
    if (b->tx_last_time > 0.0 && now >= b->tx_last_time) {
      tx_age_ms = (int)((now - b->tx_last_time) * 1000.0);
    }
    hal_log(HAL_LOG_INFO,
      "motor_chain t=%.3f ctrl=(%d,%d) ctrl_age=%.3f neutral=%d raw=(%.3f,%+.3f) raw_age=%.3f dwa=(%.3f,%+.3f) cmd_age=%.3f hard_stop=%d hall=(active:%d mode:%d target:%.3f age:%.3f fault:%d) wheel=(L:%.3f R:%.3f v:%.3f age:%.3f) escape=(fwd:%d dwa_req:%d pending:%d active:%d cmd:%d dir:%s reason:%s yaw:%+.3f clear:(%.3f,%.3f)) bridge=(state:%s xy:%d,%d write:%d reason:%s) frame=%s port=%s baud=%d fd=%d dry=%d feedback=(online:%d age:%.3f seen:%d) rx=(bytes:%ld frames:%ld drive:%ld status:%ld bad_crc:%ld dropped:%ld) tx=(ret:%ld errno:%d age_ms:%d attempts:%llu full:%llu fail:%llu)",
      hal_steady_now_sec(), b->last_controller_x, b->last_controller_y,
      ctrl_age, controller_neutral_stop, b->last_raw_linear,
      b->last_raw_angular, raw_age, b->last_linear, b->last_angular,
      cmd_age, b->hard_stop_active, b->hall_speed_control_active,
      b->hall_speed_control_mode, b->hall_speed_control_target, hall_age,
      b->hall_speed_fault_wait_neutral, b->wheel_left_mps,
      b->wheel_right_mps, b->actual_linear, wheel_age, physical_forward,
      dwa_request_fresh, b->dwa_escape_request,
      stop_escape_decision.active, stop_escape_decision.command_enabled,
      stop_escape_decision.direction == SE_DIRECTION_LEFT ? "LEFT" :
        (stop_escape_decision.direction == SE_DIRECTION_RIGHT ? "RIGHT" : "NONE"),
      stop_escape_decision.reason != NULL ? stop_escape_decision.reason : "",
      stop_escape_decision.yawrate, stop_escape_decision.left_clearance,
      stop_escape_decision.right_clearance, state, x, y, should_write,
      write_reason, hex, b->writer.port, b->writer.baud, b->writer.fd,
      b->writer.dry_run, b->controller_feedback_online,
      now - b->controller_feedback_last_valid_time, b->controller_feedback_seen,
      b->writer.rx_stats.bytes, b->writer.rx_stats.frames,
      b->writer.rx_stats.drive, b->writer.rx_stats.status,
      b->writer.rx_stats.bad_crc, b->writer.rx_stats.dropped,
      b->tx_last_result, b->tx_last_errno, tx_age_ms,
      (unsigned long long)b->tx_attempt_count,
      (unsigned long long)b->tx_full_count,
      (unsigned long long)b->tx_fail_count);
    b->last_chain_diag_time = now;
  }

  /* 日志（对齐 Python 的 verbose/frame change/定时日志） */
  {
    const int intervention_debug =
      (b->closed_loop_has_info || b->intervention_slew_has_info ||
       b->output_turn_deadzone_has_info) && now - b->last_log_time >= 0.5;
    const int hall_debug = (b->hall_speed_control_active ||
                            b->hall_speed_release_active ||
                            b->hall_speed_fault_wait_neutral) &&
      now - b->last_log_time >= 0.1;
    const int linear_profile_debug = b->config.linear_profile_enabled &&
      b->linear_output_profile.has_last_info &&
      b->linear_output_profile.last_info.limited &&
      now - b->last_log_time >= 0.1;
    if (b->config.verbose || !mp_frame_equal(&frame, &b->last_frame) ||
        now - b->last_log_time >= 2.0 ||
        intervention_debug || hall_debug || linear_profile_debug) {
      char hex[MP_FRAME_HEX_MAX];
      mp_frame_hex(&frame, hex);
      hal_log(HAL_LOG_INFO,
        "%s/%s/%s: x=%d y=%d forward_ref=%.3f raw_controller_y=%d raw_linear=%.3f cmd_linear=%.3f hall_target=%.3f hall_active=%d hall_mode=%d frame=%s",
        state, should_write ? "sent" : "held", write_reason, x, y,
        b->forward_speed_reference, b->last_controller_y, b->last_raw_linear,
        b->last_linear, b->hall_speed_control_target,
        b->hall_speed_control_active ? 1 : 0, b->hall_speed_control_mode, hex);
      b->last_frame = frame;
      b->last_log_time = now;
    }
  }
}

int mb_raw_reverse_passthrough_xy(mb_bridge_t *b, double now,
  int *out_x, int *out_y, char out_state[MB_STATE_MAX])
{
  int controller_x, controller_y;
  int limited_controller_y;
  int angular_intervened;
  double linear_scale;

  if (!mb_raw_command_is_recent(b, now)) return 0;
  if (b->last_cmd_time <= 0.0 ||
      now - b->last_cmd_time > b->config.cmd_timeout) return 0;
  if (b->last_raw_linear >= -b->config.linear_deadband) return 0;
  if (b->last_linear >= -b->config.linear_deadband) return 0;

  controller_x = mp_clamp_coord((double)b->last_controller_x);
  controller_y = mp_clamp_coord((double)b->last_controller_y);
  /* Keep the controller's reverse-turn X input, but apply any DWA linear
   * speed cap to the reverse throttle axis before forwarding the frame. */
  linear_scale = fabs(b->last_linear) / fabs(b->last_raw_linear);
  if (linear_scale > 1.0) linear_scale = 1.0;
  if (linear_scale < 0.0) linear_scale = 0.0;
  limited_controller_y = mp_clamp_coord((double)lround(
    (double)b->config.controller_center_y +
    ((double)controller_y - (double)b->config.controller_center_y) *
      linear_scale));
  if (linear_scale < 0.999) {
    static int reverse_cap_log_key;
    hal_log_throttle(now, 500, &reverse_cap_log_key, HAL_LOG_INFO,
      "reverse_linear_cap raw=%.3f cmd=%.3f scale=%.3f y=%d->%d",
      b->last_raw_linear, b->last_linear, linear_scale,
      controller_y, limited_controller_y);
  }
  angular_intervened =
    fabs(b->last_angular - b->last_raw_angular) > b->config.angular_deadband;
  if (!angular_intervened) {
    *out_x = controller_x;
    *out_y = limited_controller_y;
    state_set(out_state, linear_scale < 0.999
      ? "controller_reverse_passthrough_limited"
      : "controller_reverse_passthrough");
    return 1;
  }
  {
    double corrected_angular = b->last_angular;
    int cx, cy;
    if (b->config.invert_angular) corrected_angular = -corrected_angular;
    if (fabs(corrected_angular) < b->config.angular_deadband) {
      corrected_angular = 0.0;
    }
    mb_motion_to_xy(b, 0.0, corrected_angular, &cx, &cy);
    *out_x = cx;
    *out_y = limited_controller_y;
    state_set(out_state, linear_scale < 0.999
      ? "controller_reverse_safety_steer_limited"
      : "controller_reverse_safety_steer");
    return 1;
  }
}

void mb_stop_and_close(mb_bridge_t *b)
{
  int i;
  const int repeats =
    b->config.stop_repeats > 1 ? b->config.stop_repeats : 1;
  mc_linear_output_profile_hard_stop(&b->linear_output_profile, mb_now(b));
  if (strcmp(b->config.output_mode, "intervention-only") == 0 &&
      !b->config.stop_on_close) {
    ms_serial_writer_close(&b->writer);
    return;
  }
  for (i = 0; i < repeats; ++i) {
    b->stop_frame = mb_build_output_frame(b, MP_CENTER, MP_CENTER);
    mb_writer_write(b, &b->stop_frame);
    b->last_written_frame = b->stop_frame;
    mb_sleep_ms(b, (double)b->config.interval_ms);
  }
  ms_serial_writer_close(&b->writer);
}

void mb_send_stop_burst(mb_bridge_t *b, const char *reason, int repeats,
  double settle)
{
  int i;
  (void)reason;
  mc_linear_output_profile_hard_stop(&b->linear_output_profile, mb_now(b));
  if (repeats < 0) repeats = b->config.pre_stop_repeats;
  if (settle < 0.0) settle = b->config.pre_stop_settle;
  if (repeats < 0) repeats = 0;
  if (repeats == 0) return;
  for (i = 0; i < repeats; ++i) {
    b->stop_frame = mb_build_output_frame(b, MP_CENTER, MP_CENTER);
    mb_writer_write(b, &b->stop_frame);
    b->last_written_frame = b->stop_frame;
    mb_sleep_ms(b, (double)b->config.interval_ms);
  }
  if (settle > 0.0) {
    mb_sleep_ms(b, settle);
  }
}

void mb_send_intervention_release_stop(mb_bridge_t *b, const char *reason)
{
  int i;
  const int repeats = b->config.intervention_release_stop_repeats > 1
    ? b->config.intervention_release_stop_repeats : 1;
  (void)reason;
  mc_linear_output_profile_hard_stop(&b->linear_output_profile, mb_now(b));
  for (i = 0; i < repeats; ++i) {
    b->stop_frame = mb_build_output_frame(b, MP_CENTER, MP_CENTER);
    mb_writer_write(b, &b->stop_frame);
    b->last_written_frame = b->stop_frame;
    mb_sleep_ms(b, (double)b->config.interval_ms);
  }
  b->intervention_motion_active = 0;
  {
    const double now = mb_now(b);
    b->safe_stop_hold_until =
      (b->safe_stop_hold_until > now + b->config.safe_stop_hold_sec)
      ? b->safe_stop_hold_until
      : now + b->config.safe_stop_hold_sec;
  }
}

/* —— sink / 注入注册 —— */

void mb_bridge_set_controller_cmd_sink(mb_bridge_t *b,
  mb_controller_cmd_sink_t sink, void *user)
{
  b->inject.controller_cmd_sink = sink;
  b->inject.controller_cmd_sink_user = user;
}

void mb_bridge_set_feedback_sink(mb_bridge_t *b,
  mb_feedback_sink_t sink, void *user)
{
  b->inject.feedback_sink = sink;
  b->inject.feedback_sink_user = user;
}

void mb_bridge_set_wheel_yaw_sink(mb_bridge_t *b,
  mb_wheel_yaw_sink_t sink, void *user)
{
  b->inject.wheel_yaw_sink = sink;
  b->inject.wheel_yaw_sink_user = user;
}

void mb_bridge_set_frame_sink(mb_bridge_t *b,
  mb_frame_sink_t sink, void *user)
{
  b->inject.frame_sink = sink;
  b->inject.frame_sink_user = user;
}

void mb_bridge_set_time_source(mb_bridge_t *b, mb_now_fn_t fn, void *user)
{
  b->inject.now_fn = fn;
  b->inject.now_fn_user = user;
}

void mb_bridge_set_sleep_source(mb_bridge_t *b, mb_sleep_fn_t fn, void *user)
{
  b->inject.sleep_fn = fn;
  b->inject.sleep_fn_user = user;
}
