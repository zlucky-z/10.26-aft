/*
 * P7 电机桥 parity harness（C 侧）。
 * 与 tools/parity/parity_p7_cpp.cpp 使用同一 LCG 生成相同输入序列，
 * 输出逐字段一致的 CSV（p7,<场景>,<tick>,... + p7d 析构行）。
 *
 * 观测面：串口帧序列（frame_sink 记录）、controller_cmd/feedback/
 * wheel_yaw sink 记录、桥内部状态。
 * 时钟/sleep 均注入（时间从 0 起步，与 C++ 侧 mock chrono 一致）。
 * 修改任一侧的 rng 调用顺序必须同步另一侧！
 * 用法: test_p7 [输出csv路径] [场景数]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/hal/hal_log.h"
#include "../src/motor/motor_bridge.h"
#include "../src/motor/motor_bridge_config.h"

/* ---- RNG（与 parity_p7_cpp.cpp 逐行一致） ---- */
static unsigned long long rng_state;

static void rng_seed(int s)
{
  rng_state = ((unsigned long long)s + 1ULL) * 0x9E3779B97F4A7C15ULL;
}

static unsigned long long rng_next(void)
{
  rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return rng_state;
}

static double rng_unit(void)
{
  return (double)(rng_next() >> 11) * (1.0 / 9007199254740992.0);
}

static int rng_int(int lo, int hi)
{
  const int v = lo + (int)(rng_unit() * (double)(hi - lo + 1));
  return v > hi ? hi : v;
}

#define T_TICKS 100
#define TICK_SEC 0.2
#define FRAME_REC_MAX 65536

/* ---- 观测记录 ---- */
static mp_frame_t g_frames[FRAME_REC_MAX];
static int g_frame_count;
static int g_ccmd_count;
static double g_ccmd_lin, g_ccmd_ang;
static int g_fb_count;
static double g_fb_x, g_fb_y, g_fb_gear;
static int g_wy_count;
static double g_wy_last;

/* ---- 注入源 ---- */
static double g_now;

static double test_now_fn(void *user)
{
  (void)user;
  return g_now;
}

static void test_sleep_fn(void *user, double ms)
{
  (void)user;
  (void)ms; /* no-op：时间由 g_now 推进（与 C++ mock sleep_for 一致） */
}

static void frame_record(void *user, const mp_frame_t *frame)
{
  (void)user;
  if (g_frame_count < FRAME_REC_MAX) {
    g_frames[g_frame_count++] = *frame;
  }
}

static void ccmd_record(void *user, double linear_x, double angular_z)
{
  (void)user;
  ++g_ccmd_count;
  g_ccmd_lin = linear_x;
  g_ccmd_ang = angular_z;
}

static void fb_record(void *user, const ms_sample_t *sample)
{
  (void)user;
  ++g_fb_count;
  /* 对齐 C++ Float32MultiArray：msg.data = {float(x), float(y), ...,
   * float(gear_valid ? gear : 0)} */
  g_fb_x = (double)(float)sample->x;
  g_fb_y = (double)(float)sample->y;
  g_fb_gear = (double)(float)(sample->gear_valid ? sample->gear : 0);
}

static void wy_record(void *user, double yaw_total)
{
  (void)user;
  ++g_wy_count;
  g_wy_last = yaw_total;
}

static void frames_hex_range(int begin, int end, char *out, size_t cap)
{
  size_t pos = 0;
  static const char hex[] = "0123456789ABCDEF";
  int i;
  size_t j;
  out[0] = '\0';
  for (i = begin; i < end; ++i) {
    const mp_frame_t *f = &g_frames[i];
    if (i != begin) out[pos++] = ';';
    for (j = 0; j < f->len; ++j) {
      if (j != 0) out[pos++] = ' ';
      out[pos++] = hex[(f->bytes[j] >> 4) & 0x0F];
      out[pos++] = hex[f->bytes[j] & 0x0F];
    }
    if (pos + 8 > cap) break;
  }
  if (pos == 0) {
    out[0] = '-';
    out[1] = '\0';
  } else {
    out[pos] = '\0';
  }
}

static const char *se_dir_str(se_direction_t d)
{
  switch (d) {
    case SE_DIRECTION_LEFT: return "LEFT";
    case SE_DIRECTION_RIGHT: return "RIGHT";
    default: return "NONE";
  }
}

int main(int argc, char **argv)
{
  const char *out_path = argc > 1 ? argv[1] : "p7_c.csv";
  const int scenarios = argc > 2 ? atoi(argv[2]) : 60;
  FILE *out = fopen(out_path, "w");
  int s;

  if (out == NULL) {
    fprintf(stderr, "cannot open %s\n", out_path);
    return 1;
  }
  hal_log_set_enabled(0); /* 与 C++ 侧 PARITY_LOG 缺省关闭一致 */

  for (s = 0; s < scenarios; ++s) {
    int protocol_idx, output_mode_idx, escape_scenario, gyro_pitch;
    int require_neutral, closed_loop_idx, gear_speeds_on;
    int writes_before;
    mb_bridge_t bridge;
    char err[256];
    mb_config_t config;
    int k;

    rng_seed(s);

    /* ---- 场景配置（rng 顺序固定，与 C++ 侧一致） ---- */
    protocol_idx = rng_int(0, 3);
    output_mode_idx = rng_unit() < 0.3 ? 1 : 0;
    escape_scenario = (rng_unit() < 0.3) ? 1 : 0;
    gyro_pitch = (rng_unit() < 0.3) ? 1 : 0;
    require_neutral = (rng_unit() < 0.5) ? 1 : 0;
    closed_loop_idx = (rng_unit() < 0.3) ? 1 : 0;
    gear_speeds_on = (rng_unit() < 0.4) ? 1 : 0;
    {
      const int invert_linear = (rng_unit() < 0.15) ? 1 : 0;
      const int invert_angular = (rng_unit() < 0.15) ? 1 : 0;
      const int axis_lock = (rng_unit() < 0.15) ? 1 : 0;
      const int controller_axis_lock = (rng_unit() < 0.15) ? 1 : 0;
      const int controller_invert_x = (rng_unit() < 0.15) ? 1 : 0;
      const int controller_invert_y = (rng_unit() < 0.15) ? 1 : 0;
      const int linear_profile_enabled = (rng_unit() < 0.3) ? 1 : 0;
      const int has_ccmd_topic = (rng_unit() < 0.5) ? 1 : 0;
      const int has_fb_topic = (rng_unit() < 0.5) ? 1 : 0;
      const int has_wy_topic = (rng_unit() < 0.5) ? 1 : 0;
      const int reverse_scenario = (rng_unit() < 0.25) ? 1 : 0;
      const int emergency_scenario = (rng_unit() < 0.2) ? 1 : 0;
      const int on_change_idle = (rng_unit() < 0.3) ? 1 : 0;

      mb_config_defaults(&config);
      config.dry_run = 1;
      {
        const char *prot = protocol_idx == 0 ? "2.0"
          : protocol_idx == 1 ? "2.1"
          : protocol_idx == 2 ? "2.3" : "gear_horn";
        strncpy(config.protocol_name, prot,
          sizeof(config.protocol_name) - 1);
        config.protocol = (mp_protocol_t)protocol_idx;
      }
      if (output_mode_idx == 1) {
        strncpy(config.output_mode, "intervention-only",
          sizeof(config.output_mode) - 1);
        strncpy(config.controller_cmd_topic, "/ctrl",
          sizeof(config.controller_cmd_topic) - 1);
      } else if (has_ccmd_topic) {
        strncpy(config.controller_cmd_topic, "/ctrl",
          sizeof(config.controller_cmd_topic) - 1);
      }
      if (has_fb_topic) {
        strncpy(config.feedback_topic, "/feedback",
          sizeof(config.feedback_topic) - 1);
      }
      if (has_wy_topic) {
        strncpy(config.wheel_yaw_topic, "/wheel_yaw",
          sizeof(config.wheel_yaw_topic) - 1);
      }
      config.invert_linear = invert_linear;
      config.invert_angular = invert_angular;
      config.axis_lock = axis_lock;
      config.controller_axis_lock = controller_axis_lock;
      config.controller_invert_x = controller_invert_x;
      config.controller_invert_y = controller_invert_y;
      config.controller_require_neutral = require_neutral;
      config.linear_profile_enabled = linear_profile_enabled;
      strncpy(config.gyro_angle_source,
        gyro_pitch ? "imu_pitch" : "zero",
        sizeof(config.gyro_angle_source) - 1);
      if (closed_loop_idx == 1) {
        strncpy(config.closed_loop_mode, "intervention",
          sizeof(config.closed_loop_mode) - 1);
      }
      if (gear_speeds_on) {
        static const double speeds[5] = {0.3, 0.45, 0.6, 0.75, 0.9};
        memcpy(config.gear_peak_speeds, speeds, sizeof(speeds));
        config.gear_peak_speeds_count = 5;
      }
      if (on_change_idle) {
        strncpy(config.idle_stop_mode, "on-change",
          sizeof(config.idle_stop_mode) - 1);
      }

      /* ---- 构造（时钟从 0 起；注入须在 init 前设置） ---- */
      g_now = 0.0;
      g_frame_count = 0;
      g_ccmd_count = 0;
      g_ccmd_lin = 0.0;
      g_ccmd_ang = 0.0;
      g_fb_count = 0;
      g_fb_x = 0.0;
      g_fb_y = 0.0;
      g_fb_gear = 0.0;
      g_wy_count = 0;
      g_wy_last = 0.0;

      memset(&bridge, 0, sizeof(bridge));
      bridge.inject.now_fn = test_now_fn;
      bridge.inject.sleep_fn = test_sleep_fn;
      bridge.inject.frame_sink = frame_record;
      bridge.inject.controller_cmd_sink = ccmd_record;
      bridge.inject.feedback_sink = fb_record;
      bridge.inject.wheel_yaw_sink = wy_record;

      if (mb_bridge_init(&bridge, &config, err, sizeof(err)) != 0) {
        fprintf(stderr, "scenario %d: bridge init failed: %s\n", s, err);
        fclose(out);
        return 1;
      }
      writes_before = g_frame_count;

      /* ---- tick 循环 ---- */
      for (k = 0; k < T_TICKS; ++k) {
        const double t = (k + 1) * TICK_SEC;
        const int phase = k / 25;
        g_now = t;

        /* 1) 控制器反馈样本 */
        if (rng_unit() < 0.85) {
          ms_sample_t sample;
          const int is_status = (rng_unit() < 0.9) ? 1 : 0;
          int cx, cy, base;
          int left_raw, right_raw, gear;
          memset(&sample, 0, sizeof(sample));
          sample.kind = is_status ? MS_SAMPLE_KIND_STATUS
            : MS_SAMPLE_KIND_DRIVE;
          if (k < 3) {
            cx = 95 + rng_int(0, 10);
            cy = 95 + rng_int(0, 10);
          } else if (phase == 3 && escape_scenario) {
            cx = 95 + rng_int(0, 10);
            cy = 150 + rng_int(0, 40);
          } else if (phase == 3) {
            cx = 90 + rng_int(0, 20);
            cy = 90 + rng_int(0, 20);
          } else if (phase == 2 && reverse_scenario) {
            cx = 90 + rng_int(0, 20);
            cy = 40 + rng_int(0, 40);
          } else {
            cx = 90 + rng_int(0, 20);
            cy = 130 + rng_int(0, 50);
          }
          if (phase == 0) {
            base = 1000 + rng_int(0, 1000);
          } else if (phase == 1) {
            if (emergency_scenario) {
              base = 1850 + (k % 25) * 12 + rng_int(0, 60);
            } else {
              base = 100 + (24 - (k % 25)) * 60 + rng_int(0, 80);
            }
          } else if (phase == 2) {
            if (reverse_scenario) {
              base = -(200 + rng_int(0, 800));
            } else {
              base = 100 + rng_int(0, 300);
            }
          } else {
            base = rng_int(-30, 30);
          }
          left_raw = base + rng_int(-40, 40);
          right_raw = base + rng_int(-40, 40);
          if (rng_unit() < 0.25) {
            gear = rng_int(1, 5);
          } else {
            gear = -1;
          }
          sample.x = cx;
          sample.y = cy;
          sample.left_raw = left_raw;
          sample.right_raw = right_raw;
          sample.left_mps = (double)left_raw / 3600.0;
          sample.right_mps = (double)right_raw / 3600.0;
          sample.gear = gear;
          sample.gear_valid = (gear >= 1 && gear <= 5);
          sample.protocol = "";
          sample.time = t;
          mb_publish_sample(&bridge, &sample);
        }

        /* 2) DWA cmd_vel */
        if (phase == 0 || phase == 2) {
          if (rng_unit() < 0.6) {
            double lin;
            if (phase == 2 && reverse_scenario) {
              lin = -(0.1 + rng_unit() * 0.4);
            } else {
              lin = rng_unit() * 0.5;
            }
            {
              const double ang = (rng_unit() * 2.0 - 1.0) * 0.6;
              mb_on_cmd(&bridge, lin, ang);
            }
          }
        } else if (phase == 1) {
          if (rng_unit() < 0.5) {
            const double lin = (rng_unit() < 0.3) ? 0.0 : rng_unit() * 0.1;
            const double ang = (rng_unit() * 2.0 - 1.0) * 0.4;
            mb_on_cmd(&bridge, lin, ang);
          }
        } else { /* phase 3 */
          if (!escape_scenario && rng_unit() < 0.4) {
            const double lin = rng_unit() * 0.2;
            const double ang = (rng_unit() * 2.0 - 1.0) * 0.3;
            mb_on_cmd(&bridge, lin, ang);
          }
        }

        /* 3) 霍尔速度控制 */
        {
          const double prob = (phase == 1) ? 1.0 : 0.1;
          if (rng_unit() < prob) {
            float data[8];
            const int size = 3 + rng_int(0, 5);
            const int hard_stop = (rng_unit() < 0.02) ? 1 : 0;
            data[0] = (phase == 1 || rng_unit() < 0.5) ? 1.0f : 0.0f;
            data[1] = (phase == 1 && emergency_scenario)
              ? 0.55f
              : (float)(0.1 + rng_unit() * 0.4);
            data[2] = (float)((phase == 1)
              ? (emergency_scenario ? 2 : rng_int(1, 5))
              : rng_int(3, 4));
            data[3] = (float)(0.3 + rng_unit() * 0.2);
            data[4] = (float)(40.0 + rng_unit() * 80.0);
            data[5] = (float)rng_int(0, 6);
            data[6] = (rng_unit() < 0.3) ? 1.0f : 0.0f;
            data[7] = hard_stop ? 1.0f : 0.0f;
            mb_on_hall_speed_control(&bridge, data, (size_t)size);
          }
        }

        /* 4) 逃逸点云 */
        if (rng_unit() < 0.5) {
          const int n = rng_int(0, 16);
          const int point_step = 8;
          uint8_t buf[16 * 8];
          mb_cloud_field_t fields[2];
          mb_cloud_view_t view;
          int i;
          memset(buf, 0, sizeof(buf));
          for (i = 0; i < n; ++i) {
            const double px = (rng_unit() * 2.0 - 1.0) * 0.8;
            const double py = (rng_unit() * 2.0 - 1.0) * 1.2;
            const float fx = (float)px;
            const float fy = (float)py;
            memcpy(&buf[(size_t)i * point_step + 0], &fx, 4);
            memcpy(&buf[(size_t)i * point_step + 4], &fy, 4);
          }
          strncpy(fields[0].name, "x", sizeof(fields[0].name) - 1);
          fields[0].datatype = 7; /* FLOAT32 */
          fields[0].offset = 0;
          strncpy(fields[1].name, "y", sizeof(fields[1].name) - 1);
          fields[1].datatype = 7;
          fields[1].offset = 4;
          memset(&view, 0, sizeof(view));
          view.data = buf;
          view.data_len = (size_t)n * point_step;
          view.width = n;
          view.height = 1;
          view.point_step = point_step;
          view.row_step = point_step * n;
          view.big_endian = 0;
          view.fields = fields;
          view.field_count = 2;
          mb_on_escape_cloud(&bridge, &view);
        }

        /* 5) IMU */
        if (gyro_pitch && rng_unit() < 0.8) {
          const double ax = (rng_unit() * 2.0 - 1.0) * 2.0;
          const double ay = (rng_unit() * 2.0 - 1.0) * 2.0;
          const double az = 9.8 + (rng_unit() * 2.0 - 1.0) * 0.5;
          mb_on_imu(&bridge, ax, ay, az);
        }

        /* 6) 喇叭 */
        if (rng_unit() < 0.1) {
          const int horn = (rng_unit() < 0.5) ? 1 : 0;
          mb_on_horn(&bridge, horn);
        }

        /* 7) 雷达扫描心跳 */
        if (rng_unit() < 0.3) {
          mb_on_left_scan(&bridge);
        }
        if (rng_unit() < 0.3) {
          mb_on_right_scan(&bridge);
        }

        /* ---- tick ---- */
        mb_on_timer(&bridge);

        /* ---- CSV 行 ---- */
        {
          const int total = g_frame_count;
          const int n_this = total - writes_before;
          char hexbuf[4096];
          frames_hex_range(writes_before, total, hexbuf, sizeof(hexbuf));
          fprintf(out,
            "p7,%d,%d,%d,%d,%s,"
            "%d,%.9f,%.9f,"
            "%d,%.9f,%.9f,%.9f,"
            "%d,%.9f,"
            "%.9f,%d,%d,%d,%.9f,"
            "%d,%d,%d,"
            "%d,%d,"
            "%.9f,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,"
            "%.9f,"
            "%d,%d,"
            "%d,%s,%.9f,%.9f,"
            "%d,%d,%.9f,"
            "%.9f,%.9f,%.9f,"
            "%.9f,%s\n",
            s, k, total, n_this, hexbuf,
            g_ccmd_count,
            g_ccmd_count > 0 ? g_ccmd_lin : 0.0,
            g_ccmd_count > 0 ? g_ccmd_ang : 0.0,
            g_fb_count,
            g_fb_count > 0 ? g_fb_x : 0.0,
            g_fb_count > 0 ? g_fb_y : 0.0,
            g_fb_count > 0 ? g_fb_gear : 0.0,
            g_wy_count,
            g_wy_count > 0 ? g_wy_last : 0.0,
            bridge.hall_speed_controller.output_y,
            bridge.hall_speed_controller.initialized,
            bridge.hall_speed_control_active,
            bridge.hall_speed_control_mode,
            bridge.hall_speed_control_target,
            bridge.hall_speed_release_active,
            bridge.hall_speed_fault_wait_neutral,
            bridge.hard_stop_active,
            bridge.controller_gear_valid ? bridge.controller_gear : 0,
            bridge.output_gear_valid ? bridge.output_gear : 0,
            bridge.last_raw_linear, bridge.last_raw_angular,
            bridge.last_linear, bridge.last_angular,
            bridge.actual_linear, bridge.actual_yaw_rate,
            bridge.raw_yaw_rate,
            bridge.wheel_yaw_total,
            bridge.last_controller_x, bridge.last_controller_y,
            bridge.stop_escape_last_decision.active,
            se_dir_str(bridge.stop_escape_last_decision.direction),
            bridge.stop_escape_last_decision.left_clearance,
            bridge.stop_escape_last_decision.right_clearance,
            bridge.intervention_motion_active,
            bridge.was_motion,
            bridge.safe_stop_hold_until,
            bridge.closed_loop_angular_i,
            bridge.intervention_slew_linear,
            bridge.intervention_slew_angular,
            bridge.linear_output_profile.output_y,
            bridge.linear_output_profile_kind);
          writes_before = total;
        }
      }

      /* ---- 析构（stop_and_close 停止帧串也计入观测面） ---- */
      mb_bridge_destroy(&bridge);
      {
        const int total = g_frame_count;
        char hexbuf[4096];
        frames_hex_range(writes_before, total, hexbuf, sizeof(hexbuf));
        fprintf(out, "p7d,%d,%d,%s\n", s, total, hexbuf);
      }
    }
  }

  fclose(out);
  fprintf(stderr, "test_p7: %d scenarios done\n", scenarios);
  return 0;
}
