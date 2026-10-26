/*
 * 模块功能：单进程整机入口（P9）——把生产 ROS 流水线 10 个进程收编为
 * 一个进程：左右 cspc 雷达驱动 → 激光滤波 → 融合 → DWA 规划 → 电机桥
 * → /dev/ttyS6 串口控制。全部使用重构并通过对拍验收的 C 模块。
 *
 * 对应生产拓扑（start_dwa_motor_test.sh，逐项映射）：
 *   cspc_lidar_cpp ×2        → cspc_pair（SDK 双实例，每侧采集线程）
 *   laser_scan_preprocessor ×2 → dwa_filter_ranges（滤波，直译 P5）
 *   laser_scan_fusion         → dwa_fusion_append_scan（融合，直译 P5）
 *   tf_base/tf_left/tf_right  → 省略（仅 RViz 可视化，无控制语义）
 *   dwa_local_planner_cpp     → dwa_node（直译 P2~P6，bit-exact）
 *   motor_xy_cmd_vel_node_cpp → mb_bridge（直译 P7，bit-exact）
 *
 * 线程模型（对齐 ROS 单线程 executor 语义）：
 *   左/右雷达线程：cspc_pair_poll（阻塞整圈）→ 滤波 → 存侧扫描 →
 *                  触发融合（任一侧新到即融合，同 ROS 融合节点回调语义）
 *   桥线程（20ms）：事件队列排空（mb_on_* 全部在本线程，串行化对齐
 *                  C++ 单线程 executor）→ mb_on_timer（内含串口轮询）
 *   主线程（100ms）：dwa_node_tick + 基线同款打印（含 [循环计时]）
 *
 * 数据接线（与 ROS topic 一一对应）：
 *   /scan_left_raw /scan_right_raw      → 滤波（本进程内）
 *   /dwa/scan_left/right_filtered       → 融合输入 + mb_on_left/right_scan 心跳
 *   /dwa/fused_scan_base                → dwa_node_on_fused_cloud +
 *                                          mb_on_escape_cloud（桥的逃逸点云
 *                                          订阅的就是融合输出，字节布局同构）
 *   /cmd_vel_raw（桥解析 0x88 手柄发布）→ dwa_node_on_cmd
 *   /dwa/motor_feedback                 → dwa_node_on_motor_feedback
 *   /dwa/cmd_vel_candidate + hall_speed → mb_on_cmd / mb_on_hall_speed_control
 *   /odom                               → 生产链路无发布者，不注入（同现状）
 *   IMU                                 → 生产桥无 imu 订阅，不注入（同现状）
 *
 * 诊断输出：控制链路写入结构化 CSV，融合点云写入带序号的二进制帧，
 * 避免高频人类汇总文本掩盖决策与最终电机帧之间的对应关系。
 *
 * 安全：雷达数据就绪前不开电机串口（对齐生产"桥最后启动"原则）；
 * mb 串口带 flock——若 ROS 流水线仍在跑会打开失败并退出，杜绝双驱。
 *
 * 仅 POSIX 构建。用法：
 *   robot_main [dwa_planner.yaml] [--filter-yaml lidar_filter.yaml]
 *             [--motor-port /dev/ttyS6] [--left-port /dev/ttyS4]
 *             [--right-port /dev/ttyS5] [--dry-run]
 */
#define _GNU_SOURCE /* pthread_setaffinity_np / CPU_SET 需要（基线同款） */
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#ifdef __linux__
#include <sched.h>
#include <unistd.h> /* _exit */
#endif

#include "../src/common/dwa_geometry.h"
#include "../src/config/dwa_params.h"
#include "../src/config/perception_params.h"
#include "../src/drivers/cspc_lidar_pair.h"
#include "../src/drivers/bmi270.h"
#include "../src/hal/hal_log.h"
#include "../src/hal/hal_time.h"
#include "../src/motor/motor_bridge.h"
#include "../src/motor/motor_bridge_config.h"
#include "../src/motor/yaw_feedback.h"
#include "../src/perception/laser_filter_core.h"
#include "../src/perception/laser_scan_fusion.h"
#include "../src/runtime/dwa_node.h"
#include "../src/safety/front_safety.h"
#include "../src/fusion/dynamic_velocity_fusion.h"
#include "../../soft-point/udp_FNC.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================ 容量与常量 ============================ */

#define RM_SCAN_MAX 16384    /* 单侧扫描点上限（同 SDK 适配层截断值） */
#define RM_FUSED_MAX 65536   /* 融合点上限（双侧 × 截断值） */

/* 前方探测区（基线 predictive_FNC.c 同款几何，单位换算为本工程米制）：
 * 车头 x=0.50m、半宽 0.295m、两角向外张 5° 的梯形。 */
#define RM_CAR_FRONT_M 0.50
#define RM_CAR_HALF_WID_M 0.295
#define RM_FRONT_DETECT_ANGLE_DEG 5.0
/* 轮距（生产 WHEEL_TRACK_M=0.50，仅用于打印 wfb 轮差估计）。 */
#define RM_WHEEL_TRACK_M 0.50

/* ============================ 全局状态 ============================ */

static volatile sig_atomic_t g_stop = 0;

static dwa_node_t g_node;

typedef struct {
  float ranges[RM_SCAN_MAX];
  size_t count;
  float angle_min, angle_max, angle_increment;
  float range_min, range_max;
  int fresh;
  double updated_time;
  unsigned long frames;
  pthread_mutex_t mu;
} rm_side_t;

static rm_side_t g_side[2]; /* 0=left 1=right */

/* 滤波配置（lidar_filter.yaml 加载，两侧各自一份）。 */
static dwa_laser_filter_config_t g_filter_cfg[2];
static dwa_lidar_filter_params_t g_filter_params[2];

/* 融合外参（dwa_planner.yaml laser_scan_fusion 段）。 */
static dwa_fusion_mount_t g_mounts[2];

/* 融合输出（g_cloud_mu 保护；base 点直供 DWA，escape 字节供桥解析）。 */
static pthread_mutex_t g_cloud_mu;
static dwa_fusion_point_t g_fused[RM_FUSED_MAX];
static size_t g_fused_count;
static dwa_fused_base_point_t g_base_points[RM_FUSED_MAX * 2 + 64];
static dwa_fused_base_point_t g_front_safety_candidates[RM_FUSED_MAX];
static size_t g_front_safety_candidate_count;
/* 9.28 Fix D：跨车身侧墙救援缓冲——真实侧墙点归还给规划器
 * （进 DWA 障碍表+侧净距），不进逃逸点云/正前方距离。
 * Fix H 起救援通道退役（跨车身点全部直通主云），恒为空。 */
static dwa_fusion_point_t g_side_flank[64];
static size_t g_side_flank_count;
static uint8_t g_escape_data[RM_FUSED_MAX * 16];
static double g_fwd_gap = 1e9; /* 正前方距离 m（基线语义：无障 1e9） */

/* ---- A4(10.09)：IMU 角速度闭环补偿状态（ltw 基线方案移植） ---- */
static yaw_feedback_params_t g_yf_params;   /* 补偿参数（ltw 实车标定值） */
static yaw_feedback_state_t g_yf_state;     /* 补偿跨拍状态（积分/斜坡锚） */
static yaw_feedback_out_t g_yf_diag;        /* 最近一拍诊断量 */
static int g_imu_ready = 0;   /* bmi270 初始化成功（0=补偿整体旁路） */
static int g_imu_ok = 0;      /* 本拍 gz 有效（含失败宽限/卡零检测） */
static int g_imu_fail = 0;    /* 连续读取失败计数（≤15 拍沿用滤波值） */
static int g_gz_stuck = 0;    /* gz 卡零而轮差明显在转的计数 */
static float g_gz_sign = 1.0f;/* gz 符号约定（yaml imu_gz_sign） */
static float g_gz_filt = 0.0f;/* 陀螺 z 轴 EMA（α=0.35@10Hz，ltw 同款） */
static int g_gz_filt_init = 0;
static float g_wfb_filt = 0.0f;   /* IMU 不可用时轮差兜底 EMA */
static int g_wfb_filt_init = 0;
static float g_w_meas = 0.0f;     /* 本拍实测角速度（gz 或轮差兜底） */
static float g_v_meas = 0.0f;     /* 本拍实测线速度 m/s */
static float g_theta_joy = 0.0f;  /* 手柄转向角 rad（积分器换向清零用） */
static float g_w_max = 1.40f;     /* MOVING_DWA 角速度限幅（params.max_yawrate） */
static int g_yf_applied = 0;      /* 最近一次发布是否应用了补偿 */
static float g_yf_last_w = 0.0f;  /* 最近一次补偿后输出（窗心写回用） */

/* 桥事件队列（生产者：主线程/雷达线程；消费者：桥线程）。 */
typedef enum {
  RM_EV_CMD,
  RM_EV_HALL,
  RM_EV_SCAN_L,
  RM_EV_SCAN_R,
  RM_EV_ESCAPE,
  RM_EV_ESCAPE_IMU
} rm_ev_type_t;

typedef struct {
  rm_ev_type_t type;
  double linear_x, angular_z;
  int imu_valid;
  float hall[8];
} rm_event_t;

#define RM_EVQ_CAP 256
static rm_event_t g_evq[RM_EVQ_CAP];
static int g_ev_head = 0, g_ev_tail = 0;
static pthread_mutex_t g_evq_mu;

static mb_bridge_t g_bridge;
static int g_bridge_inited = 0;
static volatile int g_bridge_started = 0;
static pthread_t g_bridge_thread;
static pthread_t g_bridge_bootstrap_thread;
static const char *g_motor_port = "/dev/ttyS6";
static int g_dry_run = 0;

/* RM_DEBUG_CLOUD=1：每 20 帧打印原始/滤波近距光束与融合前方区域点，
 * 用于定位幻影近点（P9 排障，不影响数据面）。 */
static int g_debug_cloud = 0;
/* RM_UDP_POINT_DEBUG=1：把滤波后、车体自过滤前的统一融合点云
 * 按 soft-point UDP 协议发给已连接的 DCJProject 可视化工具。 */
static int g_udp_point_debug = 0;
static int g_udp_point_ready = 0;

/* Runtime traces.  The binary cloud trace stores exactly the point array fed
 * to dwa_node_on_fused_cloud(), including the safety-only flags, so command
 * decisions can be replayed against the same cloud instead of a later UDP
 * snapshot. */
#define RM_CLOUD_TRACE_MAGIC 0x524d434cU /* "RMCL" */
#define RM_CLOUD_TRACE_VERSION 1U
#define RM_CLOUD_TRACE_DEFAULT_MAX_MB 32U

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t point_size;
  uint64_t sequence;
  uint64_t monotonic_us;
  uint64_t wall_us;
  uint32_t primary_count;
  uint32_t total_count;
} rm_cloud_trace_header_t;

typedef struct __attribute__((packed)) {
  float x;
  float y;
  uint8_t source;
  uint8_t flags; /* bit0=side_safety_only, bit1=front_safety_only */
  uint8_t reserved[2];
} rm_cloud_trace_point_t;

static FILE *g_cloud_trace = NULL;
static FILE *g_control_trace = NULL;
static char g_cloud_trace_path[512];
static size_t g_cloud_trace_max_bytes = RM_CLOUD_TRACE_DEFAULT_MAX_MB * 1024U * 1024U;
static size_t g_cloud_trace_bytes = 0;
static uint64_t g_cloud_sequence = 0;
static double g_cloud_last_mono = 0.0;
static uint64_t g_cloud_last_wall_us = 0;
static size_t g_cloud_last_primary_count = 0;
static size_t g_cloud_last_total_count = 0;

/* ============================ 小工具 ============================ */

static double rm_now_sec(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
}

static uint64_t rm_wall_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static size_t rm_trace_max_bytes(void)
{
  const char *value = getenv("RM_POINTCLOUD_TRACE_MAX_MB");
  char *end = NULL;
  unsigned long mb;
  if (value == NULL || *value == '\0') {
    return RM_CLOUD_TRACE_DEFAULT_MAX_MB * 1024U * 1024U;
  }
  mb = strtoul(value, &end, 10);
  if (end == value || *end != '\0' || mb == 0UL || mb > 1024UL) {
    return RM_CLOUD_TRACE_DEFAULT_MAX_MB * 1024U * 1024U;
  }
  return (size_t)mb * 1024U * 1024U;
}

static void rm_open_runtime_traces(void)
{
  const char *cloud_path = getenv("RM_POINTCLOUD_TRACE");
  const char *control_path = getenv("RM_CONTROL_TRACE");

  g_cloud_trace_max_bytes = rm_trace_max_bytes();
  if (cloud_path != NULL && *cloud_path != '\0') {
    strncpy(g_cloud_trace_path, cloud_path, sizeof(g_cloud_trace_path) - 1);
    g_cloud_trace_path[sizeof(g_cloud_trace_path) - 1] = '\0';
    g_cloud_trace = fopen(g_cloud_trace_path, "wb");
    if (g_cloud_trace == NULL) {
      fprintf(stderr, "[trace] 无法打开点云文件: %s\n", g_cloud_trace_path);
    }
  }
  if (control_path != NULL && *control_path != '\0') {
    g_control_trace = fopen(control_path, "w");
    if (g_control_trace == NULL) {
      fprintf(stderr, "[trace] 无法打开控制链路文件: %s\n", control_path);
    } else {
      fprintf(g_control_trace,
        "tick,mono_sec,wall_us,cloud_seq,cloud_mono_sec,cloud_age_sec,"
        "cloud_primary,cloud_total,published,intent_v,intent_w,raw_v,raw_w,"
        "selected_v,selected_w,published_v,published_w,path,valid,takeover,"
        "front,left,right,escape_active,escape_events,escape_yaw,escape_reason,"
        "tick_wall_us,tick_cpu_us\n");
      fflush(g_control_trace);
    }
  }
}

static void rm_close_runtime_traces(void)
{
  if (g_cloud_trace != NULL) {
    fflush(g_cloud_trace);
    fclose(g_cloud_trace);
    g_cloud_trace = NULL;
  }
  if (g_control_trace != NULL) {
    fflush(g_control_trace);
    fclose(g_control_trace);
    g_control_trace = NULL;
  }
}

static void rm_rotate_cloud_trace(void)
{
  char rotated[sizeof(g_cloud_trace_path) + 8];
  if (g_cloud_trace == NULL || g_cloud_trace_path[0] == '\0') return;
  fclose(g_cloud_trace);
  g_cloud_trace = NULL;
  snprintf(rotated, sizeof(rotated), "%s.1", g_cloud_trace_path);
  (void)remove(rotated);
  (void)rename(g_cloud_trace_path, rotated);
  g_cloud_trace = fopen(g_cloud_trace_path, "wb");
  g_cloud_trace_bytes = 0;
}

static void rm_save_fused_cloud_trace(
  const dwa_fused_base_point_t *points, size_t primary_count, size_t count)
{
  rm_cloud_trace_header_t header;
  size_t i;
  const size_t record_bytes = sizeof(header) + count * sizeof(rm_cloud_trace_point_t);

  if (g_cloud_trace == NULL || points == NULL || count == 0) return;
  if (g_cloud_trace_bytes > 0 &&
      g_cloud_trace_bytes + record_bytes > g_cloud_trace_max_bytes) {
    rm_rotate_cloud_trace();
    if (g_cloud_trace == NULL) return;
  }
  memset(&header, 0, sizeof(header));
  header.magic = RM_CLOUD_TRACE_MAGIC;
  header.version = RM_CLOUD_TRACE_VERSION;
  header.point_size = (uint16_t)sizeof(rm_cloud_trace_point_t);
  header.sequence = g_cloud_sequence;
  header.monotonic_us = (uint64_t)(g_cloud_last_mono * 1000000.0);
  header.wall_us = g_cloud_last_wall_us;
  header.primary_count = (uint32_t)primary_count;
  header.total_count = (uint32_t)count;
  if (fwrite(&header, sizeof(header), 1, g_cloud_trace) != 1) return;
  for (i = 0; i < count; ++i) {
    rm_cloud_trace_point_t item;
    memset(&item, 0, sizeof(item));
    item.x = points[i].point.x;
    item.y = points[i].point.y;
    item.source = points[i].is_left ? 0U : 1U;
    item.flags = (uint8_t)((points[i].side_safety_only ? 1U : 0U) |
      (points[i].front_safety_only ? 2U : 0U));
    if (fwrite(&item, sizeof(item), 1, g_cloud_trace) != 1) return;
  }
  g_cloud_trace_bytes += record_bytes;
  g_cloud_last_primary_count = primary_count;
  g_cloud_last_total_count = count;
  fflush(g_cloud_trace);
}

static void rm_write_control_trace(const dwa_velocity_t *intent,
  int published, double tick_wall_us, double tick_cpu_us)
{
  const double now = hal_steady_now_sec();
  const double cloud_age = g_cloud_last_mono > 0.0
    ? now - g_cloud_last_mono : -1.0;
  const char *escape_reason =
    (g_bridge.stop_escape_last_decision.reason != NULL) ?
      g_bridge.stop_escape_last_decision.reason : "";

  if (g_control_trace == NULL || intent == NULL) return;
  fprintf(g_control_trace,
    "%llu,%.6f,%llu,%llu,%.6f,%.6f,%zu,%zu,%d,%.6f,%.6f,%.6f,%.6f,"
    "%.6f,%.6f,%.6f,%.6f,%s,%d,%d,%.6f,%.6f,%.6f,%d,%llu,%.6f,%s,"
    "%.0f,%.0f\n",
    (unsigned long long)g_node.tick_count, now,
    (unsigned long long)rm_wall_us(),
    (unsigned long long)g_cloud_sequence, g_cloud_last_mono, cloud_age,
    g_cloud_last_primary_count, g_cloud_last_total_count, published,
    (double)intent->v, (double)intent->w,
    (double)g_node.diag_raw_dwa_v, (double)g_node.diag_raw_dwa_w,
    (double)g_node.diag_selected_v, (double)g_node.diag_selected_w,
    (double)g_node.last_published_cmd.linear_x,
    (double)g_node.last_published_cmd.angular_z,
    g_node.diag_path != NULL ? g_node.diag_path : "", g_node.diag_valid_count,
    g_node.diag_takeover, (double)g_node.front_unclipped_clearance_this_tick,
    (double)g_node.diag_side_clear_left,
    (double)g_node.diag_side_clear_right,
    g_bridge.stop_escape_last_decision.active ? 1 : 0,
    (unsigned long long)g_bridge.stop_escape_event_count,
    g_bridge.stop_escape_last_decision.yawrate, escape_reason,
    tick_wall_us, tick_cpu_us);
  fflush(g_control_trace);
}

static void rm_sleep_ms(double ms)
{
  struct timespec ts;
  if (ms <= 0.0) return;
  ts.tv_sec = (time_t)(ms / 1000.0);
  ts.tv_nsec = (long)((ms - (double)ts.tv_sec * 1000.0) * 1e6);
  nanosleep(&ts, NULL);
}

static void rm_post_event(const rm_event_t *ev)
{
  pthread_mutex_lock(&g_evq_mu);
  {
    const int next = (g_ev_head + 1) % RM_EVQ_CAP;
    if (next != g_ev_tail) { /* 满：丢最旧（病态负载下可见，正常不发生） */
      g_evq[g_ev_head] = *ev;
      g_ev_head = next;
    } else {
      g_evq[g_ev_tail] = *ev;
      g_ev_tail = (g_ev_tail + 1) % RM_EVQ_CAP;
    }
  }
  pthread_mutex_unlock(&g_evq_mu);
}

static int rm_pop_event(rm_event_t *ev)
{
  int got = 0;
  pthread_mutex_lock(&g_evq_mu);
  if (g_ev_tail != g_ev_head) {
    *ev = g_evq[g_ev_tail];
    g_ev_tail = (g_ev_tail + 1) % RM_EVQ_CAP;
    got = 1;
  }
  pthread_mutex_unlock(&g_evq_mu);
  return got;
}

/* ============================ 前方距离（基线同款几何） ============================ */

/* Fix H(9.28)：自车足迹判定——与 DWA 感知层 inside_footprint_self_filter
 * 完全同几何（主体矩形 + 脚踏/脚尖连续梯形 + padding）。供融合后主云
 * 过滤使用：DWA 注入点在感知层还会再滤一遍（幂等），逃逸点云/正前方
 * 距离则依赖这一层删除自身结构反射（近距钳位自反点/脚踏回波），对齐
 * 9.11-ok 基线 body_mask 语义（基线融合零过滤，自滤只在足迹层做）。 */
static bool rm_point_inside_self_footprint(float x, float y)
{
  const dwa_footprint_t *f = &g_node.config.footprint;
  const float padding =
    g_node.params_snapshot.footprint_self_filter_padding > 0.0f ?
    g_node.params_snapshot.footprint_self_filter_padding : 0.0f;
  if (dwa_inside_footprint_rectangle(
      x, y, -f->back, f->front, f->half_width, padding)) {
    return true;
  }
  {
    const float trapezoid_start_x = f->front;
    const float trapezoid_shoulder_x = f->footrest_start;
    const float trapezoid_end_x = f->toe_front;
    float half_width = f->toe_half_width;
    if (trapezoid_end_x <= trapezoid_start_x ||
      trapezoid_shoulder_x < trapezoid_start_x ||
      trapezoid_shoulder_x > trapezoid_end_x)
    {
      return false;
    }
    if (x <= trapezoid_shoulder_x) {
      const float span = trapezoid_shoulder_x - trapezoid_start_x;
      const float ratio = span > 0.0f ?
        (x - trapezoid_start_x) / span : 1.0f;
      half_width = f->half_width + ratio *
        (f->footrest_half_width - f->half_width);
    }
    return x >= trapezoid_start_x - padding &&
      x <= trapezoid_end_x + padding &&
      fabsf(y) <= half_width + padding;
  }
}

static double rm_front_gap(const dwa_fusion_point_t *points, size_t count)
{
  const float tan_half =
    (float)tan(RM_FRONT_DETECT_ANGLE_DEG * M_PI / 180.0);
  float best = 1e9f;
  size_t i;
  for (i = 0; i < count; ++i) {
    const float x = points[i].x;
    const float y = points[i].y;
    float dx;
    if (x <= RM_CAR_FRONT_M) continue;
    dx = x - RM_CAR_FRONT_M;
    if (fabsf(y) >= RM_CAR_HALF_WID_M + dx * tan_half) continue;
    if (dx < best) best = dx;
  }
  return (double)best;
}

/* ============================ 融合（ROS 融合节点语义） ============================ */

/* 局部拷贝工作区（g_cloud_mu 下串行使用）。 */
static dwa_fusion_scan_t g_local_scan[2];
static float g_local_ranges[2][RM_SCAN_MAX];

/* soft-point 协议统一使用厘米：头部几何以 cm 存入 1 字节，点坐标以
 * cm 存入有符号 int16（大端）。协议最多承载 1022 个点，超出时等距抽样，
 * 保留整个视场覆盖，不改变实际规划点云。 */
static uint8_t rm_cm_u8(float meters)
{
  long cm = lroundf(meters * 100.0f);
  if (cm < 0) cm = 0;
  if (cm > 255) cm = 255;
  return (uint8_t)cm;
}

static void rm_send_pointcloud_debug(const dwa_fusion_point_t *points,
                                    size_t point_count)
{
  enum { HEADER_SIZE = 7, MAX_POINTS = (MAX_DATA_SIZE - HEADER_SIZE) / 4 };
  uint8_t payload[MAX_DATA_SIZE];
  size_t send_count, i;
  const dwa_planner_params_t *p = &g_node.params_snapshot;

  if (!g_udp_point_debug || !g_udp_point_ready || points == NULL) return;
  send_count = point_count;
  if (send_count > MAX_POINTS) send_count = MAX_POINTS;

  payload[0] = COM_PointCloudData_CMD;
  payload[1] = rm_cm_u8(p->footprint_front + p->footprint_back);
  payload[2] = rm_cm_u8(2.0f * p->footprint_half_width);
  payload[3] = rm_cm_u8(p->footrest_front_x - p->footprint_front);
  payload[4] = rm_cm_u8(2.0f * p->footrest_half_width);
  payload[5] = rm_cm_u8(g_mounts[0].x);
  payload[6] = rm_cm_u8(g_mounts[0].y);

  for (i = 0; i < send_count; ++i) {
    size_t src = i * point_count / send_count;
    long x_cm = lroundf(points[src].x * 100.0f);
    long y_cm = lroundf(points[src].y * 100.0f);
    uint16_t x, y;
    if (x_cm < -32768) x_cm = -32768;
    if (x_cm > 32767) x_cm = 32767;
    if (y_cm < -32768) y_cm = -32768;
    if (y_cm > 32767) y_cm = 32767;
    x = (uint16_t)(int16_t)x_cm;
    y = (uint16_t)(int16_t)y_cm;
    payload[HEADER_SIZE + i * 4 + 0] = (uint8_t)(x >> 8);
    payload[HEADER_SIZE + i * 4 + 1] = (uint8_t)x;
    payload[HEADER_SIZE + i * 4 + 2] = (uint8_t)(y >> 8);
    payload[HEADER_SIZE + i * 4 + 3] = (uint8_t)y;
  }
  (void)udp_sendData(payload, HEADER_SIZE + send_count * 4);
}

static void rm_fuse(void)
{
  int s;
  int both = 1;
  const double now = rm_now_sec();
  const double timeout = g_node.params_snapshot.scan_timeout_sec;

  pthread_mutex_lock(&g_cloud_mu);

  /* 必须两侧都在超时窗口内更新；不允许用一侧新扫描伪装整幅融合云新鲜。 */
  for (s = 0; s < 2; ++s) {
    size_t n;
    pthread_mutex_lock(&g_side[s].mu);
    if (!g_side[s].fresh || g_side[s].updated_time <= 0.0 ||
        now - g_side[s].updated_time > timeout) {
      both = 0;
    }
    n = g_side[s].count;
    if (n > RM_SCAN_MAX) n = RM_SCAN_MAX;
    memcpy(g_local_ranges[s], g_side[s].ranges, n * sizeof(float));
    g_local_scan[s].angle_min = g_side[s].angle_min;
    g_local_scan[s].angle_max = g_side[s].angle_max;
    g_local_scan[s].angle_increment = g_side[s].angle_increment;
    g_local_scan[s].range_min = g_side[s].range_min;
    g_local_scan[s].range_max = g_side[s].range_max;
    g_local_scan[s].ranges = g_local_ranges[s];
    g_local_scan[s].range_count = n;
    pthread_mutex_unlock(&g_side[s].mu);
  }
  if (!both) {
    pthread_mutex_unlock(&g_cloud_mu);
    return;
  }

  /* append_scan ×2（laser_scan_fusion publish_if_ready 直译）。 */
  g_fused_count = 0;
  if (!dwa_fusion_append_scan(&g_local_scan[0], &g_mounts[0], 0,
        g_fused, RM_FUSED_MAX, &g_fused_count) ||
      !dwa_fusion_append_scan(&g_local_scan[1], &g_mounts[1], 1,
        g_fused, RM_FUSED_MAX, &g_fused_count)) {
    fprintf(stderr, "[robot_main] 融合缓冲不足（points>=%zu）\n", g_fused_count);
  }
  /* 在自车足迹过滤前发送，显示滤波后融合输出及可能被自车过滤掉的点。 */
  rm_send_pointcloud_debug(g_fused, g_fused_count);

  /* 独立保存脚尖前向硬停候选：必须在通用自车足迹过滤之前提取。
   * 候选只进入安全净距统计，不进入 DWA 点云/栅格/走廊，因而不扩张
   * 弯道侧墙的前向停车包络。保留 toe 前 1 个 self-filter padding
   * 范围内的窄前向安全带点，之后由安全链使用 toe collision edge 计算。
   */
  g_front_safety_candidate_count = 0;
  {
    const dwa_front_safety_config_t front_zone = {
      g_node.params_snapshot.front_trigger_start_x,
      g_node.params_snapshot.front_trigger_base_width,
      g_node.params_snapshot.front_trigger_expand_angle_deg
    };
    size_t r, w = 0;
    for (r = 0; r < g_fused_count; ++r) {
      const float x = g_fused[r].x;
      const float y = g_fused[r].y;
      const float world_angle_deg = atan2f(y, x) * (float)(180.0 / M_PI);
      const dwa_point_t point = {x, y};
      /* 不再屏蔽雷达原点附近的 10 cm 区域，避免真实近障碍物被误删；
       * 保留既有后向扇区排除。 */
      if (world_angle_deg > 170.0f || world_angle_deg < -170.0f)
      {
        continue;
      }
      if (dwa_point_is_front_safety_candidate(
          &point, &front_zone, &g_node.config.footprint,
          g_node.params_snapshot.footprint_self_filter_padding) &&
        g_front_safety_candidate_count < RM_FUSED_MAX)
      {
        dwa_fused_base_point_t *candidate =
          &g_front_safety_candidates[g_front_safety_candidate_count++];
        candidate->point = point;
        candidate->is_left = (g_fused[r].source == 0);
        candidate->side_safety_only = 0;
        candidate->front_safety_only = 1;
      }
      if (rm_point_inside_self_footprint(x, y)) continue;
      g_fused[w++] = g_fused[r];
    }
    g_fused_count = w;
    g_side_flank_count = 0;
  }

  /* 同一遍生成：DWA 注入点 + 桥逃逸点云字节（生产 PointCloud2 布局：
   * point_step=16，x@0 y@4 z@8 FLOAT32 小端，source@12 UINT8）。
   * Fix H：跨车身点已直通主云，Fix D 救援尾部恒空（保留兼容）。 */
  {
    size_t i;
    const float zero = 0.0f;
    for (i = 0; i < g_fused_count; ++i) {
      g_base_points[i].point.x = g_fused[i].x;
      g_base_points[i].point.y = g_fused[i].y;
      g_base_points[i].is_left = (g_fused[i].source == 0);
      g_base_points[i].side_safety_only = 0;
      g_base_points[i].front_safety_only = 0;
      memcpy(g_escape_data + i * 16 + 0, &g_fused[i].x, 4);
      memcpy(g_escape_data + i * 16 + 4, &g_fused[i].y, 4);
      memcpy(g_escape_data + i * 16 + 8, &zero, 4);
      g_escape_data[i * 16 + 12] = g_fused[i].source;
    }
    for (i = 0; i < g_side_flank_count &&
         g_fused_count + i < RM_FUSED_MAX; ++i) {
      const size_t idx = g_fused_count + i;
      g_base_points[idx].point.x = g_side_flank[i].x;
      g_base_points[idx].point.y = g_side_flank[i].y;
      g_base_points[idx].is_left = (g_side_flank[i].source == 0);
      g_base_points[idx].side_safety_only = 1;
      g_base_points[idx].front_safety_only = 0;
    }
    for (i = 0; i < g_front_safety_candidate_count; ++i) {
      const size_t idx = g_fused_count + g_side_flank_count + i;
      g_base_points[idx] = g_front_safety_candidates[i];
    }
  }
  g_fwd_gap = rm_front_gap(g_fused, g_fused_count);

  /* Save the exact array handed to DWA, not the pre-self-filter UDP view. */
  g_cloud_sequence += 1;
  g_cloud_last_mono = hal_steady_now_sec();
  g_cloud_last_wall_us = rm_wall_us();
  g_cloud_last_primary_count = g_fused_count;
  g_cloud_last_total_count =
    g_fused_count + g_side_flank_count + g_front_safety_candidate_count;
  rm_save_fused_cloud_trace(g_base_points, g_fused_count,
    g_cloud_last_total_count);

  if (g_debug_cloud) {
    static unsigned fuse_seq = 0;
    if ((fuse_seq++ % 20) == 0) {
      size_t k;
      int shown = 0;
      printf("[fuse] seq=%u count=%zu fwd=%.4f\n",
        fuse_seq, g_fused_count, g_fwd_gap);
      for (k = 0; k < g_fused_count && shown < 10; ++k) {
        if (g_fused[k].x > 0.35f && g_fused[k].x < 0.80f &&
            fabsf(g_fused[k].y) < 0.40f) {
          printf("[fuse]   x=%.4f y=%.4f src=%d\n",
            (double)g_fused[k].x, (double)g_fused[k].y,
            (int)g_fused[k].source);
          ++shown;
        }
      }
    }
  }

  pthread_mutex_unlock(&g_cloud_mu);

  /* 注入 DWA（内部有锁，线程安全）。尾部为 Fix D 跨车身侧墙救援点。 */
  dwa_node_on_fused_cloud(&g_node, g_base_points,
    g_fused_count + g_side_flank_count + g_front_safety_candidate_count);

  /* 通知桥：融合点云到达（= 生产 /dwa/fused_scan_base 订阅回调）。 */
  {
    rm_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = RM_EV_ESCAPE;
    rm_post_event(&ev);
  }
}

/* ============================ 雷达采集线程 ============================ */

/* 每侧滤波工作区（仅本侧线程使用）。 */
static float s_raw_ranges[2][RM_SCAN_MAX];
static float s_filt_ranges[2][RM_SCAN_MAX];

static void rm_scan_callback(void *user, const cspc_laser_scan_t *scan)
{
  const int side = (int)(intptr_t)user;
  size_t n = scan->point_count;
  size_t i;
  dwa_laser_filter_stats_t stats;
  rm_event_t ev;

  if (n > RM_SCAN_MAX) n = RM_SCAN_MAX;

  /* node_lidar_ros.cpp publish_next_scan 直译：points → ranges 按下标直拷。
   * Fix H(9.28)：撤销本仓库 9.22 引入的 ≤0.15m→INFINITY 前置门限。
   * 该门限并非"原版 SDK 上游语义"（已核对 9.11-ok third_party SDK：
   * 全链无 0.15 门限，config.min_range=0.06 直通）。它把滤波层
   * "0<r<range_min 的有限回波钳位到 range_min 保留为危险点"的基线
   * 语义整个废掉——近距回波被当成"无回波/空闲空间"，删掉的恰是
   * "波束被腿/脚踏挡住"的证据，扩大窄通道物理盲区（右墙贴身 5~15cm
   * 时右雷达全盲，对侧雷达波束又被腿挡，规划器无从得知盲区存在）。
   * 恢复基线链路：自反点 raw≈0.05m → 滤波钳位 0.06m 危险点 → 变换
   * 后落在足迹内 → 由 rm_point_inside_self_footprint 删除（9.22 的
   * fwd≈0.07 幻影由足迹过滤兜底，不再需要源头门限）。 */
  for (i = 0; i < n; ++i) {
    s_raw_ranges[side][i] = scan->points[i].range;
  }

  /* laser_scan_preprocessor scan_callback 直译：整帧滤波。 */
  (void)dwa_filter_ranges(s_raw_ranges[side], n, scan->min_range,
    scan->max_range, &g_filter_cfg[side], s_filt_ranges[side], &stats);

  if (g_debug_cloud && (g_side[side].frames % 20) == 0) {
    size_t k;
    int shown = 0;
    printf("[cloud] side=%d n=%zu out=%zu inv=%zu near=%zu outl=%zu "
      "rmin=%.3f rmax=%.3f amin=%.4f ainc=%.6f\n",
      side, n, stats.output_count, stats.invalid_count,
      stats.near_range_clamped_count, stats.statistical_outlier_count,
      (double)scan->min_range, (double)scan->max_range,
      (double)scan->min_angle, (double)scan->angle_increment);
    for (k = 0; k < n && shown < 10; ++k) {
      if (isfinite(s_filt_ranges[side][k]) &&
          s_filt_ranges[side][k] < 0.8f) {
        printf("[cloud]   i=%zu ang=%.1fdeg raw=%.3f filt=%.3f\n",
          k,
          (scan->min_angle + (double)k * scan->angle_increment) * 180.0 / M_PI,
          (double)s_raw_ranges[side][k], (double)s_filt_ranges[side][k]);
        ++shown;
      }
    }
  }

  /* 存最新滤波扫描（= 发布 /dwa/scan_{left,right}_filtered）。 */
  pthread_mutex_lock(&g_side[side].mu);
  memcpy(g_side[side].ranges, s_filt_ranges[side], n * sizeof(float));
  g_side[side].count = n;
  g_side[side].angle_min = scan->min_angle;
  g_side[side].angle_max = scan->max_angle;
  g_side[side].angle_increment = scan->angle_increment;
  g_side[side].range_min = scan->min_range;
  g_side[side].range_max = scan->max_range;
  g_side[side].fresh = 1;
  g_side[side].updated_time = rm_now_sec();
  g_side[side].frames++;
  pthread_mutex_unlock(&g_side[side].mu);

  /* 桥扫描心跳（= 订阅回调 mb_on_left/right_scan）。 */
  memset(&ev, 0, sizeof(ev));
  ev.type = side == 0 ? RM_EV_SCAN_L : RM_EV_SCAN_R;
  rm_post_event(&ev);

  /* 融合（= 融合节点在任一侧订阅回调里 publish_if_ready）。 */
  rm_fuse();
}

static void *rm_lidar_thread(void *arg)
{
  const int side = (int)(intptr_t)arg;
  /* 不绑核（对齐生产：原 ROS 多进程链路无亲和设置；DWA 并行规划
   * worker 需要全核可用，绑核反而制造争用）。SDK 的 detached 读线程
   * 在 node_start 时创建；重处理（data_handling→滤波→融合）在本线程。 */
  while (!g_stop) {
    (void)cspc_pair_poll(side); /* 回调内完成滤波+融合 */
  }
  return NULL;
}

static int rm_wait_side_fresh(int side, double timeout_sec)
{
  const double deadline = rm_now_sec() + timeout_sec;
  while (rm_now_sec() < deadline) {
    int fresh = 0;
    if (g_stop) return -1;
    pthread_mutex_lock(&g_side[side].mu);
    fresh = g_side[side].fresh;
    pthread_mutex_unlock(&g_side[side].mu);
    if (fresh) return 0;
    rm_sleep_ms(50);
  }
  return -1;
}

/* ============================ 桥线程（mb_* 单线程串行化） ============================ */

static void rm_controller_cmd_sink(void *user, double linear_x, double angular_z)
{
  dwa_twist_t cmd;
  (void)user;
  dwa_twist_reset(&cmd);
  cmd.linear_x = (float)linear_x;
  cmd.angular_z = (float)angular_z;
  dwa_node_on_cmd(&g_node, &cmd); /* = 生产 /cmd_vel_raw 订阅回调 */
}

static void rm_feedback_sink(void *user, const ms_sample_t *sample)
{
  float data[7];
  (void)user;
  /* Float32MultiArray：x,y,left_raw,right_raw,left_mps,right_mps,gear。 */
  data[0] = (float)sample->x;
  data[1] = (float)sample->y;
  data[2] = (float)sample->left_raw;
  data[3] = (float)sample->right_raw;
  data[4] = (float)sample->left_mps;
  data[5] = (float)sample->right_mps;
  data[6] = (float)sample->gear;
  dwa_node_on_motor_feedback(&g_node, data, 7);
}

static void rm_drain_bridge_events(void)
{
  rm_event_t ev;
  while (rm_pop_event(&ev)) {
    switch (ev.type) {
    case RM_EV_CMD:
      mb_on_cmd(&g_bridge, ev.linear_x, ev.angular_z);
      break;
    case RM_EV_HALL:
      mb_on_hall_speed_control(&g_bridge, ev.hall, 8);
      break;
    case RM_EV_SCAN_L:
      mb_on_left_scan(&g_bridge);
      break;
    case RM_EV_SCAN_R:
      mb_on_right_scan(&g_bridge);
      break;
    case RM_EV_ESCAPE_IMU:
      mb_on_escape_imu_yawrate(&g_bridge, ev.angular_z, ev.imu_valid);
      break;
    case RM_EV_ESCAPE: {
      /* 生产语义：桥的逃逸点云订阅 = 融合输出（PointCloud2 布局）。 */
      static const mb_cloud_field_t fields[2] = {
        { "x", 7, 0 },
        { "y", 7, 4 }
      };
      mb_cloud_view_t view;
      size_t count;
      pthread_mutex_lock(&g_cloud_mu);
      count = g_fused_count;
      memset(&view, 0, sizeof(view));
      view.data = g_escape_data;
      view.data_len = count * 16;
      view.width = (int)count;
      view.height = 1;
      view.point_step = 16;
      view.row_step = (int)(count * 16);
      view.big_endian = 0;
      view.fields = fields;
      view.field_count = 2;
      mb_on_escape_cloud(&g_bridge, &view);
      pthread_mutex_unlock(&g_cloud_mu);
      break;
    }
    }
  }
}

static void *rm_bridge_thread(void *arg)
{
  double period_sec;
  double next_tick;
  unsigned beat = 0;
  (void)arg;
  period_sec = (double)g_bridge.config.interval_ms / 1000.0;
  next_tick = rm_now_sec() + period_sec;
  while (!g_stop) {
    const double now = rm_now_sec();
    if (now < next_tick) {
      rm_sleep_ms((next_tick - now) * 1000.0);
    }
    rm_drain_bridge_events();
    mb_on_timer(&g_bridge); /* 内含串口轮询（poll_status） */
    next_tick += period_sec;
    if (rm_now_sec() > next_tick) {
      next_tick = rm_now_sec() + period_sec;
    }
    if (g_debug_cloud && (++beat % 50) == 0) {
      const ms_rx_stats_t *rx = &g_bridge.writer.rx_stats;
      printf("[bridge] rx(bytes=%ld frames=%ld status=%ld drive=%ld "
        "bad_crc=%ld bad_ver=%ld dropped=%ld) wheel=(%.3f,%.3f) "
        "ctrl=(%d,%d) gear=%d hall_act=%d hard_stop=%d esc_act=%d "
        "esc_cnt=%llu\n",
        rx->bytes, rx->frames, rx->status, rx->drive, rx->bad_crc,
        rx->bad_version, rx->dropped,
        g_bridge.wheel_left_mps, g_bridge.wheel_right_mps,
        g_bridge.last_controller_x, g_bridge.last_controller_y,
        g_bridge.controller_gear, g_bridge.hall_speed_control_active,
        g_bridge.hard_stop_active,
        g_bridge.stop_escape_last_decision.active,
        (unsigned long long)g_bridge.stop_escape_event_count);
    }
  }
  return NULL;
}

/* ============================ 桥后台就绪线程 ============================ */

static void rm_exit(int code); /* 前置：引导线程先于定义调用 */

static int rm_build_bridge_argv(const char *motor_port, int dry_run,
  const char ***out_argv);

/* 9.24: 启动提速——主循环立即开跑（<1s 出打印），电机桥在本线程里等
 * 两侧雷达出数据后再初始化上线。安全语义不变：雷达数据就绪前电机
 * 串口不会被打开，桥未上线时控制器命令自然无去处（不会发车）。 */
static void *rm_bridge_bootstrap(void *arg)
{
  char err[256];
  int bridge_argc;
  const char **bridge_argv;
  (void)arg;

  if (rm_wait_side_fresh(0, 8.0) != 0) {
    fprintf(stderr, "fatal: 左雷达 8 秒内无数据\n");
    rm_exit(1);
  }
  if (rm_wait_side_fresh(1, 8.0) != 0) {
    fprintf(stderr, "fatal: 右雷达 8 秒内无数据\n");
    rm_exit(1);
  }

  bridge_argc = rm_build_bridge_argv(g_motor_port, g_dry_run, &bridge_argv);
  if (mb_config_parse(bridge_argc, (char **)bridge_argv, &g_bridge.config,
        err, sizeof(err)) != 0) {
    fprintf(stderr, "fatal: %s\n", err);
    rm_exit(1);
  }
  memset(&g_bridge.inject, 0, sizeof(g_bridge.inject));
  if (mb_bridge_init(&g_bridge, &g_bridge.config, err, sizeof(err)) != 0) {
    fprintf(stderr, "fatal: 电机桥初始化失败：%s\n"
      "（若 ROS 流水线仍在运行会占用串口，请先 systemctl stop hu-m40-dwa）\n",
      err);
    rm_exit(1);
  }
  g_bridge_inited = 1;
  mb_bridge_enable_sensor_interlock(&g_bridge, 1);
  mb_bridge_set_controller_cmd_sink(&g_bridge, rm_controller_cmd_sink, NULL);
  mb_bridge_set_feedback_sink(&g_bridge, rm_feedback_sink, NULL);

  g_bridge_started = 1;
  if (pthread_create(&g_bridge_thread, NULL, rm_bridge_thread, NULL) != 0) {
    g_bridge_started = 0;
    fprintf(stderr, "fatal: 桥线程创建失败\n");
    rm_exit(1);
  }
  return NULL;
}

/* ============================ DWA 输出 sink → 桥事件 ============================ */

/* A4(10.09)：IMU 采样 + 实测角速度融合（ltw feedback_dwa_step 的
 * 采样层逐项移植）。主循环每拍（10Hz）在 tick 前调用：
 *  - bmi270 读取成功且 |gz|<8（ sanity 上限）→ EMA 更新，失败计数清零
 *  - 连续失败 ≤15 拍沿用滤波值（1.5s 宽限），超时降级轮差兜底
 *  - 卡零检测：gz 恒 0 而轮差明显在转（>0.25rad/s）持续 30 拍 →
 *    判 IMU 实际不可用，强制降级（防"假装闭环"） */
static void rm_imu_poll(void)
{
  planner_motor_feedback_snapshot_t fb;
  float w_wheel;
  int imu_ok = 0;
  int imu_sample_fresh = 0;

  if (g_imu_ready) {
    Bmi270ImuSample imu;
    if (bmi270_read(&imu) == 0 && fabsf(imu.gz) < 8.0f) {
      const float gz = g_gz_sign * imu.gz;
      if (!g_gz_filt_init) {
        g_gz_filt = gz;
        g_gz_filt_init = 1;
      } else {
        g_gz_filt += 0.35f * (gz - g_gz_filt);
      }
      g_imu_fail = 0;
      imu_ok = 1;
      imu_sample_fresh = 1;
    } else {
      g_imu_fail++;
      if (g_gz_filt_init) {
        imu_ok = (g_imu_fail <= 15);
      }
    }
  }
  fb = dwa_node_motor_feedback_snapshot(&g_node);
  w_wheel = (fb.right_mps - fb.left_mps) / RM_WHEEL_TRACK_M;
  g_v_meas = 0.5f * (fb.left_mps + fb.right_mps);
  if (imu_ok && g_gz_filt_init &&
      fabsf(g_gz_filt) < 0.01f && fabsf(w_wheel) > 0.25f) {
    if (++g_gz_stuck > 30) {
      imu_ok = 0;
      g_imu_fail = 99;
    }
  } else {
    g_gz_stuck = 0;
  }
  if (imu_ok) {
    g_w_meas = g_gz_filt;
  } else {
    if (!g_wfb_filt_init) {
      g_wfb_filt = w_wheel;
      g_wfb_filt_init = 1;
    } else {
      g_wfb_filt += 0.30f * (w_wheel - g_wfb_filt);
    }
    g_w_meas = g_wfb_filt;
  }
  g_imu_ok = imu_ok;
  {
    rm_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = RM_EV_ESCAPE_IMU;
    ev.angular_z = imu_ok ? (double)g_gz_filt : 0.0;
    ev.imu_valid = imu_sample_fresh && imu_ok;
    rm_post_event(&ev);
  }
}

static void rm_cmd_sink(void *user, const dwa_twist_t *cmd,
  const dwa_hall_speed_control_msg_t *hall_control)
{
  rm_event_t ev;
  float w_out = cmd->angular_z;
  /* A4(10.09)：IMU 角速度闭环——仅在 MOVING_DWA 接管发布（hall
   * active、模式=3、非硬停）时对 w 做 P 阻尼+小积分+斜率限幅；
   * 其余链路（manual_bypass/hand_safe/hard_stop/掉线）原样透传，
   * 过渡瞬间复位补偿状态（积分器/斜坡锚点不带旧方向进手动）。
   * IMU 未就绪时整体旁路（行为与旧版逐字节一致）。 */
  const int comp_active = g_imu_ready &&
    hall_control->data[0] > 0.5f &&
    hall_control->data[7] < 0.5f &&
    (int)hall_control->data[2] == (int)DYNAMIC_CONTROL_MOVING_DWA;
  (void)user;
  if (comp_active) {
    const yaw_feedback_out_t yf = yaw_feedback_step(
      &g_yf_params, &g_yf_state, cmd->angular_z, g_w_meas,
      g_v_meas, g_w_max, g_theta_joy, g_imu_ok);
    w_out = yf.w_out;
    g_yf_diag = yf;
    g_yf_last_w = yf.w_out;
    g_yf_applied = 1;
  } else if (g_yf_applied) {
    yaw_feedback_reset(&g_yf_state);
    g_yf_applied = 0;
    g_yf_last_w = 0.0f;
  }
  memset(&ev, 0, sizeof(ev));
  ev.type = RM_EV_CMD;
  ev.linear_x = (double)cmd->linear_x;
  ev.angular_z = (double)w_out;
  rm_post_event(&ev);

  memset(&ev, 0, sizeof(ev));
  ev.type = RM_EV_HALL;
  memcpy(ev.hall, hall_control->data, sizeof(ev.hall));
  rm_post_event(&ev);

}

/* ============================ 信号 ============================ */

static void rm_on_signal(int sig)
{
  (void)sig;
  g_stop = 1;
}

/*
 * 直接 _exit：跳过静态析构与 atexit。SDK 双实例的 detached 读线程设计上
 * 无法停止，正常 return 会触发 C++ 静态析构与仍在运行的读线程竞态，
 * 板端实测表现为 "*** buffer overflow detected ***: terminated"
 * （P9）。先 fflush 保证行缓冲打印落盘。
 */
static void rm_exit(int code)
{
  fflush(NULL);
  _exit(code);
}

/* ============================ 桥配置（生产 argv 逐项同款） ============================ */

/*
 * 与 start_dwa_motor_test.sh 的 motor_bridge 启动参数一一对应。
 * 修改任一项前先同步生产脚本，保持两侧一致。
 */
static int rm_build_bridge_argv(const char *motor_port, int dry_run,
  const char ***out_argv)
{
  static const char *fixed[] = {
    "robot_main",
    "--protocol", "2.3",
    "--port", NULL, /* [4] motor_port */
    "--baud", "19200",
    "--cmd-topic", "/dwa/cmd_vel_candidate",
    "--controller-cmd-topic", "/cmd_vel_raw",
    "--output-mode", "continuous",
    "--max-forward-speed", "0.70",
    "--max-reverse-speed", "0.70",
    "--max-turn-speed", "1.40",
    /* A2(10.09)：Y 油门映射按档位峰值缩放（与 DWA 侧
     * gear_peak_speeds 同表）。无此项时映射退回 0.70 固定基准
     * = 原行为。 */
    "--gear-peak-speeds", "0.70,1.00,1.20,1.40,1.67",
    "--forward-output-deadzone-y", "120",
    "--controller-center-x", "100",
    "--controller-center-y", "100",
    "--controller-dead-low", "89",
    "--controller-dead-high", "111",
    "--hall-speed-control-topic", "/dwa/hall_speed_control",
    "--hall-speed-control-timeout", "0.25",
    "--hall-speed-y-gain", "300.0",
    "--hall-speed-y-down-rate", "40.0",
    "--hall-speed-y-up-rate", "30.0",
    "--hall-moving-hold-y-up-rate", "20.0",
    "--hall-moving-hold-max-y", "180.0",
    "--hall-moving-hold-emergency-speed", "0.52",
    "--hall-moving-hold-release-margin", "0.04",
    "--hall-speed-decel-predict-sec", "0.25",
    "--linear-profile-enabled",
    "--linear-profile-comfort-up-rate", "10.0",
    "--linear-profile-comfort-down-rate", "20.0",
    "--linear-profile-protective-up-rate", "10.0",
    "--linear-profile-protective-down-rate", "30.0",
    "--interval-ms", "20",
    "--horn-topic", "/wheelchair/horn_cmd",
    "--horn-timeout", "0.50",
    "--fallback-gear", "1",
    "--cmd-timeout", "0.50",
    "--left-scan-topic", "/dwa/scan_left_filtered",
    "--right-scan-topic", "/dwa/scan_right_filtered",
    "--feedback-topic", "/dwa/motor_feedback",
    "--wheel-yaw-topic", "/dwa/wheel_yaw_total",
    "--wheel-track-m", "0.50",
    "--wheel-yaw-max-dt", "0.15",
    "--escape-stop-hold-sec", "0.0",
    "--escape-turn-yawrate", "0.65",
    "--escape-half-width", "0.295",
    "--escape-max-duration-sec", "3.0",
    "--no-controller-require-neutral",
    /* 9.27 去掉干预路径角速度 slew：默认 0.9 rad/s²（每 0.1s 周期 w 最多
     * 变 0.09，0→1.4 需 1.55s），且只在 DWA 避障干预写帧时生效——实测
     * "角速度仍被限制"的主因。置 0 后规划器自身 w 窗口
     * （±max_d_yawrate*control_period = ±0.32/周期）成为唯一速率约束，
     * 响应与手柄直通路径（无 slew）一致。线速度 slew 0.6 m/s² 保留。
     * 注：刻意偏离 start_dwa_motor_test.sh 生产参数（该脚本无此项，
     * 走默认 0.9）；A/B 实测期临时差异，回滚删此两行即可。 */
    "--intervention-angular-slew-rate", "0",
    "--rx-debug"
  };
  static const char *argv_store[128];
  int n = (int)(sizeof(fixed) / sizeof(fixed[0]));
  int i;

  for (i = 0; i < n; ++i) {
    argv_store[i] = fixed[i];
  }
  argv_store[4] = motor_port;
  if (dry_run) {
    argv_store[n++] = "--dry-run";
  }
  *out_argv = argv_store;
  return n;
}

/* ============================ main ============================ */

int main(int argc, char **argv)
{
  const char *yaml_path = "config/dwa_planner.yaml";
  const char *filter_yaml = "config/lidar_filter.yaml";
  const char *motor_port = "/dev/ttyS6";
  const char *left_port = "/dev/ttyS4";
  const char *right_port = "/dev/ttyS5";
  int dry_run = 0;
  int i;
  int s;
  dwa_planner_params_t params;
  dwa_fusion_params_t fusion_params;
  pthread_t lidar_threads[2];
  struct timeval tv_start, tv_end;
  long loop_us;
  double period_sec;

  setvbuf(stdout, NULL, _IOLBF, 0); /* 管道下行缓冲（P8 教训） */
  /* RM_DEBUG_SPEED=1：打开速度模块每拍决策诊断（dwa_node_tick 的 INFO 行：
   * mode/front_clear 及触发点/实速/档位/target），用于压速类问题定位。 */
  if (getenv("RM_DEBUG_SPEED") != NULL) {
    hal_log_set_min_level(HAL_LOG_INFO);
  } else {
    hal_log_set_min_level(HAL_LOG_ERROR); /* 控制台只留基线同款打印+错误 */
  }
  g_debug_cloud = (getenv("RM_DEBUG_CLOUD") != NULL);
  g_udp_point_debug = (getenv("RM_UDP_POINT_DEBUG") != NULL &&
                       strcmp(getenv("RM_UDP_POINT_DEBUG"), "0") != 0);

  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--dry-run") == 0) {
      dry_run = 1;
    } else if (strcmp(argv[i], "--filter-yaml") == 0 && i + 1 < argc) {
      filter_yaml = argv[++i];
    } else if (strcmp(argv[i], "--motor-port") == 0 && i + 1 < argc) {
      motor_port = argv[++i];
    } else if (strcmp(argv[i], "--left-port") == 0 && i + 1 < argc) {
      left_port = argv[++i];
    } else if (strcmp(argv[i], "--right-port") == 0 && i + 1 < argc) {
      right_port = argv[++i];
    } else if (argv[i][0] != '-') {
      yaml_path = argv[i];
    }
  }

  signal(SIGINT, rm_on_signal);
  signal(SIGTERM, rm_on_signal);

  /* 桥后台引导线程用的全局配置。 */
  g_motor_port = motor_port;
  g_dry_run = dry_run;

  /* ---- 参数加载 ---- */
  if (dwa_planner_params_load(yaml_path, &params) != 0) {
    fprintf(stderr, "fatal: 无法读取 %s（使用默认参数继续）\n", yaml_path);
  }
  if (dwa_fusion_params_load(yaml_path, &fusion_params) != 0) {
    fprintf(stderr, "fatal: 无法读取融合段（%s），使用默认外参\n", yaml_path);
  }
  for (s = 0; s < 2; ++s) {
    if (dwa_lidar_filter_params_load(filter_yaml, s == 0,
          &g_filter_params[s]) != 0) {
      fprintf(stderr, "fatal: 无法读取 %s（%s 侧）\n", filter_yaml,
        s == 0 ? "left" : "right");
      return 1;
    }
    g_filter_cfg[s].window_size = g_filter_params[s].window_size;
    g_filter_cfg[s].stddev_multiplier = g_filter_params[s].stddev_multiplier;
    g_filter_cfg[s].min_valid_samples = g_filter_params[s].min_valid_samples;
    g_filter_cfg[s].min_cluster_size = g_filter_params[s].min_cluster_size;
    g_filter_cfg[s].max_neighbor_gap = g_filter_params[s].max_neighbor_gap;
  }
  g_mounts[0].x = fusion_params.left_laser_x;
  g_mounts[0].y = fusion_params.left_laser_y;
  g_mounts[0].yaw = fusion_params.left_laser_yaw;
  g_mounts[0].reversed = fusion_params.left_laser_angle_reversed;
  g_mounts[0].offset = fusion_params.left_laser_angle_offset;
  g_mounts[1].x = fusion_params.right_laser_x;
  g_mounts[1].y = fusion_params.right_laser_y;
  g_mounts[1].yaw = fusion_params.right_laser_yaw;
  g_mounts[1].reversed = fusion_params.right_laser_angle_reversed;
  g_mounts[1].offset = fusion_params.right_laser_angle_offset;

  if (g_udp_point_debug) {
    if (udp_FNC_INT() == 0) {
      g_udp_point_ready = 1;
      printf("[点云UDP] 已启用，监听 0.0.0.0:8080；在 DCJProject 中连接后发送融合点云\n");
    } else {
      fprintf(stderr, "[点云UDP] 初始化失败，点云可视化关闭\n");
      g_udp_point_debug = 0;
    }
  }
  rm_open_runtime_traces();

  /* 生产 -p 覆盖（start_dwa_motor_test.sh 的 DWA 节点参数）。 */
  params.max_reverse_velocity = 0.70f;
  params.max_yawrate = 1.40f;

  /* ---- DWA 节点 ---- */
  if (dwa_node_init(&g_node, &params) != 0) {
    fprintf(stderr, "fatal: DWA 节点初始化失败\n");
    return 1;
  }
  g_node.debug_nominal_speed = 0.0f;
  g_node.force_debug_intent = 0;
  g_node.publish_debug_visualization = 1;
  g_node.log_trajectory_scores = 0;
  dwa_node_set_cmd_sink(&g_node, rm_cmd_sink, NULL);

  /* ---- A4(10.09)：IMU 初始化（失败不致命——补偿自动旁路） ---- */
  yaw_feedback_reset(&g_yf_state);
  g_yf_params = yaw_feedback_default_params();
  /* A4b(10.09)：补偿强度 yaml 覆盖（加载器已保证 >0 有效值；
   * yaw_feedback.c 默认与 yaml 缺键兜底同值，此处仅搬运） */
  g_yf_params.gain_follow = params.yaw_feedback_gain_follow;
  g_yf_params.gain_kill = params.yaw_feedback_gain_kill;
  g_yf_params.fight_margin = params.yaw_feedback_fight_margin;
  g_yf_params.integ_max_frac = params.yaw_feedback_integ_max_frac;
  g_yf_params.integ_release = params.yaw_feedback_integ_release;
  g_yf_params.ref_w_max = params.yaw_feedback_ref_w_max;
  g_yf_params.manual_slew_min_theta = params.yaw_feedback_manual_slew_min_theta;
  g_w_max = params.max_yawrate;
  g_gz_sign = params.imu_gz_sign != 0.0f ? params.imu_gz_sign : 1.0f;
  if (params.yaw_feedback_enabled) {
    if (bmi270_init_and_calibrate() == 0) {
      g_imu_ready = 1;
      printf("[IMU] bmi270 初始化成功（" BMI270_I2C_DEV "，gz_sign=%+.1f），"
        "MOVING_DWA 角速度闭环已启用（gain_f=%.2f integ_frac=%.2f "
        "release=%.2f w_scale=%.1f）\n", (double)g_gz_sign,
        (double)g_yf_params.gain_follow, (double)g_yf_params.integ_max_frac,
        (double)g_yf_params.integ_release,
        (double)(g_w_max / g_yf_params.ref_w_max));
    } else {
      g_imu_ready = 0;
      printf("[IMU] bmi270 初始化失败——角速度闭环自动旁路"
        "（其余功能不受影响）\n");
    }
  } else {
    g_imu_ready = 0;
    printf("[IMU] yaw_feedback_enabled=false——角速度闭环关闭\n");
  }

  /* ---- 锁初始化 ---- */
  for (s = 0; s < 2; ++s) {
    pthread_mutex_init(&g_side[s].mu, NULL);
  }
  pthread_mutex_init(&g_cloud_mu, NULL);
  pthread_mutex_init(&g_evq_mu, NULL);

  /* ---- 雷达并行启动 + 桥后台就绪，主循环不等（9.24 启动提速） ---- */
  cspc_pair_set_callback(0, rm_scan_callback, (void *)(intptr_t)0);
  cspc_pair_set_callback(1, rm_scan_callback, (void *)(intptr_t)1);

  if (cspc_pair_start(0, left_port, "laser_left", 230400, 4) != 0) {
    fprintf(stderr, "fatal: 左雷达启动失败（%s）\n", left_port);
    rm_exit(1);
  }
  if (pthread_create(&lidar_threads[0], NULL, rm_lidar_thread,
        (void *)(intptr_t)0) != 0) {
    fprintf(stderr, "fatal: 左雷达线程创建失败\n");
    rm_exit(1);
  }

  if (cspc_pair_start(1, right_port, "laser_right", 230400, 4) != 0) {
    fprintf(stderr, "fatal: 右雷达启动失败（%s）\n", right_port);
    rm_exit(1);
  }
  if (pthread_create(&lidar_threads[1], NULL, rm_lidar_thread,
        (void *)(intptr_t)1) != 0) {
    fprintf(stderr, "fatal: 右雷达线程创建失败\n");
    rm_exit(1);
  }

  /* 电机桥后台就绪（等雷达数据后自动上线）；主循环立即开跑。 */
  if (pthread_create(&g_bridge_bootstrap_thread, NULL, rm_bridge_bootstrap,
        NULL) != 0) {
    fprintf(stderr, "fatal: 桥引导线程创建失败\n");
    rm_exit(1);
  }

  /* ---- 主循环（基线同款 10Hz + 打印；不绑核，DWA 并行 worker 全核可用） ---- */
  /* L1(9.27) 失控修复：stdout/stderr 置 O_NONBLOCK。日志经 tee 管道落盘，
   * 管道/终端背压时阻塞式写会把控制环卡死（070820 日志实测 print 段
   * 阻塞 141.6s，整车失控 2.4 分钟后才自行恢复）。非阻塞下管道满
   * ⇒写失败即丢日志，控制环绝不等打印。hal_log 走 stderr 同样受保护。 */
  {
    const int log_fds[2] = { STDOUT_FILENO, STDERR_FILENO };
    int fi;
    for (fi = 0; fi < 2; ++fi) {
      const int fl = fcntl(log_fds[fi], F_GETFL, 0);
      if (fl != -1) {
        (void)fcntl(log_fds[fi], F_SETFL, fl | O_NONBLOCK);
      }
    }
  }
  period_sec = (double)g_node.params_snapshot.control_period;
  if (period_sec <= 0.0) period_sec = 0.1;

  while (!g_stop) {
    long period_us = (long)(period_sec * 1e6);
    int published;
    struct timespec cpu_a, cpu_b;
    long tick_wall_us, tick_cpu_us, print_us;
    struct timeval tv_mid;
    const dwa_velocity_t intent = dwa_node_read_intent(&g_node);

    /* A4(10.09)：IMU 采样（tick 前，供本拍 rm_cmd_sink 补偿消费）；
     * 手柄角缓存（积分器换向清零判据，与 tick 同拍）。 */
    g_theta_joy = dwa_node_joystick_forward_angle_deg(&g_node, &intent) *
      (float)M_PI / 180.0f;
    rm_imu_poll();

    /* L1：清除上一圈可能因管道满置位的 stdio 错误标志（否则后续
     * printf 会永久静默），日志丢弃只影响本圈。 */
    clearerr(stdout);
    clearerr(stderr);

    gettimeofday(&tv_start, NULL);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_a);

    dwa_node_tick(&g_node);

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_b);
    gettimeofday(&tv_mid, NULL);
    tick_wall_us = (tv_mid.tv_sec - tv_start.tv_sec) * 1000000L +
      (tv_mid.tv_usec - tv_start.tv_usec);
    tick_cpu_us = (cpu_b.tv_sec - cpu_a.tv_sec) * 1000000L +
      (cpu_b.tv_nsec - cpu_a.tv_nsec) / 1000L;

    published = (g_node.last_published_tick == g_node.tick_count);
    /* A4(10.09)：窗心同步——补偿后的 w 写回 last_published_cmd，
     * 下周期 DWA 动态窗口中心=底盘实际应执行的角速度（预测与执行
     * 同源；ltw 同款语义）。未补偿/未发布时不动。 */
    if (published && g_yf_applied) {
      g_node.last_published_cmd.angular_z = g_yf_last_w;
    }
    rm_write_control_trace(&intent, published, (double)tick_wall_us,
      (double)tick_cpu_us);

    gettimeofday(&tv_end, NULL);
    loop_us = (tv_end.tv_sec - tv_start.tv_sec) * 1000000L +
      (tv_end.tv_usec - tv_start.tv_usec);
    print_us = loop_us - tick_wall_us;
    /* 毛刺自动定位：tick 墙钟 vs tick 线程 CPU（差=被调度/锁等待）
     * vs 打印段。loop>20ms 时输出（P9 排障）。 */
    if (loop_us > 20000) {
      printf("[stall] loop=%ldus tick_wall=%ldus tick_cpu=%ldus "
        "print=%ldus fused=%zu\n",
        loop_us, tick_wall_us, tick_cpu_us, print_us, g_fused_count);
    }

    if (loop_us > period_us) {
      rm_sleep_ms(0.01);
    } else {
      rm_sleep_ms((double)(period_us - loop_us) / 1000.0);
    }
  }

  /* ---- 收尾（桥先停：停发帧串 + 关串口） ---- */
  printf("[robot_main] 正在停止...\n");
  if (g_bridge_started) {
    pthread_join(g_bridge_thread, NULL);
  }
  if (g_bridge_inited) {
    mb_bridge_destroy(&g_bridge);
  }
  rm_close_runtime_traces();
  dwa_node_destroy(&g_node);
  printf("[robot_main] 已退出\n");
  rm_exit(0); /* 跳过静态析构（SDK detached 读线程仍在跑） */
  return 0;   /* unreachable */
}

