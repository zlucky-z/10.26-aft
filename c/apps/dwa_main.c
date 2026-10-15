/*
 * 模块功能：DWA 纯 C 重构版的主入口（dwa_local_planner_node main 直译）。
 * P6-5；架构优先：真实输入源（雷达融合点云 / 手柄 / 电机反馈 / 里程计）
 * 与输出面（电机桥串口）在 P7/P8 接入，本入口提供：
 *   1. yaml 配置加载（默认值 + 覆盖）
 *   2. 节点初始化 / 周期 tick 主循环（control_period 节拍）
 *   3. 控制输出 sink 打印（影子模式的可见输出面）
 *
 * P8：--replay <spool> 回放模式——读 shadow_recorder 录制的真实数据流，
 * 按录制时间轴（虚拟墙钟）注入 dwa_node_on_*，与录到的 C++ 原版输出
 * （topic 10/11）逐条对拍，收尾输出差异报告。
 *
 * 用法：dwa_main [yaml路径] [--ticks N]
 *            [--replay <spool>] [--speed S] [--follow] [--idle SEC]
 *   不带参数：使用内置默认参数（与 C++ 默认构造路径一致）。
 *   --ticks N：跑 N 个周期后退出（默认无限循环，Ctrl-C 退出）。
 *   --replay <spool>：回放模式（与 --ticks 互斥）。
 *   --speed S：回放速率倍数（默认 1.0 实时；0 = 全速不等待）。
 *   --follow：实时跟随（配合 --replay）——spool 由 recorder 边录边追加，
 *     本进程增量读取实时注入 + 真实节拍 tick，与原版输出实时并行计算。
 *     Ctrl-C / SIGTERM / 数据流静默 --idle 秒（默认 60）后收尾出对拍报告。
 */
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "../src/common/shadow_spool.h"
#include "../src/config/dwa_params.h"
#include "../src/hal/hal_log.h"
#include "../src/hal/hal_time.h"
#include "../src/runtime/dwa_node.h"

/* 跨平台毫秒级 sleep。 */
static void sleep_ms(double milliseconds)
{
  if (milliseconds <= 0.0) return;
#ifdef _WIN32
  Sleep((DWORD)(milliseconds + 0.5));
#else
  struct timespec ts;
  ts.tv_sec = (time_t)(milliseconds / 1000.0);
  ts.tv_nsec = (long)((milliseconds - (double)ts.tv_sec * 1000.0) * 1e6);
  nanosleep(&ts, NULL);
#endif
}

/* ---- 回放模式的输出/基准序列 ---- */
typedef struct {
  double t;      /* 录制时间轴（归零后秒） */
  float v;
  float w;
  float hall[8];
  size_t hall_n;
  long tick;
} replay_out_t;

typedef struct {
  double t;
  float v;
  float w;
} replay_base_cmd_t;

typedef struct {
  double t;
  float data[8];
  size_t n;
} replay_base_hall_t;

typedef struct {
  replay_out_t *items;
  size_t count;
  size_t capacity;
} replay_out_buf_t;

typedef struct {
  replay_base_cmd_t *items;
  size_t count;
  size_t capacity;
} replay_base_cmd_buf_t;

typedef struct {
  replay_base_hall_t *items;
  size_t count;
  size_t capacity;
} replay_base_hall_buf_t;

static void replay_out_push(
  replay_out_buf_t *buf, const replay_out_t *item)
{
  if (buf->count == buf->capacity) {
    size_t next = buf->capacity > 0 ? buf->capacity * 2 : 64;
    replay_out_t *grown = (replay_out_t *)realloc(
      buf->items, next * sizeof(*grown));
    if (grown == NULL) return;
    buf->items = grown;
    buf->capacity = next;
  }
  buf->items[buf->count++] = *item;
}

/* 控制输出 sink：正常模式打印；回放模式进序列（user_data = 缓冲）+ 打印。 */
static void on_cmd_published(
  void *user_data,
  const dwa_twist_t *cmd,
  const dwa_hall_speed_control_msg_t *hall_control)
{
  replay_out_buf_t *out = (replay_out_buf_t *)user_data;
  printf("[cmd] v=%+.3f w=%+.3f | hall active=%.0f target=%.3f mode=%.0f "
    "prebrake_v=%.3f y_rate=%.3f gear=%.0f closed_loop=%.0f hard_stop=%.0f\n",
    cmd->linear_x, cmd->angular_z,
    hall_control->data[0], hall_control->data[1], hall_control->data[2],
    hall_control->data[3], hall_control->data[4], hall_control->data[5],
    hall_control->data[6], hall_control->data[7]);
  if (out != NULL) {
    replay_out_t item;
    memset(&item, 0, sizeof(item));
    item.t = -1.0; /* 由回放循环在 tick 处覆盖 */
    item.v = cmd->linear_x;
    item.w = cmd->angular_z;
    memcpy(item.hall, hall_control->data, sizeof(item.hall));
    item.hall_n = 8;
    replay_out_push(out, &item);
  }
}

/* 功能：向基准序列压入一条原版输出（回放加载时调用）。 */
static void replay_base_cmd_push(
  replay_base_cmd_buf_t *buf, double t, float v, float w)
{
  if (buf->count == buf->capacity) {
    size_t next = buf->capacity > 0 ? buf->capacity * 2 : 64;
    replay_base_cmd_t *grown = (replay_base_cmd_t *)realloc(
      buf->items, next * sizeof(*grown));
    if (grown == NULL) return;
    buf->items = grown;
    buf->capacity = next;
  }
  buf->items[buf->count].t = t;
  buf->items[buf->count].v = v;
  buf->items[buf->count].w = w;
  ++buf->count;
}

static void replay_base_hall_push(
  replay_base_hall_buf_t *buf, double t, const float *data, size_t n)
{
  size_t i;
  if (buf->count == buf->capacity) {
    size_t next = buf->capacity > 0 ? buf->capacity * 2 : 64;
    replay_base_hall_t *grown = (replay_base_hall_t *)realloc(
      buf->items, next * sizeof(*grown));
    if (grown == NULL) return;
    buf->items = grown;
    buf->capacity = next;
  }
  buf->items[buf->count].t = t;
  for (i = 0; i < n && i < 8; ++i) buf->items[buf->count].data[i] = data[i];
  buf->items[buf->count].n = n < 8 ? n : 8;
  ++buf->count;
}

/* 功能：注入一条录制消息（payload 按 topic 解包）。
 * topic 10/11 不注入，写入基准序列。返回 0 正常。 */
static int replay_inject(
  dwa_node_t *node,
  const shadow_rec_t *rec,
  double base_wall,
  double first_rel,
  replay_base_cmd_buf_t *base_cmd,
  replay_base_hall_buf_t *base_hall)
{
  const uint8_t *p = rec->payload;
  size_t left = rec->payload_len;

  /* 虚拟墙钟对齐录制时刻（数据 time 戳相位与录制一致）。 */
  hal_wall_set_virtual(base_wall + rec->rel_t);

  switch (rec->topic) {
    case SHADOW_TOPIC_FUSED_CLOUD: {
      uint32_t count;
      dwa_fused_base_point_t *points;
      size_t i;
      if (left < 4) return -1;
      memcpy(&count, p, 4);
      p += 4;
      left -= 4;
      if ((size_t)count * sizeof(shadow_point_t) > left) return -1;
      points = (dwa_fused_base_point_t *)malloc(
        sizeof(*points) * (count > 0 ? count : 1));
      if (points == NULL) return -1;
      for (i = 0; i < count; ++i) {
        shadow_point_t sp;
        memcpy(&sp, p + i * sizeof(sp), sizeof(sp));
        points[i].point.x = sp.x;
        points[i].point.y = sp.y;
        points[i].is_left = sp.is_left != 0;
        points[i].side_safety_only = 0; /* 9.28 Fix D：解码点均为主云 */
        points[i].front_safety_only = 0;
      }
      dwa_node_on_fused_cloud(node, points, count);
      free(points);
      return 0;
    }
    case SHADOW_TOPIC_CMD_VEL_RAW: {
      double lx, az;
      dwa_twist_t cmd;
      if (left < 16) return -1;
      memcpy(&lx, p, 8);
      memcpy(&az, p + 8, 8);
      memset(&cmd, 0, sizeof(cmd));
      cmd.linear_x = (float)lx;   /* float 化语义：与 C++ 基准侧 q() 量化一致 */
      cmd.angular_z = (float)az;
      dwa_node_on_cmd(node, &cmd);
      return 0;
    }
    case SHADOW_TOPIC_MOTOR_FEEDBACK: {
      uint32_t n;
      if (left < 4) return -1;
      memcpy(&n, p, 4);
      if ((size_t)n * 4u > left - 4) return -1;
      dwa_node_on_motor_feedback(node, (const float *)(p + 4), n);
      return 0;
    }
    case SHADOW_TOPIC_ODOM: {
      double lx, az;
      dwa_velocity_t vel;
      if (left < 16) return -1;
      memcpy(&lx, p, 8);
      memcpy(&az, p + 8, 8);
      vel.v = (float)lx;
      vel.w = (float)az;
      dwa_node_on_odom(node, &vel);
      return 0;
    }
    case SHADOW_TOPIC_OUT_CMD: {
      double lx, az;
      if (left < 16) return -1;
      memcpy(&lx, p, 8);
      memcpy(&az, p + 8, 8);
      replay_base_cmd_push(base_cmd, rec->rel_t - first_rel,
        (float)lx, (float)az);
      return 0;
    }
    case SHADOW_TOPIC_OUT_HALL: {
      uint32_t n;
      if (left < 4) return -1;
      memcpy(&n, p, 4);
      if ((size_t)n * 4u > left - 4) return -1;
      replay_base_hall_push(base_hall, rec->rel_t - first_rel,
        (const float *)(p + 4), n);
      return 0;
    }
    default:
      return 0; /* 未知 topic：跳过 */
  }
}

/* 功能：回放结束后逐条对拍并打印差异报告。
 * 对齐：C 输出（10Hz tick 域）↔ 基准最近邻（|dt|<=0.06s）。
 * 容差：1e-3（P4.5 float 化验收标准）。 */
static void replay_compare(
  const replay_out_buf_t *out,
  const replay_base_cmd_buf_t *base_cmd,
  const replay_base_hall_buf_t *base_hall)
{
  const double tol = 1e-3;
  const double align_tol = 0.06;
  size_t i;
  size_t aligned = 0;
  size_t mismatch = 0;
  double max_err_v = 0.0;
  double max_err_w = 0.0;
  double max_err_h = 0.0;

  printf("\n==== REPLAY PARITY ====\n");
  printf("REPLAY c_out=%u base_cmd=%u base_hall=%u\n",
    (unsigned)out->count, (unsigned)base_cmd->count,
    (unsigned)base_hall->count);

  for (i = 0; i < out->count; ++i) {
    const replay_out_t *o = &out->items[i];
    /* 最近邻基准 cmd。 */
    long best = -1;
    double best_dt = align_tol + 1.0;
    size_t j;
    for (j = 0; j < base_cmd->count; ++j) {
      const double dt = fabs(base_cmd->items[j].t - o->t);
      if (dt < best_dt) {
        best_dt = dt;
        best = (long)j;
      }
    }
    if (best < 0) {
      printf("MISSING_BASE tick=%ld t=%.3f v=%+.6f w=%+.6f\n",
        o->tick, o->t, o->v, o->w);
      ++mismatch;
      continue;
    }
    {
      const replay_base_cmd_t *b = &base_cmd->items[best];
      const double ev = fabs((double)o->v - (double)b->v);
      const double ew = fabs((double)o->w - (double)b->w);
      double eh = 0.0;
      size_t k;
      /* 最近邻基准 hall（与 cmd 同链路发布）。 */
      long best_h = -1;
      double best_dt_h = align_tol + 1.0;
      for (k = 0; k < base_hall->count; ++k) {
        const double dt = fabs(base_hall->items[k].t - o->t);
        if (dt < best_dt_h) {
          best_dt_h = dt;
          best_h = (long)k;
        }
      }
      if (best_h >= 0) {
        const replay_base_hall_t *bh = &base_hall->items[best_h];
        size_t n = o->hall_n < bh->n ? o->hall_n : bh->n;
        for (k = 0; k < n; ++k) {
          const double e = fabs((double)o->hall[k] - (double)bh->data[k]);
          if (e > eh) eh = e;
        }
      }
      ++aligned;
      if (ev > max_err_v) max_err_v = ev;
      if (ew > max_err_w) max_err_w = ew;
      if (eh > max_err_h) max_err_h = eh;
      if (ev > tol || ew > tol || eh > tol) {
        printf("MISMATCH tick=%ld t=%.3f dt=%.3f "
          "cv=%+.6f bv=%+.6f (e=%.2e) cw=%+.6f bw=%+.6f (e=%.2e) "
          "hall_e=%.2e\n",
          o->tick, o->t, best_dt,
          o->v, b->v, ev, o->w, b->w, ew, eh);
        ++mismatch;
      }
    }
  }

  printf("PARITY aligned=%u mismatch=%u missing=%u "
    "max_err_v=%.3e max_err_w=%.3e max_err_h=%.3e (tol=%.1e)\n",
    (unsigned)aligned, (unsigned)mismatch,
    (unsigned)(out->count - aligned),
    max_err_v, max_err_w, max_err_h, tol);
  printf("==== %s ====\n",
    (mismatch == 0 && out->count > 0) ? "PASS" : "CHECK");
}

/* 功能：回放主循环：事件驱动（消息注入 + 周期 tick），虚拟墙钟与录制同相位。 */
static int run_replay(
  dwa_node_t *node,
  const char *spool_path,
  double speed,
  double tail_sec)
{
  shadow_rec_t *recs = NULL;
  uint8_t *buf = NULL;
  size_t buf_len = 0;
  double base_wall = 0.0;
  long nrecs;
  long ri = 0;
  double first_rel = 0.0;
  double tick_period = (double)node->params_snapshot.control_period;
  double next_tick_t;
  double replay_start_real;
  replay_out_buf_t out;
  replay_base_cmd_buf_t base_cmd;
  replay_base_hall_buf_t base_hall;
  long tick_index = 0;

  memset(&out, 0, sizeof(out));
  memset(&base_cmd, 0, sizeof(base_cmd));
  memset(&base_hall, 0, sizeof(base_hall));

  /* 回放序列缓冲挂到 sink（正常模式为 NULL，仅打印）。 */
  dwa_node_set_cmd_sink(node, on_cmd_published, &out);

  nrecs = shadow_spool_load(spool_path, &recs, &buf, &buf_len, &base_wall);
  if (nrecs <= 0) {
    fprintf(stderr, "dwa_main: cannot load spool %s\n", spool_path);
    return 1;
  }
  first_rel = recs[0].rel_t;
  hal_log(HAL_LOG_INFO, "replay: %ld records, span=%.1fs, tick=%.3fs",
    nrecs, recs[nrecs - 1].rel_t - first_rel, tick_period);

  if (tick_period <= 0.0) tick_period = 0.1;
  next_tick_t = tick_period;
  replay_start_real = hal_steady_now_sec();

  for (;;) {
    const double next_msg_t =
      (ri < nrecs) ? recs[ri].rel_t - first_rel : 1e30;
    const double end_t =
      ((ri < nrecs) ? recs[nrecs - 1].rel_t - first_rel : 0.0) + tail_sec;
    double target_t;

    if (ri >= nrecs && next_tick_t > end_t) break;

    target_t = next_msg_t < next_tick_t ? next_msg_t : next_tick_t;

    /* 实时等待（speed=0 全速）。 */
    if (speed > 0.0) {
      const double elapsed_real = hal_steady_now_sec() - replay_start_real;
      const double want_real = target_t / speed;
      if (want_real > elapsed_real) {
        sleep_ms((want_real - elapsed_real) * 1000.0);
      }
    }

    if (next_msg_t <= next_tick_t) {
      /* 消息到期：注入。 */
      if (replay_inject(node, &recs[ri], base_wall, first_rel,
            &base_cmd, &base_hall) != 0) {
        hal_log(HAL_LOG_WARN, "replay: bad record %ld dropped", ri);
      }
      ++ri;
    } else {
      /* tick 到期：虚拟墙钟对齐后执行。 */
      const size_t before = out.count;
      hal_wall_set_virtual(base_wall + first_rel + next_tick_t);
      dwa_node_tick(node);
      if (out.count > before) {
        /* 本 tick 产生了输出：补时间戳与 tick 号。 */
        size_t k;
        for (k = before; k < out.count; ++k) {
          out.items[k].t = next_tick_t;
          out.items[k].tick = tick_index;
        }
      }
      ++tick_index;
      next_tick_t += tick_period;
    }
  }

  hal_wall_set_virtual(-1.0);
  replay_compare(&out, &base_cmd, &base_hall);

  free(out.items);
  free(base_cmd.items);
  free(base_hall.items);
  free(recs);
  free(buf);
  return 0;
}

/* ---- 实时跟随模式（live 影子） ---- */

static volatile sig_atomic_t g_follow_stop = 0;

static void on_follow_signal(int sig)
{
  (void)sig;
  g_follow_stop = 1;
}

/* 功能：spool 由 recorder 边录边追加，本进程增量读取实时注入，
 * tick 按真实墙钟节拍（control_period），与 C++ 原版并行计算。
 * 停止：Ctrl-C / SIGTERM / 数据流静默 idle_sec 秒 → 收尾对拍报告。
 * 注：启动晚于 recorder 的积压记录会一次性补注入（无逐条节拍）；
 * 精确逐时间轴对拍用离线 run_replay（无 --follow）复算同一 spool。 */
static int run_follow(
  dwa_node_t *node,
  const char *spool_path,
  double idle_sec)
{
  shadow_spool_stream_t st;
  replay_out_buf_t out;
  replay_base_cmd_buf_t base_cmd;
  replay_base_hall_buf_t base_hall;
  double tick_period = (double)node->params_snapshot.control_period;
  double base_wall;
  double first_rel = -1.0;
  double start_steady = 0.0;
  double last_data;
  double next_tick_t = 0.0;
  long tick_index = 0;

  memset(&out, 0, sizeof(out));
  memset(&base_cmd, 0, sizeof(base_cmd));
  memset(&base_hall, 0, sizeof(base_hall));

  dwa_node_set_cmd_sink(node, on_cmd_published, &out);

  if (shadow_spool_stream_open(spool_path, &st) != 0) {
    fprintf(stderr, "dwa_main: cannot open spool %s\n", spool_path);
    return 1;
  }
  base_wall = st.base_wall_sec;
  if (tick_period <= 0.0) tick_period = 0.1;
  if (idle_sec <= 0.0) idle_sec = 60.0;

  hal_log(HAL_LOG_INFO,
    "follow: streaming %s (tick=%.3fs idle=%.0fs)",
    spool_path, tick_period, idle_sec);

#ifndef _WIN32
  signal(SIGPIPE, SIG_IGN);
#endif
  signal(SIGINT, on_follow_signal);
  signal(SIGTERM, on_follow_signal);

  last_data = hal_steady_now_sec();

  while (!g_follow_stop) {
    const long added = shadow_spool_stream_poll(&st);
    long i;

    if (added > 0) {
      last_data = hal_steady_now_sec();
      if (first_rel < 0.0) {
        /* 首条数据：建立时间轴（first_rel 为对拍基准零点）。 */
        first_rel = st.recs[0].rel_t;
        start_steady = hal_steady_now_sec();
        next_tick_t = tick_period;
        hal_log(HAL_LOG_INFO, "follow: 数据流已接入，实时跟随中（Ctrl-C 停止出报告）");
      }
      /* 新记录到达即其录制时刻：立即注入。 */
      for (i = st.count - added; i < st.count; ++i) {
        if (replay_inject(node, &st.recs[i], base_wall, first_rel,
              &base_cmd, &base_hall) != 0) {
          hal_log(HAL_LOG_WARN, "follow: bad record dropped");
        }
      }
    }

    if (first_rel >= 0.0) {
      const double elapsed = hal_steady_now_sec() - start_steady;
      if (next_tick_t <= elapsed) {
        const size_t before = out.count;
        size_t k;
        hal_wall_set_virtual(base_wall + first_rel + next_tick_t);
        dwa_node_tick(node);
        for (k = before; k < out.count; ++k) {
          out.items[k].t = next_tick_t;
          out.items[k].tick = tick_index;
        }
        ++tick_index;
        next_tick_t += tick_period;
        {
          const double now_elapsed = hal_steady_now_sec() - start_steady;
          if (next_tick_t <= now_elapsed) {
            /* 落后于节拍：对齐当前时刻，避免追赶风暴（与正常主循环一致）。 */
            next_tick_t = now_elapsed + tick_period;
          }
        }
      }
      if (hal_steady_now_sec() - last_data > idle_sec) {
        hal_log(HAL_LOG_INFO, "follow: 数据流静默 %.0fs，收尾", idle_sec);
        break;
      }
    }

    sleep_ms(2.0);
  }

  hal_wall_set_virtual(-1.0);
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
  replay_compare(&out, &base_cmd, &base_hall);

  shadow_spool_stream_close(&st);
  free(out.items);
  free(base_cmd.items);
  free(base_hall.items);
  return 0;
}

int main(int argc, char **argv)
{
  const char *yaml_path = NULL;
  const char *replay_path = NULL;
  double replay_speed = 1.0;
  int follow_mode = 0;
  double follow_idle = 60.0;
  long tick_limit = -1; /* -1 = 无限 */
  dwa_planner_params_t params;
  dwa_node_t *node;
  int i;

  /* stdout 行缓冲：管道/tee 下与 stderr（无缓冲，hal_log）混流时保持行完整，
   * 避免 [cmd] 输出行被 INFO/WARN 日志从中间截断交错。 */
  setvbuf(stdout, NULL, _IOLBF, 0);

  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
      tick_limit = strtol(argv[++i], NULL, 10);
    } else if (strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
      replay_path = argv[++i];
    } else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
      replay_speed = strtod(argv[++i], NULL);
    } else if (strcmp(argv[i], "--follow") == 0) {
      follow_mode = 1;
    } else if (strcmp(argv[i], "--idle") == 0 && i + 1 < argc) {
      follow_idle = strtod(argv[++i], NULL);
    } else if (argv[i][0] != '-') {
      yaml_path = argv[i];
    }
  }

  /* ---- 配置加载（默认值 + yaml 覆盖） ---- */
  dwa_planner_params_defaults(&params);
  if (yaml_path != NULL) {
    if (dwa_planner_params_load(yaml_path, &params) != 0) {
      fprintf(stderr, "dwa_main: yaml 加载失败（%s），使用默认参数\n", yaml_path);
    }
  }

  /* ---- 节点初始化 ---- */
  node = (dwa_node_t *)malloc(sizeof(dwa_node_t));
  if (node == NULL) {
    fprintf(stderr, "dwa_main: out of memory\n");
    return 1;
  }
  if (dwa_node_init(node, &params) != 0) {
    fprintf(stderr, "dwa_main: node init failed\n");
    free(node);
    return 1;
  }
  dwa_node_set_cmd_sink(node, on_cmd_published, NULL);

  /* ---- 回放模式（P8：真实数据注入 + 对拍） ---- */
  if (replay_path != NULL) {
    int rc;
    if (follow_mode) {
      rc = run_follow(node, replay_path, follow_idle);
    } else {
      rc = run_replay(node, replay_path, replay_speed, 1.0);
    }
    hal_log(HAL_LOG_INFO, "dwa_main: replay done");
    dwa_node_destroy(node);
    free(node);
    return rc;
  }

  /* ---- 周期主循环（RCLCPP spinner + wall timer 直译） ---- */
  {
    const double period_sec = (double)node->config.control_period;
    double next_tick = hal_steady_now_sec() + period_sec;
    long tick_index = 0;

    hal_log(HAL_LOG_INFO, "dwa_main: running (period=%.3fs, ticks=%ld)",
      period_sec, tick_limit);
    for (;;) {
      const double now = hal_steady_now_sec();

      if (tick_limit >= 0 && tick_index >= tick_limit) break;
      if (now < next_tick) {
        sleep_ms((next_tick - now) * 1000.0);
      }

      /* TODO(P7/P8)：在此接入真实输入源：
       *   - 融合点云 → dwa_node_on_fused_cloud（或经 dwa_node_decode_
       *     base_cloud 解 PointCloud2 字节）
       *   - 手柄 cmd_vel → dwa_node_on_cmd
       *   - 电机反馈 → dwa_node_on_motor_feedback
       *   - 里程计 → dwa_node_on_odom
       * 影子模式（P8）同时把 C++ 原版输出接入 on_cmd_published 对拍。 */

      dwa_node_tick(node);
      ++tick_index;

      next_tick += period_sec;
      if (hal_steady_now_sec() > next_tick) {
        /* 落后于节拍：直接对齐当前时刻，避免追赶风暴。 */
        next_tick = hal_steady_now_sec() + period_sec;
      }
    }
  }

  hal_log(HAL_LOG_INFO, "dwa_main: shutdown");
  dwa_node_destroy(node);
  free(node);
  return 0;
}
