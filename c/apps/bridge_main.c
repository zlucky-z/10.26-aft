/*
 * 模块功能：电机桥主入口（motor_xy_cmd_vel_node 的 main 直译）。
 * 职责：argv 配置解析 → 串口打开 → interval_ms 周期主循环（on_timer）
 * → SIGINT 退出并停发停止帧（析构语义）。
 *
 * P8 接线点：真实输入源（DWA cmd_vel / 霍尔速度控制 / 逃逸点云 /
 * 雷达扫描心跳 / IMU / 喇叭）由 IPC 层调用 mb_on_* 注入；本入口当前
 * 只跑空转周期（与 C++ 版无订阅数据时行为一致）。
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "../src/hal/hal_log.h"
#include "../src/hal/hal_time.h"
#include "../src/motor/motor_bridge.h"
#include "../src/motor/motor_bridge_config.h"

static volatile sig_atomic_t g_stop_requested = 0;

static void on_signal(int sig)
{
  (void)sig;
  g_stop_requested = 1;
}

/* 跨平台毫秒级 sleep。 */
static void sleep_ms(double milliseconds)
{
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

int main(int argc, char **argv)
{
  mb_config_t config;
  char err[256];
  mb_bridge_t *bridge;
  double period_sec;
  double next_tick;

  if (mb_config_parse(argc, argv, &config, err, sizeof(err)) != 0) {
    hal_log(HAL_LOG_ERROR, "fatal: %s", err);
    return 1;
  }

  bridge = (mb_bridge_t *)calloc(1, sizeof(mb_bridge_t));
  if (bridge == NULL) {
    hal_log(HAL_LOG_ERROR, "fatal: out of memory");
    return 1;
  }
  if (mb_bridge_init(bridge, &config, err, sizeof(err)) != 0) {
    hal_log(HAL_LOG_ERROR, "fatal: %s", err);
    free(bridge);
    return 1;
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  period_sec = (double)config.interval_ms / 1000.0;
  next_tick = hal_steady_now_sec() + period_sec;
  hal_log(HAL_LOG_INFO, "bridge_main: running (interval=%dms)", config.interval_ms);

  while (!g_stop_requested) {
    const double now = hal_steady_now_sec();
    if (now < next_tick) {
      sleep_ms((next_tick - now) * 1000.0);
    }
    /* TODO(P8)：在此接入 IPC 输入源（对应 ROS 订阅回调 → mb_on_*）。 */
    mb_on_timer(bridge);
    next_tick += period_sec;
    if (hal_steady_now_sec() > next_tick) {
      next_tick = hal_steady_now_sec() + period_sec;
    }
  }

  hal_log(HAL_LOG_INFO, "bridge_main: shutdown");
  mb_bridge_destroy(bridge); /* ~MotorXYBridge → stop_and_close */
  free(bridge);
  return 0;
}
