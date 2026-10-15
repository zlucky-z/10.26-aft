/*
 * 模块功能：cspc_lidar_adapter.h 对应实现（node_lidar_ros.cpp 直译）。
 * 本文件是重构工程中唯一的 C++ 编译单元（方案允许的例外）：
 * SDK core 保持 C++，此处以 extern "C" 包装。
 */
#include "cspc_lidar_adapter.h"

#include <cstdio>
#include <string>

#include "node_lidar.h"

#include "../hal/hal_log.h"
#include "../hal/hal_time.h"

/* 点缓冲：静态启动期分配（方案内存策略）。一圈典型 ~380 点（10Hz×3800/s），
 * 16K 上限远超需求；超限时截断并告警。 */
#define CSPC_POINT_MAX 16384

static cspc_laser_point_t g_points[CSPC_POINT_MAX];
static cspc_scan_callback_t g_callback = NULL;
static void *g_callback_user = NULL;

extern "C" void cspc_adapter_set_scan_callback(cspc_scan_callback_t cb,
  void *user)
{
  g_callback = cb;
  g_callback_user = user;
}

extern "C" int cspc_adapter_start(const char *port, const char *frame_id,
  int baudrate, int version)
{
  node_lidar.lidar_general_info.port = port != NULL ? port : "/dev/ttyS4";
  node_lidar.lidar_general_info.frame_id =
    frame_id != NULL ? frame_id : "laser_link";
  node_lidar.lidar_general_info.m_SerialBaudrate = baudrate;
  node_lidar.lidar_general_info.version = version;

  if (node_start() < 0) {
    return -1;
  }
  hal_log(HAL_LOG_INFO,
    "COIN-D6 lidar started: port=%s baudrate=%d version=%d frame_id=%s",
    node_lidar.lidar_general_info.port.c_str(),
    node_lidar.lidar_general_info.m_SerialBaudrate,
    node_lidar.lidar_general_info.version,
    node_lidar.lidar_general_info.frame_id.c_str());
  return 0;
}

extern "C" int cspc_adapter_poll(void)
{
  LaserScan scan;
  size_t i;
  size_t count;
  cspc_laser_scan_t out;

  /* data_handling() 等待完整一圈（SDK 内部同步语义，见 node_lidar_ros 注释） */
  if (!data_handling(scan) || scan.points.empty()) {
    return 0;
  }

  count = scan.points.size();
  if (count > CSPC_POINT_MAX) {
    hal_log(HAL_LOG_WARN,
      "COIN-D6 scan truncated: points=%zu limit=%d",
      count, CSPC_POINT_MAX);
    count = CSPC_POINT_MAX;
  }
  for (i = 0; i < count; ++i) {
    g_points[i].angle = scan.points[i].angle;
    g_points[i].range = scan.points[i].range;
    g_points[i].intensity = scan.points[i].intensity;
  }

  out.min_angle = scan.config.min_angle;
  out.max_angle = scan.config.max_angle;
  out.angle_increment = scan.config.angle_increment;
  out.time_increment = scan.config.time_increment;
  out.scan_time = scan.config.scan_time;
  out.min_range = scan.config.min_range;
  out.max_range = scan.config.max_range;
  out.stamp = scan.stamp;
  out.point_count = count;
  out.points = g_points;

  if (g_callback != NULL) {
    g_callback(g_callback_user, &out);
  }
  hal_log_throttle(hal_steady_now_sec(), 2000, (const void *)&g_callback,
    HAL_LOG_INFO, "COIN-D6 scan published: points=%zu range=[%.2f,%.2f]m",
    count, (double)out.min_range, (double)out.max_range);
  return 1;
}
