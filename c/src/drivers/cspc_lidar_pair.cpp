/*
 * 模块功能：cspc_lidar_pair.h 对应实现——SDK 双实例（命名空间复制）。
 *
 * 技术：SDK 八个 .cpp 以 #include 方式收进两个独立命名空间
 *（cspc_left / cspc_right），每个命名空间持有自己的一份全局
 * node_lidar、串口对象与读线程，互不干扰。
 *
 * 关键细节（P7 预包含技巧的演进版）：
 *   1. 系统头全部在全局作用域预包含——SDK 头/源内部的系统 include
 *      命中 include guard 变 no-op，不会被拉进命名空间。
 *   2. locker.h 在全局预包含：它是无状态工具类（Locker/Event/
 *      ScopedLocker，每实例自持锁对象），两侧共享类定义、各建各的
 *      实例。它用 #pragma once 做 guard，无法用 #undef 复制进第二
 *      命名空间，且放进命名空间反而会因类型同时存在于全局与命名
 *      空间触发 ADL 歧义。
 *   3. 其余 SDK 头随命名空间包含（类型/函数完整落在命名空间内，
 *      避免与全局副本 ADL 歧义）；进入第二命名空间前 #undef 这些头
 *      的 guard 宏（guard 是编译单元全局的，否则第二份全部 no-op）。
 *
 * 本文件是重构工程中继 cspc_lidar_adapter.cpp 之后的第二个（也是
 * 最后一个）C++ 编译单元；仅 POSIX 构建（SDK 依赖 termios）。
 */
#include "cspc_lidar_pair.h"

#include <cstdio>
#include <string>

/* ---- 全局预包含：locker.h（无状态工具类，两侧共享） ---- */
#include "locker.h"

/* ---- 全局预包含：SDK 传递引入的全部系统头。
 * 清单来自对 sdk/*.h sdk/*.cpp 的 #include 全枚举（Windows 专属头在
 * SDK 内被 _WIN32 守卫，Linux 构建不触达，无须预包含）。 ---- */
#include <algorithm>
#include <assert.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <linux/serial.h>
#include <math.h>
#include <memory>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

/* ---- SDK 噪声静音（P9） ----
 * SDK 在数据线程里直接 printf（"full----- ..." 转速异常心跳、version
 * 枚举打印等），并在 lidar_set_port() 里 std::cout << "***" << port。
 * 单进程版里这些输出：① 与 robot_main 的基线同款打印争用 stdout 的
 * stdio 全局锁；② 污染与 9.20 基线对照的日志。
 * printf 在命名空间展开前重定义为 no-op（仅影响本编译单元内 SDK 副本，
 * 系统头已全局预包含看不到该宏）；cout 由 cspc_pair_start 里把
 * std::cout 指向空 streambuf 静音（本工程全用 printf，进程内无人用 cout）。 */
#define printf RM_SDK_QUIET_PRINTF
#define RM_SDK_QUIET_PRINTF(...) ((void)0)

/* ---- 左雷达：SDK 头 + 全部 .cpp 收进独立命名空间（main.cpp 不含；
 * locker.h 已全局预包含，此处经 #pragma once 自动 no-op） ---- */
namespace cspc_left {
#include "calibration.h"
#include "lidar_data_processing.h"
#include "lidar_information.h"
#include "msg_recept.h"
#include "mtime.h"
#include "node_lidar.h"
#include "point_cloud_optimize.h"
#include "serial_port.h"
#include "timer.h"

#include "calibration.cpp"
#include "lidar_data_processing.cpp"
#include "lidar_information.cpp"
#include "mtime.cpp"
#include "node_lidar.cpp"
#include "point_cloud_optimize.cpp"
#include "serial_port.cpp"
#include "timer.cpp"
} /* namespace cspc_left */

/* ---- 右雷达：SDK 第二份独立副本。
 * SDK 头 guard 宏是编译单元全局的（第一份已置位），必须先全部
 * #undef 才能让头文件在第二命名空间重新展开。 ---- */
namespace cspc_right {
#undef LIDAR_CALIBRATION
#undef LIDAR_DATA_PROCESSING
#undef LIDAR_INFORMATION_H
#undef __H_MESSAGE_RECEPT_H__
#undef __H_MTIME_H__
#undef NODE_LIDAR_H
#undef POINT_CLOUD_OPTIMIZE_H
#undef NODE_SERIAL_H
#undef NODE_TIMER_H

#include "calibration.h"
#include "lidar_data_processing.h"
#include "lidar_information.h"
#include "msg_recept.h"
#include "mtime.h"
#include "node_lidar.h"
#include "point_cloud_optimize.h"
#include "serial_port.h"
#include "timer.h"

#include "calibration.cpp"
#include "lidar_data_processing.cpp"
#include "lidar_information.cpp"
#include "mtime.cpp"
#include "node_lidar.cpp"
#include "point_cloud_optimize.cpp"
#include "serial_port.cpp"
#include "timer.cpp"
} /* namespace cspc_right */

#undef printf /* 还原：以下本文件代码与后续包含不再静音 */

/* ---- SDK std::cout 静音（P9）：空 streambuf，吞掉全部输出 ---- */
namespace {
struct rm_null_streambuf : public std::streambuf {
  int overflow(int c) override { return c; }
  std::streamsize xsputn(const char *, std::streamsize n) override { return n; }
};
rm_null_streambuf g_rm_null_streambuf;
} /* namespace */

/* ---- 每侧独立状态（cspc_lidar_adapter 单例版的成对化） ---- */
#define CSPC_PAIR_POINT_MAX 16384

typedef struct {
  cspc_laser_point_t points[CSPC_PAIR_POINT_MAX];
  cspc_scan_callback_t cb;
  void *cb_user;
} cspc_pair_state_t;

static cspc_pair_state_t g_pair_state[2];

/* 单侧实现宏（两侧代码完全对称，宏避免复制漂移）。
 * 语义逐行对齐 cspc_lidar_adapter.cpp 的 start/poll。 */
#define CSPC_PAIR_SIDE_IMPL(NS, TAG)                                          \
  static int cspc_##TAG##_start(const char *port, const char *frame_id,       \
    int baudrate, int version)                                                \
  {                                                                           \
    NS::node_lidar.lidar_general_info.port =                                  \
      port != NULL ? port : "/dev/ttyS4";                                     \
    NS::node_lidar.lidar_general_info.frame_id =                              \
      frame_id != NULL ? frame_id : "laser_link";                             \
    NS::node_lidar.lidar_general_info.m_SerialBaudrate = baudrate;            \
    NS::node_lidar.lidar_general_info.version = version;                      \
    if (NS::node_start() < 0) {                                               \
      return -1;                                                              \
    }                                                                         \
    return 0;                                                                 \
  }                                                                           \
                                                                              \
  static int cspc_##TAG##_poll(void)                                          \
  {                                                                           \
    NS::LaserScan scan;                                                       \
    cspc_pair_state_t *st = &g_pair_state[CSPC_PAIR_##TAG];                   \
    size_t i;                                                                 \
    size_t count;                                                             \
    cspc_laser_scan_t out;                                                    \
                                                                              \
    /* data_handling() 等待完整一圈（SDK 内部同步语义）。 */                  \
    if (!NS::data_handling(scan) || scan.points.empty()) {                    \
      return 0;                                                               \
    }                                                                         \
                                                                              \
    count = scan.points.size();                                               \
    if (count > CSPC_PAIR_POINT_MAX) {                                        \
      std::fprintf(stderr,                                                    \
        "COIN-D6(" #TAG ") scan truncated: points=%zu limit=%d\n",            \
        count, CSPC_PAIR_POINT_MAX);                                          \
      count = CSPC_PAIR_POINT_MAX;                                             \
    }                                                                         \
    for (i = 0; i < count; ++i) {                                             \
      st->points[i].angle = scan.points[i].angle;                             \
      st->points[i].range = scan.points[i].range;                             \
      st->points[i].intensity = scan.points[i].intensity;                     \
    }                                                                         \
                                                                              \
    out.min_angle = scan.config.min_angle;                                    \
    out.max_angle = scan.config.max_angle;                                    \
    out.angle_increment = scan.config.angle_increment;                        \
    out.time_increment = scan.config.time_increment;                          \
    out.scan_time = scan.config.scan_time;                                    \
    out.min_range = scan.config.min_range;                                    \
    out.max_range = scan.config.max_range;                                    \
    out.stamp = scan.stamp;                                                   \
    out.point_count = count;                                                  \
    out.points = st->points;                                                  \
                                                                              \
    if (st->cb != NULL) {                                                     \
      st->cb(st->cb_user, &out);                                              \
    }                                                                         \
    return 1;                                                                 \
  }

CSPC_PAIR_SIDE_IMPL(cspc_left, LEFT)
CSPC_PAIR_SIDE_IMPL(cspc_right, RIGHT)

extern "C" int cspc_pair_start(int side, const char *port, const char *frame_id,
  int baudrate, int version)
{
  /* SDK cout 静音（幂等）：lidar_set_port() 的 "***<port>" 打印等。 */
  std::cout.rdbuf(&g_rm_null_streambuf);
  if (side == CSPC_PAIR_LEFT) {
    return cspc_LEFT_start(port, frame_id, baudrate, version);
  }
  if (side == CSPC_PAIR_RIGHT) {
    return cspc_RIGHT_start(port, frame_id, baudrate, version);
  }
  return -1;
}

extern "C" int cspc_pair_poll(int side)
{
  if (side == CSPC_PAIR_LEFT) {
    return cspc_LEFT_poll();
  }
  if (side == CSPC_PAIR_RIGHT) {
    return cspc_RIGHT_poll();
  }
  return -1;
}

extern "C" void cspc_pair_set_callback(int side, cspc_scan_callback_t cb,
  void *user)
{
  if (side == CSPC_PAIR_LEFT || side == CSPC_PAIR_RIGHT) {
    g_pair_state[side].cb = cb;
    g_pair_state[side].cb_user = user;
  }
}
