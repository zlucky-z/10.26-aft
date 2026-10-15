/*
 * 模块功能：cspc COIN-D6 雷达 SDK 的双实例接口（单进程左右雷达）。
 *
 * 背景：SDK（third_party/cspc_lidar_sdk_ros2/sdk）以全局单例
 * node_lidar + 全局函数 node_start()/data_handling() 组织，一个进程
 * 只能驱动一台雷达。ROS 生产链路用两个 cspc_lidar_cpp 进程分别驱动
 * /dev/ttyS4（左）与 /dev/ttyS5（右）。单进程集成（P9）需要同进程
 * 双实例：实现层用命名空间把 SDK 八个 .cpp 复制为两份独立副本
 * （cspc_lidar_pair.cpp，预包含技巧见其注释）。
 *
 * 语义与 cspc_lidar_adapter（单雷达版）一致，仅按 side 分派。
 * 仅 POSIX 构建。
 */
#ifndef DWA_CSPC_LIDAR_PAIR_H
#define DWA_CSPC_LIDAR_PAIR_H

#include "cspc_lidar_adapter.h" /* cspc_laser_scan_t / cspc_scan_callback_t */

#ifdef __cplusplus
extern "C" {
#endif

#define CSPC_PAIR_LEFT 0
#define CSPC_PAIR_RIGHT 1

/*
 * 初始化并启动指定侧雷达驱动（CspcLidarNode 构造直译，双实例版）。
 * side：CSPC_PAIR_LEFT / CSPC_PAIR_RIGHT。
 * 返回 0 成功；-1 失败（对应 throw runtime_error）。
 */
int cspc_pair_start(int side, const char *port, const char *frame_id,
  int baudrate, int version);

/*
 * 取指定侧一圈扫描并触发该侧回调（publish_next_scan 直译）。
 * 返回 1 = 发布了一圈；0 = 无完整圈；-1 = side 非法。
 * 阻塞在 SDK data_handling 的整圈等待上（与 ROS 单进程轮询语义一致），
 * 供每侧独立采集线程调用。
 */
int cspc_pair_poll(int side);

/* 注册指定侧扫描回调（回调在调用 cspc_pair_poll 的线程内执行）。 */
void cspc_pair_set_callback(int side, cspc_scan_callback_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* DWA_CSPC_LIDAR_PAIR_H */
