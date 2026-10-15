/*
 * 模块功能：COIN-D6 雷达 SDK 接口层（node_lidar_ros.cpp 97 行 ROS 层的
 * C 回调注册直译）。
 * 方案允许的唯一 C++ 例外：SDK core（third_party/cspc_lidar_sdk_ros2/sdk，
 * ~2500 行 C 风格 C++）保持 C++ 编译，由本适配层以 extern "C" 包装。
 *
 * 语义对齐 node_lidar_ros.cpp：
 *   - 启动参数（port/frame_id/baudrate/version）写入
 *     node_lidar.lidar_general_info 后 node_start()
 *   - publish_next_scan()：data_handling() 等整圈 → 转换为 C 扫描结构 →
 *     回调（对应 LaserScan publisher）
 *   - 空圈（points 为空）不触发回调
 *   - THROTTLE 日志：2 秒窗口（hal_log_throttle 直译）
 *
 * 仅 POSIX 构建（SDK serial_port 依赖 termios）。
 */
#ifndef DWA_CSPC_LIDAR_ADAPTER_H
#define DWA_CSPC_LIDAR_ADAPTER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float angle;
  float range;
  unsigned short intensity;
} cspc_laser_point_t;

typedef struct {
  /* LaserConfig 直译 */
  float min_angle;
  float max_angle;
  float angle_increment;
  float time_increment;
  float scan_time;
  float min_range;
  float max_range;
  unsigned long long stamp;
  /* 点数组指针指向适配层内部缓冲，仅回调内有效 */
  size_t point_count;
  const cspc_laser_point_t *points;
} cspc_laser_scan_t;

typedef void (*cspc_scan_callback_t)(void *user,
  const cspc_laser_scan_t *scan);

/*
 * 初始化并启动雷达驱动（CspcLidarNode 构造直译）。
 * 返回 0 成功；-1 失败（对应 throw runtime_error）。
 */
int cspc_adapter_start(const char *port, const char *frame_id,
  int baudrate, int version);

/*
 * 取一圈扫描并触发回调（publish_next_scan 直译）。
 * 返回 1 = 发布了一圈；0 = 无完整圈（不触发回调）；-1 = 错误。
 */
int cspc_adapter_poll(void);

void cspc_adapter_set_scan_callback(cspc_scan_callback_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* DWA_CSPC_LIDAR_ADAPTER_H */
