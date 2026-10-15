/*
 * 模块功能：将左右已滤波 LaserScan 各自按统一外参转换到 base_link 并合并。
 * 修改提示：本模块只做坐标转换和合并，不做簇过滤、车体过滤或二次去重；
 * DWA 与 RViz 共用同一输出。
 *
 * 直译自 native/laser_scan_fusion.cpp（P5）。只移植纯计算核心
 * （Mount/Point/valid_range/append_scan），ROS 节点外壳（话题/锁/PointCloud2
 * 序列化）不属本层。全 float 规范（P4.5）。
 */
#ifndef DWA_LASER_SCAN_FUSION_H
#define DWA_LASER_SCAN_FUSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 融合节点激光外参（fusion 节点自己的 Mount，区别于 DWA 节点的 LaserMount）。
 * yaw 为弧度；offset 为弧度（yaml 的 angle_offset_deg 加载时已转换）。 */
typedef struct {
  float x;
  float y;
  float yaw;
  bool reversed;
  float offset;
} dwa_fusion_mount_t;

dwa_fusion_mount_t dwa_fusion_mount_default(void);

/* 融合输出点（base_link 坐标）。source: 0=left, 1=right; preserves side
 * semantics after fusion. */
typedef struct {
  float x;
  float y;
  float z;
  uint8_t source;
} dwa_fusion_point_t;

/* 一帧已滤波 LaserScan 的最小数据面（fusion 只读这些字段）。 */
typedef struct {
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;
  const float *ranges;
  size_t range_count;
} dwa_fusion_scan_t;

/* valid_range(scan, range) 直译：有限且落在该帧声明的量程内。 */
bool dwa_fusion_valid_range(const dwa_fusion_scan_t *scan, float range);

/*
 * append_scan(scan, mount, source, points) 直译：
 * 将一帧扫描按外参变换到 base_link 追加进 points。
 * points：调用方提供的点数组缓冲（容量 points_cap，已有点数 *points_count）；
 * 返回 true（缓冲足够）；缓冲不足返回 false 且 *points_count 不变。
 */
bool dwa_fusion_append_scan(
  const dwa_fusion_scan_t *scan,
  const dwa_fusion_mount_t *mount,
  uint8_t source,
  dwa_fusion_point_t *points,
  size_t points_cap,
  size_t *points_count);

#ifdef __cplusplus
}
#endif

#endif /* DWA_LASER_SCAN_FUSION_H */
