/*
 * 模块功能：将左右已滤波 LaserScan 各自按统一外参转换到 base_link 并合并。
 * 修改提示：本模块只做坐标转换和合并，不做簇过滤、车体过滤或二次去重；
 * DWA 与 RViz 共用同一输出。
 *
 * 逐行直译自 native/laser_scan_fusion.cpp（P5），全 float 规范（P4.5）。
 * 原版变换中间量（cos_yaw/sin_yaw/local_x/local_y/angle）为 double，此处 float。
 */
#include "laser_scan_fusion.h"

#include <math.h>

dwa_fusion_mount_t dwa_fusion_mount_default(void)
{
  dwa_fusion_mount_t mount;
  mount.x = 0.0f;
  mount.y = 0.0f;
  mount.yaw = 0.0f;
  mount.reversed = false;
  mount.offset = 0.0f;
  return mount;
}

/* valid_range(scan, range) 直译：有限且落在该帧声明的量程内。 */
bool dwa_fusion_valid_range(const dwa_fusion_scan_t *scan, float range)
{
  return isfinite(range) && range >= scan->range_min && range <= scan->range_max;
}

/*
 * append_scan(scan, mount, source, points) 直译。
 * 原版 points.reserve 后 push_back；此处带容量检查，缓冲不足返回 false。
 */
bool dwa_fusion_append_scan(
  const dwa_fusion_scan_t *scan,
  const dwa_fusion_mount_t *mount,
  uint8_t source,
  dwa_fusion_point_t *points,
  size_t points_cap,
  size_t *points_count)
{
  const float cos_yaw = cosf(mount->yaw);
  const float sin_yaw = sinf(mount->yaw);
  float scan_angle = scan->angle_min;
  size_t i;

  for (i = 0; i < scan->range_count; ++i) {
    const float range = scan->ranges[i];
    const float angle = (mount->reversed ?
      (scan->angle_min + scan->angle_max - scan_angle) : scan_angle) + mount->offset;
    if (dwa_fusion_valid_range(scan, range)) {
      const float local_x = range * cosf(angle);
      const float local_y = range * sinf(angle);
      if (*points_count >= points_cap) return false;
      points[*points_count].x =
        mount->x + cos_yaw * local_x - sin_yaw * local_y;
      points[*points_count].y =
        mount->y + sin_yaw * local_x + cos_yaw * local_y;
      points[*points_count].z = 0.0f;
      points[*points_count].source = source;
      ++(*points_count);
    }
    scan_angle += scan->angle_increment;
  }
  return true;
}
