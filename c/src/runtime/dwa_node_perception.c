/*
 * 模块功能：DWA 感知栅格处理：投影雷达扫描、射线清空、障碍占用、
 * 侧距和前方保护诊断。
 * 直译自 native/nodes/impl/node_perception_grid.inc 与
 * node_perception_scan.inc（逐行对应，不改逻辑/默认值/执行顺序）。
 * P6；全 float 规范。
 *
 * 注：inside_planning_scan_angle（左右雷达规划扇区裁剪）在 C++ 侧已被
 * 融合点云直通取代（project_base_cloud_to_grid 不再调用），故不移植；
 * left/right_angle_cropped_points_this_tick_ 计数器因此恒为 0，与 C++ 一致。
 */
#include "dwa_node_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../common/dwa_geometry.h"
#include "../hal/hal_log.h"
#include "../perception/local_occupancy_grid.h"
#include "../safety/front_safety.h" /* V4d(10.08)：dwa_point_inside_front_trigger_zone 预组装判定 */

#define NODE_PI ((float)3.14159265358979323846)

/* normalize_angle_deg(angle_deg) 直译（前向声明）。 */
float normalize_angle_deg(float angle_deg);

/* ---------- F3(9.27) 骑乘过滤宽簇墙闸 ---------- */

/* Fix E：带内点按 y 升序排序（簇切分依据），同时携带 x 供跨度判据。 */
typedef struct {
  float y;
  float x;
} dnp_band_point_t;

static int dnp_cmp_band_point_y_asc(const void *a, const void *b)
{
  const float fa = ((const dnp_band_point_t *)a)->y;
  const float fb = ((const dnp_band_point_t *)b)->y;
  return fa < fb ? -1 : (fa > fb ? 1 : 0);
}

/*
 * 每周期先扫一遍融合点云的骑乘过滤带：带内点按 y 排序聚类，
 * 存在横向宽度 ≥ rider_filter_preserve_width 的连续簇，或簇内 x 跨度
 * ≥ rider_filter_preserve_width ⇒ 判定带内是墙/大面积障碍，本周期
 * 停用骑乘过滤——防 F2(a) 盲区带复发（宽墙整面从障碍表消失→DWA
 * 满速冲墙→硬停脱困循环）。
 *
 * 9.28 Fix E：新增 x 跨度判据。窄通道侧墙是一条平行于 x 的直线，
 * 在带内 y 宽度仅数厘米（原判据永远不触发→墙点被当膝盖整段删除，
 * 044009 日志实测：右净距 0.05m 时侧点仍在带内被滤、有效轨迹 19~48
 * 条、满速贴墙）；但其 x 跨度可达带全长（0.76~1.35）。膝盖/脚尖簇
 * 的 x 跨度上限约 0.28m（0.83→1.11），按 0.40m 一刀切开：侧墙进表、
 * 膝盖照删。结果写入 node 诊断字段供 [DWA链路] 打印。
 */
static void dnp_update_rider_band_wall_gate(
  const dwa_node_t *node,
  const dwa_fused_base_point_t *base_points,
  size_t base_point_count)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  enum { MAX_BAND_POINTS = 512 };
  dnp_band_point_t pts[MAX_BAND_POINTS];
  size_t n = 0;
  size_t i;
  bool wall = false;
  float cluster_first;   /* 簇内 y 最小值 */
  float cluster_last;    /* 簇内 y 最大值 */
  float cluster_x_min;   /* 簇内 x 最小值（Fix E） */
  float cluster_x_max;   /* 簇内 x 最大值（Fix E） */

  /* C++ 侧 mutable 成员直译：const 方法内写诊断字段。 */
  *(size_t *)&node->rider_band_points_this_tick = 0;
  *(bool *)&node->rider_band_wall_gate_this_tick = false;
  if (!p->rider_self_filter_enabled) return;
  if (p->rider_self_filter_front_x <= p->rider_self_filter_start_x) return;

  for (i = 0; i < base_point_count; ++i) {
    const dwa_point_t *pt = &base_points[i].point;
    if (base_points[i].front_safety_only) continue;
    if (!isfinite(pt->x) || !isfinite(pt->y)) continue;
    if (pt->x < p->rider_self_filter_start_x ||
      pt->x > p->rider_self_filter_front_x ||
      fabsf(pt->y) > p->rider_self_filter_half_width)
    {
      continue;
    }
    if (n >= MAX_BAND_POINTS) {
      /* 带内点爆表本身就是大面积结构特征：直接判墙。 */
      *(size_t *)&node->rider_band_points_this_tick = n;
      *(bool *)&node->rider_band_wall_gate_this_tick = true;
      return;
    }
    pts[n].y = pt->y;
    pts[n].x = pt->x;
    n++;
  }
  *(size_t *)&node->rider_band_points_this_tick = n;
  if (n < 2) return;

  qsort(pts, n, sizeof(pts[0]), dnp_cmp_band_point_y_asc);
  cluster_first = pts[0].y;
  cluster_last = pts[0].y;
  cluster_x_min = pts[0].x;
  cluster_x_max = pts[0].x;
  for (i = 1; i < n; ++i) {
    if (pts[i].y - pts[i - 1].y > p->rider_filter_cluster_gap) {
      /* 判据一（原）：簇 y 宽度≥阈值——正前方横墙；
       * 判据二（Fix E）：簇 x 跨度≥阈值——平行侧墙（窄通道场景）。 */
      if (cluster_last - cluster_first >= p->rider_filter_preserve_width ||
        cluster_x_max - cluster_x_min >= p->rider_filter_preserve_width)
      {
        wall = true;
        break;
      }
      cluster_first = pts[i].y;
      cluster_x_min = pts[i].x;
      cluster_x_max = pts[i].x;
    }
    cluster_last = pts[i].y;
    if (pts[i].x < cluster_x_min) cluster_x_min = pts[i].x;
    if (pts[i].x > cluster_x_max) cluster_x_max = pts[i].x;
  }
  if (!wall &&
    (cluster_last - cluster_first >= p->rider_filter_preserve_width ||
      cluster_x_max - cluster_x_min >= p->rider_filter_preserve_width))
  {
    wall = true;
  }
  *(bool *)&node->rider_band_wall_gate_this_tick = wall;
}

/* ---------- node_perception_scan.inc 直译 ---------- */

/* angle_in_window_deg(angle_deg, start_deg, end_deg) 直译。 */
static bool angle_in_window_deg(float angle_deg, float start_deg, float end_deg)
{
  angle_deg = normalize_angle_deg(angle_deg);
  start_deg = normalize_angle_deg(start_deg);
  end_deg = normalize_angle_deg(end_deg);
  if (start_deg <= end_deg) {
    return angle_deg >= start_deg && angle_deg <= end_deg;
  }
  return angle_deg >= start_deg || angle_deg <= end_deg;
}

/* normalize_angle_deg(angle_deg) 直译。 */
float normalize_angle_deg(float angle_deg)
{
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

/* inside_local_foot_mask(is_left_scan, scan_angle_rad, sensor_range) 直译。 */
static bool inside_local_foot_mask(
  const dwa_node_t *node, bool is_left_scan, float scan_angle_rad,
  float sensor_range)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  float angle_deg;

  if (!p->local_foot_mask_enabled || !isfinite(sensor_range) ||
    sensor_range <= 0.0f || sensor_range > p->local_foot_mask_max_range)
  {
    return false;
  }

  angle_deg = normalize_angle_deg(scan_angle_rad * 180.0f / NODE_PI);
  if (is_left_scan) {
    return angle_in_window_deg(
      angle_deg, p->left_local_foot_angle_min_deg, p->left_local_foot_angle_max_deg);
  }
  return angle_in_window_deg(
    angle_deg, p->right_local_foot_angle_min_deg, p->right_local_foot_angle_max_deg);
}

/* inside_footprint_self_filter(x, y) 直译。 */
static bool inside_footprint_self_filter(const dwa_node_t *node, float x, float y)
{
  const float padding =
    node->params_snapshot.footprint_self_filter_padding > 0.0f ?
    node->params_snapshot.footprint_self_filter_padding : 0.0f;
  return dwa_point_inside_footprint_self_filter(
    x, y, &node->config.footprint, padding);
}

/* 安全净距独立留出脚尖前缘检测带：通用自车过滤仍服务于规划点云，
 * 但不能让通用过滤的0.05m padding吞掉实际toe_front前的障碍点。
 * 因此安全统计仅沿用自车过滤到 toe_front-0.05m；toe-front 窄带中的
 * 传感器自反射仍需依赖后续实测/角度特征确认，不能再用空间 padding 一刀切。 */
static bool inside_front_safety_self_filter(const dwa_node_t *node, float x, float y)
{
  const float safety_filter_front = node->config.footprint.toe_front - 0.05f;
  if (node->config.footprint.toe_front > 0.0f && x >= safety_filter_front) {
    return false;
  }
  return inside_footprint_self_filter(node, x, y);
}

/* ---------- node_perception_grid.inc 直译 ---------- */

/* reset_occupancy_grid(grid) 直译。 */
void dnp_reset_occupancy_grid(const dwa_node_t *node, dwa_occupancy_grid_t *grid)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  dwa_occupancy_grid_config_t grid_config;
  grid_config.resolution = p->grid_resolution;
  grid_config.min_x = p->grid_min_x;
  grid_config.max_x = p->grid_max_x;
  grid_config.max_abs_y = p->grid_max_abs_y;
  dwa_reset_occupancy_grid(grid, &grid_config);
}

/* 物理侧距按 base_link 点的位置判断，不按 source 归属。
 * 内向安装时，对侧雷达可能是某一物理侧墙的唯一观测者；若继续使用
 * 雷达局部角窗，跨车体墙点会被当成非本侧点而从侧距中消失。 */
bool dnp_inside_physical_side_sector(
  const dwa_node_t *node, const dwa_point_t *point)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const dwa_footprint_t *f = &node->config.footprint;
  const float side_detection_angle_deg = p->side_detection_angle_deg;
  const float angle_deg = normalize_angle_deg(
    atan2f(point->y, point->x) * 180.0f / NODE_PI);
  /* Forward-facing corner returns belong to front clearance, not side range.
   * Otherwise a frontal wall just outside toe width appears as a 5-10 cm side
   * obstacle and can incorrectly activate side-hold/speed limiting. */
  if (point->x > 0.0f &&
    fabsf(point->y) < 0.50f * fabsf(point->x)) return false;
  const float side_lookahead =
    node->dynamic_config.moving_prebrake_base_distance +
    node->dynamic_config.forward_speed_limit *
    node->dynamic_config.moving_prebrake_speed_ratio;
  const float safety_front = dwa_footprint_front_extent(f, true);
  float local_half_width = dwa_footprint_half_width_at_x(point->x, f, true);
  const bool in_angular_side_sector =
    side_detection_angle_deg > 0.0f &&
    ((point->y > 0.0f &&
      angle_deg >= 90.0f - side_detection_angle_deg && angle_deg <= 180.0f) ||
     (point->y < 0.0f &&
      angle_deg >= -180.0f && angle_deg <= -90.0f + side_detection_angle_deg));
  bool outside_lateral_footprint = false;

  /*
   * The angular sector is retained for rear/true side observations.  For a
   * diagonal wall seen across the front of the chair, however, the world
   * angle can be less than 70 degrees even though the point is physically
   * outside the corresponding side of the footprint.  Admit that point by
   * the footprint boundary, but only inside the same forward/rear lookahead
   * used by side diagnostics.  This keeps centre-front points (small |y|)
   * out of side clearance while preventing cross-body side walls from being
   * discarded solely because of their vehicle-centred angle.
   */
  if (local_half_width <= 0.0f && point->x > safety_front) {
    local_half_width = f->toe_half_width + f->padding;
  }
  if (point->x >= -f->back &&
    point->x <= safety_front + side_lookahead &&
    local_half_width > 0.0f)
  {
    outside_lateral_footprint =
      point->y > local_half_width || point->y < -local_half_width;
  }
  return in_angular_side_sector || outside_lateral_footprint;
}

static void update_side_clearance_by_physical_side(
  const dwa_node_t *node, const dwa_point_t *point,
  planner_side_clearance_diagnostics_t *diagnostics)
{
  const float footprint_distance =
    dwa_distance_to_expanded_footprint(point, &node->config.footprint);
  const float clearance = footprint_distance < 0.0f ? 0.0f : footprint_distance;

  if (!dnp_inside_physical_side_sector(node, point)) return;
  if (point->y > 0.0f) {
    if (clearance < diagnostics->left) {
      diagnostics->left = clearance;
      diagnostics->left_point = *point;
    }
    return;
  }
  if (point->y < 0.0f && clearance < diagnostics->right) {
    diagnostics->right = clearance;
    diagnostics->right_point = *point;
  }
}

/* 9.28 Fix D：跨车身侧墙救援点的侧净距更新——左右归属按点自身 y 符号
 * （两雷达交叉安装，source 与车侧相反），不走 update_side_clearance 的
 * 雷达归属+侧向角窗（跨车身视角下必被角窗剔除=侧净距失明根因）。
 * 喂 side_wall_hold / 速度侧门控消费。 */
static void update_side_clearance_by_point_side(
  const dwa_node_t *node, const dwa_point_t *point,
  planner_side_clearance_diagnostics_t *diagnostics)
{
  const float footprint_distance =
    dwa_distance_to_expanded_footprint(point, &node->config.footprint);
  const float clearance =
    footprint_distance < 0.0f ? 0.0f : footprint_distance;
  if (point->y > 0.0f) {
    if (clearance < diagnostics->left) {
      diagnostics->left = clearance;
      diagnostics->left_point = *point;
    }
    return;
  }
  if (point->y < 0.0f && clearance < diagnostics->right) {
    diagnostics->right = clearance;
    diagnostics->right_point = *point;
  }
}

/* update_side_hard_stop_circle(point, diagnostics) 直译。 */
static void update_side_hard_stop_circle(  const dwa_node_t *node, const dwa_point_t *point,
  planner_side_hard_stop_circle_diagnostics_t *diagnostics)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  const float dx = point->x - p->side_hard_stop_circle_center_x;
  const float dy = point->y - p->side_hard_stop_circle_center_y;
  const float radius = hypotf(dx, dy);
  if (!p->side_hard_stop_circle_enabled) return;
  if (!isfinite(radius) || radius > p->side_hard_stop_circle_radius) return;

  /* 圆形侧边硬停固定按左右半圆归属，避免再用角度参数重复裁剪。 */
  if (dy > 0.0f) {
    if (radius < diagnostics->left_radius) {
      diagnostics->left_radius = radius;
      diagnostics->left_point = *point;
    }
    return;
  }
  if (dy < 0.0f && radius < diagnostics->right_radius) {
    diagnostics->right_radius = radius;
    diagnostics->right_point = *point;
  }
}

/* front_no_turn_stop_max_x() 直译。 */
static float front_no_turn_stop_max_x(const dwa_node_t *node)
{
  const dwa_footprint_t *f = &node->config.footprint;
  /* 硬停区延伸到脚踏板中点，给前部转弯保留足够的安全距离。 */
  return f->footrest_start + 0.5f * (f->footrest_front - f->footrest_start);
}

/* update_front_no_turn_stop(point, diagnostics) 直译。 */
static void update_front_no_turn_stop(
  const dwa_node_t *node, const dwa_point_t *point,
  planner_front_no_turn_stop_diagnostics_t *diagnostics)
{
  /* 雷达横杆到脚踏板之间没有可用转弯半径，真实障碍进入后只能停车或倒退。 */
  if (point->x < node->params_snapshot.self_filter_body_front_x ||
    point->x > front_no_turn_stop_max_x(node) ||
    fabsf(point->y) > node->config.footprint.half_width)
  {
    return;
  }

  ++diagnostics->point_count;
  if (point->x < diagnostics->nearest_x) {
    diagnostics->nearest_x = point->x;
    diagnostics->nearest_point = *point;
  }
}

/* decode_base_cloud(cloud, points) 直译：输入侧工具（P8 回放真实数据用）。 */
size_t dwa_node_decode_base_cloud(
  const uint8_t *data,
  size_t data_size,
  size_t point_step,
  size_t x_offset,
  size_t y_offset,
  size_t source_offset,
  size_t point_count,
  dwa_fused_base_point_t *out,
  size_t max_out)
{
  size_t written = 0;
  size_t i;

  if (data == NULL || point_step == 0 ||
    x_offset + sizeof(float) > point_step ||
    y_offset + sizeof(float) > point_step ||
    source_offset + 1 > point_step)
  {
    return 0;
  }
  if (data_size < point_count * point_step) return 0;

  for (i = 0; i < point_count; ++i) {
    const uint8_t *base = data + i * point_step;
    float x;
    float y;
    uint8_t source;
    memcpy(&x, base + x_offset, sizeof(float));
    memcpy(&y, base + y_offset, sizeof(float));
    source = *(base + source_offset);
    if (isfinite(x) && isfinite(y) && source <= 1U) {
      if (written < max_out) {
        out[written].point.x = x;
        out[written].point.y = y;
        out[written].is_left = source == 0U;
        out[written].side_safety_only = 0; /* 9.28 Fix D：解码点均为主云 */
        out[written].front_safety_only = 0;
        ++written;
      }
    }
  }
  return written;
}

/* project_base_cloud_to_grid(...) 直译。 */
void dnp_project_base_cloud_to_grid(
  const dwa_node_t *node,
  const dwa_fused_base_point_t *base_points,
  size_t base_point_count,
  planner_side_clearance_diagnostics_t *side_clearance,
  dwa_occupancy_grid_t *grid,
  dwa_point_buf_t *points,
  dwa_geometric_scan_buf_t *left_geometric_scan_points,
  dwa_geometric_scan_buf_t *right_geometric_scan_points,
  planner_side_hard_stop_circle_diagnostics_t *side_hard_stop_circle,
  planner_front_no_turn_stop_diagnostics_t *front_no_turn_stop,
  dwa_point_buf_t *side_safety_points)
{
  size_t i;
  *(size_t *)&node->front_safety_candidates_this_tick = 0;
  *(size_t *)&node->front_safety_hits_this_tick = 0;
  /* 安装角是常量：三角函数外提到循环外（原逐点重算 500 点×2 次）。 */
  const bool grid_build = node->params_snapshot.occupancy_grid_build_enabled;
  const float left_mount_cos = cosf(node->left_mount.yaw);
  const float left_mount_sin = sinf(node->left_mount.yaw);
  const float right_mount_cos = cosf(node->right_mount.yaw);
  const float right_mount_sin = sinf(node->right_mount.yaw);

  /* V4d(10.08)：F1d 前向净距的未截断统计——trigger 楔形几何与车头
   * 前缘每 tick 只组装一次（与上方安装角外提同一优化模式；组装内容
   * 与 front_safety_config()（dwa_node_safety.c）保持一致）。 */
  const dwa_front_safety_config_t v4d_trigger_zone = {
    node->params_snapshot.front_trigger_start_x,
    node->params_snapshot.front_trigger_base_width,
    node->params_snapshot.front_trigger_expand_angle_deg
  };
  const float v4d_collision_front_edge = dns_collision_front_edge(node);

  /* F3(9.27)：先更新骑乘过滤带宽簇墙闸（逐点过滤前算好，本周期内
   * dnp_point_inside_rider_self_filter 与诊断链共用同一闸值）。 */
  dnp_update_rider_band_wall_gate(node, base_points, base_point_count);

  if (left_geometric_scan_points != NULL) left_geometric_scan_points->count = 0;
  if (right_geometric_scan_points != NULL) right_geometric_scan_points->count = 0;
  for (i = 0; i < base_point_count; ++i) {
    const dwa_point_t *point = &base_points[i].point;
    const bool is_left_scan = base_points[i].is_left;
    const planner_laser_mount_t *mount =
      is_left_scan ? &node->left_mount : &node->right_mount;
    const float mount_cos = is_left_scan ? left_mount_cos : right_mount_cos;
    const float mount_sin = is_left_scan ? left_mount_sin : right_mount_sin;
    float range;
    bool real_hit;
    bool body_self_reflection;
    int cell_x = 0;
    int cell_y = 0;
    dwa_geometric_scan_point_t geometric_point;
    float dx;
    float dy;
    float local_x;
    float local_y;
    float sensor_angle;

    if (!isfinite(point->x) || !isfinite(point->y)) continue;

    /* 脚尖前向独立安全候选来自融合后、通用自车过滤前的点云。
     * 只更新脚尖窄安全带净距，禁止进入规划、侧墙/骑乘带统计；候选
     * 已在 robot_main 按脚尖前缘附近的过滤释放带筛选，故不再套用
     * 会删掉它的通用足迹/骑乘过滤。 */
    if (base_points[i].front_safety_only) {
      ++*(size_t *)&node->front_safety_candidates_this_tick;
      if (dwa_point_inside_front_safety_lane(
          point, &v4d_trigger_zone, &node->config.footprint))
      {
        const float clearance = point->x - v4d_collision_front_edge;
        ++*(size_t *)&node->front_safety_hits_this_tick;
        if (clearance <
          *(float *)&node->front_unclipped_clearance_this_tick)
        {
          *(float *)&node->front_unclipped_clearance_this_tick = clearance;
        }
      }
      continue;
    }

    /* 前向净距改用未截断点云。下方 real_hit 的
     * obstacle_range(板端3.50) 截断把远点直接丢弃（不进障碍表），
     * dnp_diagnose_obstacles 的 nearest_front 因此被钳在
     * obstacle_range−车头前缘(0.90)≈2.6m，F1d 的
     * bypass_near_zone_far_distance(板端3.80) 永远够不着——通道远墙
     * 超出感知圈时 F1d"看不见"，latch 释放回档位峰值，墙进圈才急刹
     * +大舵（093514 日志定案：4/5 档撞对侧、通道内出现 3 档线速度
     * 同根因）。这里在截断【之前】对前方触发楔形区取最小净距，写入
     * front_unclipped_clearance_this_tick 供速度控制和硬停止消费
     * （dwa_node_tick.c 的 dynamic_input.front_clearance 优先读它）。
     * 只取前向物理包络窄道：弯道侧墙留给侧向/走廊逻辑，不纳入正前净距。
     * 排除跨车身侧救援点(side_safety_only)、已确认自点、骑乘带点（F3 宽簇墙闸语义随 dnp_point_inside_rider_
     * self_filter 自动生效）全部排除；local_foot_mask 板端关闭且仅挡
     * 0.4m 内近点、planning 裁剪(|y|≤2.5m)远大于楔形区半宽，对本
     * 统计均无差别。不进障碍表/栅格/几何扫描/评分——DWA 行为与
     * CPU 预算不变。 */
    if (!base_points[i].side_safety_only &&
      dwa_point_inside_front_safety_lane(
        point, &v4d_trigger_zone, &node->config.footprint) &&
      !inside_front_safety_self_filter(node, point->x, point->y) &&
      !dnp_point_inside_rider_self_filter(node, point->x, point->y))
    {
      const float v4d_clearance = point->x - v4d_collision_front_edge;
      if (v4d_clearance <
        *(float *)&node->front_unclipped_clearance_this_tick)
      {
        *(float *)&node->front_unclipped_clearance_this_tick = v4d_clearance;
      }
    }

    range = hypotf(point->x - mount->x, point->y - mount->y);
    real_hit = range > 0.0f && range <= node->config.obstacle_range;
    if (!real_hit) continue;

    if (base_points[i].side_safety_only) {
      /* 9.28 Fix D：跨车身侧墙救援点（rm_fuse 捞出，幻影/骑乘带已
       * 在源头剔除，这里足迹过滤兜底）。
       * 与 9.27 旧 Fix B 的关键区别：除侧净距外【同时进 DWA 障碍表】
       * ——025152 日志实锤：v3 门控期意图 w 恒 0 仍左右撞、规划器
       * 持续朝 0.12m 近墙打满舵（sel w=-0.32），根因是晨间跨车身
       * 裁剪令侧墙从未进过障碍表，规划器对侧墙全盲。现在碰撞判废
       * 对称生效，让它自己看见墙。
       * 仍不进：side_safety_points/侧硬停圆/front_no_turn_stop/
       * 几何扫描缓冲/自由空间射线（角窗与圆判定对跨车身视角无
       * 意义，且避免误触发既有安全通道）。栅格标记经由
       * dnp_mark_fused_obstacles_in_grid(obstacles) 自动覆盖。 */
      if (inside_footprint_self_filter(node, point->x, point->y)) {
        continue;
      }
      ++*(size_t *)&node->side_flank_points_this_tick;
      update_side_clearance_by_point_side(node, point, side_clearance);
      if (point->x < node->params_snapshot.planning_min_x ||
        fabsf(point->y) > node->params_snapshot.planning_max_abs_y) continue;
      if (dnp_point_inside_rider_self_filter(node, point->x, point->y)) continue;
      if (!dwa_grid_world_to_cell(grid, point->x, point->y, &cell_x, &cell_y)) continue;
      /* points.push_back(point)——与主路径同一缓冲、同一容量增长 */
      if (points->count == points->capacity) {
        const size_t fresh_capacity = points->capacity == 0 ?
          16 : points->capacity * 2;
        dwa_point_t *fresh = (dwa_point_t *)realloc(
          points->points, fresh_capacity * sizeof(dwa_point_t));
        if (fresh == NULL) continue;
        points->points = fresh;
        points->capacity = fresh_capacity;
      }
      points->points[points->count++] = *point;
      continue;
    }

    /* 仅对既有的角度掩码，从 base_link 点反推传感器本地角；
     * 障碍点本身保持 base_link 坐标。 */
    dx = point->x - mount->x;
    dy = point->y - mount->y;
    local_x = mount_cos * dx + mount_sin * dy;
    local_y = -mount_sin * dx + mount_cos * dy;
    sensor_angle = atan2f(local_y, local_x);
    if (inside_local_foot_mask(node, is_left_scan, sensor_angle, range)) continue;
    body_self_reflection = inside_footprint_self_filter(node, point->x, point->y);
    if (!body_self_reflection) {
      /* side_safety_points.push_back(point) */
      if (side_safety_points->count == side_safety_points->capacity) {
        const size_t fresh_capacity = side_safety_points->capacity == 0 ?
          16 : side_safety_points->capacity * 2;
        dwa_point_t *fresh = (dwa_point_t *)realloc(
          side_safety_points->points, fresh_capacity * sizeof(dwa_point_t));
        if (fresh == NULL) continue; /* vector 抛异常路径此处忽略（与 ROS 不同） */
        side_safety_points->points = fresh;
        side_safety_points->capacity = fresh_capacity;
      }
      side_safety_points->points[side_safety_points->count++] = *point;
      update_side_hard_stop_circle(node, point, side_hard_stop_circle);
      update_front_no_turn_stop(node, point, front_no_turn_stop);
    }
    /* 融合点云已表达在 base_link。不再应用旧的左右传感器角裁剪：
     * 雷达 yaw 或安装位变化后，旧扫描语义会丢弃有效障碍点。 */

    /* 融合点云没有逐点传感器射线元数据。只用已知安装位做自由空间射线
     * 追踪；障碍点本身已经在 base_link。9.24: 规划不再查询栅格，
     * 建图开关关闭时跳过射线清空（每点数十格的主要建图开销）。 */
    if (grid_build) {
      dwa_raytrace_free(grid, mount->x, mount->y, point->x, point->y);
    }
    if (body_self_reflection) {
      /* C++ 侧 body_masked_points_this_tick_ 为 mutable（const 方法内递增）。 */
      ++*(size_t *)&node->body_masked_points_this_tick;
      continue;
    }
    if (point->x < node->params_snapshot.planning_min_x ||
      fabsf(point->y) > node->params_snapshot.planning_max_abs_y) continue;

    /* 9.24 转向修复：骑乘者身体包络（脚尖+膝盖，随车刚性同动）不进入
     * 静态障碍表与几何扫描缓冲。留在这里会让 DWA 把骑乘者自身当障碍
     * 绕行（转向意图失效、速度永久压接管）。 */
    if (dnp_point_inside_rider_self_filter(node, point->x, point->y)) continue;

    if (!dwa_grid_world_to_cell(grid, point->x, point->y, &cell_x, &cell_y)) continue;
    /* points.push_back(point) */
    if (points->count == points->capacity) {
      const size_t fresh_capacity = points->capacity == 0 ?
        16 : points->capacity * 2;
      dwa_point_t *fresh = (dwa_point_t *)realloc(
        points->points, fresh_capacity * sizeof(dwa_point_t));
      if (fresh == NULL) continue;
      points->points = fresh;
      points->capacity = fresh_capacity;
    }
    points->points[points->count++] = *point;
    geometric_point.valid = true;
    geometric_point.point = *point;
    geometric_point.range = range;
    if (is_left_scan) {
      if (left_geometric_scan_points != NULL) {
        if (left_geometric_scan_points->count == left_geometric_scan_points->capacity) {
          const size_t fresh_capacity = left_geometric_scan_points->capacity == 0 ?
            16 : left_geometric_scan_points->capacity * 2;
          dwa_geometric_scan_point_t *fresh = (dwa_geometric_scan_point_t *)realloc(
            left_geometric_scan_points->items,
            fresh_capacity * sizeof(dwa_geometric_scan_point_t));
          if (fresh == NULL) continue;
          left_geometric_scan_points->items = fresh;
          left_geometric_scan_points->capacity = fresh_capacity;
        }
        left_geometric_scan_points->items[left_geometric_scan_points->count++] =
          geometric_point;
      }
    } else {
      if (right_geometric_scan_points != NULL) {
        if (right_geometric_scan_points->count == right_geometric_scan_points->capacity) {
          const size_t fresh_capacity = right_geometric_scan_points->capacity == 0 ?
            16 : right_geometric_scan_points->capacity * 2;
          dwa_geometric_scan_point_t *fresh = (dwa_geometric_scan_point_t *)realloc(
            right_geometric_scan_points->items,
            fresh_capacity * sizeof(dwa_geometric_scan_point_t));
          if (fresh == NULL) continue;
          right_geometric_scan_points->items = fresh;
          right_geometric_scan_points->capacity = fresh_capacity;
        }
        right_geometric_scan_points->items[right_geometric_scan_points->count++] =
          geometric_point;
      }
    }
    update_side_clearance_by_physical_side(node, point, side_clearance);
  }
}

/* mark_fused_obstacles_in_grid(points, grid) 直译。 */
void dnp_mark_fused_obstacles_in_grid(
  const dwa_node_t *node, const dwa_point_buf_t *points, dwa_occupancy_grid_t *grid)
{
  size_t i;
  (void)node;
  if (!node->params_snapshot.occupancy_grid_build_enabled) return;
  for (i = 0; i < points->count; ++i) {
    int cell_x = 0;
    int cell_y = 0;
    if (dwa_grid_world_to_cell(grid, points->points[i].x, points->points[i].y,
        &cell_x, &cell_y))
    {
      dwa_mark_grid_occupied(grid, cell_x, cell_y);
    }
  }
}

/* inflate_occupied_cells(grid) 直译。 */
void dnp_inflate_occupied_cells(const dwa_node_t *node, dwa_occupancy_grid_t *grid)
{
  if (!node->params_snapshot.occupancy_grid_build_enabled) return;
  dwa_inflate_occupied_cells(grid, node->params_snapshot.grid_inflation_radius);
}

/* clear_current_footprint(grid) 直译。 */
void dnp_clear_current_footprint(const dwa_node_t *node, dwa_occupancy_grid_t *grid)
{
  if (!node->params_snapshot.occupancy_grid_build_enabled) return;
  dwa_clear_current_footprint(grid, &node->config.footprint);
}

/* 骑乘者身体区自点判定（P9 新增）。
 * 骑乘者膝盖/大腿的点（实测 x≈1.11，|y|≤0.35）落在前方触发区内，
 * 会把速度模块的前方净距永久钳在小值上，导致 1.5km/h 锁速：
 * 预减速锁存距离最高 0.90+1.60×实速（≈3.5m），坐着不可能满足解除条件。
 * 9.24 转向修复：起点下探到碰撞前缘（0.76），覆盖脚尖区（实测脚尖点
 * x≈0.83 落在旧起点 0.90 之下）；且本判定同时用于障碍表构建
 * （dnp_project_base_cloud_to_grid）——障碍点按"世界中静止"仿真，而
 * 骑乘者与车刚性同动，留在障碍表会把所有前向候选轨迹判废
 * （骑行 72% 圈前方净距被脚尖钳在 0.08m，DWA 全程绕行骑乘者自身，
 * 转向意图失效）。9.20 基线以脚尖安全区常量显式忽略该区域。
 * 注意：9.24 曾尝试把本判定扩到安全诊断（禁转区/侧边点集）并将起点
 * 下探 0.50，实测导致转向剧烈抖动，已回滚——安全诊断链维持原状。 */
bool dnp_point_inside_rider_self_filter(
  const dwa_node_t *node, float x, float y)
{
  const dwa_planner_params_t *p = &node->params_snapshot;
  if (!p->rider_self_filter_enabled) return false;
  if (p->rider_self_filter_front_x <= p->rider_self_filter_start_x) return false;
  /* F3(9.27) 宽簇墙闸：带内检出墙/大面积障碍时本周期不过滤——
   * 盲区带（墙整面消失→满速冲墙）比膝盖点压速危险得多。 */
  if (node->rider_band_wall_gate_this_tick) return false;
  return x >= p->rider_self_filter_start_x &&
    x <= p->rider_self_filter_front_x &&
    fabsf(y) <= p->rider_self_filter_half_width;
}

/* diagnose_obstacles(obstacles) 直译。 */
planner_obstacle_diagnostics_t dnp_diagnose_obstacles(
  const dwa_node_t *node, const dwa_point_buf_t *obstacles)
{
  const dwa_footprint_t *f = &node->config.footprint;
  const float side_lookahead =
    node->dynamic_config.moving_prebrake_base_distance +
    node->dynamic_config.forward_speed_limit *
    node->dynamic_config.moving_prebrake_speed_ratio;
  const float safety_front = dwa_footprint_front_extent(f, true);
  planner_obstacle_diagnostics_t diag = planner_obstacle_diagnostics_default();
  size_t i;

  diag.total = obstacles->count;
  for (i = 0; i < obstacles->count; ++i) {
    const dwa_point_t *point = &obstacles->points[i];
    const float left_radar_distance = hypotf(
      point->x - node->left_mount.x, point->y - node->left_mount.y);
    const float right_radar_distance = hypotf(
      point->x - node->right_mount.x, point->y - node->right_mount.y);
    const float nearest_radar_distance =
      left_radar_distance < right_radar_distance ?
      left_radar_distance : right_radar_distance;
    float local_half_width = dwa_footprint_half_width_at_x(
      point->x, f, true);
    bool inside_side_lookahead;

    if (dwa_distance_to_expanded_footprint(point, f) < 0.0f) {
      ++diag.inside_current_footprint;
    }
    if (nearest_radar_distance < diag.nearest_radar_distance) {
      diag.nearest_radar_distance = nearest_radar_distance;
      diag.nearest_radar_point = *point;
    }
    if (dns_point_inside_front_safety_lane(node, point))
    {
      const bool rider_self_point =
        dnp_point_inside_rider_self_filter(node, point->x, point->y);
      if (!rider_self_point) {
        const float clearance = point->x - dns_collision_front_edge(node);
        if (clearance < diag.nearest_front) {
          diag.nearest_front = clearance;
          diag.nearest_front_point = *point;
        }
        /* 近量程硬停止只统计前方保护区域，不能使用全局最近雷达点。 */
        if (nearest_radar_distance < diag.nearest_front_radar_distance) {
          diag.nearest_front_radar_distance = nearest_radar_distance;
        }
      }
    }
    if (dns_point_inside_gear_front_trigger_zone(node, point))
    {
      if (!dnp_point_inside_rider_self_filter(node, point->x, point->y)) {
        const float clearance = point->x - dns_collision_front_edge(node);
        if (clearance < diag.nearest_gear_front) {
          diag.nearest_gear_front = clearance;
        }
      }
    }
    if (local_half_width <= 0.0f && point->x > safety_front) {
      local_half_width = f->toe_half_width + f->padding;
    }
    inside_side_lookahead = point->x >= -f->back &&
      point->x <= safety_front + side_lookahead;
    if (local_half_width > 0.0f && point->y > local_half_width && inside_side_lookahead) {
      const float side_distance = point->y - local_half_width;
      if (side_distance < diag.nearest_left) {
        diag.nearest_left = side_distance;
        diag.nearest_left_point = *point;
      }
    }
    if (local_half_width > 0.0f && point->y < -local_half_width && inside_side_lookahead) {
      const float side_distance = -point->y - local_half_width;
      if (side_distance < diag.nearest_right) {
        diag.nearest_right = side_distance;
        diag.nearest_right_point = *point;
      }
    }
  }
  return diag;
}

/* diagnose_grid(grid) 直译。 */
planner_grid_diagnostics_t dnp_diagnose_grid(const dwa_occupancy_grid_t *grid)
{
  planner_grid_diagnostics_t diag;
  size_t i;
  diag.free_cells = 0;
  diag.occupied_cells = 0;
  diag.unknown_cells = 0;
  for (i = 0; i < grid->cells_size; ++i) {
    const int8_t value = grid->cells[i];
    if (value >= 100) ++diag.occupied_cells;
    else if (value == 0) ++diag.free_cells;
    else ++diag.unknown_cells;
  }
  return diag;
}
