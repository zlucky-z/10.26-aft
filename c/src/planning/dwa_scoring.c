/* 直译自 native/planning/dwa_scoring.hpp，逻辑/顺序不变。 */
#include "dwa_scoring.h"

#include <float.h>

#include "../common/dwa_geometry.h"

static float dwa_min_double(float a, float b) { return b < a ? b : a; }
static float dwa_max_double(float a, float b) { return a < b ? b : a; }

/*
 * C++ lambda evaluate_obstacle 的捕获展开：
 * [&, state, cos_yaw, sin_yaw, &min_clearance_squared, &min_clearance,
 *  &useful_radius, &useful_radius_squared, obstacle_range_squared,
 *  footprint_radius, &config]
 */
typedef struct {
  const dwa_footprint_t *collision_footprint;
  const dwa_state_t *state;
  float cos_yaw;
  float sin_yaw;
  float obstacle_range_squared;
  float footprint_radius;
  float *min_clearance_squared;
  float *min_clearance;
  float *useful_radius;
  float *useful_radius_squared;
  /* 10.08 Fix V3：本步是否在判死视界内。false = 净距视界带（记真实
   * 净距、不判死）——足迹内点净距显式记 0（距离函数对内点返回 -1），
   * 满障碍罚作渐进转弯引导。 */
  bool veto_armed;
} evaluate_obstacle_ctx_t;

static bool dwa_evaluate_obstacle(const dwa_point_t *obstacle, void *user)
{
  const evaluate_obstacle_ctx_t *ctx = (const evaluate_obstacle_ctx_t *)user;
  const dwa_state_t *state = ctx->state;
  const float dx = obstacle->x - state->x;
  const float dy = obstacle->y - state->y;
  const float center_distance_squared = dx * dx + dy * dy;
  if (center_distance_squared > ctx->obstacle_range_squared) return false;

  if (center_distance_squared > *ctx->useful_radius_squared) return false;

  const float local_x = ctx->cos_yaw * dx + ctx->sin_yaw * dy;
  const float local_y = -ctx->sin_yaw * dx + ctx->cos_yaw * dy;
  if (dwa_footprint_contains_local(local_x, local_y, ctx->collision_footprint, true)) {
    if (ctx->veto_armed) return true;
    /* Fix V3：判死视界外的碰撞 = "现在刹车来不及转"——不否决候选，
     * 但净距记 0：障碍代价满罚（0.40）压过 to_goal 直行偏好，逼评分
     * 自觉选转弯/减速弧。这正是 V1（记 0 但全 2s 视界→悬崖太早）与
     * V2（不记→墙隐形不转弯，053316 走廊尽头 16 次硬停）的折中：
     * 罚分梯度只延伸到 clearance 视界边沿，起转距离可调。 */
    if (*ctx->min_clearance_squared > 0.0f) {
      *ctx->min_clearance_squared = 0.0f;
      *ctx->min_clearance = 0.0f;
      *ctx->useful_radius = ctx->footprint_radius + 1e-12f;
      *ctx->useful_radius_squared = *ctx->useful_radius * *ctx->useful_radius;
    }
    return false;
  }

  const dwa_point_t local_point = {local_x, local_y};
  const float clearance = dwa_distance_to_expanded_footprint(
    &local_point, ctx->collision_footprint);
  const float clearance_squared = clearance * clearance;
  if (clearance_squared < *ctx->min_clearance_squared) {
    *ctx->min_clearance_squared = clearance_squared;
    *ctx->min_clearance = clearance;
    *ctx->useful_radius = ctx->footprint_radius + *ctx->min_clearance + 1e-12f;
    *ctx->useful_radius_squared = *ctx->useful_radius * *ctx->useful_radius;
  }
  return false;
}

void dwa_evaluate_trajectory(
  dwa_trajectory_t *trajectory,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_point_t *local_goal,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  const dwa_occupancy_grid_t *grid,
  const dwa_obstacle_spatial_index_t *spatial_index,
  const dwa_grid_blocking_index_t *blocking_index)
{
  /* 9.24 性能修复：逐步碰撞只保留点云空间桶查询（与 9.20 基线一致）。
   * 占用栅格的占据格来自同一份融合点云并叠加射线清空，含陈旧占据时
   * 反而比当前点云更保守；原先每步先做栅格 footprint 盒查询再做一遍
   * 点云桶查询，是 DWA 接管圈耗时主因之一。栅格碰撞检查在此停用，
   * dwa_footprint_hits_grid 函数保留（test_p5 仍直接覆盖）。 */
  (void)grid;
  (void)blocking_index;
  const float obstacle_range_squared = config->obstacle_range * config->obstacle_range;
  dwa_footprint_t collision_footprint = config->footprint;
  if (collision_footprint.toe_front > collision_footprint.footrest_front &&
    config->toe_collision_margin_m > 0.0f)
  {
    collision_footprint.toe_front += config->toe_collision_margin_m;
  }
  const float footprint_front = dwa_footprint_front_extent(&collision_footprint, true) +
    collision_footprint.padding;
  const float footprint_back = collision_footprint.back + collision_footprint.padding;
  const float footprint_half_width = dwa_footprint_half_width_extent(
    &collision_footprint, true) + collision_footprint.padding;
  const float footprint_radius = hypotf(
    dwa_max_double(footprint_front, footprint_back), footprint_half_width);
  /* 10.08 Fix V3b（CPU 修复）：净距统计初始值钳到软边距。净距 ≥
   * obstacle_cost_free_margin 时障碍代价恒 0，评分逐位等价（几何选择
   * 器板端关闭，min_clearance 其余活跃消费方仅日志打印）；但
   * useful_radius 从 footprint+obstacle_range(3.5m) 缩到 footprint+
   * 0.40m，每步空间桶查询点数 ~400→~100，修 V3 净距视界带来的圈耗
   * 时暴涨（061641 实测运动圈 p90 26.8ms、[stall] 2069 次；V2 时代
   * 7-12ms）。注意：若将来重开几何选择器，其 raw_clearance-
   * max_clearance_loss 相对比较基于本值（上限从 3.5 变 0.40），需
   * 重审此钳制。 */
  const float initial_clearance = dwa_min_double(
    config->obstacle_range, config->obstacle_cost_free_margin);
  float min_clearance_squared = initial_clearance * initial_clearance;
  float min_clearance = initial_clearance;
  float useful_radius = footprint_radius + min_clearance + 1e-12f;
  float useful_radius_squared = useful_radius * useful_radius;

  /* 10.08 Fix V3c：判死/净距双视界解耦（仅 DWA 候选评分调用方开启）。
   * 判死视界 T_veto = |v|/max_deceleration + reaction_sec（钳 [min,max]）
   * 不变——该视界内的碰撞"现在刹车已经来不及"，否决合理；近距保命
   * 与通道贴墙防护（红区）与 V2/V3/V3b 逐位相同。
   * 净距视界 = min(horizon_m, 刹车视距弧长 + 0.30m)。V3b 的纯固定弧长
   * 在低速通道也拉满 1.4m——修正舵候选（持续左舵对抗机械右漂）的
   * 弧线在 1.4m 内扫到对侧墙（净距→0 满罚），可持续修正角速度被钳
   * 到 ~2×侧余量×v/arc²≈0.28 < 机械右漂 0.16~0.36 → 修正不动 → 骑
   * 右墙（065301 实测 L-R +0.090、R_min 0.05；对照 V2 arc0.54 →
   * +0.011、V3 arc0.82 → +0.050，偏移随弧长单调）。改回"刹车视距+
   * 0.30m"（V3 语义：v=0.7 时 0.84m——V3 实测足够走廊尽头起转），
   * 并以 horizon_m 封顶高速端（v=1.4 时 1.6m，不再像 V3 拉到
   * 2.28m 系统性压高速）。veto 视界外只记净距不判死（足迹内点记 0
   * 满罚）；两视界都过期才 break。v=0 原地旋转弧长恒 0 不会提前退出。
   * 开关关时 veto=∞、视界=horizon_m 固定弧长，行为同 V3b。 */
  float veto_seconds = INFINITY;
  if (config->collision_veto_enabled &&
    config->max_deceleration > 1e-6f && config->sim_dt > 1e-6f) {
    veto_seconds = fabsf(trajectory->command.v) / config->max_deceleration +
      config->collision_veto_reaction_sec;
    veto_seconds = dwa_max_double(config->collision_veto_min_sec,
      dwa_min_double(config->collision_veto_max_sec, veto_seconds));
  }
  const float clearance_distance = isfinite(veto_seconds)
    ? dwa_min_double(config->clearance_horizon_m,
      fabsf(trajectory->command.v) * veto_seconds + 0.30f)
    : config->clearance_horizon_m;

  for (size_t si = 0; si < trajectory->state_count; ++si) {
    const dwa_state_t *state = &trajectory->states[si];
    const float step_time = (float)(si + 1) * config->sim_dt;
    const bool step_veto_armed = step_time <= veto_seconds;
    if (!step_veto_armed &&
      fabsf(trajectory->command.v) * step_time > clearance_distance) {
      /* Fix V3b：判死视界与净距弧长都过期——后续步对评分无贡献，
       * 整体跳过。states 数组完整保留，path_cost/to_goal 端点评分
       * 不受影响。v=0 原地旋转候选弧长恒 0，不会由此提前退出。 */
      break;
    }
    const float cos_yaw = cosf(state->yaw);
    const float sin_yaw = sinf(state->yaw);

    evaluate_obstacle_ctx_t ctx;
    ctx.collision_footprint = &collision_footprint;
    ctx.state = state;
    ctx.cos_yaw = cos_yaw;
    ctx.sin_yaw = sin_yaw;
    ctx.obstacle_range_squared = obstacle_range_squared;
    ctx.footprint_radius = footprint_radius;
    ctx.min_clearance_squared = &min_clearance_squared;
    ctx.min_clearance = &min_clearance;
    ctx.useful_radius = &useful_radius;
    ctx.useful_radius_squared = &useful_radius_squared;
    ctx.veto_armed = step_veto_armed;

    bool footprint_collision = false;
    if (spatial_index && dwa_spatial_index_valid(spatial_index)) {
      const float query_radius = dwa_min_double(config->obstacle_range, useful_radius);
      footprint_collision = dwa_spatial_index_visit_square(
        spatial_index, state->x, state->y, query_radius,
        dwa_evaluate_obstacle, &ctx);
    } else {
      for (size_t oi = 0; oi < obstacle_count; ++oi) {
        if (dwa_evaluate_obstacle(&obstacles[oi], &ctx)) {
          footprint_collision = true;
          break;
        }
      }
    }
    if (footprint_collision) {
      /* 走到这里的碰撞必在判死视界内（净距视界外的步已 break 跳过；
       * 净距视界内、判死视界外的碰撞已被 veto_armed=false 转为净距 0
       * 满罚，不会到达此分支）：现在刹车来不及，否决该候选。 */
      trajectory->valid = false;
      trajectory->reject_reason = "footprint_collision";
      trajectory->min_clearance = 0.0f;
      trajectory->total_cost = INFINITY;
      return;
    }
  }

  const dwa_state_t *end = &trajectory->states[trajectory->state_count - 1];
  trajectory->min_clearance = min_clearance;
  /* 9.24 转向修复：障碍代价改软边距（对齐 9.20 基线）。
   * 原实现 = obstacle_range - min_clearance 无饱和，净距 2m 和 1m 的候选
   * 代价差被 min-max 归一化放大，导致障碍还很远就选大转避让。
   * 现在净距 >= obstacle_cost_free_margin（默认 0.40m）时代价归零：
   * "够安全就平等"，只有真正逼近障碍才被惩罚。 */
  trajectory->obstacle_cost = min_clearance >= config->obstacle_cost_free_margin ?
    0.0f : config->obstacle_cost_free_margin - min_clearance;
  trajectory->to_goal_cost = hypotf(end->x - local_goal->x, end->y - local_goal->y);
  trajectory->speed_cost = config->use_speed_cost ?
    dwa_max_double(0.0f, dynamic_max_velocity - trajectory->command.v) : 0.0f;
  trajectory->path_cost = 0.0f;
  if (config->use_path_cost && config->path_reference.enabled) {
    const float lookahead = dwa_max_double(config->path_reference.lookahead, 1e-6f);
    float lateral_error_sum = 0.0f;
    float heading_error_sum = 0.0f;
    int sample_count = 0;
    const dwa_state_t *progress_state = NULL;
    for (size_t si = 0; si < trajectory->state_count; ++si) {
      const dwa_state_t *state = &trajectory->states[si];
      if (state->x <= 0.05f) continue;
      const float alpha = dwa_clamp_value(state->x / lookahead, 0.0f, 1.0f);
      lateral_error_sum += fabsf(state->y - alpha * config->path_reference.target_y);
      heading_error_sum += fabsf(dwa_normalize_angle(
        state->yaw - alpha * config->path_reference.target_heading));
      ++sample_count; progress_state = state;
      if (state->x >= lookahead) break;
    }
    if (sample_count > 0 && progress_state != NULL) {
      const float progress = dwa_clamp_value(
        (progress_state->x * cosf(config->path_reference.target_heading) +
        progress_state->y * sinf(config->path_reference.target_heading)) / lookahead,
        0.0f, 1.5f);
      trajectory->path_cost = config->path_reference.lateral_weight *
        lateral_error_sum / sample_count + config->path_reference.heading_weight *
        heading_error_sum / sample_count - config->path_reference.progress_weight * progress;
    }
  }
  trajectory->total_cost = trajectory->obstacle_cost + trajectory->to_goal_cost +
    trajectory->speed_cost + trajectory->path_cost;
  trajectory->valid = true;
  trajectory->reject_reason = NULL;
}

void dwa_evaluate_command_trajectory_into(
  const dwa_velocity_t *command,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *intent,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  dwa_trajectory_t *trajectory,
  const dwa_occupancy_grid_t *grid,
  const dwa_obstacle_spatial_index_t *spatial_index,
  const dwa_grid_blocking_index_t *blocking_index)
{
  dwa_generate_trajectory_into(command, config, trajectory);
  /* 10.09 刀1（直行小角度死区修复）：目标点 = 意图弧端点（速度匹配）。
   * 旧实现横向偏移 = itw/max_yawrate×gain，与车速无关——隐含按 0.7m/s
   * 标定；候选端点横向 ≈ v·w·T²/2 随车速线性放大，评分最优转向被压到
   * 恒定横向/(v·T²/2)：5 档 1.67m/s 时仅 0.43×itw，小意图时最优解落
   * 在 w=0 附近恒选直行（直行带小方向无响应的体感来源）。
   * 现改为沿 (intent.v, w_eff) 恒曲率弧走 goal_distance 弧长的端点：
   * 任意车速下"与意图同曲率的候选"to_goal 代价恒为零 → 评分最优转向
   * = 意图本身（1:1 跟随）。intent_yaw_to_lateral 语义改为跟随增益：
   * 1.0=精确跟随意图，>1=目标点超越意图（更激进），<1=更温和。
   * 9.24 的纵向可达性收缩（goal_distance=min(lookahead, v·T)）保留；
   * 低速/静止时 v_ref 取 0.10 下限，目标点收近但仍带横向牵引。 */
  const float follow_gain = config->intent_yaw_to_lateral;
  const float w_eff = dwa_clamp_value(
    intent->w * follow_gain, -config->max_yawrate, config->max_yawrate);
  const float reachable_distance = dwa_max_double(
    0.10f, intent->v * config->predict_time);
  const float goal_distance = dwa_min_double(
    config->intent_lookahead, reachable_distance);
  const float v_ref = dwa_max_double(fabsf(intent->v), 0.10f);
  const float goal_time = goal_distance / v_ref;
  dwa_point_t local_goal;
  if (fabsf(w_eff) > 1e-6f) {
    const float radius = v_ref / w_eff;   /* 带符号转弯半径 */
    const float psi = w_eff * goal_time;  /* 沿弧累计航向变化 */
    local_goal.x = radius * sinf(psi);
    local_goal.y = radius * (1.0f - cosf(psi));
  } else {
    local_goal.x = goal_distance;         /* 直行意图：目标在正前方 */
    local_goal.y = 0.0f;
  }
  dwa_evaluate_trajectory(
    trajectory, obstacles, obstacle_count, &local_goal, dynamic_max_velocity, config,
    grid, spatial_index, blocking_index);
}

void dwa_evaluate_forward_clearance_trajectory(
  float lookahead_distance,
  float probe_speed,
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *intent,
  float dynamic_max_velocity,
  const dwa_config_t *config,
  const dwa_occupancy_grid_t *grid,
  dwa_trajectory_t *trajectory)
{
  if (!isfinite(lookahead_distance) || lookahead_distance <= 0.0f ||
    !isfinite(probe_speed) || !isfinite(config->sim_dt) || config->sim_dt <= 0.0f)
  {
    /* DwaTrajectory invalid; invalid.command = {}; reject_reason = ...（复用缓冲） */
    const dwa_velocity_t zero = {0.0f, 0.0f};
    dwa_reset_trajectory(trajectory, &zero);
    trajectory->reject_reason = "invalid_forward_clearance_probe";
    return;
  }

  const float linear = dwa_max_double(0.10f, fabsf(probe_speed));
  dwa_config_t validation_config = *config;
  validation_config.predict_time = dwa_max_double(
    validation_config.sim_dt, lookahead_distance / linear);
  /* Fix V 豁免：前向净距探针是 prebrake/前向硬停的数据源，安全层
   * 必须看到全 lookahead 视界的碰撞，不参与判死视距解耦。 */
  validation_config.collision_veto_enabled = false;
  const float spatial_bucket_size = grid && dwa_grid_valid(grid) ?
    dwa_max_double(0.50f, grid->resolution * 10.0f) : 0.50f;
  dwa_obstacle_spatial_index_t spatial_index;
  dwa_obstacle_spatial_index_init(&spatial_index);
  dwa_build_obstacle_spatial_index(obstacles, obstacle_count, spatial_bucket_size,
    &spatial_index);
  dwa_grid_blocking_index_t blocking_index;
  dwa_grid_blocking_index_init(&blocking_index);
  if (grid && dwa_grid_valid(grid)) {
    dwa_build_grid_blocking_index(grid, validation_config.unknown_is_obstacle,
      &blocking_index);
  } else {
    /* blocking_index = DwaGridBlockingIndex{}：置空 */
    dwa_grid_blocking_index_destroy(&blocking_index);
    dwa_grid_blocking_index_init(&blocking_index);
  }
  const dwa_velocity_t probe_command = {linear, 0.0f};
  dwa_evaluate_command_trajectory_into(
    &probe_command, obstacles, obstacle_count, intent, dynamic_max_velocity,
    &validation_config, trajectory, grid, &spatial_index, &blocking_index);
  dwa_grid_blocking_index_destroy(&blocking_index);
  dwa_obstacle_spatial_index_destroy(&spatial_index);
}

float dwa_normalize_cost_component(float value, float minimum, float maximum)
{
  const float span = maximum - minimum;
  if (span <= FLT_EPSILON) return 0.0f;
  return (value - minimum) / span;
}

void dwa_normalize_trajectory_costs(
  dwa_trajectory_t *trajectories,
  size_t trajectory_count,
  const dwa_config_t *config)
{
  float min_obstacle = INFINITY;
  float max_obstacle = -INFINITY;
  float min_goal = INFINITY;
  float max_goal = -INFINITY;
  float min_speed = INFINITY;
  float max_speed = -INFINITY;
  float min_path = INFINITY;
  float max_path = -INFINITY;

  for (size_t i = 0; i < trajectory_count; ++i) {
    const dwa_trajectory_t *trajectory = &trajectories[i];
    if (!trajectory->valid) continue;
    min_obstacle = dwa_min_double(min_obstacle, trajectory->obstacle_cost);
    max_obstacle = dwa_max_double(max_obstacle, trajectory->obstacle_cost);
    min_goal = dwa_min_double(min_goal, trajectory->to_goal_cost);
    max_goal = dwa_max_double(max_goal, trajectory->to_goal_cost);
    min_speed = dwa_min_double(min_speed, trajectory->speed_cost);
    max_speed = dwa_max_double(max_speed, trajectory->speed_cost);
    min_path = dwa_min_double(min_path, trajectory->path_cost);
    max_path = dwa_max_double(max_path, trajectory->path_cost);
  }

  for (size_t i = 0; i < trajectory_count; ++i) {
    dwa_trajectory_t *trajectory = &trajectories[i];
    if (!trajectory->valid) continue;
    trajectory->obstacle_cost = dwa_normalize_cost_component(
      trajectory->obstacle_cost, min_obstacle, max_obstacle);
    trajectory->to_goal_cost = dwa_normalize_cost_component(
      trajectory->to_goal_cost, min_goal, max_goal);
    trajectory->speed_cost = config->use_speed_cost ? dwa_normalize_cost_component(
      trajectory->speed_cost, min_speed, max_speed) : 0.0f;
    trajectory->path_cost = config->use_path_cost ? dwa_normalize_cost_component(
      trajectory->path_cost, min_path, max_path) : 0.0f;
    float obstacle_scale = 1.0f;
    /* 旧做法只在 40→30cm 额外惩罚直行，却没有压住 DWA 已经在约1m
     * 就开始选弯的原 obstacle_cost，所以实车看起来完全没变化。
     * 现在只在“正向直行 DWA + 两侧都有余量”时，把前方避障评分从
     * 40cm起平滑打开、30cm达到原强度。碰撞否决/硬停止不受此缩放影响；
     * 任一侧靠墙时不缩放，弯道侧墙仍走原避障评分。 */
    if (config->front_turn_bias_active &&
      isfinite(config->front_turn_bias_clearance_m) &&
      config->front_turn_bias_start_m > config->front_turn_bias_full_m)
    {
      const float t = (config->front_turn_bias_start_m -
        config->front_turn_bias_clearance_m) /
        (config->front_turn_bias_start_m - config->front_turn_bias_full_m);
      if (t <= 0.0f) {
        obstacle_scale = 0.0f;
      } else {
        const float u = t >= 1.0f ? 1.0f : t;
        obstacle_scale = u * u * (3.0f - 2.0f * u);
      }
    }
    trajectory->total_cost =
      config->obstacle_cost_gain * obstacle_scale * trajectory->obstacle_cost +
      config->to_goal_cost_gain * trajectory->to_goal_cost +
      config->speed_cost_gain * trajectory->speed_cost +
      config->path_cost_gain * trajectory->path_cost;
  }
}
