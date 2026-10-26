/* 直译自 native/planning/dwa_sampling.hpp。 */
#include "dwa_sampling.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../common/dwa_geometry.h"
#include "dwa_trajectory.h"

#if defined(__linux__)
/* 恢复 C++ OpenMP 并行（#pragma omp parallel for schedule(static)
 * num_threads(planning_worker_count)）的 pthread 等价实现。候选轨迹互不写
 * 共享状态（各写自己的 result->trajectories 槽位），obstacles/config/grid/
 * 空间索引/阻塞索引在评估期间只读——并行与串行结果逐位一致。仅 Linux
 * 启用；Windows 宿主对拍走串行路径。 */
#define DWA_PLAN_PTHREAD 1
#include <pthread.h>
#endif

static int dwa_max_int(int a, int b) { return a < b ? b : a; }
static float dwa_min_double(float a, float b) { return b < a ? b : a; }
static float dwa_max_double(float a, float b) { return a < b ? b : a; }

/* Select the lowest-cost finite candidate, optionally requiring forward
 * progress.  A zero-speed candidate remains available as the explicit stop
 * fallback when no finite positive-speed candidate exists. */
static bool dwa_select_best_candidate(
  dwa_plan_result_t *result, const dwa_velocity_t *intent,
  bool require_positive_speed)
{
  const dwa_trajectory_t *best = NULL;
  const float tie_epsilon = 1e-3f;

  for (size_t i = 0; i < result->trajectory_count; ++i) {
    const dwa_trajectory_t *candidate = &result->trajectories[i];
    if (!candidate->valid || !isfinite(candidate->total_cost)) continue;
    if (require_positive_speed && candidate->command.v <= 1e-6f) continue;
    if (best == NULL || candidate->total_cost < best->total_cost - tie_epsilon ||
        (fabsf(candidate->total_cost - best->total_cost) <= tie_epsilon &&
         fabsf(candidate->command.w - intent->w) <
           fabsf(best->command.w - intent->w) - 1e-6f)) {
      best = candidate;
    }
  }

  if (best == NULL) return false;
  result->selected = best->command;
  dwa_trajectory_copy(&result->selected_trajectory, best);
  return true;
}

/* ---- 候选评估分片（原循环体提取，字段全只读/独立） ---- */
typedef struct {
  int begin;
  int end;
  int candidates_per_velocity;
  int v_samples;
  int w_samples;
  float v_low;
  float v_high;
  float w_low;
  float w_high;
  const dwa_point_t *obstacles;
  size_t obstacle_count;
  dwa_velocity_t intent;
  const dwa_config_t *config;
  dwa_plan_result_t *result;
  const dwa_occupancy_grid_t *grid;
  const dwa_obstacle_spatial_index_t *spatial_index;
} dwa_plan_chunk_t;

static void dwa_plan_evaluate_range(const dwa_plan_chunk_t *c)
{
  for (int candidate_index = c->begin; candidate_index < c->end;
       ++candidate_index) {
    const int i = candidate_index / c->candidates_per_velocity;
    const int j = candidate_index % c->candidates_per_velocity;
    const float alpha = (float)i / (float)(c->v_samples - 1);
    const float v = c->v_low + (c->v_high - c->v_low) * alpha;
    float w = 0.0f;
    if (j < c->w_samples) {
      const float beta = (float)j / (float)(c->w_samples - 1);
      w = c->w_low + (c->w_high - c->w_low) * beta;
      if (v < c->config->slow_velocity_threshold) {
        w = w > 0.0f ?
          dwa_max_double(w, c->config->min_yawrate) :
          dwa_min_double(w, -c->config->min_yawrate);
        w = dwa_clamp_value(w, -c->config->max_yawrate, c->config->max_yawrate);
      }
    }
    const dwa_velocity_t command = {v, w};
    dwa_evaluate_command_trajectory_into(
      &command, c->obstacles, c->obstacle_count, &c->intent, c->v_high,
      c->config, &c->result->trajectories[(size_t)candidate_index],
      c->grid, c->spatial_index, NULL);
  }
}

#ifdef DWA_PLAN_PTHREAD
#define DWA_PLAN_MAX_WORKERS 16

static void *dwa_plan_chunk_thread(void *arg)
{
  dwa_plan_evaluate_range((const dwa_plan_chunk_t *)arg);
  return NULL;
}
#endif

void dwa_plan_result_init(dwa_plan_result_t *result)
{
  memset(result, 0, sizeof(*result));
  dwa_trajectory_init(&result->selected_trajectory);
}

void dwa_plan_result_destroy(dwa_plan_result_t *result)
{
  dwa_trajectory_destroy(&result->selected_trajectory);
  if (result->trajectories != NULL) {
    for (size_t i = 0; i < result->trajectory_capacity; ++i) {
      dwa_trajectory_destroy(&result->trajectories[i]);
    }
    free(result->trajectories);
  }
  result->trajectories = NULL;
  result->trajectory_count = 0;
  result->trajectory_capacity = 0;
}

void dwa_plan_dwa_into(
  const dwa_point_t *obstacles,
  size_t obstacle_count,
  const dwa_velocity_t *current_velocity,
  const dwa_velocity_t *user_intent,
  const dwa_config_t *config,
  dwa_plan_result_t *result,
  const dwa_occupancy_grid_t *grid,
  dwa_obstacle_spatial_index_t *spatial_index_workspace,
  dwa_grid_blocking_index_t *blocking_index_workspace)
{
  const dwa_velocity_t zero = {0.0f, 0.0f};
  result->selected = zero;
  dwa_reset_trajectory(&result->selected_trajectory, &zero);
  result->valid_count = 0;
  dwa_velocity_t intent;
  intent.v = dwa_clamp_value(user_intent->v, config->min_velocity, config->max_velocity);
  intent.w = dwa_clamp_value(user_intent->w, -config->max_yawrate, config->max_yawrate);

  dwa_velocity_t current;
  current.v = dwa_clamp_value(current_velocity->v, config->min_velocity, config->max_velocity);
  current.w = dwa_clamp_value(current_velocity->w, -config->max_yawrate, config->max_yawrate);

  /* G1(10.09)：窗口下界用 sampling_deceleration_window（>0 时），
   * 与判死视界 T_veto（dwa_scoring.c 仍用 max_deceleration）解耦。
   * 拐角断崖时窗口能下探更深（3.00×0.10=0.30/tick），保留中间刹车
   * 候选，避免"全死→fallback(0,0)→实速崩 0→窗口塌缩"级联（摇摇车）。
   * <=0 时退回 max_deceleration，行为与原版逐位一致。 */
  const float window_decel = config->sampling_deceleration_window > 0.0f ?
    config->sampling_deceleration_window : config->max_deceleration;
  float v_low = dwa_max_double(
    current.v - window_decel * config->control_period,
    config->min_velocity);
  const float v_high = dwa_min_double(
    current.v + config->max_acceleration * config->control_period,
    intent.v);
  if (v_low > v_high) v_low = v_high;
  const float w_low = dwa_max_double(
    current.w - config->max_d_yawrate * config->control_period,
    -config->max_yawrate);
  const float w_high = dwa_min_double(
    current.w + config->max_d_yawrate * config->control_period,
    config->max_yawrate);

  const int v_samples = dwa_max_int(2, config->velocity_samples);
  const int w_samples = dwa_max_int(2, config->yawrate_samples);
  const bool include_straight = w_low < 0.0f && w_high > 0.0f;
  const int candidates_per_velocity = w_samples + (include_straight ? 1 : 0);
  const int candidate_count = v_samples * candidates_per_velocity;
  /* result.trajectories.resize(candidate_count)：新增元素默认构造，容量复用。 */
  if (result->trajectory_capacity < (size_t)candidate_count) {
    size_t next = result->trajectory_capacity > 0 ? result->trajectory_capacity : 16;
    while (next < (size_t)candidate_count) next *= 2;
    dwa_trajectory_t *grown = (dwa_trajectory_t *)realloc(
      result->trajectories, next * sizeof(dwa_trajectory_t));
    for (size_t i = result->trajectory_capacity; i < next; ++i) {
      dwa_trajectory_init(&grown[i]);
    }
    result->trajectories = grown;
    result->trajectory_capacity = next;
  }
  result->trajectory_count = (size_t)candidate_count;
  /* 9.24 性能修复：空间桶 0.25→0.50（栅格分辨率×10），对齐 9.20 基线，
   * 减少逐步碰撞的桶扫描次数。 */
  const float spatial_bucket_size = grid && dwa_grid_valid(grid) ?
    dwa_max_double(0.50f, grid->resolution * 10.0f) : 0.50f;
  dwa_obstacle_spatial_index_t local_spatial_index;
  dwa_obstacle_spatial_index_init(&local_spatial_index);
  dwa_obstacle_spatial_index_t *spatial_index = spatial_index_workspace ?
    spatial_index_workspace : &local_spatial_index;
  dwa_build_obstacle_spatial_index(obstacles, obstacle_count, spatial_bucket_size,
    spatial_index);
  /* 9.24 性能修复：逐步碰撞已停用栅格盒查询（见 dwa_scoring.c），不再
   * 每次规划重建栅格阻塞索引；workspace 参数保留以兼容既有调用方。 */
  (void)blocking_index_workspace;

  /* 候选轨迹互不写共享状态；C++ OpenMP 静态分片的 pthread 等价实现
   * （schedule(static)：块大小 = ceil(N/W)，线程 t 处理
   * [t*block, min((t+1)*block, N))），结果与串行/C++ 并行逐位一致。 */
  {
#ifdef DWA_PLAN_PTHREAD
    int worker_count = config->planning_worker_count;
    if (worker_count < 1) worker_count = 1;
    if (worker_count > DWA_PLAN_MAX_WORKERS) worker_count = DWA_PLAN_MAX_WORKERS;
    if (worker_count > 1 && candidate_count >= 4) {
      const int block = (candidate_count + worker_count - 1) / worker_count;
      dwa_plan_chunk_t chunks[DWA_PLAN_MAX_WORKERS];
      pthread_t threads[DWA_PLAN_MAX_WORKERS - 1];
      int spawned = 0;
      int t;
      for (t = 0; t < worker_count; ++t) {
        const int end = (t + 1) * block;
        chunks[t].begin = t * block;
        chunks[t].end = end > candidate_count ? candidate_count : end;
        chunks[t].candidates_per_velocity = candidates_per_velocity;
        chunks[t].v_samples = v_samples;
        chunks[t].w_samples = w_samples;
        chunks[t].v_low = v_low;
        chunks[t].v_high = v_high;
        chunks[t].w_low = w_low;
        chunks[t].w_high = w_high;
        chunks[t].obstacles = obstacles;
        chunks[t].obstacle_count = obstacle_count;
        chunks[t].intent = intent;
        chunks[t].config = config;
        chunks[t].result = result;
        chunks[t].grid = grid;
        chunks[t].spatial_index = spatial_index;
      }
      for (t = 1; t < worker_count && chunks[t].begin < chunks[t].end; ++t) {
        if (pthread_create(&threads[spawned], NULL, dwa_plan_chunk_thread,
              &chunks[t]) != 0) {
          break;
        }
        ++spawned;
      }
      if (chunks[0].begin < chunks[0].end) {
        dwa_plan_evaluate_range(&chunks[0]);
      }
      /* 创建失败的线程的分片由主线程兜底（保持全部候选被评估）。 */
      for (t = 1; t < worker_count; ++t) {
        if (t - 1 >= spawned && chunks[t].begin < chunks[t].end) {
          dwa_plan_evaluate_range(&chunks[t]);
        }
      }
      for (t = 0; t < spawned; ++t) {
        pthread_join(threads[t], NULL);
      }
    } else {
      dwa_plan_chunk_t chunk;
      chunk.begin = 0;
      chunk.end = candidate_count;
      chunk.candidates_per_velocity = candidates_per_velocity;
      chunk.v_samples = v_samples;
      chunk.w_samples = w_samples;
      chunk.v_low = v_low;
      chunk.v_high = v_high;
      chunk.w_low = w_low;
      chunk.w_high = w_high;
      chunk.obstacles = obstacles;
      chunk.obstacle_count = obstacle_count;
      chunk.intent = intent;
      chunk.config = config;
      chunk.result = result;
      chunk.grid = grid;
      chunk.spatial_index = spatial_index;
      dwa_plan_evaluate_range(&chunk);
    }
#else
    dwa_plan_chunk_t chunk;
    chunk.begin = 0;
    chunk.end = candidate_count;
    chunk.candidates_per_velocity = candidates_per_velocity;
    chunk.v_samples = v_samples;
    chunk.w_samples = w_samples;
    chunk.v_low = v_low;
    chunk.v_high = v_high;
    chunk.w_low = w_low;
    chunk.w_high = w_high;
    chunk.obstacles = obstacles;
    chunk.obstacle_count = obstacle_count;
    chunk.intent = intent;
    chunk.config = config;
    chunk.result = result;
    chunk.grid = grid;
    chunk.spatial_index = spatial_index;
    dwa_plan_evaluate_range(&chunk);
#endif
  }

  for (size_t i = 0; i < result->trajectory_count; ++i) {
    if (result->trajectories[i].valid) ++result->valid_count;
  }

  if (result->valid_count == 0) {
    result->selected = zero;
    dwa_generate_trajectory_into(&zero, config, &result->selected_trajectory);
    result->selected_trajectory.valid = false;
    result->selected_trajectory.reject_reason = "no_valid_trajectory";
    dwa_obstacle_spatial_index_destroy(&local_spatial_index);
    return;
  }

  /* AC2(10.10) 方案C：意图方向候选全判死 → 冻结输出(0,0)。
   * 003347 定案：窄缝塌缩段强打杆(|iw|>=0.5) 1529 行中 6% 输出与
   * 意图反向（实例：用户推左 -1.10，sel_w=+0.10 右转持续 10+ 拍）。
   * 机理：意图方向弧被几何判死（扫掠半径超侧净距）后，评分只剩
   * 反向小弧/原地候选——v=0 候选 w 对位置代价无影响，方向任意；
   * 反向弧被障碍罚分化后胜出 = "不受控"体感。改为：|intent.w|>=门槛
   * 且有效候选中无任何同号 w 候选时，走 (0,0) fallback（同
   * no_valid_trajectory 路径：selected_trajectory.invalid → tick 层
   * 几何选择器跳过、安全层既有兜底）。同号小弧/原地转向候选仍活时
   * 不触发（评分照常）。门槛<=0 禁用（原行为）。 */
  if (config->intent_freeze_min_yawrate > 0.0f &&
    fabsf(intent.w) >= config->intent_freeze_min_yawrate)
  {
    bool intent_direction_alive = false;
    for (size_t i = 0; i < result->trajectory_count; ++i) {
      const dwa_trajectory_t *trajectory = &result->trajectories[i];
      if (trajectory->valid && trajectory->command.w * intent.w > 0.0f) {
        intent_direction_alive = true;
        break;
      }
    }
    if (!intent_direction_alive) {
      result->selected = zero;
      dwa_generate_trajectory_into(&zero, config,
        &result->selected_trajectory);
      result->selected_trajectory.valid = false;
      result->selected_trajectory.reject_reason = "intent_direction_all_vetoed";
      dwa_obstacle_spatial_index_destroy(&local_spatial_index);
      return;
    }
  }

  dwa_normalize_trajectory_costs(result->trajectories, result->trajectory_count, config);
  /* A forward intent must keep moving whenever a safe positive-speed
   * candidate exists.  Previously the normalized score could select a
   * zero-speed arc because it had lower obstacle/goal cost, collapsing the
   * next dynamic window around the measured near-zero speed. */
  {
    const bool forward_progress_requested =
      intent.v > dwa_max_double(config->min_velocity, 1e-3f);
    bool selected = dwa_select_best_candidate(
      result, &intent, forward_progress_requested);
    if (!selected && forward_progress_requested) {
      /* No safe forward progress remains: select a finite stop candidate if
       * one exists.  The caller can then execute the normal zero-speed stop
       * path without inventing a non-safe velocity. */
      selected = dwa_select_best_candidate(result, &intent, false);
    }
    if (!selected) {
      result->selected = zero;
      dwa_generate_trajectory_into(&zero, config, &result->selected_trajectory);
      result->selected_trajectory.valid = false;
      result->selected_trajectory.reject_reason = "no_finite_valid_trajectory";
    }
  }

  dwa_obstacle_spatial_index_destroy(&local_spatial_index);
}

int dwa_turn_direction(float yawrate, float epsilon)
{
  if (yawrate > epsilon) return 1;
  if (yawrate < -epsilon) return -1;
  return 0;
}
