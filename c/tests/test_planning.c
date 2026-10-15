/*
 * P2 parity（C 侧）：固定种子伪随机场景（速度 0~1.7m/s、角速度 ±1.2、随机点云），
 * 跑 dwa_plan_dwa_into，dump CSV。
 * 用法: test_planning [输出csv路径] [场景数]
 * 场景生成与 tools/parity/parity_cpp.cpp 逐字节一致（同一 LCG）。
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/planning/dwa_sampling.h"

static uint64_t rng_state = 0x9E3779B97F4A7C15ULL;

static uint64_t rng_next(void)
{
  rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return rng_state;
}

static double rng_unit(void)
{
  return (double)(rng_next() >> 11) * (1.0 / 9007199254740992.0);
}

#define MAX_OBSTACLES 128
#define SCENARIO_COUNT_DEFAULT 256

int main(int argc, char **argv)
{
  const char *out_path = argc > 1 ? argv[1] : "planning_c.csv";
  const int scenarios = argc > 2 ? atoi(argv[2]) : SCENARIO_COUNT_DEFAULT;
  FILE *out = fopen(out_path, "w");
  if (out == NULL) {
    fprintf(stderr, "cannot open %s\n", out_path);
    return 1;
  }

  /* 与 C++ 版相同的测试配置：DwaConfig 默认 + 打开速度/路径代价分支。 */
  dwa_config_t config = dwa_config_default();
  config.use_speed_cost = true;
  config.use_path_cost = true;
  config.path_reference.enabled = true;
  config.path_reference.target_y = 0.30;
  config.path_reference.target_heading = 0.20;

  dwa_plan_result_t result;
  dwa_plan_result_init(&result);

  for (int s = 0; s < scenarios; ++s) {
    const int n_obs = 1 + (int)(rng_unit() * 80.0); /* 1..80 */
    dwa_point_t obstacles[MAX_OBSTACLES];
    for (int i = 0; i < n_obs && i < MAX_OBSTACLES; ++i) {
      obstacles[i].x = -1.0 + rng_unit() * 5.0;
      obstacles[i].y = -3.0 + rng_unit() * 6.0;
    }
    const int obs_count = n_obs < MAX_OBSTACLES ? n_obs : MAX_OBSTACLES;
    dwa_velocity_t current;
    current.v = rng_unit() * 1.7;          /* 0 ~ 1.7 m/s */
    current.w = rng_unit() * 2.4 - 1.2;    /* -1.2 ~ 1.2 */
    dwa_velocity_t intent;
    intent.v = rng_unit() * 1.7;
    intent.w = rng_unit() * 2.4 - 1.2;

    dwa_plan_dwa_into(obstacles, obs_count, &current, &intent, &config, &result,
      NULL, NULL, NULL);

    const dwa_trajectory_t *sel = &result.selected_trajectory;
    fprintf(out,
      "s,%d,%d,%.17g,%.17g,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s\n",
      s, obs_count, current.v, current.w, result.valid_count,
      sel->valid ? 1 : 0,
      sel->min_clearance, sel->total_cost, sel->obstacle_cost, sel->to_goal_cost,
      sel->speed_cost, sel->path_cost,
      sel->reject_reason ? sel->reject_reason : "-");
    for (size_t i = 0; i < result.trajectory_count; ++i) {
      const dwa_trajectory_t *t = &result.trajectories[i];
      fprintf(out,
        "c,%d,%zu,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s\n",
        s, i, t->command.v, t->command.w, t->valid ? 1 : 0,
        t->min_clearance, t->total_cost, t->obstacle_cost, t->to_goal_cost,
        t->speed_cost, t->path_cost, t->reject_reason ? t->reject_reason : "-");
    }
  }

  dwa_plan_result_destroy(&result);
  fclose(out);
  printf("test_planning: %d scenarios -> %s\n", scenarios, out_path);
  return 0;
}
