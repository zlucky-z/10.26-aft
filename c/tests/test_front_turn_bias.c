#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../src/planning/dwa_scoring.h"

static void fail(const char *message)
{
  fprintf(stderr, "test_front_turn_bias: %s\n", message);
  exit(1);
}

static void prepare(dwa_trajectory_t *trajectories)
{
  for (int i = 0; i < 2; ++i) {
    trajectories[i].valid = true;
    trajectories[i].command.v = 0.50f;
    trajectories[i].command.w = 0.0f;
    trajectories[i].obstacle_cost = (float)i;
    trajectories[i].to_goal_cost = 0.0f;
    trajectories[i].speed_cost = 0.0f;
    trajectories[i].path_cost = 0.0f;
  }
}

int main(void)
{
  dwa_config_t config = dwa_config_default();
  dwa_trajectory_t trajectories[2] = {0};

  config.front_turn_bias_active = true;
  config.front_turn_bias_clearance_m = 0.80f;
  prepare(trajectories);
  dwa_normalize_trajectory_costs(trajectories, 2, &config);
  if (fabsf(trajectories[1].total_cost - trajectories[0].total_cost) > 1e-6f)
    fail("above 40cm, front soft scoring should not cause early avoidance");

  config.front_turn_bias_clearance_m = 0.35f;
  prepare(trajectories);
  dwa_normalize_trajectory_costs(trajectories, 2, &config);
  if (fabsf((trajectories[1].total_cost - trajectories[0].total_cost) - 0.5f) > 1e-5f)
    fail("at 35cm, front soft scoring should be half-strength");

  config.front_turn_bias_clearance_m = 0.30f;
  prepare(trajectories);
  dwa_normalize_trajectory_costs(trajectories, 2, &config);
  if (fabsf((trajectories[1].total_cost - trajectories[0].total_cost) - 1.0f) > 1e-5f)
    fail("at/below 30cm, front soft scoring should reach full strength");

  config.front_turn_bias_active = false;
  config.front_turn_bias_clearance_m = 0.80f;
  prepare(trajectories);
  dwa_normalize_trajectory_costs(trajectories, 2, &config);
  if (fabsf((trajectories[1].total_cost - trajectories[0].total_cost) - 1.0f) > 1e-5f)
    fail("outside straight/open-side gate, preserve existing obstacle scoring");

  puts("test_front_turn_bias: PASS");
  return 0;
}
