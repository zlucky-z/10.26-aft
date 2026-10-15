/* 直译自 native/planning/dwa_trajectory.hpp。 */
#include "dwa_trajectory.h"

#include <string.h>

#include "../common/dwa_geometry.h"

static int dwa_max_int(int a, int b) { return a < b ? b : a; }

void dwa_reset_trajectory(dwa_trajectory_t *trajectory, const dwa_velocity_t *command)
{
  trajectory->state_count = 0;  /* states.clear()：容量保留 */
  trajectory->command = *command;
  trajectory->valid = false;
  trajectory->min_clearance = INFINITY;
  trajectory->total_cost = INFINITY;
  trajectory->obstacle_cost = 0.0f;
  trajectory->to_goal_cost = 0.0f;
  trajectory->speed_cost = 0.0f;
  trajectory->path_cost = 0.0f;
  trajectory->reject_reason = NULL; /* reject_reason.clear() */
}

void dwa_generate_trajectory_into(
  const dwa_velocity_t *command,
  const dwa_config_t *config,
  dwa_trajectory_t *trajectory)
{
  dwa_reset_trajectory(trajectory, command);

  dwa_state_t state;
  memset(&state, 0, sizeof(state)); /* DwaState 默认成员全 0 */
  const int steps = dwa_max_int(
    1, (int)ceilf(config->predict_time / config->sim_dt));
  if (trajectory->state_capacity < (size_t)steps) {
    dwa_trajectory_states_reserve(trajectory, (size_t)steps);
  }
  for (int i = 0; i < steps; ++i) {
    dwa_state_t next;
    dwa_motion_step(&state, command->v, command->w, config->sim_dt, &next);
    state = next;
    dwa_trajectory_states_push_back(trajectory, &state);
  }
}
