#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../src/planning/dwa_sampling.h"

static void configure(dwa_config_t *config)
{
  *config = dwa_config_default();
  config->max_velocity = 0.70f;
  config->max_yawrate = 1.40f;
  config->max_acceleration = 1.50f;
  config->max_deceleration = 1.50f;
  config->sampling_deceleration_window = 3.00f;
  config->max_d_yawrate = 12.00f;
  config->predict_time = 2.00f;
  config->sim_dt = 0.10f;
  config->control_period = 0.10f;
  config->velocity_samples = 4;
  config->yawrate_samples = 27;
  config->obstacle_range = 3.50f;
  config->obstacle_cost_free_margin = 0.08f;
  config->collision_veto_enabled = true;
  config->collision_veto_reaction_sec = 0.30f;
  config->collision_veto_min_sec = 0.50f;
  config->collision_veto_max_sec = 2.00f;
  config->clearance_horizon_m = 1.60f;
  config->dwa_clearance_buffer_m = 0.15f;
  config->toe_collision_margin_m = 0.10f;
  config->use_speed_cost = false;
  config->intent_freeze_min_yawrate = 0.50f;
}

int main(void)
{
  dwa_config_t config;
  configure(&config);
  const dwa_point_t obstacles[] = {
    {1.160651207f, 0.059677899f}, {0.862104297f, 0.578437567f},
    {0.833050847f, 0.297736526f}, {0.502307951f, 0.482613444f},
    {1.521963120f, 0.186711252f}, {0.589385390f, -0.245015681f},
    {1.583587170f, 0.547864676f}, {0.582877159f, 0.380965948f},
    {0.827234387f, 0.242275536f}, {1.594690323f, -0.680997670f},
    {0.451239586f, 0.332551003f}, {0.583410203f, -0.513747096f},
    {0.706609845f, 0.570222974f}, {1.335721016f, 0.284807205f},
    {1.182155848f, 0.232265532f}, {0.478686601f, 0.129364192f},
    {0.504857957f, 0.705170870f}, {0.666639388f, 0.312314034f},
    {0.949217200f, 0.090021193f}, {1.386430144f, -0.082840204f},
    {0.949339986f, 0.476994276f}, {0.924785912f, 0.366216660f},
    {0.807507277f, -0.587259471f}, {1.341527820f, -0.436474800f},
    {1.592978716f, 0.459067941f}, {1.392398596f, -0.531082034f},
    {1.524780273f, 0.116373420f}
  };
  const dwa_velocity_t current = {0.266f, 0.280f};
  const dwa_velocity_t intent = {0.679f, 0.0f};
  dwa_plan_result_t result;
  dwa_plan_result_init(&result);
  dwa_plan_dwa_into(obstacles, sizeof(obstacles) / sizeof(obstacles[0]),
    &current, &intent, &config, &result, NULL, NULL, NULL);

  const dwa_trajectory_t *zero_straight = NULL;
  const dwa_trajectory_t *best_forward = NULL;
  for (size_t i = 0; i < result.trajectory_count; ++i) {
    const dwa_trajectory_t *t = &result.trajectories[i];
    if (!t->valid) continue;
    if (fabsf(t->command.v) <= 1e-3f && fabsf(t->command.w) <= 1e-3f) {
      zero_straight = t;
    }
    if (t->command.v > 1e-3f && t->min_clearance > 1e-4f &&
        (best_forward == NULL || t->total_cost < best_forward->total_cost)) {
      best_forward = t;
    }
  }
  const int passed = result.valid_count > 0 && zero_straight != NULL &&
    best_forward != NULL && zero_straight->total_cost < best_forward->total_cost &&
    result.selected.v > 1e-3f && result.selected_trajectory.valid &&
    result.selected_trajectory.min_clearance > 1e-4f;
  printf("valid=%d zero_cost=%.6f best_forward_cost=%.6f selected=(%.3f,%+.3f) clearance=%.3f\n",
    result.valid_count,
    zero_straight ? zero_straight->total_cost : INFINITY,
    best_forward ? best_forward->total_cost : INFINITY,
    result.selected.v, result.selected.w,
    result.selected_trajectory.min_clearance);
  dwa_plan_result_destroy(&result);
  if (!passed) {
    fprintf(stderr, "zero-speed progress regression failed\n");
    return 1;
  }
  return 0;
}

