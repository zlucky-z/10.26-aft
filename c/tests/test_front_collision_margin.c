#include <stdio.h>

#include "../src/planning/dwa_scoring.h"

static int evaluate(const dwa_point_t *obstacle, float margin)
{
  dwa_config_t config = dwa_config_default();
  dwa_trajectory_t trajectory;
  dwa_velocity_t command = {0.50f, 0.0f};
  dwa_velocity_t intent = {0.0f, 0.0f};
  config.predict_time = 0.20f;
  config.sim_dt = 0.10f;
  config.obstacle_range = 2.0f;
  config.toe_collision_margin_m = margin;
  config.collision_veto_enabled = true;
  config.collision_veto_min_sec = 0.50f;
  config.collision_veto_max_sec = 2.00f;
  config.collision_veto_reaction_sec = 0.30f;
  config.max_deceleration = 1.50f;
  config.footprint.front = 0.50f;
  config.footprint.footrest_start = 0.57f;
  config.footprint.footrest_front = 0.75f;
  config.footprint.footrest_half_width = 0.23f;
  config.footprint.toe_front = 0.90f;
  config.footprint.toe_half_width = 0.23f;
  config.footprint.padding = 0.0f;
  dwa_trajectory_init(&trajectory);
  dwa_evaluate_command_trajectory_into(
    &command, obstacle, 1, &intent, 0.0f, &config, &trajectory,
    NULL, NULL, NULL);
  {
    const int valid = trajectory.valid ? 1 : 0;
    dwa_trajectory_destroy(&trajectory);
    return valid;
  }
}

int main(void)
{
  const dwa_point_t frontal_point = {1.05f, 0.0f};
  const dwa_point_t side_wall = {0.95f, 0.50f};
  if (!evaluate(&frontal_point, 0.0f)) {
    fprintf(stderr, "unbuffered toe footprint should allow this non-contact trajectory\n");
    return 1;
  }
  if (evaluate(&frontal_point, 0.10f)) {
    fprintf(stderr, "toe margin must veto a point inside the forward buffer\n");
    return 1;
  }
  if (!evaluate(&side_wall, 0.10f)) {
    fprintf(stderr, "toe margin must not widen the footprint toward corridor walls\n");
    return 1;
  }
  puts("test_front_collision_margin: passed");
  return 0;
}
