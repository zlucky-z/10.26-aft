#include <stdio.h>

#include "../src/safety/side_safety.h"

static dwa_side_safety_config_t test_config(void)
{
  dwa_side_safety_config_t config = dwa_side_safety_config_default();
  config.triangle_enabled = true;
  config.triangle_stop_gap_m = 0.20f;
  config.footprint.front = 0.50f;
  config.footprint.back = 0.14f;
  config.footprint.half_width = 0.295f;
  config.footprint.footrest_start = 0.57f;
  config.footprint.footrest_front = 0.75f;
  config.footprint.footrest_half_width = 0.23f;
  config.footprint.toe_front = 0.90f;
  config.footprint.toe_half_width = 0.23f;
  return config;
}

static int expect_stop(
  const char *name, const dwa_velocity_t *command,
  const dwa_point_t *point, int expected)
{
  const dwa_side_safety_config_t config = test_config();
  const planner_side_hard_stop_decision_t result =
    dwa_evaluate_side_triangle_stop(command, true, point, 1, &config);
  if ((result.stop ? 1 : 0) != expected) {
    fprintf(stderr, "%s: expected stop=%d got=%d\n",
      name, expected, result.stop ? 1 : 0);
    return 1;
  }
  return 0;
}

int main(void)
{
  const dwa_point_t left_near = {0.50f, 0.45f};
  const dwa_point_t left_far = {0.50f, 0.60f};
  const dwa_point_t right_near = {0.50f, -0.45f};
  const dwa_velocity_t forward_left = {0.70f, 1.00f};
  const dwa_velocity_t forward_right = {0.70f, -1.00f};
  const dwa_velocity_t reverse_left = {-0.20f, 0.50f};
  int failures = 0;

  failures += expect_stop("forward left toward left obstacle",
    &forward_left, &left_near, 1);
  failures += expect_stop("forward left beyond triangle gap",
    &forward_left, &left_far, 0);
  failures += expect_stop("forward left with right obstacle",
    &forward_left, &right_near, 0);
  failures += expect_stop("forward right with left obstacle",
    &forward_right, &left_near, 0);
  failures += expect_stop("reverse left toward left obstacle",
    &reverse_left, &left_near, 1);

  if (failures != 0) return 1;
  puts("test_side_triangle_stop: passed");
  return 0;
}
