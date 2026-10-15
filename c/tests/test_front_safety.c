/* Front-edge and front-lane regression tests. */
#include <math.h>
#include <stdio.h>

#include "../src/safety/front_safety.h"
#include "../src/common/dwa_types.h"

static int near(float a, float b)
{
  return fabsf(a - b) < 1e-6f;
}

int main(void)
{
  dwa_footprint_t footprint = dwa_footprint_default();
  dwa_front_safety_config_t config = dwa_front_safety_config_default();
  dwa_point_t direct_obstacle = {1.00f, 0.00f};
  dwa_point_t corridor_wall = {1.00f, 0.26f};
  dwa_point_t toe_edge_probe = {0.90f, 0.20f};
  dwa_point_t body_edge_probe = {0.49f, 0.26f};
  dwa_point_t toe_contact_probe = {0.90f, 0.00f};
  dwa_point_t toe_lateral_wall = {0.90f, 0.27f};
  dwa_point_t behind_release_band = {0.84f, 0.00f};

  footprint.front = 0.50f;
  footprint.footrest_start = 0.57f;
  footprint.footrest_front = 0.75f;
  footprint.footrest_half_width = 0.23f;
  footprint.toe_front = 0.90f;
  footprint.toe_half_width = 0.23f;
  footprint.padding = 0.0f;

  if (!near(dwa_collision_front_edge(&footprint), 0.90f)) {
    fprintf(stderr, "front edge must use toe front (0.90m)\n");
    return 1;
  }
  footprint.padding = 0.02f;
  if (!near(dwa_collision_front_edge(&footprint), 0.92f)) {
    fprintf(stderr, "front edge must include footprint padding\n");
    return 1;
  }
  footprint.padding = 0.0f;
  if (!dwa_point_inside_front_safety_lane(&direct_obstacle, &config, &footprint)) {
    fprintf(stderr, "direct frontal obstacle must remain in front safety lane\n");
    return 1;
  }
  if (dwa_point_inside_front_safety_lane(&corridor_wall, &config, &footprint)) {
    fprintf(stderr, "lateral corridor wall must not be treated as a frontal stop\n");
    return 1;
  }
  if (!dwa_point_inside_front_safety_lane(&toe_edge_probe, &config, &footprint)) {
    fprintf(stderr, "toe-edge obstacle must remain included\n");
    return 1;
  }
  if (!dwa_point_inside_front_safety_lane(&body_edge_probe, &config, &footprint)) {
    fprintf(stderr, "body-front obstacle must retain the wider body envelope\n");
    return 1;
  }
  if (!dwa_point_is_front_safety_candidate(
      &toe_contact_probe, &config, &footprint, 0.05f))
  {
    fprintf(stderr, "toe-contact point must be preserved for hard-stop safety\n");
    return 1;
  }
  if (dwa_point_is_front_safety_candidate(
      &toe_lateral_wall, &config, &footprint, 0.05f))
  {
    fprintf(stderr, "side wall must stay outside the narrow toe safety lane\n");
    return 1;
  }
  if (dwa_point_is_front_safety_candidate(
      &behind_release_band, &config, &footprint, 0.05f))
  {
    fprintf(stderr, "points behind the self-filter release strip must not be preserved\n");
    return 1;
  }
  puts("test_front_safety: passed");
  return 0;
}
