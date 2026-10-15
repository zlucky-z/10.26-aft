#include <stdio.h>

#include "../src/common/dwa_geometry.h"

int main(void)
{
  dwa_footprint_t footprint = dwa_footprint_default();
  footprint.front = 0.50f;
  footprint.back = 0.14f;
  footprint.half_width = 0.27f;
  footprint.footrest_start = 0.57f;
  footprint.footrest_front = 0.75f;
  footprint.footrest_half_width = 0.23f;
  footprint.toe_front = 0.90f;
  footprint.toe_half_width = 0.23f;

  if (!dwa_point_inside_footprint_self_filter(0.90f, 0.0f, &footprint, 0.05f)) {
    fprintf(stderr, "returns within the physical toe envelope should remain self-filtered\n");
    return 1;
  }
  if (dwa_point_inside_footprint_self_filter(0.95f, 0.0f, &footprint, 0.05f)) {
    fprintf(stderr, "external returns beyond toe_front must reach DWA\n");
    return 1;
  }
  if (!dwa_point_inside_footprint_self_filter(
      0.95f, 0.25f, &footprint, 0.05f)) {
    fprintf(stderr, "toe-forward lateral padding must remain filtered to protect wall behavior\n");
    return 1;
  }
  if (dwa_point_inside_footprint_self_filter(
      0.95f, 0.29f, &footprint, 0.05f)) {
    fprintf(stderr, "points outside lateral self-filter padding must remain visible\n");
    return 1;
  }
  if (!dwa_point_inside_footprint_self_filter(
      0.89f, 0.25f, &footprint, 0.05f)) {
    fprintf(stderr, "self-filter padding inside the physical toe envelope must remain unchanged\n");
    return 1;
  }
  puts("test_self_filter_toe_boundary: passed");
  return 0;
}
