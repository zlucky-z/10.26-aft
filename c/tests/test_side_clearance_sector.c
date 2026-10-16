#include <stdio.h>
#include <string.h>

#include "../src/runtime/dwa_node_internal.h"

int main(void)
{
  dwa_node_t node;
  dwa_point_t point;
  int failures = 0;
  memset(&node, 0, sizeof(node));
  node.params_snapshot.side_detection_angle_deg = 70.0f;
  node.dynamic_config.moving_prebrake_base_distance = 0.40f;
  node.dynamic_config.forward_speed_limit = 0.70f;
  node.dynamic_config.moving_prebrake_speed_ratio = 0.50f;
  node.config.footprint.front = 0.50;
  node.config.footprint.back = 0.14;
  node.config.footprint.half_width = 0.27;
  node.config.footprint.padding = 0.0;
  node.config.footprint.footrest_start = 0.57;
  node.config.footprint.footrest_front = 0.75;
  node.config.footprint.footrest_half_width = 0.23;
  node.config.footprint.toe_front = 0.90;
  node.config.footprint.toe_half_width = 0.23;

  point.x = 0.90f;
  point.y = 0.30f;
  if (dnp_inside_physical_side_sector(&node, &point)) {
    fprintf(stderr, "FAIL: forward corner return was counted as side clearance\n");
    ++failures;
  }

  point.x = 0.10f;
  point.y = 0.50f;
  if (!dnp_inside_physical_side_sector(&node, &point)) {
    fprintf(stderr, "FAIL: true lateral obstacle was dropped from side clearance\n");
    ++failures;
  }

  point.x = 0.70f;
  point.y = 0.50f;
  if (!dnp_inside_physical_side_sector(&node, &point)) {
    fprintf(stderr, "FAIL: forward-side wall outside front sector was dropped\n");
    ++failures;
  }

  if (failures != 0) return 1;
  puts("side clearance sector tests passed");
  return 0;
}
