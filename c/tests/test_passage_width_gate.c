#include <math.h>
#include <stdio.h>

#include "../src/guidance/corridor_guide.h"
#include "../src/runtime/dwa_node_internal.h"

static int check_width(float width, int expected_blocked)
{
  dwa_point_t points[64];
  size_t count = 0;
  for (int i = 0; i < 32; ++i) {
    const float x = 0.30f + 0.04f * (float)i;
    points[count++] = (dwa_point_t){x, 0.5f * width};
    points[count++] = (dwa_point_t){x, -0.5f * width};
  }

  dwa_corridor_guide_config_t config = dwa_corridor_guide_config_default();
  config.enabled = false; /* The hard gate must not depend on soft path guidance. */
  config.minimum_passage_width = 0.74f;
  dwa_corridor_guide_t guide;
  dwa_corridor_guide_init(&guide, &config);

  dwa_corridor_guide_input_t input = dwa_corridor_guide_input_default();
  input.points = points;
  input.points_count = count;
  input.perception_fresh = true;
  input.forward_intent = true;
  input.footprint = dwa_footprint_default();
  const dwa_corridor_guidance_t *result =
    dwa_corridor_guide_update(&guide, &input);
  const int passed =
    (result->narrow_passage_detected ? 1 : 0) == expected_blocked;
  if (!passed) {
    fprintf(stderr,
      "width %.3f: blocked=%d width_detected=%.3f x=%.3f expected=%d reason=%s\n",
      width, result->narrow_passage_detected ? 1 : 0,
      result->narrow_passage_width, result->narrow_passage_x,
      expected_blocked, result->reason ? result->reason : "");
  }
  if (expected_blocked &&
    (!isfinite(result->narrow_passage_width) ||
    result->narrow_passage_width >= config.minimum_passage_width)) {
    fprintf(stderr, "blocked passage did not report its measured width\n");
    dwa_corridor_guide_destroy(&guide);
    return 0;
  }

  input.forward_intent = false;
  result = dwa_corridor_guide_update(&guide, &input);
  if (result->narrow_passage_detected) {
    fprintf(stderr, "width gate remained active without forward intent\n");
    dwa_corridor_guide_destroy(&guide);
    return 0;
  }
  dwa_corridor_guide_destroy(&guide);
  return passed;
}

int main(void)
{
  int failures = 0;
  failures += !check_width(0.739f, 1);
  failures += !check_width(0.740f, 0);
  failures += !check_width(0.750f, 0);

  {
    dwa_node_t node = {0};
    dwa_twist_t command = {0};
    node.narrow_passage_blocked = true;
    node.tick_count = 1;

    command.linear_x = 0.25f;
    command.angular_z = 0.30f;
    dns_publish_regular_command(&node, &command, true,
      DYNAMIC_CONTROL_MOVING_DWA, false);
    if (node.last_published_cmd.linear_x != 0.0f ||
      node.last_published_cmd.angular_z != command.angular_z)
    {
      fprintf(stderr, "forward gate must stop positive speed but preserve steering\n");
      ++failures;
    }

    command.linear_x = -0.20f;
    command.angular_z = -0.15f;
    dns_publish_regular_command(&node, &command, true,
      DYNAMIC_CONTROL_MOVING_DWA, false);
    if (node.last_published_cmd.linear_x != command.linear_x ||
      node.last_published_cmd.angular_z != command.angular_z)
    {
      fprintf(stderr, "forward gate must preserve reverse and steering\n");
      ++failures;
    }
  }

  if (failures != 0) {
    fprintf(stderr, "test_passage_width_gate: %d failure(s)\n", failures);
    return 1;
  }
  puts("test_passage_width_gate: PASS");
  return 0;
}
