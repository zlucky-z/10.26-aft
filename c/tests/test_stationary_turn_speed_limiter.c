#include <math.h>
#include <stdio.h>

#include "../src/safety/stationary_turn_speed_limiter.h"

static int failures;

static void expect_true(const char *name, bool actual, bool expected)
{
  if (actual != expected) {
    printf("FAIL %s: actual=%d expected=%d\n", name, actual, expected);
    ++failures;
  }
}

static void expect_near(const char *name, float actual, float expected)
{
  if (!isfinite(actual) || fabsf(actual - expected) > 1e-5f) {
    printf("FAIL %s: actual=%.6f expected=%.6f\n", name, actual, expected);
    ++failures;
  }
}

int main(void)
{
  const stsl_config_t config = {
    true, 100, 100, 89, 111, 89, 10.0f, 1.0f, 0.91f, 0.70f, 0.50f
  };
  const dwa_point_t radial_points[] = {{0.60f, 0.0f}, {0.0f, -0.80f}};
  stsl_result_t result;

  result = stsl_evaluate(&config, 150, 50, true, true, 0.95f);
  expect_true("reverse triggers outer ring", result.speed_limited, true);
  expect_near("outer ring cap is gear one", result.speed_limit_mps, 0.70f);
  result = stsl_evaluate(&config, 50, 50, true, true, 0.95f);
  expect_true("reverse triggers regardless of lateral direction", result.speed_limited, true);

  result = stsl_evaluate(&config, 150, 100, true, true, 0.90f);
  expect_true("pure lateral turn triggers inner ring", result.speed_limited, true);
  expect_near("inner ring cap is half gear one", result.speed_limit_mps, 0.35f);
  result = stsl_evaluate(&config, 150, 95, true, true, 0.90f);
  expect_true("lateral input with rearward deadzone drift triggers", result.speed_limited, true);

  result = stsl_evaluate(&config, 150, 108, true, true, 0.95f);
  expect_true("forward-side red sector triggers", result.speed_limited, true);
  expect_near("forward-side red sector uses outer cap", result.speed_limit_mps, 0.70f);

  result = stsl_evaluate(&config, 150, 110, true, true, 0.95f);
  expect_true("forward turn outside red sector does not trigger", result.speed_limited, false);
  result = stsl_evaluate(&config, 164, 111, true, true, 0.95f);
  expect_true("forward red-sector boundary includes edge", result.speed_limited, true);
  result = stsl_evaluate(&config, 164, 112, true, true, 0.95f);
  expect_true("forward red-sector beyond edge does not trigger", result.speed_limited, false);

  result = stsl_evaluate(&config, 100, 150, true, true, 0.50f);
  expect_true("straight forward does not trigger", result.speed_limited, false);

  result = stsl_evaluate(&config, 150, 150, true, true, 0.50f);
  expect_true("diagonal forward does not trigger", result.speed_limited, false);

  result = stsl_evaluate(&config, 150, 50, true, true, 0.50f);
  expect_near("nearest radial obstacle selects inner ring", result.speed_limit_mps, 0.35f);
  expect_near("reverse sign is preserved", stsl_apply_linear_limit(-0.70f, result.speed_limit_mps), -0.35f);
  expect_near("slower request is never increased", stsl_apply_linear_limit(0.20f, result.speed_limit_mps), 0.20f);
  expect_near("circle distance uses origin", stsl_nearest_obstacle_distance(
    radial_points, 2, INFINITY), 0.60f);
  result = stsl_evaluate(&config, 150, 100, true, true, 1.01f);
  expect_true("obstacle outside outer ring does not trigger", result.speed_limited, false);

  result = stsl_evaluate(&config, 150, 100, false, true, 0.50f);
  expect_true("stale joystick does not trigger", result.speed_limited, false);
  result = stsl_evaluate(&config, 150, 100, true, false, 0.50f);
  expect_true("stale point cloud does not trigger", result.speed_limited, false);

  if (failures == 0) {
    puts("test_stationary_turn_speed_limiter: ALL PASS");
    return 0;
  }
  printf("test_stationary_turn_speed_limiter: %d FAILURES\n", failures);
  return 1;
}
