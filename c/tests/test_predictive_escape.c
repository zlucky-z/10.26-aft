#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../src/motor/stop_escape_compensator.h"

static int failures;

static void expect(int condition, const char *message)
{
  if (!condition) {
    fprintf(stderr, "FAIL: %s\n", message);
    failures += 1;
  }
}

int main(void)
{
  se_stop_escape_compensator_t escape;
  se_stop_escape_decision_t decision;
  se_point_t cloud[2];
  double now;
  int i;

  se_stop_escape_compensator_init(&escape, 0.0, 0.01, 0.40, 0.6, 0.295);
  cloud[0].x = 0.60;
  cloud[0].y = 0.50;
  cloud[1].x = 0.60;
  cloud[1].y = -0.40;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 1.0);

  /* The backup passes a 10cm triangle gap.  This rear-body point is inside
   * the old 20cm triangle but outside the migrated 10cm triangle. */
  cloud[0].x = 0.40;
  cloud[0].y = 0.435;
  cloud[1].x = 0.40;
  cloud[1].y = -0.435;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 1.0);
  expect(!escape.left_hit && !escape.right_hit,
    "rear-body triangle must retain the backup 10cm gap");

  cloud[0].x = 0.60;
  cloud[0].y = 0.50;
  cloud[1].x = 0.60;
  cloud[1].y = -0.40;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 1.0);

  /* Moving wheels must settle before the backup module can take over. */
  decision = se_stop_escape_compensator_step(
    &escape, 1.0, 0.5, 0.0, 0.20, 0.20, 1.40, 0);
  expect(!decision.active && strcmp(decision.reason, "waiting_wheel_settle") == 0,
    "moving wheels must not start escape");

  /* No direction: the side with the larger free gap is selected. */
  decision = se_stop_escape_compensator_step(
    &escape, 1.1, 0.5, 0.0, 0.0, 0.0, 1.40, 0);
  expect(decision.active && decision.direction == SE_DIRECTION_LEFT &&
      decision.yawrate > 0.0,
    "open-side selection must choose the larger gap");

  /* Explicit joystick direction is authoritative when that side is safe. */
  se_stop_escape_compensator_reset(&escape);
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 2.0);
  decision = se_stop_escape_compensator_step(
    &escape, 2.0, 0.5, -0.8, 0.0, 0.0, 1.40, 0);
  expect(decision.active && decision.direction == SE_DIRECTION_RIGHT &&
      decision.yawrate < 0.0,
    "explicit safe joystick direction must be preserved");

  /* An explicit blocked side must not silently reverse to the other side. */
  se_stop_escape_compensator_reset(&escape);
  cloud[0].x = 0.60;
  cloud[0].y = 0.50;
  cloud[1].x = 0.60;
  cloud[1].y = -0.30;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 3.0);
  decision = se_stop_escape_compensator_step(
    &escape, 3.0, 0.5, -0.8, 0.0, 0.0, 1.40, 0);
  expect(!decision.active && strcmp(decision.reason, "no_safe_side") == 0,
    "blocked explicit direction must not reverse automatically");

  /* A non-zero DWA path immediately hands control back and clears the session. */
  decision = se_stop_escape_compensator_step(
    &escape, 3.1, 0.5, 0.0, 0.0, 0.0, 1.40, 1);
  expect(!decision.active && escape.session == 0,
    "DWA path must interrupt escape session");

  /* The backup module has a cumulative three-second gate and re-arms only
   * after neutral input followed by a new forward input. */
  se_stop_escape_compensator_reset(&escape);
  cloud[1].y = -0.40;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 4.0);
  decision = se_stop_escape_compensator_step(
    &escape, 4.0, 0.5, 0.0, 0.0, 0.0, 1.40, 0);
  expect(decision.active, "escape must start from a stopped no-path state");
  for (i = 1; i <= 31; ++i) {
    now = 4.0 + 0.1 * (double)i;
    decision = se_stop_escape_compensator_step(
      &escape, now, 0.5, 0.0, 0.0, 0.0, 1.40, 0);
  }
  expect(!decision.active && strcmp(decision.reason, "duration_limit") == 0,
    "escape must stop after the cumulative three-second limit");
  decision = se_stop_escape_compensator_step(
    &escape, 7.2, 0.0, 0.0, 0.0, 0.0, 1.40, 0);
  expect(!decision.active, "duration lockout must wait for neutral input");
  decision = se_stop_escape_compensator_step(
    &escape, 7.3, 0.5, 0.0, 0.0, 0.0, 1.40, 0);
  expect(decision.active, "forward input after neutral must re-arm escape");

  if (failures != 0) return 1;
  puts("predictive escape tests passed");
  return 0;
}
