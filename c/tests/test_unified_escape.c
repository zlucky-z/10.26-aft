#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../src/motor/motor_bridge.h"
#include "../src/motor/motor_bridge_config.h"

static double g_now;
static double test_now(void *user)
{
  (void)user;
  return g_now;
}
static void test_sleep(void *user, double ms)
{
  (void)user;
  (void)ms;
}

static mp_frame_t g_last_frame;
static unsigned long g_frame_count;
static void capture_frame(void *user, const mp_frame_t *frame)
{
  (void)user;
  g_last_frame = *frame;
  ++g_frame_count;
}

static int expect(int condition, const char *message)
{
  if (!condition) {
    fprintf(stderr, "FAIL: %s\n", message);
    return 1;
  }
  return 0;
}

int main(void)
{
  int failures = 0;
  se_stop_escape_compensator_t escape;
  se_point_t cloud[3];
  se_stop_escape_decision_t decision;
  mb_bridge_t bridge;
  mb_config_t config;
  ms_sample_t sample;
  char err[256];

  mb_config_defaults(&config);
  failures += expect(config.escape_stop_hold_sec == 0.0,
    "escape must start without the former 0.5-second delay");

  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.27);
  cloud[0].x = 0.5; cloud[0].y = 0.68;  /* left clearance 0.41 */
  cloud[1].x = 0.5; cloud[1].y = -0.48; /* right clearance 0.21 */
  cloud[2].x = 0.55; cloud[2].y = 0.10; /* frontal bearing favors right */
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 10.0);
  decision = se_stop_escape_compensator_update(&escape,
    10.0, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_LEFT,
    "clearly wider side must override frontal-bearing preference");

  /* Equal measured side clearances use the nearer frontal half as a bias. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.27);
  cloud[0].x = 0.5; cloud[0].y = 0.47;
  cloud[1].x = 0.5; cloud[1].y = -0.47;
  cloud[2].x = 0.55; cloud[2].y = 0.10;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 10.05);
  decision = se_stop_escape_compensator_update(&escape,
    10.05, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_RIGHT,
    "tied side clearances should turn away from the nearer frontal obstacle half");

  /* A newly blocked latched side must switch only to the open opposite side. */
  cloud[0].y = 0.29; /* left clearance 0.02, below the 0.03 safety floor */
  cloud[1].y = -0.68; /* right clearance 0.41 */
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 10.1);
  decision = se_stop_escape_compensator_update(&escape,
    10.1, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_RIGHT,
    "escape must switch away from a newly blocked side");

  decision = se_stop_escape_compensator_update(&escape,
    10.2, 1, 1, 1, 1, 0, 0);
  failures += expect(!decision.active,
    "neutral controller must immediately cancel escape");

  /* With tied/unknown side clearances, the front obstacle bearing breaks the tie. */
  cloud[0].x = 0.60; cloud[0].y = 0.10;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 10.3);
  decision = se_stop_escape_compensator_update(&escape,
    10.3, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_RIGHT,
    "front obstacle left-of-center should produce an away-to-right escape");

  /* A centered frontal obstacle has no geometric side preference: pick an
   * arbitrary direction, then alternate it for the next otherwise-tied event. */
  decision = se_stop_escape_compensator_update(&escape,
    10.4, 1, 1, 1, 1, 0, 0);
  cloud[0].x = 0.60; cloud[0].y = 0.0;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 10.5);
  decision = se_stop_escape_compensator_update(&escape,
    10.5, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_LEFT,
    "centered frontal obstacle should allow arbitrary escape direction");
  decision = se_stop_escape_compensator_update(&escape,
    10.6, 1, 1, 1, 1, 0, 0);
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 10.7);
  decision = se_stop_escape_compensator_update(&escape,
    10.7, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_RIGHT,
    "arbitrary tied direction should alternate across escape events");
  se_stop_escape_compensator_reset(&escape);
  decision = se_stop_escape_compensator_update(&escape,
    11.4, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active,
    "stale obstacle cloud must inhibit initial escape");

  /* Staleness after activation pauses output without ending the maneuver;
   * fresh sensing resumes the same direction without another start delay. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.27);
  cloud[0].x = 0.60; cloud[0].y = 0.0;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 11.5);
  decision = se_stop_escape_compensator_update(&escape,
    11.5, 0, 1, 1, 1, 0, 0);
  {
    const se_direction_t latched = decision.direction;
    decision = se_stop_escape_compensator_update(&escape,
      12.2, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && !decision.command_enabled &&
        decision.direction == latched &&
        strcmp(decision.reason, "paused_cloud_stale") == 0,
      "stale scan must pause, not exit, an active escape");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 12.21);
    decision = se_stop_escape_compensator_update(&escape,
      12.22, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && decision.command_enabled &&
        decision.direction == latched,
      "fresh scan must resume the same escape without a new start delay");

    cloud[0].x = 0.10; cloud[0].y = 0.29;
    cloud[1].x = 0.10; cloud[1].y = -0.29;
    se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 12.23);
    decision = se_stop_escape_compensator_update(&escape,
      12.24, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && !decision.command_enabled &&
        decision.direction == latched &&
        strcmp(decision.reason, "paused_no_safe_side") == 0,
      "temporarily unsafe sides must pause without clearing the escape latch");

    cloud[0].y = 0.60;
    cloud[1].y = -0.60;
    se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 12.25);
    decision = se_stop_escape_compensator_update(&escape,
      12.26, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && decision.command_enabled &&
        decision.direction == latched,
      "safe side clearance must resume the retained direction");
  }

  /* Frontal wall returns just outside the footprint width must not be
   * mistaken for blocked left/right escape sectors. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.27);
  cloud[0].x = 0.90; cloud[0].y = 0.28;
  cloud[1].x = 0.90; cloud[1].y = -0.29;
  cloud[2].x = 0.80; cloud[2].y = 0.0;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 11.5);
  failures += expect(isinf(escape.left_clearance) &&
      isinf(escape.right_clearance),
    "frontal near-corner returns must not contaminate lateral clearance");
  decision = se_stop_escape_compensator_update(&escape,
    11.5, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active,
    "open lateral sectors must permit escape beside a frontal obstacle");

  /* A point actually beside the chassis remains a side hazard. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.27);
  cloud[0].x = 0.10; cloud[0].y = 0.28;
  cloud[1].x = 0.10; cloud[1].y = -0.29;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 11.6);
  decision = se_stop_escape_compensator_update(&escape,
    11.6, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "no_safe_side") == 0,
    "genuinely close obstacles on both sides must still inhibit escape");

  /* DWA handoff requires a sustained command, then sustained measured forward
   * wheel motion. A failed trial resumes the same latched direction. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.9, 0.6, 0.27);
  cloud[0].x = 0.60; cloud[0].y = 0.0;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 12.0);
  decision = se_stop_escape_compensator_update(&escape,
    12.0, 0, 1, 1, 1, 0, 0);
  {
    const se_direction_t latched_direction = decision.direction;
    failures += expect(decision.active,
      "escape should activate after stopped forward intent");
    decision = se_stop_escape_compensator_update(&escape,
      12.10, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && !decision.yield_to_dwa,
      "brief DWA forward command must not yet yield control");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 12.41);
    decision = se_stop_escape_compensator_update(&escape,
      12.41, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && decision.yield_to_dwa &&
        decision.direction == latched_direction,
      "sustained positive DWA command should start handoff with direction latched");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 12.70);
    decision = se_stop_escape_compensator_update(&escape,
      12.70, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && decision.yield_to_dwa,
      "handoff should remain active while measured forward motion is absent");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.02);
    decision = se_stop_escape_compensator_update(&escape,
      13.02, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && !decision.yield_to_dwa &&
        decision.command_enabled && decision.direction == latched_direction,
      "failed handoff must resume the same escape without restarting");

    /* A second handoff succeeds only after 0.25 s of measured forward motion. */
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.10);
    decision = se_stop_escape_compensator_update(&escape,
      13.10, 0, 1, 1, 1, 1, 0);
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.41);
    decision = se_stop_escape_compensator_update(&escape,
      13.41, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.yield_to_dwa,
      "a new sustained positive command should retry DWA handoff");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.42);
    decision = se_stop_escape_compensator_update(&escape,
      13.42, 0, 1, 1, 1, 1, 1);
    failures += expect(decision.active && decision.yield_to_dwa,
      "forward movement must be verified continuously, not on one sample");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.68);
    decision = se_stop_escape_compensator_update(&escape,
      13.68, 0, 1, 1, 1, 1, 1);
    failures += expect(!decision.active && strcmp(decision.reason, "recovered") == 0,
      "sustained measured forward motion should end the escape");

    /* Losing the forward command during handoff immediately resumes the turn. */
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 13.70);
    decision = se_stop_escape_compensator_update(&escape,
      13.70, 0, 1, 1, 1, 0, 0);
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 14.01);
    decision = se_stop_escape_compensator_update(&escape,
      14.01, 0, 1, 1, 1, 0, 0);
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 14.02);
    decision = se_stop_escape_compensator_update(&escape,
      14.02, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && !decision.yield_to_dwa,
      "new positive command must qualify before handing off again");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 14.33);
    decision = se_stop_escape_compensator_update(&escape,
      14.33, 0, 1, 1, 1, 1, 0);
    failures += expect(decision.active && decision.yield_to_dwa,
      "qualified command should re-enter handoff");
    se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 14.34);
    decision = se_stop_escape_compensator_update(&escape,
      14.34, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && decision.command_enabled &&
        !decision.yield_to_dwa,
      "command disappearance must immediately resume the latched escape");

    decision = se_stop_escape_compensator_update(&escape,
      14.40, 1, 1, 1, 1, 0, 0);
    failures += expect(!decision.active,
      "releasing forward intent must immediately cancel an escape");
  }

  /* Bridge feedback timestamps must be stamped in the same relative clock as mb_now(). */
  mb_config_defaults(&config);
  config.dry_run = 1;
  config.escape_stop_hold_sec = 0.0;
  memset(&bridge, 0, sizeof(bridge));
  bridge.inject.now_fn = test_now;
  bridge.inject.sleep_fn = test_sleep;
  g_now = 10.0;
  if (mb_bridge_init(&bridge, &config, err, sizeof(err)) != 0) {
    fprintf(stderr, "FAIL: bridge init: %s\n", err);
    return 2;
  }
  memset(&sample, 0, sizeof(sample));
  sample.left_mps = 0.0;
  sample.right_mps = 0.0;
  sample.time = 123456.0; /* serial parser's absolute CLOCK_MONOTONIC timestamp */
  mb_update_wheel_feedback(&bridge, &sample);
  failures += expect(fabs(bridge.last_wheel_feedback_time - g_now) < 1e-9,
    "wheel feedback timestamp must be translated to the bridge clock domain");
  failures += expect(mb_wheel_feedback_is_fresh(&bridge, g_now),
    "newly received wheel feedback should be fresh");
  g_now += config.wheel_feedback_timeout + 0.01;
  failures += expect(!mb_wheel_feedback_is_fresh(&bridge, g_now),
    "wheel feedback must expire after the configured timeout");

  /* Serial TX pauses only after 1s without a valid parsed controller frame. */
  {
    const uint64_t attempts_before = bridge.tx_attempt_count;
    g_now = 10.99;
    mb_on_cmd(&bridge, 0.20, 0.0);
    mb_on_timer(&bridge);
    failures += expect(bridge.tx_attempt_count > attempts_before,
      "TX must remain enabled during the 1s feedback grace window");

    g_now = 11.01;
    mb_on_cmd(&bridge, 0.20, 0.0);
    mb_on_timer(&bridge);
    failures += expect(bridge.tx_last_result == -3,
      "TX must be suppressed after 1s without a valid feedback frame");
    failures += expect(!bridge.controller_feedback_online,
      "controller feedback status must become offline after the timeout");

    memset(&sample, 0, sizeof(sample));
    sample.kind = MS_SAMPLE_KIND_DRIVE;
    g_now = 11.02;
    mb_publish_sample(&bridge, &sample);
    failures += expect(bridge.controller_feedback_online,
      "a valid parsed drive frame must restore the online state immediately");
    mb_on_cmd(&bridge, 0.20, 0.0);
    mb_on_timer(&bridge);
    failures += expect(bridge.tx_last_result != -3,
      "TX must resume immediately after valid feedback returns");

    /* Raw bytes/unknown samples are not valid feedback and must not refresh. */
    g_now = 12.03;
    mb_on_cmd(&bridge, 0.20, 0.0);
    mb_on_timer(&bridge);
    failures += expect(bridge.tx_last_result == -3,
      "unknown/non-feedback samples must not keep the controller online");
  }
  mb_stop_and_close(&bridge);

  /* Production sensor interlock: movement is blocked and cause is encoded in
   * the controller status byte (left=bit0, right=bit1, IMU=bit2). */
  mb_config_defaults(&config);
  config.dry_run = 1;
  config.protocol = MP_PROTOCOL_23;
  memset(&bridge, 0, sizeof(bridge));
  bridge.inject.now_fn = test_now;
  bridge.inject.sleep_fn = test_sleep;
  bridge.inject.frame_sink = capture_frame;
  g_now = 30.0;
  g_frame_count = 0;
  if (mb_bridge_init(&bridge, &config, err, sizeof(err)) != 0) {
    fprintf(stderr, "FAIL: sensor bridge init: %s\n", err);
    return 2;
  }
  mb_bridge_enable_sensor_interlock(&bridge, 1);
  mb_on_left_scan(&bridge);
  mb_on_right_scan(&bridge);
  mb_on_escape_imu_yawrate(&bridge, 0.0, 1);
  memset(&sample, 0, sizeof(sample));
  sample.kind = MS_SAMPLE_KIND_DRIVE;
  mb_publish_sample(&bridge, &sample);
  mb_on_cmd(&bridge, 0.20, 0.0);
  g_now = 30.10;
  mb_on_timer(&bridge);
  failures += expect(g_last_frame.bytes[9] == 0,
    "healthy sensors must report no sensor-fault status bits");

  /* Keep radar and controller feedback fresh, but let IMU exceed 1s. */
  g_now = 31.20;
  mb_on_left_scan(&bridge);
  mb_on_right_scan(&bridge);
  mb_publish_sample(&bridge, &sample);
  mb_on_timer(&bridge);
  failures += expect(g_last_frame.bytes[9] == MP_SENSOR_STATUS_IMU_FAULT,
    "IMU timeout must be identified in the controller status byte");
  failures += expect(g_last_frame.bytes[5] == 0x08 &&
      g_last_frame.bytes[6] == 0x00 &&
      g_last_frame.bytes[7] == 0x08 &&
      g_last_frame.bytes[8] == 0x00,
    "IMU fault must force both commanded axes to neutral");

  /* Refresh IMU/right radar only: left radar timeout is independently reported. */
  g_now = 31.80;
  mb_on_right_scan(&bridge);
  mb_on_escape_imu_yawrate(&bridge, 0.0, 1);
  mb_publish_sample(&bridge, &sample);
  mb_on_timer(&bridge);
  failures += expect(g_last_frame.bytes[9] ==
      MP_SENSOR_STATUS_LEFT_RADAR_FAULT,
    "left radar timeout must be reported independently");

  /* Refresh left/IMU only: right radar timeout is independently reported. */
  g_now = 32.40;
  mb_on_left_scan(&bridge);
  mb_on_escape_imu_yawrate(&bridge, 0.0, 1);
  mb_publish_sample(&bridge, &sample);
  mb_on_timer(&bridge);
  failures += expect(g_last_frame.bytes[9] ==
      MP_SENSOR_STATUS_RIGHT_RADAR_FAULT,
    "right radar timeout must be reported independently");

  /* Recovery of all required sensors releases the actuator interlock. */
  g_now = 32.50;
  mb_on_right_scan(&bridge);
  mb_on_escape_imu_yawrate(&bridge, 0.0, 1);
  mb_publish_sample(&bridge, &sample);
  mb_on_timer(&bridge);
  failures += expect(g_last_frame.bytes[9] == 0,
    "fresh feedback from all required sensors must clear fault status");
  failures += expect(g_frame_count > 0,
    "sensor interlock must continue sending neutral/status frames");
  mb_stop_and_close(&bridge);

  if (failures != 0) return 1;
  puts("unified escape tests passed");
  return 0;
}
