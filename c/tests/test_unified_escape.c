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
  failures += expect(fabs(config.escape_half_width - 0.295) < 1e-9,
    "escape side clearance must use the 59 cm chassis width");
  failures += expect(fabs(config.escape_max_duration_sec - 2.0) < 1e-9,
    "escape maximum duration must default to two seconds");

  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.5; cloud[0].y = 0.68;  /* left clearance 0.41 */
  cloud[1].x = 0.5; cloud[1].y = -0.48; /* right clearance 0.21 */
  cloud[2].x = 0.55; cloud[2].y = 0.10; /* frontal bearing favors right */
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 10.0);
  decision = se_stop_escape_compensator_update(&escape,
    10.0, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_LEFT,
    "clearly wider side must override frontal-bearing preference");

  /* Equal measured side clearances use the nearer frontal half as a bias. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.5; cloud[0].y = 0.47;
  cloud[1].x = 0.5; cloud[1].y = -0.47;
  cloud[2].x = 0.55; cloud[2].y = 0.10;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 10.05);
  decision = se_stop_escape_compensator_update(&escape,
    10.05, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.direction == SE_DIRECTION_RIGHT,
    "tied side clearances should turn away from the nearer frontal obstacle half");

  /* A newly blocked latched side must stop and latch out, not reverse escape. */
  cloud[0].y = 0.70; /* left clearance 0.405 */
  cloud[1].y = -0.32; /* right clearance 0.025, below the safety floor */
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 10.1);
  decision = se_stop_escape_compensator_update(&escape,
    10.1, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && !decision.command_enabled &&
      strcmp(decision.reason, "selected_side_blocked") == 0,
    "escape must command stop and lock out when its selected side becomes unsafe");

  /* Even if the vehicle/output state changes and only the opposite side is
   * safe, the held trigger must not reselect or reverse the escape direction. */
  cloud[0].y = 0.40; /* left clearance 0.105 */
  cloud[1].y = -0.32; /* right remains blocked; LEFT is now the only safe side */
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 10.14);
  decision = se_stop_escape_compensator_update(&escape,
    10.14, 0, 1, 0, 0, 0, 0);
  failures += expect(decision.active && !decision.command_enabled &&
      decision.direction == SE_DIRECTION_RIGHT &&
      strcmp(decision.reason, "escape_locked_out") == 0,
    "locked escape must keep its original direction until forward intent is released");

  decision = se_stop_escape_compensator_update(&escape,
    10.15, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && !decision.command_enabled &&
      decision.direction == SE_DIRECTION_RIGHT &&
      strcmp(decision.reason, "escape_locked_out") == 0,
    "blocked escape must stay stopped in the same lockout while the trigger remains");

  decision = se_stop_escape_compensator_update(&escape,
    10.2, 1, 1, 1, 1, 0, 0);
  failures += expect(!decision.active,
    "neutral controller must immediately cancel escape");

  /* A manual turn cancels an active escape immediately and prevents it from
   * restarting until the forward/turn trigger has been released. */
  se_stop_escape_compensator_reset(&escape);
  cloud[0].x = 0.60; cloud[0].y = 0.70;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 10.21);
  decision = se_stop_escape_compensator_update(&escape,
    10.21, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.command_enabled,
    "escape should be active before manual turn takeover");
  decision = se_stop_escape_compensator_update_with_turn(&escape,
    10.22, 0, 1, 1, 1, 0, 0, SE_DIRECTION_RIGHT);
  failures += expect(!decision.active && !decision.command_enabled &&
      decision.direction == SE_DIRECTION_RIGHT &&
      strcmp(decision.reason, "manual_turn_takeover") == 0,
    "manual turn must cancel escape and hand control back to DWA");
  decision = se_stop_escape_compensator_update(&escape,
    10.23, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "escape_locked_out") == 0,
    "manual-turn cancellation must not re-enter while forward intent remains");
  decision = se_stop_escape_compensator_update(&escape,
    10.24, 0, 0, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "conditions_not_met") == 0,
    "releasing forward intent must clear the lockout without escaping");
  decision = se_stop_escape_compensator_update(&escape,
    10.25, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active,
    "escape may be evaluated again after its trigger has cleared");

  /* Unknown side space is not treated as safe; hand direction stays with DWA. */
  se_stop_escape_compensator_reset(&escape);
  cloud[0].x = 0.60; cloud[0].y = 0.10;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 10.3);
  decision = se_stop_escape_compensator_update_with_turn(&escape,
    10.3, 0, 1, 1, 1, 0, 0, SE_DIRECTION_LEFT);
  failures += expect(!decision.active &&
      decision.direction == SE_DIRECTION_LEFT &&
      strcmp(decision.reason, "manual_turn_takeover") == 0,
    "unknown side clearances must leave the requested turn to DWA");
  se_stop_escape_compensator_reset(&escape);
  decision = se_stop_escape_compensator_update(&escape,
    10.5, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "no_safe_side") == 0,
    "unknown side clearances without a turn intent must not pick an arbitrary side");
  se_stop_escape_compensator_reset(&escape);
  decision = se_stop_escape_compensator_update(&escape,
    11.4, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active,
    "stale obstacle cloud must inhibit initial escape");

  /* Staleness after activation pauses output without ending the maneuver;
   * fresh sensing resumes the same direction without another start delay. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.60; cloud[0].y = 0.70;
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

    cloud[0].x = 0.10; cloud[0].y = 0.32;
    cloud[1].x = 0.10; cloud[1].y = -0.32;
    se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 12.23);
    decision = se_stop_escape_compensator_update(&escape,
      12.24, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && !decision.command_enabled &&
        strcmp(decision.reason, "selected_side_blocked") == 0,
      "unsafe selected side must stop and retain the escape direction");

    cloud[0].y = 0.60;
    cloud[1].y = -0.60;
    se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 12.25);
    decision = se_stop_escape_compensator_update(&escape,
      12.26, 0, 1, 1, 1, 0, 0);
    failures += expect(decision.active && !decision.command_enabled &&
        strcmp(decision.reason, "escape_locked_out") == 0,
      "a newly clear scan must not silently resume a blocked escape");
  }

  /* Frontal wall returns just outside the footprint width must not be
   * mistaken for blocked left/right escape sectors. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.90; cloud[0].y = 0.28;
  cloud[1].x = 0.90; cloud[1].y = -0.29;
  cloud[2].x = 0.80; cloud[2].y = 0.0;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 3, 11.5);
  failures += expect(isinf(escape.left_clearance) &&
      isinf(escape.right_clearance),
    "frontal near-corner returns must not contaminate lateral clearance");
  decision = se_stop_escape_compensator_update(&escape,
    11.5, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "no_safe_side") == 0,
    "unobserved lateral sectors must not be assumed clear for escape");

  /* A point actually beside the chassis remains a side hazard. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.10; cloud[0].y = 0.32;
  cloud[1].x = 0.10; cloud[1].y = -0.32;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 2, 11.6);
  decision = se_stop_escape_compensator_update(&escape,
    11.6, 0, 1, 1, 1, 0, 0);
  failures += expect(!decision.active &&
      strcmp(decision.reason, "no_safe_side") == 0,
    "genuinely close obstacles on both sides must still inhibit escape");

  /* DWA handoff requires a sustained command, then sustained measured forward
   * wheel motion. A failed trial resumes the same latched direction. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.9, 0.6, 0.295);
  escape.max_duration_sec = 10.0; /* This block exercises handoff, not timeout. */
  cloud[0].x = 0.60; cloud[0].y = 0.70;
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

  /* The configured hard limit stops one escape and locks it out until the
   * initiating forward intent or stop trigger clears. */
  se_stop_escape_compensator_init(&escape, 0.0, 0.03, 0.5, 0.6, 0.295);
  cloud[0].x = 0.60; cloud[0].y = 0.70;
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 20.0);
  decision = se_stop_escape_compensator_update(&escape,
    20.0, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.command_enabled,
    "escape must start before the duration limit");
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 21.99);
  decision = se_stop_escape_compensator_update(&escape,
    21.99, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && decision.command_enabled,
    "escape must remain available immediately before the duration limit");
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 22.0);
  decision = se_stop_escape_compensator_update(&escape,
    22.0, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && !decision.command_enabled &&
      decision.direction == SE_DIRECTION_LEFT &&
      strcmp(decision.reason, "duration_limit") == 0,
    "escape must stop at its two-second maximum duration and retain its direction");
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 22.01);
  decision = se_stop_escape_compensator_update(&escape,
    22.01, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active && !decision.command_enabled &&
      strcmp(decision.reason, "escape_locked_out") == 0,
    "duration timeout must not immediately restart while trigger remains");
  decision = se_stop_escape_compensator_update(&escape,
    22.02, 0, 0, 1, 1, 0, 0);
  failures += expect(!decision.active,
    "clearing the trigger must release the duration lockout");
  se_stop_escape_compensator_update_cloud(&escape, cloud, 1, 22.03);
  decision = se_stop_escape_compensator_update(&escape,
    22.03, 0, 1, 1, 1, 0, 0);
  failures += expect(decision.active,
    "a fresh trigger after release may start a new bounded escape");

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

  /* Reverse manual bypass preserves native joystick Y when no linear speed
   * limit is active; gear-only intervention must not remap the throttle. */
  memset(&bridge, 0, sizeof(bridge));
  mb_config_defaults(&config);
  bridge.config = config;
  g_now = 40.0;
  bridge.last_raw_cmd_time = g_now;
  bridge.last_cmd_time = g_now;
  bridge.last_raw_linear = -0.70;
  bridge.last_linear = -0.70;
  bridge.last_controller_x = 31;
  bridge.last_controller_y = 0;
  {
    int x = -1, y = -1;
    char state[MB_STATE_MAX];
    failures += expect(mb_raw_reverse_passthrough_xy(
        &bridge, g_now, &x, &y, state) == 1 && x == 31 && y == 0,
      "unlimited reverse must pass the original controller X/Y through");
    bridge.output_gear_closed_loop_active = 1;
    failures += expect(mb_raw_reverse_passthrough_xy(
        &bridge, g_now, &x, &y, state) == 1 && y == 0,
      "gear-only override must not remap raw reverse Y");
    bridge.last_linear = -0.35;
    failures += expect(mb_raw_reverse_passthrough_xy(
        &bridge, g_now, &x, &y, state) == 1 &&
        y == (int)mp_py_round((double)MP_CENTER +
          ((double)0 - (double)MP_CENTER) * 0.5),
      "active half-speed limit must scale raw reverse joystick deflection");
  }

  if (failures != 0) return 1;
  puts("unified escape tests passed");
  return 0;
}
