/*
 * P3/P4 parity（C 侧）：状态序列对拍（fusion 状态机 / side_wall_hold /
 * corridor_guide / dwa_geometric_selector（链 P2 planning）/ safety 纯函数）。
 * 用法: test_p3p4 [输出csv路径] [场景数]
 * 场景生成与 tools/parity/parity_p3p4_cpp.cpp 逐字节一致（同一 LCG、
 * 同一调用顺序）。修改本文件时必须同步修改 parity_p3p4_cpp.cpp。
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/fusion/dynamic_velocity_fusion.h"
#include "../src/guidance/corridor_guide.h"
#include "../src/guidance/dwa_geometric_selector.h"
#include "../src/planning/dwa_sampling.h"
#include "../src/safety/escape_controller.h"
#include "../src/safety/front_safety.h"
#include "../src/safety/side_safety.h"
#include "../src/safety/side_wall_hold_control.h"

static uint64_t rng_state = 0x9E3779B97F4A7C15ULL;

static uint64_t rng_next(void)
{
  rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return rng_state;
}

static double rng_unit(void)
{
  return (double)(rng_next() >> 11) * (1.0 / 9007199254740992.0);
}

static double clampd(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

#define TICKS 96

/* P4.5：DWA_PI 已 float 化；harness 输入生成保持 double π（与 C++ 基线一致）。 */
static const double T_PI = 3.14159265358979323846;

static FILE *g_out = NULL;

/* ---------------- P4: DynamicVelocityFusion 状态序列 ---------------- */

static void run_fusion(int scenarios)
{
  for (int s = 0; s < scenarios; ++s) {
    dwa_dynamic_velocity_config_t config = dwa_dynamic_velocity_config_default();
    const double tweak = rng_unit();
    if (s & 1) {
      config.moving_prebrake_base_distance = 0.60 + 0.60 * tweak;
    }

    dwa_dynamic_velocity_fusion_t fusion;
    dwa_dynamic_velocity_fusion_init(&fusion, &config);

    double now = 0.0;
    double speed = rng_unit() * 1.7;
    double front = 1.0 + rng_unit() * 4.0;
    double gear_front = 1.0 + rng_unit() * 4.0;
    double left = rng_unit() * 2.0;
    double right = rng_unit() * 2.0;

    const int pattern = s % 4;
    static const double DT[4] = {0.02, 0.05, 0.10, 0.50};

    for (int t = 0; t < TICKS; ++t) {
      const double rb = rng_unit();
      const double dt = DT[(int)(rb * 4.0)];
      now += dt;

      dwa_dynamic_velocity_input_t input = dwa_dynamic_velocity_input_default();
      input.now_sec = now;
      input.forward_intent = rng_unit() < 0.75;
      input.forward_start_rising = rng_unit() < 0.08;
      input.perception_fresh = rng_unit() < 0.92;
      input.hall_speed_fresh = rng_unit() < 0.92;

      const double rs = rng_unit();
      if (pattern == 0) {
        speed = clampd(speed + (rs - 0.5) * 0.5, 0.0, 1.7);
      } else if (pattern == 1) {
        speed = clampd(speed - 0.05 - 0.10 * rs, 0.0, 1.7);
      } else if (pattern == 2) {
        speed = clampd(speed + 0.05 + 0.10 * rs, 0.0, 1.7);
      } else {
        speed = clampd(rs * 0.10, 0.0, 1.7);
      }
      input.actual_forward_speed = speed;

      const double rg = rng_unit();
      input.raw_gear = 1 + (int)(rg * 5.0);
      input.raw_gear_valid = rng_unit() < 0.85;
      input.near_obstacle_gear_active = rng_unit() < 0.30;

      const double rf = rng_unit();
      if (pattern == 0) {
        front = clampd(front + (rf - 0.5) * 1.0, 0.02, 8.0);
      } else if (pattern == 1) {
        front = clampd(front - 0.08 - 0.16 * rf, 0.02, 8.0);
      } else if (pattern == 2) {
        front = clampd(front + 0.08 + 0.16 * rf, 0.02, 8.0);
      } else {
        front = 0.05 + 1.20 * rf;
      }
      input.front_clearance = front;

      const double rgf = rng_unit();
      gear_front = clampd(front + (rgf - 0.5) * 0.30, 0.02, 8.0);
      input.gear_front_clearance = gear_front;

      const double rl = rng_unit();
      if (rl < 0.15) {
        left = INFINITY;
      } else {
        const double rl2 = rng_unit();
        left = clampd(left + (rl2 - 0.5) * 0.8, 0.01, 3.0);
      }
      input.left_side_clearance = left;

      const double rr = rng_unit();
      if (rr < 0.15) {
        right = INFINITY;
      } else {
        const double rr2 = rng_unit();
        right = clampd(right + (rr2 - 0.5) * 0.8, 0.01, 3.0);
      }
      input.right_side_clearance = right;

      input.global_nearest_radar_distance = 0.05 + 8.0 * rng_unit();

      const double rsw = rng_unit();
      input.sweep_clearance = rsw < 0.20 ? INFINITY : 0.02 + 2.5 * rsw;
      input.restore_pose_stable = rng_unit() < 0.60;

      const dwa_dynamic_velocity_decision_t d =
        dwa_dynamic_velocity_fusion_update(&fusion, &input);

      fprintf(g_out,
        "f,%d,%d,%.17g,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.17g,%.17g,"
        "%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
        s, t, input.now_sec,
        (int)d.mode,
        d.dwa_takeover ? 1 : 0, d.hall_speed_control_active ? 1 : 0,
        d.side_cap_active ? 1 : 0,
        d.raw_gear, d.effective_gear, d.raw_gear_valid ? 1 : 0,
        d.near_obstacle_gear_active ? 1 : 0, d.gear_closed_loop_active ? 1 : 0,
        d.gear_restore_hold_count, d.gear_target,
        d.gear_transition_active ? 1 : 0, d.gear_speed_transition_active ? 1 : 0,
        d.gear_speed_transition_target, d.gear_speed_transition_progress,
        d.gear, d.gear_peak_speed,
        d.side_clearance, d.side_predicted_clearance, d.side_closing_rate,
        d.side_speed_limit,
        d.moving_prebrake_distance, d.gear_trigger_distance,
        d.gear_transition_remaining_sec, d.gear_transition_compensation_distance,
        d.target_max_velocity, d.held_forward_speed);
    }
  }
}

/* ---------------- P3: SideWallHoldController 状态序列 ---------------- */

static void run_side_wall(int scenarios)
{
  for (int s = 0; s < scenarios; ++s) {
    dwa_side_wall_hold_config_t config = dwa_side_wall_hold_config_default();
    const double tweak = rng_unit();
    if (s & 1) {
      config.hold_distance = 0.15 + 0.20 * tweak;
      config.release_distance = config.hold_distance + 0.05;
    }

    dwa_side_wall_hold_controller_t controller;
    dwa_side_wall_hold_controller_init(&controller, &config);

    double now = 0.0;
    double left = 0.5 + rng_unit() * 1.5;
    double right = 0.5 + rng_unit() * 1.5;
    const int pattern = s % 4;

    for (int t = 0; t < TICKS; ++t) {
      now += 0.10;

      dwa_side_wall_hold_input_t input = dwa_side_wall_hold_input_default();
      input.now_sec = now;
      input.dt_sec = 0.10;
      input.forward = rng_unit() < 0.80;
      input.perception_fresh = rng_unit() < 0.90;

      const double rl = rng_unit();
      if (pattern == 0) {
        left = clampd(left + (rl - 0.5) * 0.4, 0.005, 3.0);
      } else if (pattern == 1) {
        left = clampd(left - 0.04 - 0.04 * rl, 0.005, 3.0);
      } else if (pattern == 2) {
        left = clampd(left + (rl - 0.5) * 0.2, 0.005, 3.0);
      } else {
        left = clampd(left - 0.03 - 0.03 * rl, 0.005, 3.0);
      }
      input.left_clearance = left;

      const double rr = rng_unit();
      if (pattern == 0) {
        right = clampd(right + (rr - 0.5) * 0.4, 0.005, 3.0);
      } else if (pattern == 1) {
        right = clampd(right + (rr - 0.5) * 0.2, 0.005, 3.0);
      } else if (pattern == 2) {
        right = clampd(right - 0.04 - 0.04 * rr, 0.005, 3.0);
      } else {
        right = clampd(right - 0.03 - 0.03 * rr, 0.005, 3.0);
      }
      input.right_clearance = right;

      input.requested_yawrate = (rng_unit() - 0.5) * 1.2;

      const dwa_side_wall_hold_result_t r =
        dwa_side_wall_hold_controller_update(&controller, &input);

      fprintf(g_out,
        "w,%d,%d,%d,%d,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%d,%d\n",
        s, t,
        r.active ? 1 : 0, r.left_latched ? 1 : 0, r.right_latched ? 1 : 0,
        r.left_active ? 1 : 0, r.right_active ? 1 : 0,
        (int)r.side,
        r.correction_yawrate, r.output_yawrate,
        r.left_clearance, r.right_clearance,
        r.left_predicted_clearance, r.right_predicted_clearance,
        r.left_closing_rate, r.right_closing_rate,
        r.left_clearance_held ? 1 : 0, r.right_clearance_held ? 1 : 0);
    }
  }
}

/* ---------------- P3: DwaCorridorGuide 状态序列 ---------------- */

static void run_corridor(int scenarios)
{
  for (int s = 0; s < scenarios; ++s) {
    dwa_corridor_guide_config_t config = dwa_corridor_guide_config_default();
    const double tweak = rng_unit();
    if (s & 1) {
      config.search_half_width = 0.90 + 0.60 * tweak;
    }

    dwa_corridor_guide_t guide;
    dwa_corridor_guide_init(&guide, &config);

    const double corridor_width = 0.55 + rng_unit() * 0.60;
    const double center_y = (rng_unit() - 0.5) * 0.80;
    const bool wall_present = (s % 3) != 2;

    dwa_footprint_t footprint = dwa_footprint_default();
    footprint.front = 0.34;
    footprint.back = 0.32;
    footprint.half_width = 0.28;
    footprint.padding = 0.0;
    footprint.footrest_start = 0.34;
    footprint.footrest_front = 0.75;
    footprint.footrest_half_width = 0.14;
    footprint.toe_front = 0.90;
    footprint.toe_half_width = 0.10;

    dwa_point_t points[64];

    for (int t = 0; t < TICKS; ++t) {
      const bool wall_on = wall_present && ((t / 24) % 2 == 0);

      int count = 0;
      if (wall_on) {
        for (int k = 0; k < 11; ++k) {
          const double x = 0.30 + 0.15 * k;
          const double jx1 = (rng_unit() - 0.5) * 0.04;
          const double jy1 = (rng_unit() - 0.5) * 0.04;
          points[count].x = x + jx1;
          points[count].y = center_y + 0.5 * corridor_width + jy1;
          ++count;
          const double jx2 = (rng_unit() - 0.5) * 0.04;
          const double jy2 = (rng_unit() - 0.5) * 0.04;
          points[count].x = x + jx2;
          points[count].y = center_y - 0.5 * corridor_width + jy2;
          ++count;
        }
        const int n_noise = (int)(rng_unit() * 4.0);
        for (int i = 0; i < n_noise; ++i) {
          points[count].x = -1.0 + rng_unit() * 5.0;
          points[count].y = -3.0 + rng_unit() * 6.0;
          ++count;
        }
      } else {
        const int n = 1 + (int)(rng_unit() * 40.0);
        for (int i = 0; i < n; ++i) {
          points[count].x = -1.0 + rng_unit() * 5.0;
          points[count].y = -3.0 + rng_unit() * 6.0;
          ++count;
        }
      }

      dwa_corridor_guide_input_t input = dwa_corridor_guide_input_default();
      input.points = points;
      input.points_count = (size_t)count;
      input.footprint = footprint;
      input.intent.v = rng_unit() * 0.70;
      input.intent.w = (rng_unit() - 0.5) * 1.40;
      input.perception_fresh = rng_unit() < 0.90;
      input.forward_intent = input.intent.v > 0.05;
      input.max_yawrate = 1.40;

      const dwa_corridor_guidance_t *g = dwa_corridor_guide_update(&guide, &input);

      fprintf(g_out,
        "c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%s\n",
        s, t,
        g->detected ? 1 : 0, g->active ? 1 : 0, g->held ? 1 : 0,
        g->stable_cycles, g->lost_cycles,
        g->slice_count, g->candidate_count, g->cluster_count,
        g->raw_gate_count, g->blocked_gate_count,
        g->entry_x, g->entry_y, g->entry_distance, g->minimum_width,
        g->target_y, g->target_heading,
        g->reason ? g->reason : "-");

      const int n_slices = g->slice_count < 4 ? g->slice_count : 4;
      for (int i = 0; i < n_slices; ++i) {
        const dwa_corridor_slice_t *sl = &g->slices[i];
        fprintf(g_out, "cs,%d,%d,%d,%.17g,%.17g,%.17g\n",
          s, t, i, sl->center.x, sl->center.y, sl->width);
      }
    }
  }
}

/* ---------------- P3: DwaGeometricTrajectorySelector（链 P2 planning） ---------------- */

static void run_geometric(int scenarios)
{
  dwa_config_t plan_config = dwa_config_default();
  plan_config.use_speed_cost = true;
  plan_config.use_path_cost = true;
  plan_config.path_reference.enabled = true;
  plan_config.path_reference.target_y = 0.30;
  plan_config.path_reference.target_heading = 0.20;

  dwa_plan_result_t plan;
  dwa_plan_result_init(&plan);

  for (int s = 0; s < scenarios; ++s) {
    dwa_geometric_selector_config_t config =
      dwa_geometric_selector_config_default();
    const double tweak = rng_unit();
    if (s & 1) {
      config.lookahead_distance = 0.35 + 0.40 * tweak;
    }

    dwa_geometric_trajectory_selector_t selector;
    dwa_geometric_selector_init(&selector, &config);
    dwa_geometric_selection_t selection;
    dwa_geometric_selection_init(&selection);

    for (int t = 0; t < TICKS; ++t) {
      const bool cluster_mode = ((t / 24) % 2 == 0);

      const int n_obs = 1 + (int)(rng_unit() * 60.0);
      dwa_point_t obstacles[64];
      for (int i = 0; i < n_obs; ++i) {
        obstacles[i].x = -0.5 + rng_unit() * 3.0;
        obstacles[i].y = -1.5 + rng_unit() * 3.0;
      }
      const int obs_count = n_obs < 64 ? n_obs : 64;
      dwa_velocity_t current;
      current.v = rng_unit() * 0.50;
      current.w = (rng_unit() - 0.5) * 1.00;
      dwa_velocity_t intent;
      intent.v = rng_unit() * 0.70;
      intent.w = (rng_unit() - 0.5) * 1.40;

      dwa_plan_dwa_into(obstacles, obs_count, &current, &intent, &plan_config,
        &plan, NULL, NULL, NULL);

      const int n_left = 3 + (int)(rng_unit() * 10.0);
      const int n_right = 3 + (int)(rng_unit() * 10.0);

      const dwa_point_t left_laser = {0.50, 0.27};
      const dwa_point_t right_laser = {0.50, -0.27};

      dwa_geometric_scan_point_t left_scan[16];
      for (int i = 0; i < n_left; ++i) {
        left_scan[i].valid = rng_unit() < 0.92;
        if (cluster_mode) {
          left_scan[i].point.x = 0.55 + 0.80 * rng_unit();
          left_scan[i].point.y = -0.60 + 1.20 * rng_unit();
        } else {
          const double ang = (rng_unit() - 0.5) * 2.4;
          const double dist = 0.30 + 3.0 * rng_unit();
          left_scan[i].point.x = left_laser.x + dist * cos(ang);
          left_scan[i].point.y = left_laser.y + dist * sin(ang);
        }
        left_scan[i].range =
          hypot(left_scan[i].point.x - left_laser.x,
                left_scan[i].point.y - left_laser.y);
      }

      dwa_geometric_scan_point_t right_scan[16];
      for (int i = 0; i < n_right; ++i) {
        right_scan[i].valid = rng_unit() < 0.92;
        if (cluster_mode) {
          right_scan[i].point.x = 0.55 + 0.80 * rng_unit();
          right_scan[i].point.y = -0.60 + 1.20 * rng_unit();
        } else {
          const double ang = (rng_unit() - 0.5) * 2.4;
          const double dist = 0.30 + 3.0 * rng_unit();
          right_scan[i].point.x = right_laser.x + dist * cos(ang);
          right_scan[i].point.y = right_laser.y + dist * sin(ang);
        }
        right_scan[i].range =
          hypot(right_scan[i].point.x - right_laser.x,
                right_scan[i].point.y - right_laser.y);
      }

      dwa_geometric_selector_input_t input =
        dwa_geometric_selector_input_default();
      input.left_scan_points = left_scan;
      input.left_scan_points_count = (size_t)n_left;
      input.right_scan_points = right_scan;
      input.right_scan_points_count = (size_t)n_right;
      input.left_laser = left_laser;
      input.right_laser = right_laser;
      input.front_trigger_start_x = 0.50;
      input.clearance_front_edge = 0.75;
      input.front_trigger_base_half_width = 0.27;
      input.front_trigger_expand_angle = (float)(T_PI / 36.0);
      input.intent = intent;
      const double rb = rng_unit();
      input.base_direction = (int)(rb * 3.0) - 1;
      input.direction_epsilon = 0.02;

      dwa_geometric_selector_select(&selector, &plan, &input, &selection);

      const dwa_trajectory_t *st = &selection.selected_trajectory;
      fprintf(g_out,
        "g,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%zu,"
                "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%.17g,%d,%s\n",
        s, t,
        selection.applied ? 1 : 0, selection.geometry_valid ? 1 : 0,
        selection.obstacle_edge_valid ? 1 : 0, selection.cluster_valid ? 1 : 0,
        selection.handoff_ready ? 1 : 0, selection.tracking_active ? 1 : 0,
        selection.manual_reverse ? 1 : 0,
        selection.raw_base_direction, selection.base_direction,
        selection.requested_direction,
        selection.eligible_count, selection.obstacle_segment_size,
        selection.edge_candidate_count,
        selection.obstacle_segment_start, selection.obstacle_segment_end,
        selection.obstacle_edge_index,
        selection.obstacle_from_left_scan ? 1 : 0, selection.cluster_point_count,
        selection.stable_cycles, selection.selected_index,
        selection.obstacle_point.x, selection.obstacle_point.y,
        selection.cluster_centroid.x, selection.cluster_centroid.y,
        selection.radar_point.x, selection.radar_point.y,
        selection.reference_origin.x, selection.reference_origin.y,
        selection.selected_lookahead_point.x, selection.selected_lookahead_point.y,
        selection.selected.v, selection.selected.w,
        st->valid ? 1 : 0, st->command.v, st->command.w,
        st->min_clearance, st->total_cost,
        selection.raw_target_heading, selection.target_heading,
        selection.target_heading_compensation, selection.selected_heading,
        selection.heading_error, selection.edge_radar_distance,
        selection.edge_front_clearance,
        selection.cluster_min_x, selection.cluster_max_x,
        selection.cluster_min_y, selection.cluster_max_y,
        selection.raw_heading, selection.raw_heading_error,
        selection.lookahead_distance,
        selection.target_heading_compensated ? 1 : 0,
        selection.reason ? selection.reason : "-");
    }

    dwa_geometric_selection_destroy(&selection);
  }

  dwa_plan_result_destroy(&plan);
}

/* ---------------- P3: safety 纯函数 ---------------- */

static void run_safety(int scenarios)
{
  const dwa_front_safety_config_t front_cfg = dwa_front_safety_config_default();
  const dwa_turn_classification_config_t turn_cfg =
    dwa_turn_classification_config_default();
  const dwa_side_safety_config_t side_cfg = dwa_side_safety_config_default();

  for (int s = 0; s < scenarios; ++s) {
    for (int t = 0; t < TICKS; ++t) {
      /* --- front_safety --- */
      dwa_footprint_t fp = dwa_footprint_default();
      fp.front = 0.30 + 0.20 * rng_unit();
      fp.back = 0.10 + 0.10 * rng_unit();
      fp.half_width = 0.20 + 0.15 * rng_unit();
      fp.padding = 0.05 * rng_unit();
      fp.footrest_start = 0.30 + 0.10 * rng_unit();
      fp.footrest_front = fp.footrest_start + 0.20 + 0.30 * rng_unit();
      fp.footrest_half_width = 0.10 + 0.08 * rng_unit();
      fp.toe_front = fp.footrest_front + 0.05 + 0.10 * rng_unit();
      fp.toe_half_width = 0.05 + 0.06 * rng_unit();

      const double edge = dwa_collision_front_edge(&fp);
      const double x_query = 4.0 * rng_unit();
      const double half_w = dwa_front_trigger_half_width_at(x_query, &front_cfg);
      dwa_point_t probe;
      probe.x = -1.0 + rng_unit() * 5.0;
      probe.y = -3.0 + rng_unit() * 6.0;
      const bool inside = dwa_point_inside_front_trigger_zone(&probe, &front_cfg);

      fprintf(g_out, "sf,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",
        s, t, edge, x_query, half_w, probe.x, probe.y, inside ? 1 : 0);

      /* --- escape_controller --- */
      dwa_velocity_t intent;
      intent.v = -0.30 + 1.10 * rng_unit();
      intent.w = -1.40 + 2.80 * rng_unit();
      float ang1 = 90.0f;
      const planner_manual_turn_direction_t dir1 =
        dwa_manual_turn_direction(&intent, &turn_cfg, &ang1);
      float ang2 = 90.0f;
      const planner_manual_turn_direction_t dir2 =
        dwa_manual_turn_direction_with_reverse(&intent, &turn_cfg, &ang2);

      planner_side_clearance_diagnostics_t clearance =
        planner_side_clearance_diagnostics_default();
      const int bucket_l = (int)(rng_unit() * 4.0);
      const double val_l = rng_unit();
      if (bucket_l == 0) {
        clearance.left = 0.30 * val_l;
      } else if (bucket_l == 1) {
        clearance.left = 0.30 + 2.0 * val_l;
      } else if (bucket_l == 2) {
        clearance.left = INFINITY;
      } else {
        clearance.left = NAN;
      }
      const int bucket_r = (int)(rng_unit() * 4.0);
      const double val_r = rng_unit();
      if (bucket_r == 0) {
        clearance.right = 0.30 * val_r;
      } else if (bucket_r == 1) {
        clearance.right = 0.30 + 2.0 * val_r;
      } else if (bucket_r == 2) {
        clearance.right = INFINITY;
      } else {
        clearance.right = NAN;
      }
      const bool esc_enabled = rng_unit() < 0.90;
      const double esc_threshold = 0.30 * rng_unit();
      const dwa_front_escape_decision_t esc =
        dwa_choose_hard_stop_escape_direction(&clearance, esc_enabled,
          esc_threshold);

      fprintf(g_out, "se,%d,%d,%.17g,%.17g,%.17g,%.17g,%d,%d,%.17g,%d,%d,%d\n",
        s, t, intent.v, intent.w, ang1, ang2,
        (int)dir1, (int)dir2,
        esc_threshold, esc_enabled ? 1 : 0,
        esc.allowed ? 1 : 0, (int)esc.direction);

      /* --- side_safety: evaluate_side_hard_stop_circle --- */
      dwa_velocity_t intent2;
      intent2.v = -0.20 + 1.00 * rng_unit();
      intent2.w = -1.40 + 2.80 * rng_unit();
      planner_side_hard_stop_circle_diagnostics_t diag =
        planner_side_hard_stop_circle_diagnostics_default();
      const double rl = rng_unit();
      if (rl < 0.30) {
        diag.left_radius = INFINITY;
      } else {
        diag.left_radius = 0.60 * rng_unit();
      }
      diag.left_point.x = -0.5 + rng_unit() * 2.0;
      diag.left_point.y = -1.0 + rng_unit() * 2.0;
      const double rr = rng_unit();
      if (rr < 0.30) {
        diag.right_radius = INFINITY;
      } else {
        diag.right_radius = 0.60 * rng_unit();
      }
      diag.right_point.x = -0.5 + rng_unit() * 2.0;
      diag.right_point.y = -1.0 + rng_unit() * 2.0;
      const bool circle_fresh = rng_unit() < 0.90;
      const planner_side_hard_stop_decision_t circ =
        dwa_evaluate_side_hard_stop_circle(&intent2, circle_fresh, &diag,
          &side_cfg);

      fprintf(g_out, "sc,%d,%d,%d,%d,%d,%.17g,%.17g,%.17g\n",
        s, t, circ.stop ? 1 : 0, (int)circ.direction,
        (int)circ.side, circ.radius,
        circ.point.x, circ.point.y);

      /* --- side_safety: evaluate_side_sweep_guard --- */
      dwa_velocity_t command;
      command.v = 0.70 * rng_unit();
      command.w = (rng_unit() - 0.5) * 1.60;
      dwa_velocity_t manual;
      manual.v = 0.50 * rng_unit();
      manual.w = (rng_unit() - 0.5) * 1.60;
      planner_motor_feedback_snapshot_t feedback =
        planner_motor_feedback_snapshot_default();
      feedback.fresh = rng_unit() < 0.80;
      feedback.left_mps = rng_unit() * 1.0;
      feedback.right_mps = rng_unit() * 1.0;
      const bool sweep_fresh = rng_unit() < 0.90;
      const int n_pts = (int)(rng_unit() * 25.0);
      dwa_point_t safety_points[32];
      for (int i = 0; i < n_pts; ++i) {
        safety_points[i].x = 1.2 * rng_unit();
        safety_points[i].y = -0.8 + 1.6 * rng_unit();
      }
      const planner_side_hard_stop_decision_t sweep =
        dwa_evaluate_side_sweep_guard(&command, &manual, &feedback, sweep_fresh,
          safety_points, (size_t)n_pts, &side_cfg);

      fprintf(g_out,
        "sg,%d,%d,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g\n",
        s, t, sweep.stop ? 1 : 0, sweep.yaw_limited ? 1 : 0,
        (int)sweep.direction, (int)sweep.side,
        sweep.radius, sweep.prediction_time_sec, sweep.limited_yawrate,
        sweep.point.x, sweep.point.y);
    }
  }
}

int main(int argc, char **argv)
{
  const char *out_path = argc > 1 ? argv[1] : "p3p4_c.csv";
  const int scenarios = argc > 2 ? atoi(argv[2]) : 48;
  g_out = fopen(out_path, "w");
  if (g_out == NULL) {
    fprintf(stderr, "cannot open %s\n", out_path);
    return 1;
  }

  run_fusion(scenarios);
  run_side_wall(scenarios);
  run_corridor(scenarios);
  run_geometric(scenarios);
  run_safety(scenarios);

  fclose(g_out);
  printf("test_p3p4: %d scenarios -> %s\n", scenarios, out_path);
  return 0;
}
