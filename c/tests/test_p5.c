/*
 * P5 perception parity harness（C 侧）。
 * 覆盖：laser_filter_core（filter_ranges/sector_minimums）、
 * laser_scan_fusion（append_scan 坐标变换）、local_occupancy_grid
 * （reset/mark/raytrace/inflate/clear_footprint）、planning 的
 * grid/blocking_index 路径（footprint_hits_grid + plan_dwa_into 带 grid，
 * 补 P3 遗留缺口）。
 *
 * 与 tools/parity/parity_p5_cpp.cpp 使用同一 LCG 生成相同输入；
 * 本侧输入经 float 字段隐式量化（P4.5 规范），C++ 侧显式 q()。
 * 修改任一侧的 rng 调用顺序必须同步另一侧！
 * 用法: test_p5 [输出csv路径] [场景数]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/perception/laser_filter_core.h"
#include "../src/perception/laser_scan_fusion.h"
#include "../src/perception/local_occupancy_grid.h"
#include "../src/planning/dwa_sampling.h"
#include "../src/planning/dwa_collision.h"
#include "../src/common/dwa_types.h"

static unsigned long long rng_state = 0x9E3779B97F4A7C15ULL;

static unsigned long long rng_next(void)
{
  rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return rng_state;
}

static double rng_unit(void)
{
  return (double)(rng_next() >> 11) * (1.0 / 9007199254740992.0);
}

#define T_PI 3.14159265358979323846
#define MAX_RANGES 704
#define MAX_POINTS 512
#define MAX_CELLS 16384
#define CHUNK_VALS 48

/* 统一特殊值字符串化（两侧必须一致；NaN 不区分符号）。 */
static void fmt_val(char *buf, size_t cap, float v)
{
  if (isnan(v)) snprintf(buf, cap, "nan");
  else if (isinf(v)) snprintf(buf, cap, v > 0.0f ? "inf" : "-inf");
  else snprintf(buf, cap, "%.17g", (double)v);
}

static void dump_val_chunk(
  FILE *out, int s, int chunk, const float *vals,
  size_t begin, size_t count, const char *tag)
{
  size_t i;
  fprintf(out, "%s,%d,%d", tag, s, chunk);
  for (i = 0; i < count; ++i) {
    char buf[40];
    fmt_val(buf, sizeof(buf), vals[begin + i]);
    fprintf(out, ",%s", buf);
  }
  fputc('\n', out);
}

int main(int argc, char **argv)
{
  const char *out_path = argc > 1 ? argv[1] : "p5_c.csv";
  const int scenarios = argc > 2 ? atoi(argv[2]) : 48;
  FILE *out = fopen(out_path, "w");
  int s;

  if (out == NULL) {
    fprintf(stderr, "cannot open %s\n", out_path);
    return 1;
  }

  for (s = 0; s < scenarios; ++s) {
    /* ============ 1) filter_ranges ============ */
    {
      float ranges[MAX_RANGES];
      float output[MAX_RANGES];
      dwa_laser_filter_config_t config;
      dwa_laser_filter_stats_t stats;
      float range_min, range_max;
      int n = 64 + (int)(rng_unit() * 640.0); /* 64..703 */
      int i;
      size_t out_n;

      range_min = 0.05 + rng_unit() * 0.15;
      range_max = 8.0 + rng_unit() * 22.0;
      config = dwa_laser_filter_config_default();
      config.window_size = 3 + (int)(rng_unit() * 7.0);
      config.stddev_multiplier = 0.5 + rng_unit() * 2.0;
      config.min_valid_samples = 1 + (int)(rng_unit() * 4.0);
      config.min_cluster_size = 1 + (int)(rng_unit() * 4.0);
      config.max_neighbor_gap = (float)(rng_unit() * 0.3);

      for (i = 0; i < n; ++i) {
        const double r = rng_unit();
        if (r < 0.05) ranges[i] = (float)NAN;
        else if (r < 0.08) ranges[i] = (float)INFINITY;
        else if (r < 0.11) ranges[i] = (float)-INFINITY;
        else if (r < 0.15) ranges[i] = -(float)(rng_unit() * 5.0);
        else if (r < 0.28) ranges[i] = rng_unit() * (double)range_min;
        else ranges[i] = (double)range_min +
          rng_unit() * ((double)range_max - (double)range_min);
      }

      out_n = dwa_filter_ranges(
        ranges, (size_t)n, range_min, range_max, &config, output, &stats);

      fprintf(out,
        "fh,%d,%d,%.17g,%.17g,%d,%.17g,%d,%d,%.17g,%zu,%zu,%zu,%zu,%zu\n",
        s, n, (double)range_min, (double)range_max, config.window_size,
        (double)config.stddev_multiplier, config.min_valid_samples,
        config.min_cluster_size, (double)config.max_neighbor_gap,
        stats.input_count, stats.invalid_count, stats.near_range_clamped_count,
        stats.statistical_outlier_count, stats.output_count);
      {
        size_t done = 0;
        int chunk = 0;
        while (done < out_n) {
          const size_t cnt = out_n - done < CHUNK_VALS ? out_n - done : CHUNK_VALS;
          dump_val_chunk(out, s, chunk, output, done, cnt, "fv");
          done += cnt;
          ++chunk;
        }
      }

      /* ============ 2) sector_minimums（同一帧原始输入） ============ */
      {
        float sectors[64];
        float angle_min = (float)(-T_PI + rng_unit() * 0.5);
        float angle_increment = (float)(0.005 + rng_unit() * 0.05);
        float sector_start = (float)(-T_PI + rng_unit() * 0.5);
        float sector_end = (float)((double)sector_start + T_PI + rng_unit() * 3.0);
        int sector_count = 8 + (int)(rng_unit() * 40.0);
        int sn;
        int k;

        sn = dwa_sector_minimums(
          ranges, (size_t)n, angle_min, angle_increment,
          sector_start, sector_end, sector_count, sectors);
        fprintf(out, "sm,%d,%d,%.17g,%.17g,%.17g,%.17g",
          s, sn, (double)angle_min, (double)angle_increment,
          (double)sector_start, (double)sector_end);
        for (k = 0; k < sn; ++k) {
          char buf[40];
          fmt_val(buf, sizeof(buf), sectors[k]);
          fprintf(out, ",%s", buf);
        }
        fputc('\n', out);
      }
    }

    /* ============ 3) laser_scan_fusion append_scan ============ */
    {
      dwa_fusion_point_t points[MAX_POINTS];
      size_t points_count = 0;
      float lr[MAX_RANGES / 2], rr[MAX_RANGES / 2];
      dwa_fusion_scan_t lscan, rscan;
      dwa_fusion_mount_t lmount, rmount;
      int nl = 32 + (int)(rng_unit() * 224.0);
      int nr = 32 + (int)(rng_unit() * 224.0);
      int i;
      size_t left_kept;

      memset(&lscan, 0, sizeof(lscan));
      memset(&rscan, 0, sizeof(rscan));
      lmount = dwa_fusion_mount_default();
      rmount = dwa_fusion_mount_default();
      lmount.x = (float)(0.3 + rng_unit() * 0.5);
      lmount.y = (float)(0.1 + rng_unit() * 0.3);
      lmount.yaw = (float)(-T_PI + rng_unit() * 2.0 * T_PI);
      lmount.reversed = rng_unit() < 0.5;
      lmount.offset = (float)((rng_unit() - 0.5) * 0.2);
      rmount.x = (float)(0.3 + rng_unit() * 0.5);
      rmount.y = (float)-(0.1 + rng_unit() * 0.3);
      rmount.yaw = (float)(-T_PI + rng_unit() * 2.0 * T_PI);
      rmount.reversed = rng_unit() < 0.5;
      rmount.offset = (float)((rng_unit() - 0.5) * 0.2);

      lscan.angle_min = (float)(-T_PI + rng_unit() * 0.5);
      lscan.angle_increment = (float)(0.005 + rng_unit() * 0.05);
      lscan.angle_max = (float)((double)lscan.angle_min +
        (double)lscan.angle_increment * (double)(nl - 1));
      lscan.range_min = (float)(0.05 + rng_unit() * 0.1);
      lscan.range_max = (float)(6.0 + rng_unit() * 20.0);
      rscan.angle_min = (float)(-T_PI + rng_unit() * 0.5);
      rscan.angle_increment = (float)(0.005 + rng_unit() * 0.05);
      rscan.angle_max = (float)((double)rscan.angle_min +
        (double)rscan.angle_increment * (double)(nr - 1));
      rscan.range_min = (float)(0.05 + rng_unit() * 0.1);
      rscan.range_max = (float)(6.0 + rng_unit() * 20.0);

      for (i = 0; i < nl; ++i) {
        const double r = rng_unit();
        if (r < 0.08) lr[i] = (float)NAN;
        else if (r < 0.16) lr[i] = (float)INFINITY;
        else lr[i] = (double)lscan.range_min +
          rng_unit() * ((double)lscan.range_max - (double)lscan.range_min);
      }
      for (i = 0; i < nr; ++i) {
        const double r = rng_unit();
        if (r < 0.08) rr[i] = (float)NAN;
        else if (r < 0.16) rr[i] = (float)INFINITY;
        else rr[i] = (double)rscan.range_min +
          rng_unit() * ((double)rscan.range_max - (double)rscan.range_min);
      }
      lscan.ranges = lr;
      lscan.range_count = (size_t)nl;
      rscan.ranges = rr;
      rscan.range_count = (size_t)nr;

      dwa_fusion_append_scan(&lscan, &lmount, 0U, points, MAX_POINTS, &points_count);
      left_kept = points_count;
      dwa_fusion_append_scan(&rscan, &rmount, 1U, points, MAX_POINTS, &points_count);

      fprintf(out,
        "fuh,%d,%d,%d,%.17g,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g,%d,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%zu\n",
        s, nl, nr,
        (double)lmount.x, (double)lmount.y, (double)lmount.yaw,
        lmount.reversed ? 1 : 0, (double)lmount.offset,
        (double)rmount.x, (double)rmount.y, (double)rmount.yaw,
        rmount.reversed ? 1 : 0, (double)rmount.offset,
        (double)lscan.angle_min, (double)lscan.angle_max,
        (double)lscan.angle_increment,
        (double)rscan.angle_min, (double)rscan.angle_max,
        (double)rscan.angle_increment,
        (double)lscan.range_min, (double)lscan.range_max,
        (double)rscan.range_min, (double)rscan.range_max,
        left_kept);
      {
        size_t done = 0;
        int chunk = 0;
        while (done < points_count) {
          const size_t cnt = points_count - done < 24 ? points_count - done : 24;
          size_t i2;
          fprintf(out, "fuv,%d,%d", s, chunk);
          for (i2 = 0; i2 < cnt; ++i2) {
            fprintf(out, ",%.17g,%.17g",
              (double)points[done + i2].x, (double)points[done + i2].y);
          }
          fputc('\n', out);
          done += cnt;
          ++chunk;
        }
      }
    }

    /* ============ 4) local_occupancy_grid + planning grid 路径 ============ */
    {
      dwa_occupancy_grid_t grid;
      dwa_occupancy_grid_config_t gconfig;
      dwa_footprint_t footprint;
      dwa_point_t obstacles[80];
      int m = 5 + (int)(rng_unit() * 60.0); /* 5..64 */
      int k = 3 + (int)(rng_unit() * 20.0);
      int i;
      size_t occ = 0, free_c = 0, unk = 0;
      int w, h;
      size_t cells_n;

      memset(&grid, 0, sizeof(grid));
      gconfig = dwa_occupancy_grid_config_default();
      gconfig.resolution = (float)(0.05 + rng_unit() * 0.10);
      gconfig.min_x = (float)-(rng_unit() * 1.2);
      gconfig.max_x = (float)(1.5 + rng_unit() * 2.0);
      gconfig.max_abs_y = (float)(0.8 + rng_unit() * 1.4);
      gconfig.inflation_radius = (float)(rng_unit() * 0.35);

      footprint = dwa_footprint_default();
      footprint.padding = (float)(rng_unit() * 0.05);

      dwa_reset_occupancy_grid(&grid, &gconfig);
      w = grid.width;
      h = grid.height;
      cells_n = grid.cells_size;
      if (cells_n > MAX_CELLS) cells_n = MAX_CELLS; /* 防御：上限内 dump */

      for (i = 0; i < m && i < 80; ++i) {
        int cx, cy;
        obstacles[i].x = (double)gconfig.min_x +
          rng_unit() * ((double)gconfig.max_x - (double)gconfig.min_x);
        obstacles[i].y = -(double)gconfig.max_abs_y +
          rng_unit() * 2.0 * (double)gconfig.max_abs_y;
        if (dwa_grid_world_to_cell(&grid, obstacles[i].x, obstacles[i].y, &cx, &cy)) {
          dwa_mark_grid_occupied(&grid, cx, cy);
        }
      }
      for (i = 0; i < k; ++i) {
        const float sx = (float)((double)gconfig.min_x +
          rng_unit() * ((double)gconfig.max_x - (double)gconfig.min_x));
        const float sy = (float)(-(double)gconfig.max_abs_y +
          rng_unit() * 2.0 * (double)gconfig.max_abs_y);
        const float ex = (float)((double)gconfig.min_x +
          rng_unit() * ((double)gconfig.max_x - (double)gconfig.min_x));
        const float ey = (float)(-(double)gconfig.max_abs_y +
          rng_unit() * 2.0 * (double)gconfig.max_abs_y);
        dwa_raytrace_free(&grid, sx, sy, ex, ey);
      }
      dwa_inflate_occupied_cells(&grid, gconfig.inflation_radius);
      dwa_clear_current_footprint(&grid, &footprint);

      for (i = 0; i < (int)cells_n; ++i) {
        if (grid.cells[i] >= 100) ++occ;
        else if (grid.cells[i] == 0) ++free_c;
        else ++unk;
      }
      fprintf(out, "gh,%d,%.17g,%.17g,%.17g,%d,%d,%zu,%zu,%zu\n",
        s, (double)grid.resolution, (double)grid.origin_x, (double)grid.origin_y,
        w, h, occ, free_c, unk);
      {
        size_t done = 0;
        int chunk = 0;
        while (done < cells_n) {
          const size_t cnt = cells_n - done < CHUNK_VALS ? cells_n - done : CHUNK_VALS;
          size_t i2;
          fprintf(out, "gk,%d,%d", s, chunk);
          for (i2 = 0; i2 < cnt; ++i2) {
            fprintf(out, ",%d", (int)grid.cells[done + i2]);
          }
          fputc('\n', out);
          done += cnt;
          ++chunk;
        }
      }

      /* ---- footprint_hits_grid 纯函数对拍（含/不含 blocking index）---- */
      {
        dwa_config_t config = dwa_config_default();
        dwa_grid_blocking_index_t blocking;
        const bool unknown_is_obstacle = rng_unit() < 0.5;
        int p;
        config.footprint = footprint;
        config.unknown_is_obstacle = unknown_is_obstacle;
        dwa_grid_blocking_index_init(&blocking);
        dwa_build_grid_blocking_index(&grid, unknown_is_obstacle, &blocking);
        for (p = 0; p < 4; ++p) {
          dwa_state_t state;
          bool hit_unknown = false, hit_unknown_b = false;
          bool hit, hit_b;
          float cos_yaw, sin_yaw;
          state.x = (float)((double)gconfig.min_x +
            rng_unit() * ((double)gconfig.max_x - (double)gconfig.min_x));
          state.y = (float)(-(double)gconfig.max_abs_y +
            rng_unit() * 2.0 * (double)gconfig.max_abs_y);
          state.yaw = (float)(-T_PI + rng_unit() * 2.0 * T_PI);
          state.v = (float)(rng_unit() * 1.7);
          state.w = (float)(rng_unit() * 2.4 - 1.2);
          cos_yaw = cosf(state.yaw);
          sin_yaw = sinf(state.yaw);
          hit = dwa_footprint_hits_grid(
            &state, &grid, &config, cos_yaw, sin_yaw, &hit_unknown, NULL);
          hit_b = dwa_footprint_hits_grid(
            &state, &grid, &config, cos_yaw, sin_yaw, &hit_unknown_b, &blocking);
          fprintf(out, "fhg,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%d\n",
            s, (double)state.x, (double)state.y, (double)state.yaw,
            (double)state.v, (double)state.w, (double)cos_yaw, (double)sin_yaw,
            hit ? 1 : 0, hit_unknown ? 1 : 0, hit_b ? 1 : 0, hit_unknown_b ? 1 : 0);
        }
        dwa_grid_blocking_index_destroy(&blocking);
      }

      /* ---- plan_dwa_into 带 grid + blocking/spatial 工作区 ---- */
      {
        dwa_config_t config = dwa_config_default();
        dwa_plan_result_t result;
        dwa_velocity_t current, intent;
        dwa_obstacle_spatial_index_t spatial;
        dwa_grid_blocking_index_t blocking;
        const dwa_trajectory_t *sel;

        config.use_speed_cost = true;
        config.use_path_cost = true;
        config.path_reference.enabled = true;
        config.path_reference.target_y = 0.30f;
        config.path_reference.target_heading = 0.20f;
        config.footprint = footprint;
        config.unknown_is_obstacle = rng_unit() < 0.5;

        current.v = (float)(rng_unit() * 1.7);
        current.w = (float)(rng_unit() * 2.4 - 1.2);
        intent.v = (float)(rng_unit() * 1.7);
        intent.w = (float)(rng_unit() * 2.4 - 1.2);

        dwa_plan_result_init(&result);
        dwa_obstacle_spatial_index_init(&spatial);
        dwa_grid_blocking_index_init(&blocking);
        dwa_plan_dwa_into(obstacles, (size_t)m, &current, &intent, &config,
          &result, &grid, &spatial, &blocking);
        sel = &result.selected_trajectory;
        fprintf(out,
          "pg,%d,%d,%.17g,%.17g,%.17g,%.17g,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s\n",
          s, m, (double)current.v, (double)current.w, (double)intent.v,
          (double)intent.w, result.valid_count, sel->valid ? 1 : 0,
          (double)result.selected.v, (double)result.selected.w,
          (double)sel->min_clearance, (double)sel->total_cost,
          (double)sel->obstacle_cost, (double)sel->to_goal_cost,
          (double)sel->speed_cost, (double)sel->path_cost,
          sel->reject_reason ? sel->reject_reason : "-");
        dwa_grid_blocking_index_destroy(&blocking);
        dwa_obstacle_spatial_index_destroy(&spatial);
        dwa_plan_result_destroy(&result);
      }

      /* ---- plan_dwa_into 带 grid、NULL blocking 工作区（内部自建路径）---- */
      {
        dwa_config_t config = dwa_config_default();
        dwa_plan_result_t result;
        dwa_velocity_t current, intent;
        const dwa_trajectory_t *sel;

        config.use_speed_cost = true;
        config.use_path_cost = true;
        config.path_reference.enabled = true;
        config.path_reference.target_y = 0.30f;
        config.path_reference.target_heading = 0.20f;
        config.footprint = footprint;
        config.unknown_is_obstacle = rng_unit() < 0.5;

        current.v = (float)(rng_unit() * 1.7);
        current.w = (float)(rng_unit() * 2.4 - 1.2);
        intent.v = (float)(rng_unit() * 1.7);
        intent.w = (float)(rng_unit() * 2.4 - 1.2);

        dwa_plan_result_init(&result);
        dwa_plan_dwa_into(obstacles, (size_t)m, &current, &intent, &config,
          &result, &grid, NULL, NULL);
        sel = &result.selected_trajectory;
        fprintf(out,
          "pgn,%d,%d,%.17g,%.17g,%.17g,%.17g,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s\n",
          s, m, (double)current.v, (double)current.w, (double)intent.v,
          (double)intent.w, result.valid_count, sel->valid ? 1 : 0,
          (double)result.selected.v, (double)result.selected.w,
          (double)sel->min_clearance, (double)sel->total_cost,
          (double)sel->obstacle_cost, (double)sel->to_goal_cost,
          (double)sel->speed_cost, (double)sel->path_cost,
          sel->reject_reason ? sel->reject_reason : "-");
        dwa_plan_result_destroy(&result);
      }

      dwa_occupancy_grid_destroy(&grid);
    }
  }

  fclose(out);
  printf("test_p5: %d scenarios -> %s\n", scenarios, out_path);
  return 0;
}
