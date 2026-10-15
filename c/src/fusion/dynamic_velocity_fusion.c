/*
 * 妯″潡鍔熻兘锛氬姩鎬侀€熷害鐘舵€佹満瀹炵幇銆? * 鐩磋瘧鑷?native/dynamic_velocity_fusion.hpp锛堥€愯瀵瑰簲锛屼笉鏀归€昏緫/榛樿鍊?鎵ц椤哄簭锛夈€? */
#include <string.h>

#include "dynamic_velocity_fusion.h"

static inline float dvf_min(float a, float b) { return a < b ? a : b; }
static inline float dvf_max(float a, float b) { return a > b ? a : b; }

/* ---------------- 閰嶇疆 ---------------- */

dwa_dynamic_velocity_config_t dwa_dynamic_velocity_config_default(void)
{
  dwa_dynamic_velocity_config_t config;
  config.forward_speed_limit = 0.70f;
  config.moving_prebrake_base_distance = 0.70f;
  config.moving_prebrake_speed_ratio = 0.70f;
  config.moving_prebrake_target_speed = 0.40f;
  config.moving_prebrake_deceleration = 1.20f;
  config.moving_prebrake_profile_far_distance = 2.20f;
  config.moving_prebrake_profile_zero_distance = 0.40f;
  config.moving_prebrake_recovery_acceleration = 0.40f;
  config.moving_dwa_speed_limit = 0.42f;
  config.moving_dwa_yawrate_limit = 0.70f;
  config.moving_dwa_force_gear = true;
  config.moving_dwa_gear_level = 1;
  config.near_obstacle_gear_enabled = true;
  config.near_obstacle_gear_center_x = 0.38f;
  config.near_obstacle_gear_center_y = 0.0f;
  config.near_obstacle_gear_radius = 1.0f;
  config.near_obstacle_gear_release_radius = 1.10f;
  config.near_obstacle_gear_level = 1;
  config.moving_dwa_trigger_distance = 0.50f;
  config.moving_dwa_entry_speed_max = 0.45f;
  config.moving_release_hysteresis = 0.80f;
  config.static_start_distance = 0.50f;
  config.static_vdwa_gain = 2.50f;
  config.release_margin = 0.0f;
  config.release_clear_sec = 0.0f;
  config.speed_restore_side_clearance = 0.45f;
  config.speed_restore_sweep_clearance = 0.45f;
  config.speed_restore_stable_sec = 1.0f;
  config.speed_restore_max_yawrate = 0.12f;
  config.static_recalculate_sec = 0.10f;
  config.gear_peak_speeds[0] = 0.70f;
  config.gear_peak_speeds[1] = 1.00f;
  config.gear_peak_speeds[2] = 1.20f;
  config.gear_peak_speeds[3] = 1.40f;
  config.gear_peak_speeds[4] = 1.67f;
  config.fallback_gear = 1;
  config.gear_closed_loop_enabled = true;
  config.gear_restore_hysteresis = 0.10f;
  config.gear_force_level = 1;
  config.gear_force_forward_only = true;
  config.gear_front_trigger_extra_angle_deg = 10.0f;
  config.gear_restore_hold_cycles = 10;
  config.gear_transition_step_sec = 0.15f;
  config.gear_transition_safety_margin = 0.05f;
  config.gear_speed_transition_enabled = true;
  config.gear_speed_transition_duration_sec = 2.0f;
  config.side_speed_trigger = 1.20f;
  config.side_speed_target = 1.20f;
  config.side_limit_distance = 0.30f;
  config.side_prediction_reaction_sec = 0.25f;
  config.side_prediction_deceleration = 1.50f;
  config.side_closing_rate_filter_alpha = 0.35f;
  config.side_closing_rate_max = 1.00f;
  config.side_cap_release_distance = 0.30f;
  config.side_cap_release_clear_sec = 0.30f;
  config.side_target_down_rate = 1.50f;
  config.side_target_up_rate = 0.40f;
  config.bypass_near_zone_release_hold_cycles = 3;
  config.bypass_near_zone_ramp_enabled = true;
  config.bypass_near_zone_ramp_deceleration = 0.40f;
  config.bypass_near_zone_far_distance = 2.20f;
  /* A3(10.09)：释放侧净距门控（板端 yaml 覆盖 0.40）。 */
  config.bypass_near_zone_release_side_distance = 0.40f;
  return config;
}

bool dwa_validate_dynamic_velocity_config(
  const dwa_dynamic_velocity_config_t *config,
  const char **reason)
{
  /* 鍘熶唬鐮侊細涓€涓暱 || 閾?+ gear_peak_speeds 寰幆鏍￠獙锛堟亽 5 涓厓绱狅級銆?*/
  const bool invalid = (    !isfinite(config->forward_speed_limit) || config->forward_speed_limit <= 0.0f ||
    !isfinite(config->moving_prebrake_base_distance) ||
    config->moving_prebrake_base_distance <= 0.0f ||
    !isfinite(config->moving_prebrake_speed_ratio) ||
    config->moving_prebrake_speed_ratio < 0.0f ||
    !isfinite(config->moving_prebrake_target_speed) ||
    config->moving_prebrake_target_speed <= 0.0f ||
    config->moving_prebrake_target_speed > config->forward_speed_limit ||
    !isfinite(config->moving_prebrake_deceleration) ||
    config->moving_prebrake_deceleration <= 0.0f ||
    !isfinite(config->moving_prebrake_profile_far_distance) ||
    config->moving_prebrake_profile_far_distance <= 0.0f ||
    !isfinite(config->moving_prebrake_profile_zero_distance) ||
    config->moving_prebrake_profile_zero_distance < 0.0f ||
    config->moving_prebrake_profile_zero_distance >=
    config->moving_prebrake_profile_far_distance ||
    !isfinite(config->moving_prebrake_recovery_acceleration) ||
    config->moving_prebrake_recovery_acceleration <= 0.0f ||
    !isfinite(config->moving_dwa_speed_limit) ||
    config->moving_dwa_speed_limit <= 0.0f ||
    config->moving_dwa_speed_limit > config->forward_speed_limit ||
    !isfinite(config->moving_dwa_yawrate_limit) ||
    config->moving_dwa_yawrate_limit <= 0.0f ||
    !config->moving_dwa_gear_level ||
    config->moving_dwa_gear_level < 1 ||
    config->moving_dwa_gear_level > DWA_FUSION_GEAR_COUNT ||
    !isfinite(config->near_obstacle_gear_center_x) ||
    !isfinite(config->near_obstacle_gear_center_y) ||
    !isfinite(config->near_obstacle_gear_radius) ||
    config->near_obstacle_gear_radius <= 0.0f ||
    !isfinite(config->near_obstacle_gear_release_radius) ||
    config->near_obstacle_gear_release_radius < config->near_obstacle_gear_radius ||
    !config->near_obstacle_gear_level ||
    config->near_obstacle_gear_level < 1 ||
    config->near_obstacle_gear_level > DWA_FUSION_GEAR_COUNT ||
    !isfinite(config->moving_dwa_trigger_distance) ||
    config->moving_dwa_trigger_distance <= 0.0f ||
    !isfinite(config->moving_dwa_entry_speed_max) ||
    config->moving_dwa_entry_speed_max < config->moving_prebrake_target_speed ||
    config->moving_dwa_entry_speed_max > config->forward_speed_limit ||
    !isfinite(config->moving_release_hysteresis) ||
    config->moving_release_hysteresis < 0.0f ||
    !isfinite(config->static_start_distance) ||
    config->static_start_distance <= 0.0f ||
    !isfinite(config->static_vdwa_gain) || config->static_vdwa_gain <= 0.0f ||
    !isfinite(config->release_margin) || config->release_margin < 0.0f ||
    !isfinite(config->release_clear_sec) || config->release_clear_sec < 0.0f ||
    !isfinite(config->static_recalculate_sec) ||
    config->static_recalculate_sec < 0.0f ||
    !isfinite(config->gear_restore_hysteresis) ||
    config->gear_restore_hysteresis < 0.0f ||
    config->gear_force_level < 1 ||
    config->gear_force_level > DWA_FUSION_GEAR_COUNT ||
    !isfinite(config->gear_front_trigger_extra_angle_deg) ||
    config->gear_front_trigger_extra_angle_deg < 0.0f ||
    config->gear_front_trigger_extra_angle_deg > 45.0f ||
    config->gear_restore_hold_cycles < 0 ||
    !isfinite(config->gear_transition_step_sec) ||
    config->gear_transition_step_sec < 0.0f ||
    !isfinite(config->gear_transition_safety_margin) ||
    config->gear_transition_safety_margin < 0.0f ||
    !isfinite(config->gear_speed_transition_duration_sec) ||
    config->gear_speed_transition_duration_sec <= 0.0f ||
    config->fallback_gear < 1 ||
    config->fallback_gear > DWA_FUSION_GEAR_COUNT ||
    !isfinite(config->side_speed_trigger) || config->side_speed_trigger <= 0.0f ||
    !isfinite(config->side_speed_target) || config->side_speed_target <= 0.0f ||
    config->side_speed_target > config->side_speed_trigger ||
    !isfinite(config->side_limit_distance) ||
    config->side_limit_distance <= 0.0f ||
    !isfinite(config->side_prediction_reaction_sec) ||
    config->side_prediction_reaction_sec < 0.0f ||
    !isfinite(config->side_prediction_deceleration) ||
    config->side_prediction_deceleration <= 0.0f ||
    !isfinite(config->side_closing_rate_filter_alpha) ||
    config->side_closing_rate_filter_alpha < 0.0f ||
    config->side_closing_rate_filter_alpha > 1.0f ||
    !isfinite(config->side_closing_rate_max) ||
    config->side_closing_rate_max <= 0.0f ||
    !isfinite(config->side_cap_release_distance) ||
    config->side_cap_release_distance < config->side_limit_distance ||
    !isfinite(config->side_cap_release_clear_sec) ||
    config->side_cap_release_clear_sec < 0.0f ||
    !isfinite(config->side_target_down_rate) ||
    config->side_target_down_rate <= 0.0f ||
    !isfinite(config->side_target_up_rate) || config->side_target_up_rate <= 0.0f ||
    !isfinite(config->bypass_near_zone_far_distance) ||
    config->bypass_near_zone_far_distance <= 0.0f ||
    /* A3(10.09)：侧净距门控有限性校验（0=禁用，允许）。 */
    !isfinite(config->bypass_near_zone_release_side_distance) ||
    config->bypass_near_zone_release_side_distance < 0.0f);
  if (invalid) {
    if (reason != NULL) *reason = "dynamic velocity scalar parameters are invalid";
    return false;
  }
  for (size_t index = 0; index < DWA_FUSION_GEAR_COUNT; ++index) {
    const float peak = config->gear_peak_speeds[index];
    if (!isfinite(peak) || peak <= 0.0f ||
      (index > 0 && peak <= config->gear_peak_speeds[index - 1]))
    {
      if (reason != NULL) {
        *reason = "gear peak speeds must be positive and strictly increasing";
      }
      return false;
    }
  }
  return true;
}

const char *dwa_dynamic_control_mode_name(dwa_dynamic_control_mode_t mode)
{
  switch (mode) {
    case DYNAMIC_CONTROL_HAND: return "HAND";
    case DYNAMIC_CONTROL_MOVING_PREBRAKE: return "MOVING_PREBRAKE";
    case DYNAMIC_CONTROL_MOVING_HOLD: return "MOVING_HOLD";
    case DYNAMIC_CONTROL_MOVING_DWA: return "MOVING_DWA";
    case DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA: return "STATIC_DYNAMIC_DWA";
    case DYNAMIC_CONTROL_SIDE_CAP: return "SIDE_CAP";
  }
  return "UNKNOWN";
}

/* ---------------- 绉佹湁杈呭姪锛坰tatic 鎴愬憳鍑芥暟鐩磋瘧锛?---------------- */

static float dvf_finite_nonnegative(float value)
{
  return isfinite(value) ? dvf_max(0.0f, value) : 0.0f;
}

static int dvf_clamp_gear(const dwa_dynamic_velocity_fusion_t *fusion, int gear)
{
  return dvf_max(1, dvf_min(gear, DWA_FUSION_GEAR_COUNT));
}

static float dvf_gear_peak_speed(
  const dwa_dynamic_velocity_fusion_t *fusion, int gear)
{
  const int safe_gear = dvf_max(
    1, dvf_min(gear, DWA_FUSION_GEAR_COUNT));
  return fusion->config.gear_peak_speeds[safe_gear - 1];
}

/* prebrake_distance(actual_speed) 鐩磋瘧銆?*/
static float dvf_prebrake_distance(
  const dwa_dynamic_velocity_fusion_t *fusion, float actual_speed)
{
  return fusion->config.moving_prebrake_base_distance +
    dvf_finite_nonnegative(actual_speed) *
    fusion->config.moving_prebrake_speed_ratio;
}

/* active_prebrake_distance(fallback) 鐩磋瘧銆?*/
static float dvf_active_prebrake_distance(
  const dwa_dynamic_velocity_fusion_t *fusion, float fallback)
{
  return fusion->latched_prebrake_distance > 0.0f ?
    fusion->latched_prebrake_distance : fallback;
}

/* 9.27 距离比例减速目标（替代 9.24 两档制的恒定 2.5km/h 避障档）：
 * 前方净距 >= far 时满速；far -> zero 线性下降；<= zero 目标 0。
 * 两档制下避障档 0.6944 与满速 0.70 几乎相等（差 0.6%），接近正前
 * 障碍时观察不到任何减速——9.27 实测 v 恒 66.5 冲到障碍跟前。 */
static float dvf_prebrake_profile_speed(
  const dwa_dynamic_velocity_fusion_t *fusion, float front_clearance)
{
  const float far = fusion->config.moving_prebrake_profile_far_distance;
  const float zero = fusion->config.moving_prebrake_profile_zero_distance;
  if (!isfinite(front_clearance)) return fusion->config.forward_speed_limit;
  if (front_clearance >= far) return fusion->config.forward_speed_limit;
  if (front_clearance <= zero) return 0.0f;
  return fusion->config.forward_speed_limit *
    (front_clearance - zero) / (far - zero);
}

/* 9.27 距离比例目标的限速率跟踪器：PREBRAKE/HOLD/MOVING_DWA 三模式
 * 共用 prebrake_target_velocity 一个状态，模式切换时目标连续不跳变。
 * 下行被 moving_prebrake_deceleration 限制（防指令突降），上行被
 * moving_prebrake_recovery_acceleration 限制（防净距噪声速度抖动）。
 * dt 由本函数统一维护（last_prebrake_update_sec 跨模式连续计时）。 */
static float dvf_advance_profile_target(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  /* 9.27 盲区锁存：目标用有效净距（含盲区保持），不再直接用原始读数——
   * 障碍进骑乘过滤盲区时原始读数跳 INF 会让目标瞬间回满速。 */
  const float profile =
    dvf_prebrake_profile_speed(fusion, fusion->effective_front_clearance);
  const float dt = fusion->last_prebrake_update_sec >= 0.0f ?
    dvf_max(0.0f, input->now_sec - fusion->last_prebrake_update_sec) : 0.0f;
  fusion->last_prebrake_update_sec = input->now_sec;
  float target = fusion->prebrake_target_velocity;
  if (!isfinite(target) || target < 0.0f) target = 0.0f;
  if (profile < target) {
    target = dvf_max(profile,
      target - fusion->config.moving_prebrake_deceleration * dt);
  } else {
    target = dvf_min(profile,
      target + fusion->config.moving_prebrake_recovery_acceleration * dt);
  }
  fusion->prebrake_target_velocity = target;
  return target;
}

/* PREBRAKE/HOLD/MOVING_DWA release threshold (9.24 fix):
 * original = latched prebrake distance (or DWA trigger distance if unlatched)
 * + release_margin. The latch equals base + ratio * entry speed, ~3.57m at
 * full 5th gear; indoor front clearance can never exceed it, which pinned
 * all three modes at the prebrake target speed forever. The threshold is
 * now capped at DWA trigger distance + moving_release_hysteresis so the
 * release condition is actually reachable once the obstacle is passed. */
static float dvf_release_distance(const dwa_dynamic_velocity_fusion_t *fusion)
{
  const float trigger = fusion->config.moving_dwa_trigger_distance;
  const float original = dvf_max(
    trigger,
    dvf_active_prebrake_distance(fusion, trigger) +
    fusion->config.release_margin);
  const float capped = trigger +
    dvf_max(0.0f, fusion->config.moving_release_hysteresis);
  return dvf_min(original, capped);
}

/* 9.27 盲区净距锁存维护（"一走一停"根因修复）。
 * 骑乘过滤区（膝盖区）对窄障碍是盲区：障碍进入该区后 front_clearance
 * 读数跳 INF，距离比例目标立刻回满速——9.27 实测 #1426-1438：障碍已
 * 逼近到 0.30m，输出却从 8cm/s 爬行直接跳回 66.5cm/s 全速顶障碍，
 * 直到近距硬停止救场、倒车、再来一轮 = 走走停停循环。
 * 有效净距 = min(当前净距, 锁存最小净距)；锁存只在两种情况下复位：
 *  1) 读数恢复到 >= 解除距离（障碍真正远离）；
 *  2) 读数 INF 且自最后一次有限读数起已行进 > moving_release_hysteresis
 *     ——真障碍盲区纵深 <0.6m，爬行穿越不会触发；骑乘者-only（膝盖被
 *     过滤→恒 INF）行进超过阈值即复位，不会锁速。
 * 每个更新周期（含 HAND 模式）都在 dwa_dynamic_velocity_fusion_update
 * 顶部调用，所有消费方（比例目标/释放闸门）读 effective 字段。 */
static void dvf_maintain_front_latch(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  const float fc = input->front_clearance;
  const float dt = fusion->last_front_latch_sec >= 0.0f ?
    dvf_max(0.0f, input->now_sec - fusion->last_front_latch_sec) : 0.0f;
  fusion->last_front_latch_sec = input->now_sec;
  if (!input->perception_fresh) {
    /* 感知过期：保持上一周期已算好的有效净距与锁存不变，不累积行进
     * 距离（锁存为 INF 时不能被 zero 钳制，否则空旷行进中感知一过期
     * 就停车）。 */
    return;
  }
  if (isfinite(fc)) {
    fusion->front_inf_gap_distance = 0.0f;
    if (fc >= dvf_release_distance(fusion)) {
      fusion->latched_min_front_clearance = fc;
    } else if (fc < fusion->latched_min_front_clearance) {
      fusion->latched_min_front_clearance = fc;
    }
    fusion->effective_front_clearance =
      dvf_min(fc, fusion->latched_min_front_clearance);
    return;
  }
  if (input->hall_speed_fresh) {
    fusion->front_inf_gap_distance +=
      dvf_finite_nonnegative(input->actual_forward_speed) * dt;
  }
  if (fusion->front_inf_gap_distance >
    dvf_max(0.0f, fusion->config.moving_release_hysteresis))
  {
    fusion->latched_min_front_clearance = INFINITY;
  }
  /* 障碍进盲区（读数 INF 但锁存仍有限）：按"已贴近"处理，目标归 0。
   * 窄障碍进骑乘过滤盲区后，障碍表/前方硬停止同样看不见它，按锁存值
   * 低速爬行会一直顶住障碍推进；停在盲区外缘等 DWA 原地转向让开
   * （v=0 时 w 不受限）或用户接管，更安全。 */
  fusion->effective_front_clearance =
    dvf_min(fusion->latched_min_front_clearance,
      fusion->config.moving_prebrake_profile_zero_distance);
}

/* speed_restore_gate_clear(input, front_release_distance) 鐩磋瘧銆?*/
static bool dvf_speed_restore_gate_clear(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float front_release_distance)
{
  if (!input->perception_fresh || !input->restore_pose_stable) return false;
  /* 鎭㈠楂橀€?鍘熷妗ｄ綅蹇呴』浣跨敤鏅€氳溅澶村噣璺濆拰鐗╃悊杞︿綋/妗ｄ綅鍓嶇紭鍑€璺濅腑
   * 鏇翠繚瀹堢殑涓€涓紱閬垮厤 front_clearance 宸茬粡鍙樺ぇ浣?gear_front_clearance
   * 浠嶈创杩戦棬妗嗘椂璇仮澶嶉珮閫熴€?*/
  /* 恢复高速的原始档位必须使用普通车头净距和物理车体/档位前缘净距中
   * 更保守的一个；避免 front_clearance 已经变大但 gear_front_clearance
   * 仍贴近门槛时恢复高速。9.27 再纳入盲区锁存的有效净距：障碍进骑乘
   * 过滤盲区时原始读数跳 INF，不能当成"前方已清空"提前释放。 */
  const float conservative_front_clearance =
    dvf_min(dvf_min(input->front_clearance, input->gear_front_clearance),
      fusion->effective_front_clearance);
  const bool front_clear = !isfinite(conservative_front_clearance) ||
    conservative_front_clearance > front_release_distance;
  const float side_limit =
    dvf_max(0.0f, fusion->config.speed_restore_side_clearance);
  const bool sides_clear =
    (!isfinite(input->left_side_clearance) ||
    input->left_side_clearance > side_limit) &&
    (!isfinite(input->right_side_clearance) ||
    input->right_side_clearance > side_limit);
  const float sweep_limit =
    dvf_max(0.0f, fusion->config.speed_restore_sweep_clearance);
  const bool sweep_clear = !isfinite(input->sweep_clearance) ||
    input->sweep_clearance > sweep_limit;
  return front_clear && sides_clear && sweep_clear;
}

/* moving_obstacle_cleared(input) 鐩磋瘧銆?*/
static bool dvf_moving_obstacle_cleared(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  return dvf_speed_restore_gate_clear(
    fusion, input, dvf_release_distance(fusion));
}

/* side_obstacle_blocks_speed_restore(input) 鐩磋瘧锛堝綋鍓嶆湭琚?update 璋冪敤锛屼繚鐣欑瓑浠凤級銆?*/
static bool dvf_side_obstacle_blocks_speed_restore(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  if (!input->perception_fresh) return true;
  const float release_distance =
    dvf_max(0.0f, fusion->config.side_cap_release_distance);
  const bool left_blocked = isfinite(input->left_side_clearance) &&
    input->left_side_clearance <= release_distance;
  const bool right_blocked = isfinite(input->right_side_clearance) &&
    input->right_side_clearance <= release_distance;
  return left_blocked || right_blocked;
}

/* moving_dwa_triggered(input) 鐩磋瘧銆?*/
static bool dvf_moving_dwa_triggered(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  const bool distance_triggered = input->perception_fresh &&
    isfinite(input->front_clearance) &&
    input->front_clearance < fusion->config.moving_dwa_trigger_distance;
  const bool speed_ready = !input->hall_speed_fresh ||
    dvf_finite_nonnegative(input->actual_forward_speed) <=
    fusion->config.moving_dwa_entry_speed_max;
  return distance_triggered && speed_ready;
}

/* compose_decision(moving_distance, target_velocity) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_compose_decision(
  const dwa_dynamic_velocity_fusion_t *fusion,
  float moving_distance,
  float target_velocity)
{
  dwa_dynamic_velocity_decision_t decision = dwa_dynamic_velocity_decision_default();
  decision.mode = fusion->base_mode;
  decision.dwa_takeover =
    fusion->base_mode == DYNAMIC_CONTROL_MOVING_DWA ||
    fusion->base_mode == DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA;
  decision.hall_speed_control_active = fusion->base_mode != DYNAMIC_CONTROL_HAND;
  decision.side_cap_active = fusion->side_cap_active;
  decision.raw_gear = fusion->observed_gear;
  decision.raw_gear_valid = fusion->raw_gear_valid;
  decision.effective_gear = fusion->effective_gear;
  /* 缁熶竴鏈夋晥妗ｄ綅浠茶锛欴WA涓€妗ｄ紭鍏堬紱闈濪WA杩戦殰纰嶄竴妗ｅ繀椤绘潵鑷?   * gear_closed_loop_active_/effective_gear_锛屼粠鑰岀粡杩囬€愮骇妗ｄ綅鍜岄€熷害杩囨浮锛?   * 涓嶈兘鍦ㄨ繖閲岀洿鎺ヨ鐩?effective_gear 缁曡繃閫熷害妯″潡銆?*/
  if (fusion->base_mode == DYNAMIC_CONTROL_MOVING_DWA &&
    fusion->config.moving_dwa_force_gear)
  {
    decision.effective_gear =
      dvf_clamp_gear(fusion, fusion->config.moving_dwa_gear_level);
  }
  decision.near_obstacle_gear_active = fusion->near_obstacle_gear_active;
  decision.gear_closed_loop_active = fusion->gear_closed_loop_active;
  decision.gear_restore_hold_count = fusion->gear_restore_hold_count;
  decision.gear_target = fusion->gear_target;
  decision.gear_transition_active = fusion->gear_transition_active;
  decision.gear_speed_transition_active = fusion->gear_speed_transition_active;
  decision.gear_speed_transition_target = fusion->gear_speed_transition_output_speed;
  decision.gear_speed_transition_progress = fusion->gear_speed_transition_progress;
  decision.gear = decision.effective_gear;
  /* 璇ュ€煎彧淇濈暀缁欒瘖鏂紱瀹為檯涓€妗ｇ嚎/瑙掗€熷害鐢?x81杈撳嚭甯х殑妗ｄ綅瀛楁浜ょ粰鎺у埗鍣ㄦ墽琛屻€?*/
  decision.gear_peak_speed = dvf_gear_peak_speed(fusion, decision.effective_gear);
  decision.moving_prebrake_distance = moving_distance;
  decision.gear_trigger_distance = fusion->current_gear_trigger_distance;
  decision.gear_transition_remaining_sec =
    fusion->current_gear_transition_remaining_sec;
  decision.gear_transition_compensation_distance =
    fusion->current_gear_transition_compensation_distance;
  decision.target_max_velocity = dvf_max(0.0f, target_velocity);
  decision.held_forward_speed = fusion->held_moving_velocity;
  return decision;
}

/* moving_target_decision(moving_distance) 直译（9.27：目标改距离比例限速率跟踪）。 */
static dwa_dynamic_velocity_decision_t dvf_moving_target_decision(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance)
{
  const float target_speed = dvf_advance_profile_target(fusion, input);
  return dvf_compose_decision(
    fusion, dvf_active_prebrake_distance(fusion, moving_distance), target_speed);
}

/* static_velocity(nearest_radar_distance) 鐩磋瘧銆?*/
static float dvf_static_velocity(
  const dwa_dynamic_velocity_fusion_t *fusion, float nearest_radar_distance)
{
  return dvf_min(
    fusion->config.forward_speed_limit,
    dvf_finite_nonnegative(nearest_radar_distance) *
    fusion->config.forward_speed_limit * fusion->config.static_vdwa_gain);
}

/* enter_moving_dwa(input, moving_distance) 直译（9.27：按实速播种跟踪器）。 */
static void dvf_enter_moving_dwa(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance)
{
  fusion->base_mode = DYNAMIC_CONTROL_MOVING_DWA;
  fusion->clear_since_sec = -1.0f;
  /* 9.27：从 HAND 直跳接管（障碍突然进锥）时按霍尔实速播种限速率
   * 跟踪器——目标从实速按减速度平滑降到距离比例值，不瞬间钳零。 */
  if (!(fusion->prebrake_target_velocity > 0.0f)) {
    fusion->prebrake_target_velocity =
      dvf_finite_nonnegative(input->actual_forward_speed);
  }
  fusion->held_moving_velocity = fusion->prebrake_target_velocity;
  if (fusion->latched_prebrake_distance <= 0.0f) {
    fusion->latched_prebrake_distance = dvf_max(
      fusion->config.moving_dwa_trigger_distance, moving_distance);
  }
}

/* reset_motion_state() 鐩磋瘧銆?*/
static void dvf_reset_motion_state(dwa_dynamic_velocity_fusion_t *fusion)
{
  fusion->base_mode = DYNAMIC_CONTROL_HAND;
  fusion->clear_since_sec = -1.0f;
  fusion->last_static_update_sec = -1.0f;
  fusion->held_static_velocity = 0.0f;
  fusion->prebrake_target_velocity = 0.0f;
  fusion->held_moving_velocity = 0.0f;
  fusion->latched_prebrake_distance = 0.0f;
  fusion->latched_prebrake_entry_speed = 0.0f;
  fusion->last_prebrake_update_sec = -1.0f;
  /* 9.27 盲区净距锁存随运动状态一并复位（非前进意图每轮都会走到这里，
   * 重推油门后按当前真实净距重新锁存，不保留上一轮的陈旧值）。 */
  fusion->effective_front_clearance = INFINITY;
  fusion->latched_min_front_clearance = INFINITY;
  fusion->front_inf_gap_distance = 0.0f;
  fusion->last_front_latch_sec = -1.0f;
}

/* maybe_enter_static_dwa(input) 鐩磋瘧銆?*/
static void dvf_maybe_enter_static_dwa(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  if (!input->forward_start_rising || !input->perception_fresh ||
    !isfinite(input->front_clearance) ||
    input->front_clearance >= fusion->config.static_start_distance)
  {
    return;
  }

  fusion->base_mode = DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA;
  fusion->clear_since_sec = -1.0f;
  fusion->last_static_update_sec = input->now_sec;
  fusion->held_static_velocity =
    dvf_static_velocity(fusion, input->global_nearest_radar_distance);
}

/* update_static_dwa(input, moving_distance) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_update_static_dwa(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance)
{
  const float release_distance =
    fusion->config.static_start_distance + fusion->config.release_margin;
  const bool release_clear =
    dvf_speed_restore_gate_clear(fusion, input, release_distance);
  if (release_clear) {
    if (fusion->clear_since_sec < 0.0f) fusion->clear_since_sec = input->now_sec;
    if (input->now_sec - fusion->clear_since_sec + 1e-9f >=
      dvf_max(fusion->config.release_clear_sec,
        fusion->config.speed_restore_stable_sec))
    {
      dvf_reset_motion_state(fusion);
      return dvf_compose_decision(
        fusion, moving_distance, fusion->config.forward_speed_limit);
    }
  } else {
    fusion->clear_since_sec = -1.0f;
    const bool update_due = fusion->last_static_update_sec < 0.0f ||
      input->now_sec - fusion->last_static_update_sec + 1e-9f >=
      fusion->config.static_recalculate_sec;
    if (input->perception_fresh &&
      isfinite(input->global_nearest_radar_distance) && update_due)
    {
      fusion->held_static_velocity =
        dvf_static_velocity(fusion, input->global_nearest_radar_distance);
      fusion->last_static_update_sec = input->now_sec;
    }
  }
  return dvf_compose_decision(
    fusion, moving_distance, fusion->held_static_velocity);
}

/* update_moving_dwa(input, moving_distance) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_update_moving_dwa(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance)
{
  const float release_distance = dvf_release_distance(fusion);
  const bool release_clear =
    dvf_speed_restore_gate_clear(fusion, input, release_distance);
  if (release_clear) {
    if (fusion->clear_since_sec < 0.0f) fusion->clear_since_sec = input->now_sec;
    if (input->now_sec - fusion->clear_since_sec + 1e-9f >=
      dvf_max(fusion->config.release_clear_sec,
        fusion->config.speed_restore_stable_sec))
    {
      dvf_reset_motion_state(fusion);
      return dvf_compose_decision(
        fusion, moving_distance, fusion->config.forward_speed_limit);
    }
  } else {
    fusion->clear_since_sec = -1.0f;
  }
  return dvf_moving_target_decision(fusion, input, moving_distance);
}

/* update_prebrake_target 已被 9.27 距离比例跟踪器 dvf_advance_profile_target
 * 取代（两档制恒定目标随 9.24 修复一并退役，函数删除）。 */

/* update_moving_prebrake(input, actual_speed, moving_distance, dwa_triggered) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_update_moving_prebrake(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float actual_speed,
  float moving_distance,
  bool dwa_triggered)
{
  if (dwa_triggered) {
    dvf_enter_moving_dwa(fusion, input, moving_distance);
    return dvf_moving_target_decision(fusion, input, moving_distance);
  }
  if (dvf_moving_obstacle_cleared(fusion, input)) {
    dvf_reset_motion_state(fusion);
    return dvf_compose_decision(
      fusion, moving_distance, fusion->config.forward_speed_limit);
  }

  const float target = dvf_advance_profile_target(fusion, input);

  /* 9.27：实速降到（正在跟踪的）距离比例目标附近即转 HOLD；
   * 两模式输出同源同值，切换只影响日志模式名与诊断。 */
  const float hold_threshold = target + 0.02f;
  if (input->hall_speed_fresh && actual_speed <= hold_threshold) {
    fusion->base_mode = DYNAMIC_CONTROL_MOVING_HOLD;
    fusion->held_moving_velocity = target;
    return dvf_compose_decision(
      fusion, dvf_active_prebrake_distance(fusion, moving_distance),
      fusion->held_moving_velocity);
  }
  return dvf_compose_decision(
    fusion, dvf_active_prebrake_distance(fusion, moving_distance),
    target);
}

/* update_moving_hold(input, moving_distance, dwa_triggered) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_update_moving_hold(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance,
  bool dwa_triggered)
{
  if (dwa_triggered) {
    dvf_enter_moving_dwa(fusion, input, moving_distance);
    return dvf_moving_target_decision(fusion, input, moving_distance);
  }
  if (dvf_moving_obstacle_cleared(fusion, input)) {
    dvf_reset_motion_state(fusion);
    return dvf_compose_decision(
      fusion, moving_distance, fusion->config.forward_speed_limit);
  }
  /* 9.27：HOLD 跟随距离比例目标（限速率跟踪），不再钉在恒定 2.5km/h。 */
  const float target = dvf_advance_profile_target(fusion, input);
  fusion->held_moving_velocity = target;
  return dvf_compose_decision(
    fusion, dvf_active_prebrake_distance(fusion, moving_distance),
    fusion->held_moving_velocity);
}

/* update_hand(input, actual_speed, moving_distance, dwa_triggered) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_update_hand(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float actual_speed,
  float moving_distance,
  bool dwa_triggered)
{
  if (dwa_triggered) {
    dvf_enter_moving_dwa(fusion, input, moving_distance);
    return dvf_moving_target_decision(fusion, input, moving_distance);
  }

  /* 9.27：入口门槛从 actual > moving_prebrake_target_speed 改为
   * actual > 0.02（任意前进运动）。旧门槛下实速 0.665 永远不超过
   * 0.6944（满速 0.70 的电机损耗实速），预减速模式从未进入——
   * 9.27 实测 v 恒 66.5 冲向障碍的直接原因。目标现按实速播种，
   * 无需旧的地板速度门槛。 */
  const bool should_prebrake = input->hall_speed_fresh &&
    actual_speed > 0.02f &&
    input->perception_fresh && isfinite(input->front_clearance) &&
    input->front_clearance <= moving_distance;
  if (!should_prebrake) {
    return dvf_compose_decision(
      fusion, moving_distance, fusion->config.forward_speed_limit);
  }

  fusion->base_mode = DYNAMIC_CONTROL_MOVING_PREBRAKE;
  fusion->clear_since_sec = -1.0f;
  fusion->latched_prebrake_distance = moving_distance;
  fusion->latched_prebrake_entry_speed = actual_speed;
  fusion->prebrake_target_velocity = actual_speed;
  fusion->last_prebrake_update_sec = input->now_sec;
  return dvf_compose_decision(
    fusion, fusion->latched_prebrake_distance,
    fusion->prebrake_target_velocity);
}

/* update_side_prediction(clearance, now_sec, prediction_sec, &trend) 鐩磋瘧銆?*/
static dwa_fusion_side_prediction_t dvf_update_side_prediction(
  dwa_dynamic_velocity_fusion_t *fusion,
  float clearance,
  float now_sec,
  float prediction_sec,
  dwa_fusion_side_clearance_trend_t *trend)
{
  dwa_fusion_side_prediction_t prediction =
    dwa_fusion_side_prediction_default();
  prediction.clearance = clearance;
  prediction.predicted_clearance = clearance;
  if (!isfinite(clearance)) {
    *trend = dwa_fusion_side_clearance_trend_default();
    return prediction;
  }

  if (!trend->initialized) {
    trend->last_clearance = clearance;
    trend->last_update_sec = now_sec;
    trend->initialized = true;
    return prediction;
  }

  const float dt = now_sec - trend->last_update_sec;
  if (dt <= 1e-3f || dt > 0.50f) {
    trend->last_clearance = clearance;
    trend->last_update_sec = now_sec;
    trend->closing_rate = 0.0f;
    return prediction;
  }

  const float raw_rate = dvf_max(
    -fusion->config.side_closing_rate_max,
    dvf_min(
      fusion->config.side_closing_rate_max,
      (trend->last_clearance - clearance) / dt));
  const float alpha = fusion->config.side_closing_rate_filter_alpha;
  trend->closing_rate = alpha * raw_rate + (1.0f - alpha) * trend->closing_rate;
  trend->last_clearance = clearance;
  trend->last_update_sec = now_sec;

  prediction.closing_rate = dvf_max(0.0f, trend->closing_rate);
  prediction.predicted_clearance =
    clearance - prediction.closing_rate * prediction_sec;
  return prediction;
}

/* evaluate_side_protection(input) 鐩磋瘧銆?*/
static dwa_fusion_side_protection_state_t dvf_evaluate_side_protection(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  dwa_fusion_side_protection_state_t state =
    dwa_fusion_side_protection_state_default();
  state.fresh = input->perception_fresh;
  if (!input->perception_fresh) return state;

  const float actual_speed = input->hall_speed_fresh ?
    dvf_finite_nonnegative(input->actual_forward_speed) : 0.0f;
  const float prediction_sec = input->hall_speed_fresh &&
    actual_speed > fusion->config.side_speed_target ?
    dvf_min(
      1.0f,
      fusion->config.side_prediction_reaction_sec +
      (actual_speed - fusion->config.side_speed_target) /
      fusion->config.side_prediction_deceleration) : 0.0f;
  const dwa_fusion_side_prediction_t left = dvf_update_side_prediction(
    fusion, input->left_side_clearance, input->now_sec, prediction_sec,
    &fusion->left_side_trend);
  const dwa_fusion_side_prediction_t right = dvf_update_side_prediction(
    fusion, input->right_side_clearance, input->now_sec, prediction_sec,
    &fusion->right_side_trend);

  state.clearance = dvf_min(left.clearance, right.clearance);
  if (left.predicted_clearance <= right.predicted_clearance) {
    state.predicted_clearance = left.predicted_clearance;
    state.closing_rate = left.closing_rate;
  } else {
    state.predicted_clearance = right.predicted_clearance;
    state.closing_rate = right.closing_rate;
  }

  const bool high_speed = input->hall_speed_fresh &&
    actual_speed > fusion->config.side_speed_trigger;
  state.cap_requested = high_speed &&
    ((isfinite(state.clearance) &&
    state.clearance <= fusion->config.side_limit_distance) ||
    (isfinite(state.predicted_clearance) &&
    state.predicted_clearance <= fusion->config.side_limit_distance));
  if (state.cap_requested) {
    state.speed_limit = fusion->config.side_speed_target;
  }
  return state;
}

/* update_side_cap_latch(input, side, cap_requested) 鐩磋瘧銆?*/
static void dvf_update_side_cap_latch(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  const dwa_fusion_side_protection_state_t *side,
  bool cap_requested)
{
  if (cap_requested) {
    fusion->side_cap_active = true;
    fusion->side_cap_clear_since_sec = -1.0f;
    return;
  }
  if (!fusion->side_cap_active) return;

  const bool release_clear = side->fresh &&
    (!isfinite(side->clearance) ||
    side->clearance > fusion->config.side_cap_release_distance);
  if (!release_clear) {
    fusion->side_cap_clear_since_sec = -1.0f;
    return;
  }
  if (fusion->side_cap_clear_since_sec < 0.0f) {
    fusion->side_cap_clear_since_sec = input->now_sec;
    return;
  }
  if (input->now_sec - fusion->side_cap_clear_since_sec + 1e-9f <
    fusion->config.side_cap_release_clear_sec)
  {
    return;
  }

  fusion->side_cap_active = false;
  fusion->side_cap_clear_since_sec = -1.0f;
  fusion->side_target_velocity = 0.0f;
  fusion->side_target_initialized = false;
}

/* update_side_target(input, desired_limit) 鐩磋瘧銆?*/
static float dvf_update_side_target(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float desired_limit)
{
  const float dt = fusion->last_side_target_update_sec < 0.0f ? 0.0f :
    dvf_max(0.0f, dvf_min(0.25f,
      input->now_sec - fusion->last_side_target_update_sec));
  fusion->last_side_target_update_sec = input->now_sec;
  desired_limit = dvf_max(0.0f, desired_limit);

  if (!fusion->side_target_initialized) {
    const float actual_speed = input->hall_speed_fresh ?
      dvf_finite_nonnegative(input->actual_forward_speed) : desired_limit;
    fusion->side_target_velocity = dvf_max(
      desired_limit,
      dvf_min(actual_speed,
        fusion->config.gear_peak_speeds[DWA_FUSION_GEAR_COUNT - 1]));
    fusion->side_target_initialized = true;
  }

  const float delta = desired_limit - fusion->side_target_velocity;
  const float rate = delta < 0.0f ?
    fusion->config.side_target_down_rate :
    fusion->config.side_target_up_rate;
  const float max_step = rate * dt;
  if (fabsf(delta) <= max_step) {
    fusion->side_target_velocity = desired_limit;
  } else if (max_step > 0.0f) {
    fusion->side_target_velocity += delta < 0.0f ? -max_step : max_step;
  }
  return fusion->side_target_velocity;
}

/* apply_side_protection(decision, side, input) 鐩磋瘧銆?*/
static dwa_dynamic_velocity_decision_t dvf_apply_side_protection(
  dwa_dynamic_velocity_fusion_t *fusion,
  dwa_dynamic_velocity_decision_t decision,
  const dwa_fusion_side_protection_state_t *side,
  const dwa_dynamic_velocity_input_t *input)
{
  decision.side_clearance = side->clearance;
  decision.side_predicted_clearance = side->predicted_clearance;
  decision.side_closing_rate = side->closing_rate;

  dvf_update_side_cap_latch(fusion, input, side, side->cap_requested);
  decision.side_cap_active = fusion->side_cap_active;
  if (!fusion->side_cap_active) {
    decision.side_speed_limit = INFINITY;
    fusion->last_side_target_update_sec = input->now_sec;
    return decision;
  }

  decision.side_speed_limit = fusion->config.side_speed_target;
  const float effective_limit = dvf_update_side_target(
    fusion, input, fusion->config.side_speed_target);

  if (decision.mode == DYNAMIC_CONTROL_HAND) {
    decision.target_max_velocity = effective_limit;
  } else {
    decision.target_max_velocity = dvf_min(
      decision.target_max_velocity, effective_limit);
  }
  decision.hall_speed_control_active = true;
  decision.side_cap_active = true;
  if (decision.mode == DYNAMIC_CONTROL_HAND) {
    decision.mode = DYNAMIC_CONTROL_SIDE_CAP;
  }
  return decision;
}

/* gear_transition_steps_to_force_gear(input) 鐩磋瘧銆?*/
static int dvf_gear_transition_steps_to_force_gear(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  if (!fusion->config.gear_closed_loop_enabled) return 0;
  int current_gear = dvf_clamp_gear(fusion, fusion->effective_gear);
  /* 鍦ㄧǔ瀹氱殑闈為棴鐜姸鎬佷笅锛屽厛鐢ㄦ湰鍛ㄦ湡鍒氳鍒扮殑鍘熷妗ｄ綅锛岄伩鍏?   * 鍚姩鍚庣涓€涓懆鏈熶粛鎷块粯璁や竴妗ｈ绠楄€屾紡鎺夐珮閫熸彁鍓嶉噺銆?*/
  if (!fusion->gear_closed_loop_active && !fusion->gear_transition_active &&
    input->raw_gear_valid && input->raw_gear >= 1 &&
    input->raw_gear <= DWA_FUSION_GEAR_COUNT)
  {
    current_gear = dvf_clamp_gear(fusion, input->raw_gear);
  }
  const int target_gear = dvf_clamp_gear(fusion, fusion->config.gear_force_level);
  return dvf_max(0, current_gear - target_gear);
}

/* gear_transition_remaining_time_sec(input) 鐩磋瘧銆?*/
static float dvf_gear_transition_remaining_time_sec(
  const dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  return (float)dvf_gear_transition_steps_to_force_gear(fusion, input) *
    dvf_max(0.0f, fusion->config.gear_transition_step_sec);
}

/* advance_effective_gear(now_sec, target_gear) 鐩磋瘧銆?*/
static void dvf_advance_effective_gear(
  dwa_dynamic_velocity_fusion_t *fusion, float now_sec, int target_gear)
{
  target_gear = dvf_clamp_gear(fusion, target_gear);
  fusion->effective_gear = dvf_clamp_gear(fusion, fusion->effective_gear);
  if (fusion->effective_gear == target_gear) {
    fusion->gear_transition_active = false;
    fusion->last_gear_transition_sec = -1.0f;
    return;
  }

  const bool clock_reset = isfinite(now_sec) &&
    isfinite(fusion->last_gear_transition_sec) &&
    now_sec < fusion->last_gear_transition_sec;
  const bool interval_elapsed =
    fusion->config.gear_transition_step_sec <= 0.0f ||
    fusion->last_gear_transition_sec < 0.0f || clock_reset ||
    !isfinite(now_sec) ||
    now_sec - fusion->last_gear_transition_sec + 1e-9f >=
    fusion->config.gear_transition_step_sec;
  if (interval_elapsed) {
    fusion->effective_gear +=
      fusion->effective_gear < target_gear ? 1 : -1;
    fusion->effective_gear = dvf_clamp_gear(fusion, fusion->effective_gear);
    fusion->last_gear_transition_sec = isfinite(now_sec) ? now_sec : -1.0f;
  }
  fusion->gear_transition_active = fusion->effective_gear != target_gear;
}

/* update_gear_state(input, moving_distance, gear_trigger_distance) 鐩磋瘧銆?*/
static void dvf_update_gear_state(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input,
  float moving_distance,
  float gear_trigger_distance)
{
  const int max_gear = DWA_FUSION_GEAR_COUNT;

  if (input->raw_gear_valid &&
    input->raw_gear >= 1 &&
    input->raw_gear <= max_gear)
  {
    fusion->observed_gear = input->raw_gear;
    fusion->raw_gear_valid = true;
  } else {
    fusion->observed_gear = fusion->config.fallback_gear;
    fusion->raw_gear_valid = false;
  }

  /* gear_force_forward_only 鍙檺鍒舵櫘閫氬墠鏂硅窛绂婚棴鐜紱杩戦殰纰嶄繚鎶や竴鏃﹁Е鍙戯紝
   * 鍗充娇褰撳墠鍛ㄦ湡鎵嬫焺鎰忓浘宸叉帴杩戜腑绔嬶紝涔熷繀椤昏繘鍏ュ悓涓€濂楀钩婊戞。浣?閫熷害杩囨浮閾捐矾銆?*/
  const bool was_closed_loop_active = fusion->gear_closed_loop_active;
  /* 杩戦殰纰嶉檺妗ｆ槸闈濪WA璺緞鍞竴鐨勫閮ㄩ檺妗ｈЕ鍙戞簮锛?   * 涓嶅啀鐢ㄥ彟涓€濂?gear_front_clearance 璺濈闂幆閲嶅瑙﹀彂銆?*/
  const bool near_obstacle_force_requested =
    fusion->config.near_obstacle_gear_enabled && fusion->near_obstacle_gear_active;
  const bool can_enter_force_gear =
    fusion->config.gear_closed_loop_enabled &&
    (near_obstacle_force_requested ||
    !fusion->config.gear_force_forward_only || input->forward_intent);

  const float restore_distance =
    moving_distance + fusion->config.gear_restore_hysteresis;

  /* 闈濪WA杩戦殰纰嶉檺妗ｄ笌妗ｄ綅闂幆缁熶竴涓轰竴涓姸鎬侊細
   * - 杩戦殰纰嶈繘鍏ユ椂閿佸畾涓€妗ｏ紱
   * - 鍙湁杩戦殰纰嶆潯浠惰В闄ゃ€佸墠鏂?涓や晶/鎵帬鍖?濮挎€佽繛缁畨鍏ㄥ悗鎵嶆仮澶嶅師濮嬫。浣嶏紱
   * - DWA鑷韩鐨勫己鍒朵竴妗ｇ敱 compose_decision 鍗曠嫭浠茶锛屼笉閲嶅瑙﹀彂杩欓噷鐨勯棴鐜€?*/
  const bool restore_clear =
    dvf_speed_restore_gate_clear(fusion, input, restore_distance);

  if (fusion->gear_closed_loop_active) {
    if (!near_obstacle_force_requested && restore_clear) {
      ++fusion->gear_restore_hold_count;
      const int required_cycles =
        dvf_max(1, fusion->config.gear_restore_hold_cycles);
      if (fusion->gear_restore_hold_count >= required_cycles) {
        fusion->gear_closed_loop_active = false;
        fusion->gear_restore_hold_count = 0;
      }
    } else {
      fusion->gear_restore_hold_count = 0;
    }
  } else if (can_enter_force_gear && near_obstacle_force_requested) {
    fusion->gear_closed_loop_active = true;
    fusion->gear_restore_hold_count = 0;
  } else {
    fusion->gear_restore_hold_count = 0;
  }

  const int target_gear = dvf_clamp_gear(fusion, fusion->gear_closed_loop_active ?
    fusion->config.near_obstacle_gear_level : fusion->observed_gear);
  fusion->gear_target = target_gear;

  /* 绌烘椃鍖轰笖娌℃湁姝ｅ湪閫€鍑洪棴鐜椂锛岀户缁繚鎸佸師鏈夎涓猴細鍘熷妗ｄ綅绔嬪嵆璺熼殢銆?   * 鍙湁鏈疆鏇剧粡瑙﹀彂杩囬棴鐜€佸綋鍓嶄粛鍦ㄩ棴鐜垨浠嶆湁鏈畬鎴愮殑杩囨浮鏃舵墠閫愮骇鍙樺寲銆?*/
  const bool transition_required = fusion->gear_closed_loop_active ||
    was_closed_loop_active || fusion->gear_transition_active;
  if (!transition_required) {
    fusion->effective_gear = fusion->observed_gear;
    fusion->gear_transition_active = false;
    fusion->last_gear_transition_sec = -1.0f;
    return;
  }
  dvf_advance_effective_gear(fusion, input->now_sec, target_gear);
}

/* update_gear_speed_transition(input) 鐩磋瘧銆?*/
static void dvf_update_gear_speed_transition(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  if (!fusion->config.gear_speed_transition_enabled) {
    fusion->gear_speed_transition_active = false;
    fusion->gear_speed_transition_progress = 1.0f;
    fusion->gear_speed_transition_output_speed = 0.0f;
    return;
  }
  const float target_speed = dvf_gear_peak_speed(fusion, fusion->gear_target);
  const bool target_changed =
    !isfinite(fusion->gear_speed_transition_target_speed) ||
    fabsf(target_speed - fusion->gear_speed_transition_target_speed) > 1e-6f;
  const bool should_transition = fusion->gear_closed_loop_active ||
    fusion->gear_transition_active ||
    fusion->gear_speed_transition_active;
  if (!should_transition) {
    fusion->gear_speed_transition_active = false;
    fusion->gear_speed_transition_progress = 1.0f;
    fusion->gear_speed_transition_output_speed = 0.0f;
    return;
  }
  const float now = input->now_sec;
  if (!fusion->gear_speed_transition_active || target_changed) {
    float seed = dvf_finite_nonnegative(input->actual_forward_speed);
    if (fusion->gear_speed_transition_active &&
      isfinite(fusion->gear_speed_transition_output_speed))
    {
      seed = fusion->gear_speed_transition_output_speed;
    }
    fusion->gear_speed_transition_start_speed = seed;
    fusion->gear_speed_transition_target_speed = target_speed;
    fusion->gear_speed_transition_start_sec = isfinite(now) ? now : -1.0f;
    fusion->gear_speed_transition_progress = 0.0f;
    fusion->gear_speed_transition_active =
      fabsf(seed - target_speed) > 1e-3f;
  }
  if (!fusion->gear_speed_transition_active) {
    fusion->gear_speed_transition_output_speed = target_speed;
    fusion->gear_speed_transition_progress = 1.0f;
    return;
  }
  float elapsed = 0.0f;
  if (isfinite(now) && isfinite(fusion->gear_speed_transition_start_sec)) {
    elapsed = dvf_max(0.0f, now - fusion->gear_speed_transition_start_sec);
  }
  fusion->gear_speed_transition_progress = dvf_max(0.0f, dvf_min(1.0f,
    elapsed / dvf_max(1e-3f,
      fusion->config.gear_speed_transition_duration_sec)));
  fusion->gear_speed_transition_output_speed =
    fusion->gear_speed_transition_start_speed +
    (fusion->gear_speed_transition_target_speed -
    fusion->gear_speed_transition_start_speed) *
    fusion->gear_speed_transition_progress;
  if (fusion->gear_speed_transition_progress >= 1.0f - 1e-6f) {
    fusion->gear_speed_transition_output_speed =
      fusion->gear_speed_transition_target_speed;
    fusion->gear_speed_transition_active = false;
  }
}

/* ---------------- 瀵瑰鎺ュ彛 ---------------- */

void dwa_dynamic_velocity_fusion_init(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_config_t *config)
{
  fusion->config = *config;
  fusion->near_obstacle_gear_active = false;
  fusion->base_mode = DYNAMIC_CONTROL_HAND;
  fusion->clear_since_sec = -1.0f;
  fusion->last_static_update_sec = -1.0f;
  fusion->held_static_velocity = 0.0f;
  fusion->prebrake_target_velocity = 0.0f;
  fusion->held_moving_velocity = 0.0f;
  fusion->latched_prebrake_distance = 0.0f;
  fusion->latched_prebrake_entry_speed = 0.0f;
  fusion->last_prebrake_update_sec = -1.0f;
  /* 9.27 盲区净距锁存：初始视为"前方无障"（INFINITY），由每周期的
   * dvf_maintain_front_latch 按实际读数接管。 */
  fusion->effective_front_clearance = INFINITY;
  fusion->latched_min_front_clearance = INFINITY;
  fusion->front_inf_gap_distance = 0.0f;
  fusion->last_front_latch_sec = -1.0f;
  fusion->side_cap_active = false;
  fusion->side_cap_clear_since_sec = -1.0f;
  fusion->side_target_velocity = 0.0f;
  fusion->last_side_target_update_sec = -1.0f;
  fusion->side_target_initialized = false;
  fusion->left_side_trend = dwa_fusion_side_clearance_trend_default();
  fusion->right_side_trend = dwa_fusion_side_clearance_trend_default();
  fusion->observed_gear = 1;
  fusion->raw_gear_valid = false;
  fusion->effective_gear = 1;
  fusion->gear_target = 1;
  fusion->gear_closed_loop_active = false;
  fusion->gear_transition_active = false;
  fusion->gear_speed_transition_active = false;
  fusion->gear_speed_transition_start_speed = 0.0f;
  fusion->gear_speed_transition_target_speed = 0.0f;
  fusion->gear_speed_transition_start_sec = -1.0f;
  fusion->gear_speed_transition_output_speed = 0.0f;
  fusion->gear_speed_transition_progress = 1.0f;
  fusion->gear_restore_hold_count = 0;
  fusion->last_gear_transition_sec = -1.0f;
  fusion->current_gear_trigger_distance = 0.0f;
  fusion->current_gear_transition_remaining_sec = 0.0f;
  fusion->current_gear_transition_compensation_distance = 0.0f;
}

void dwa_dynamic_velocity_fusion_init_default(
  dwa_dynamic_velocity_fusion_t *fusion)
{
  const dwa_dynamic_velocity_config_t config =
    dwa_dynamic_velocity_config_default();
  dwa_dynamic_velocity_fusion_init(fusion, &config);
}

bool dwa_dynamic_velocity_fusion_set_config(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_config_t *config,
  const char **reason)
{
  if (!dwa_validate_dynamic_velocity_config(config, reason)) return false;
  fusion->config = *config;
  return true;
}

void dwa_dynamic_velocity_fusion_reset(dwa_dynamic_velocity_fusion_t *fusion)
{
  dvf_reset_motion_state(fusion);
  fusion->side_cap_active = false;
  fusion->side_cap_clear_since_sec = -1.0f;
  fusion->side_target_velocity = 0.0f;
  fusion->last_side_target_update_sec = -1.0f;
  fusion->side_target_initialized = false;
  fusion->left_side_trend = dwa_fusion_side_clearance_trend_default();
  fusion->right_side_trend = dwa_fusion_side_clearance_trend_default();
  fusion->observed_gear = fusion->config.fallback_gear;
  fusion->raw_gear_valid = false;
  fusion->effective_gear = fusion->config.fallback_gear;
  fusion->gear_target = fusion->config.fallback_gear;
  fusion->gear_closed_loop_active = false;
  fusion->gear_transition_active = false;
  fusion->gear_speed_transition_active = false;
  fusion->gear_speed_transition_start_speed = 0.0f;
  fusion->gear_speed_transition_target_speed = 0.0f;
  fusion->gear_speed_transition_start_sec = -1.0f;
  fusion->gear_speed_transition_output_speed = 0.0f;
  fusion->gear_restore_hold_count = 0;
  fusion->last_gear_transition_sec = -1.0f;
  fusion->current_gear_trigger_distance = 0.0f;
  fusion->current_gear_transition_remaining_sec = 0.0f;
  fusion->current_gear_transition_compensation_distance = 0.0f;
}

void dwa_dynamic_velocity_fusion_complete_dwa_handoff(
  dwa_dynamic_velocity_fusion_t *fusion)
{
  dvf_reset_motion_state(fusion);
  fusion->side_cap_clear_since_sec = -1.0f;
}

dwa_dynamic_velocity_decision_t dwa_dynamic_velocity_fusion_update(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input)
{
  fusion->near_obstacle_gear_active = input->near_obstacle_gear_active;
  const float actual_speed = dvf_finite_nonnegative(input->actual_forward_speed);
  const float moving_distance = dvf_prebrake_distance(fusion, actual_speed);
  /* 妗ｄ綅闂幆鍙湪鏈眰澧炲姞鍔ㄦ€佽ˉ鍋匡細鐢ㄥ綋鍓嶆湁鏁堟。浣嶅埌寮哄埗妗ｄ綅鐨勫墿浣?   * 杩囨浮鏃堕棿涔樹互瀹炴椂闇嶅皵閫熷害锛屼笉鑳芥妸瀹冭鐩栧洖DWA鐨刴oving_distance銆?*/
  const float transition_remaining_sec =
    dvf_gear_transition_remaining_time_sec(fusion, input);
  const float transition_compensation_distance = actual_speed *
    transition_remaining_sec;
  const float gear_trigger_margin =
    dvf_gear_transition_steps_to_force_gear(fusion, input) > 0 ?
    fusion->config.gear_transition_safety_margin : 0.0f;
  const float gear_trigger_distance = moving_distance +
    transition_compensation_distance + gear_trigger_margin;
  fusion->current_gear_trigger_distance = gear_trigger_distance;
  fusion->current_gear_transition_remaining_sec = transition_remaining_sec;
  fusion->current_gear_transition_compensation_distance =
    transition_compensation_distance;

  /* 9.27 盲区净距锁存：所有模式（含 HAND）每周期维护一次，消费方统一
   * 读 effective_front_clearance（比例目标 + 释放闸门）。 */
  dvf_maintain_front_latch(fusion, input);

  if (!input->forward_intent) {
    /* 闈炲墠杩涘姩浣滃彧娓呴櫎鏈疆DWA杩愬姩鐘舵€侊紝涓嶆竻闄ゅ凡缁忛攣瀛樼殑寮哄埗涓€妗ｃ€?     * 杩涘叆S鍚庯紝鍊掕溅銆佸師鍦拌浆鍚戝拰涓珛閮藉繀椤荤户缁娇鐢ㄦ湁鏁堟。浣?锛?     * 鐩村埌鍓嶆柟闅滅鐗╂弧瓒砈+H鎭㈠鏉′欢銆?*/
    dvf_reset_motion_state(fusion);
    dvf_update_gear_state(fusion, input, moving_distance, gear_trigger_distance);
    dvf_update_gear_speed_transition(fusion, input);
    return dvf_compose_decision(
      fusion, moving_distance, fusion->config.forward_speed_limit);
  }

  dvf_update_gear_state(fusion, input, moving_distance, gear_trigger_distance);
  dvf_update_gear_speed_transition(fusion, input);
  const dwa_fusion_side_protection_state_t side =
    dvf_evaluate_side_protection(fusion, input);

  dvf_maybe_enter_static_dwa(fusion, input);
  if (fusion->base_mode == DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA) {
    return dvf_apply_side_protection(fusion,
      dvf_update_static_dwa(fusion, input, moving_distance), &side, input);
  }

  const bool dwa_triggered = dvf_moving_dwa_triggered(fusion, input);
  dwa_dynamic_velocity_decision_t decision;
  switch (fusion->base_mode) {
    case DYNAMIC_CONTROL_MOVING_DWA:
      decision = dvf_update_moving_dwa(fusion, input, moving_distance);
      break;
    case DYNAMIC_CONTROL_MOVING_PREBRAKE:
      decision = dvf_update_moving_prebrake(
        fusion, input, actual_speed, moving_distance, dwa_triggered);
      break;
    case DYNAMIC_CONTROL_MOVING_HOLD:
      decision = dvf_update_moving_hold(
        fusion, input, moving_distance, dwa_triggered);
      break;
    case DYNAMIC_CONTROL_HAND:
    case DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA:
    case DYNAMIC_CONTROL_SIDE_CAP:
      decision = dvf_update_hand(
        fusion, input, actual_speed, moving_distance, dwa_triggered);
      break;
    default:
      /* C++ 鏃?default 鍒嗘敮锛堟灇涓剧┓灏斤級锛涘埌姝や笉浼氬彂鐢熴€?*/
      decision = dvf_update_hand(
        fusion, input, actual_speed, moving_distance, dwa_triggered);
      break;
  }

  return dvf_apply_side_protection(fusion, decision, &side, input);
}
