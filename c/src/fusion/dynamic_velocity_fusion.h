/*
 * 模块功能：动态速度状态机：融合霍尔速度、前方距离、侧边保护和DWA接管速度。
 * 直译自 native/dynamic_velocity_fusion.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * gear_peak_speeds 为固定 5 元素数组（原 vector 长度恒 5，校验 size()==5）。
 */
#ifndef DWA_DYNAMIC_VELOCITY_FUSION_H
#define DWA_DYNAMIC_VELOCITY_FUSION_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DWA_FUSION_GEAR_COUNT 5

/* enum class DynamicControlMode */
typedef enum {
  DYNAMIC_CONTROL_HAND = 0,              /* 手柄直接控制，动态限速状态机不接管。 */
  DYNAMIC_CONTROL_MOVING_PREBRAKE = 1,   /* 运动中进入距离S后，按距离曲线降低目标速度。 */
  DYNAMIC_CONTROL_MOVING_HOLD = 2,       /* 减速完成后保持低速，等待二级距离判断。 */
  DYNAMIC_CONTROL_MOVING_DWA = 3,        /* 运动中前方进入DWA触发距离，由DWA选择轨迹。 */
  DYNAMIC_CONTROL_STATIC_DYNAMIC_DWA = 4, /* 静止近障起步，按最近雷达距离动态限制DWA速度。 */
  DYNAMIC_CONTROL_SIDE_CAP = 5           /* HAND阶段根据侧边净距限制线速度。 */
} dwa_dynamic_control_mode_t;

typedef struct {
  /* 这里速度都用m/s。 */
  float forward_speed_limit;
  /* 一级减速距离S就是：基础距离 + 霍尔实速 * 比例。 */
  float moving_prebrake_base_distance;
  float moving_prebrake_speed_ratio;
  /* 进了S以后先往这个低速降；距离数据不可用时就按下面的减速度降。 */
  float moving_prebrake_target_speed;
  float moving_prebrake_deceleration;
  /* 9.27 距离比例减速（替代两档制恒定避障档）：前方净距 >= far 时满速，
   * far -> zero 线性下降，<= zero 目标 0。两档制下避障档 0.6944 与满速
   * 0.70 几乎相等，接近正前障碍时观察不到减速（9.27 实测 v 恒 66.5）。
   * 净距回升时目标按 recovery_acceleration 平滑恢复，防净距噪声抖动。 */
  float moving_prebrake_profile_far_distance;
  float moving_prebrake_profile_zero_distance;
  float moving_prebrake_recovery_acceleration;
  /* 仅运动DWA接管阶段的线速度上限，不影响前置预减速目标。 */
  float moving_dwa_speed_limit;
  /* 仅MOVING_DWA阶段的角速度上限，不使用手柄档位决定。 */
  float moving_dwa_yawrate_limit;
  /* MOVING_DWA输出给电机桥时固定使用的档位；不影响预减速或空旷手动档位。 */
  bool moving_dwa_force_gear;
  int moving_dwa_gear_level;
  /* 未进入DWA但物理车体近障碍物时，独立把输出协议档位切到低档。 */
  bool near_obstacle_gear_enabled;
  float near_obstacle_gear_center_x;
  float near_obstacle_gear_center_y;
  float near_obstacle_gear_radius;
  float near_obstacle_gear_release_radius;
  int near_obstacle_gear_level;
  /* 前方进入二级距离且霍尔实速降到门槛内，才让普通运动DWA接管。 */
  float moving_dwa_trigger_distance;
  float moving_dwa_entry_speed_max;
  /* PREBRAKE/HOLD/MOVING_DWA 解除阈值的封顶滞回：解除距离不超过
   * moving_dwa_trigger_distance + 该值。高速进入时锁存的预减速距离
   * （基础距离+比例*实速，五档满速约3.57m）在室内净距无法超过，
   * 曾把三个模式一旦进入就永久钉在预减速目标速度。 */
  float moving_release_hysteresis;
  /* 静止近障启动主要看这两个参数。 */
  float static_start_distance;
  float static_vdwa_gain;
  /* 这两个值决定DWA什么时候把控制权还给手柄。 */
  float release_margin;
  float release_clear_sec;
  float speed_restore_side_clearance;
  float speed_restore_sweep_clearance;
  float speed_restore_stable_sec;
  float speed_restore_max_yawrate;
  /* 静止分支隔多久重算一次Vdwa。 */
  float static_recalculate_sec;
  /* 既有DWA内部速度上限/诊断表；不用于替代控制器原厂档位的线速度或角速度。 */
  float gear_peak_speeds[DWA_FUSION_GEAR_COUNT];
  int fallback_gear;
  /* 前方进入现有减速距离S后，是否临时强制使用低档位。 */
  bool gear_closed_loop_enabled;
  float gear_restore_hysteresis;
  int gear_force_level;
  bool gear_force_forward_only;
  float gear_front_trigger_extra_angle_deg;
  int gear_restore_hold_cycles;
  /* 档位闭环进入/退出时每隔多少秒最多变化一级；非闭环时不生效。 */
  float gear_transition_step_sec;
  /* 档位闭环专用触发安全余量；不改变DWA自己的预减速距离。 */
  float gear_transition_safety_margin;
  /* 档位闭环实际线速度只在进入/退出闭环时平滑过渡；不改变DWA目标和角速度。 */
  bool gear_speed_transition_enabled;
  float gear_speed_transition_duration_sec;
  /* 霍尔实速超过门槛后，侧距进入或预测将进入30cm时限到三档附近. */
  float side_speed_trigger;
  float side_speed_target;
  float side_limit_distance;
  float side_prediction_reaction_sec;
  float side_prediction_deceleration;
  float side_closing_rate_filter_alpha;
  float side_closing_rate_max;
  /* 两侧都回到30cm外并保持0.30s才退出，避免边界抖动。 */
  float side_cap_release_distance;
  float side_cap_release_clear_sec;
  /* 限速目标逐步下降；解除后由电机桥平滑恢复手柄速度。 */
  float side_target_down_rate;
  float side_target_up_rate;
  /* V4a(10.08)：F1d latch 释放防抖——前方净距须连续 N 个周期超过
   * release 阈值才解除近障锁存。治感知单周期闪烁导致的 yo-yo
   * 回冲（cap 瞬间回档位峰值→重新加速→再次断崖）。0=原单周期行为。 */
  int bypass_near_zone_release_hold_cycles;
  /* V4b(10.08)：F1d 断崖斜坡——进近障区瞬间 target 不瞬降
   * forward_speed_limit，而是从断崖时 hall 实速按
   * bypass_near_zone_ramp_deceleration 斜坡降到 forward_speed_limit。
   * 治 5 档进通道时 DWA 按 0.7 假想速度评分、判死视界乐观 4 倍
   * 导致的追赶式急转撞对侧。 */
  bool bypass_near_zone_ramp_enabled;
  float bypass_near_zone_ramp_deceleration;
  /* V4c(10.08)：F1d 近障锁存判定距离（独立参数）。原与
   * moving_prebrake_profile_far_distance(2.20) 共用，但该参数还喂
   * 距离比例减速 profile，动它会改通道内减速曲线。090443 日志定案：
   * 减速跑道不足是 4/5 档撞对侧真根因——0.4m/s² 物理减速下 5 档
   * 1.67→0.70 需 2.87m 跑道 > 2.2m，到拐角仍 1.1~1.3m/s，转弯半径
   * P90=1.36~1.79m（3 档 0.92m），通道侧净距仅 5~10cm 必扫对侧。
   * 板端调 3.80：latch 提前触发 + V4b 斜坡配合，5 档到拐角降到
   * 0.70~0.75 与 3 档趋同。代码默认 2.20=原行为。 */
  float bypass_near_zone_far_distance;
  /* A3(10.09)：1 档避障释放侧净距门控（m，0=禁用）。释放须
   * 前方净空且两侧任一侧净距>本值——1m 窄弯内视线穿过弯道
   * 开口的 fwd 真实瞬跳不再触发通道内加速（040707 实锤）。 */
  float bypass_near_zone_release_side_distance;
} dwa_dynamic_velocity_config_t;

/* DynamicVelocityConfig{} 成员默认值直译。 */
dwa_dynamic_velocity_config_t dwa_dynamic_velocity_config_default(void);

/* validate_dynamic_velocity_config(config, reason) 直译。 */
bool dwa_validate_dynamic_velocity_config(
  const dwa_dynamic_velocity_config_t *config,
  const char **reason);

/* dynamic_control_mode_name(mode) 直译。 */
const char *dwa_dynamic_control_mode_name(dwa_dynamic_control_mode_t mode);

typedef struct {
  /* 当前时间，以及手柄是不是刚开始前推。 */
  float now_sec;
  bool forward_intent;
  bool forward_start_rising;
  /* 雷达和霍尔数据太旧时，这两个标志会变成false。 */
  bool perception_fresh;
  bool hall_speed_fresh;
  /* 这是滤波后的霍尔实速。 */
  float actual_forward_speed;
  /* 控制器反馈帧中的原始档位；无效或过期时使用配置回退档位。 */
  int raw_gear;
  bool raw_gear_valid;
  /* 独立近障碍物限档模块的当前周期状态；不表示DWA接管。 */
  bool near_obstacle_gear_active;
  /* 一个是车头净距离，一个是融合点云里的最近距离。 */
  float front_clearance;
  float gear_front_clearance;
  float left_side_clearance;
  float right_side_clearance;
  float global_nearest_radar_distance;
  float sweep_clearance;
  bool restore_pose_stable;
} dwa_dynamic_velocity_input_t;

/* DynamicVelocityInput{} 默认值直译。 */
static inline dwa_dynamic_velocity_input_t dwa_dynamic_velocity_input_default(void)
{
  dwa_dynamic_velocity_input_t input;
  input.now_sec = 0.0f;
  input.forward_intent = false;
  input.forward_start_rising = false;
  input.perception_fresh = false;
  input.hall_speed_fresh = false;
  input.actual_forward_speed = 0.0f;
  input.raw_gear = 1;
  input.raw_gear_valid = false;
  input.near_obstacle_gear_active = false;
  input.front_clearance = INFINITY;
  input.gear_front_clearance = INFINITY;
  input.left_side_clearance = INFINITY;
  input.right_side_clearance = INFINITY;
  input.global_nearest_radar_distance = INFINITY;
  input.sweep_clearance = INFINITY;
  input.restore_pose_stable = false;
  return input;
}

typedef struct {
  /* 当前走到哪个状态，以及控制权是不是在DWA。 */
  dwa_dynamic_control_mode_t mode;
  bool dwa_takeover;
  bool hall_speed_control_active;
  bool side_cap_active;
  /* raw_gear 是控制器真实档位，effective_gear 是本周期要写入0x81输出帧的有效档位。 */
  int raw_gear;
  int effective_gear;
  bool raw_gear_valid;
  bool near_obstacle_gear_active;
  bool gear_closed_loop_active;
  int gear_restore_hold_count;
  int gear_target;
  bool gear_transition_active;
  bool gear_speed_transition_active;
  float gear_speed_transition_target;
  float gear_speed_transition_progress;
  /* 保留 gear 字段兼容现有日志/调用方；其值等于 effective_gear. */
  int gear;
  float gear_peak_speed;
  float side_clearance;
  float side_predicted_clearance;
  float side_closing_rate;
  float side_speed_limit;
  /* 本轮DWA预减速使用的S，以及档位闭环独立使用的动态触发距离。 */
  float moving_prebrake_distance;
  float gear_trigger_distance;
  float gear_transition_remaining_sec;
  float gear_transition_compensation_distance;
  float target_max_velocity;
  /* 这个值主要留给日志和诊断看。 */
  float held_forward_speed;
} dwa_dynamic_velocity_decision_t;

/* DynamicVelocityDecision{} 默认值直译。 */
static inline dwa_dynamic_velocity_decision_t
dwa_dynamic_velocity_decision_default(void)
{
  dwa_dynamic_velocity_decision_t decision;
  decision.mode = DYNAMIC_CONTROL_HAND;
  decision.dwa_takeover = false;
  decision.hall_speed_control_active = false;
  decision.side_cap_active = false;
  decision.raw_gear = 1;
  decision.effective_gear = 1;
  decision.raw_gear_valid = false;
  decision.near_obstacle_gear_active = false;
  decision.gear_closed_loop_active = false;
  decision.gear_restore_hold_count = 0;
  decision.gear_target = 1;
  decision.gear_transition_active = false;
  decision.gear_speed_transition_active = false;
  decision.gear_speed_transition_target = 0.0f;
  decision.gear_speed_transition_progress = 1.0f;
  decision.gear = 1;
  decision.gear_peak_speed = 0.70f;
  decision.side_clearance = INFINITY;
  decision.side_predicted_clearance = INFINITY;
  decision.side_closing_rate = 0.0f;
  decision.side_speed_limit = INFINITY;
  decision.moving_prebrake_distance = 0.70f;
  decision.gear_trigger_distance = 0.70f;
  decision.gear_transition_remaining_sec = 0.0f;
  decision.gear_transition_compensation_distance = 0.0f;
  decision.target_max_velocity = 0.70f;
  decision.held_forward_speed = 0.0f;
  return decision;
}

/* ---- DynamicVelocityFusion 类 ---- */

/* 私有内部结构（SideClearanceTrend / SidePrediction / SideProtectionState）。 */
typedef struct {
  float last_clearance;
  float last_update_sec;
  float closing_rate;
  bool initialized;
} dwa_fusion_side_clearance_trend_t;

static inline dwa_fusion_side_clearance_trend_t
dwa_fusion_side_clearance_trend_default(void)
{
  dwa_fusion_side_clearance_trend_t trend;
  trend.last_clearance = INFINITY;
  trend.last_update_sec = -1.0f;
  trend.closing_rate = 0.0f;
  trend.initialized = false;
  return trend;
}

typedef struct {
  float clearance;
  float predicted_clearance;
  float closing_rate;
} dwa_fusion_side_prediction_t;

static inline dwa_fusion_side_prediction_t
dwa_fusion_side_prediction_default(void)
{
  dwa_fusion_side_prediction_t prediction;
  prediction.clearance = INFINITY;
  prediction.predicted_clearance = INFINITY;
  prediction.closing_rate = 0.0f;
  return prediction;
}

typedef struct {
  float clearance;
  float predicted_clearance;
  float closing_rate;
  float speed_limit;
  bool fresh;
  bool cap_requested;
} dwa_fusion_side_protection_state_t;

static inline dwa_fusion_side_protection_state_t
dwa_fusion_side_protection_state_default(void)
{
  dwa_fusion_side_protection_state_t state;
  state.clearance = INFINITY;
  state.predicted_clearance = INFINITY;
  state.closing_rate = 0.0f;
  state.speed_limit = INFINITY;
  state.fresh = false;
  state.cap_requested = false;
  return state;
}

typedef struct {
  dwa_dynamic_velocity_config_t config;

  bool near_obstacle_gear_active;
  dwa_dynamic_control_mode_t base_mode;

  /* 这里记释放时间，也缓存静止分支算出来的速度。 */
  float clear_since_sec;
  float last_static_update_sec;
  float held_static_velocity;

  /* 这里记运动减速和低速保持用到的速度。 */
  float prebrake_target_velocity;
  float held_moving_velocity;

  /* 一轮减速开始后，把入口距离、入口速度和更新时间记在这里。 */
  float latched_prebrake_distance;
  float latched_prebrake_entry_speed;
  float last_prebrake_update_sec;

  /* 9.27 盲区净距锁存（"一走一停"修复）：骑乘过滤区对窄障碍是盲区，
   * 障碍进入后 front_clearance 读数跳 INF，距离比例目标会立刻回满速
   * （实测 v=66.5 顶着障碍推进）。有效净距 = min(当前净距, 锁存最小
   * 净距)，每周期由 dvf_maintain_front_latch 更新；复位条件见该函数。 */
  float effective_front_clearance;
  float latched_min_front_clearance;
  float front_inf_gap_distance;
  float last_front_latch_sec;

  /* 侧边限速记住退出时间和上一个平滑目标。 */
  bool side_cap_active;
  float side_cap_clear_since_sec;
  float side_target_velocity;
  float last_side_target_update_sec;
  bool side_target_initialized;
  dwa_fusion_side_clearance_trend_t left_side_trend;
  dwa_fusion_side_clearance_trend_t right_side_trend;
  int observed_gear;
  bool raw_gear_valid;
  int effective_gear;
  int gear_target;

  bool gear_closed_loop_active;
  bool gear_transition_active;
  bool gear_speed_transition_active;
  float gear_speed_transition_start_speed;
  float gear_speed_transition_target_speed;
  float gear_speed_transition_start_sec;
  float gear_speed_transition_output_speed;
  float gear_speed_transition_progress;
  int gear_restore_hold_count;
  float last_gear_transition_sec;
  float current_gear_trigger_distance;
  float current_gear_transition_remaining_sec;
  float current_gear_transition_compensation_distance;
} dwa_dynamic_velocity_fusion_t;

/* 构造：config_ = config；其余按类内初始化器。 */
void dwa_dynamic_velocity_fusion_init(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_config_t *config);
void dwa_dynamic_velocity_fusion_init_default(dwa_dynamic_velocity_fusion_t *fusion);

/* set_config(config, reason) 直译（成功时不 reset，与 C++ 一致）。 */
bool dwa_dynamic_velocity_fusion_set_config(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_config_t *config,
  const char **reason);

/* reset() 直译。 */
void dwa_dynamic_velocity_fusion_reset(dwa_dynamic_velocity_fusion_t *fusion);

/* complete_dwa_handoff() 直译。 */
void dwa_dynamic_velocity_fusion_complete_dwa_handoff(
  dwa_dynamic_velocity_fusion_t *fusion);

/* update(input) 直译，结果按值返回。 */
dwa_dynamic_velocity_decision_t dwa_dynamic_velocity_fusion_update(
  dwa_dynamic_velocity_fusion_t *fusion,
  const dwa_dynamic_velocity_input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* DWA_DYNAMIC_VELOCITY_FUSION_H */
