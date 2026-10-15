/*
 * 模块功能：侧墙保持控制器：依据侧距、接近趋势和手动方向限制或修正角速度。
 * 直译自 native/side_wall_hold_control.hpp（逐行对应，不改逻辑/默认值/执行顺序）。
 * std::string reason → const char*（固定字面量）；类 → 结构体 + 函数。
 */
#ifndef DWA_SIDE_WALL_HOLD_CONTROL_H
#define DWA_SIDE_WALL_HOLD_CONTROL_H

#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool enabled;
  float hold_distance;
  float correction_distance;
  float prediction_sec;
  float invalid_clearance_hold_sec;
  float closing_rate_filter_alpha;
  float closing_rate_max;
  float release_distance;
  float proportional_gain;
  float min_yawrate;
  float max_yawrate;
  float yawrate_acceleration;
  float release_clear_sec;
} dwa_side_wall_hold_config_t;

/* SideWallHoldConfig{} 成员默认值直译。 */
dwa_side_wall_hold_config_t dwa_side_wall_hold_config_default(void);

/* validate_side_wall_hold_config 直译；reason 可为 NULL。 */
bool dwa_validate_side_wall_hold_config(
  const dwa_side_wall_hold_config_t *config,
  const char **reason);

/* enum class SideWallHoldSide { kNone, kLeft, kRight, kBoth } */
typedef enum {
  DWA_SIDE_HOLD_NONE = 0,
  DWA_SIDE_HOLD_LEFT = 1,
  DWA_SIDE_HOLD_RIGHT = 2,
  DWA_SIDE_HOLD_BOTH = 3
} dwa_side_wall_hold_side_t;

/* side_wall_hold_side_name 直译。 */
const char *dwa_side_wall_hold_side_name(dwa_side_wall_hold_side_t side);

typedef struct {
  float now_sec;
  float dt_sec;
  bool forward;
  bool perception_fresh;
  float left_clearance;
  float right_clearance;
  float requested_yawrate;
} dwa_side_wall_hold_input_t;

/* SideWallHoldInput{} 默认值直译。 */
static inline dwa_side_wall_hold_input_t dwa_side_wall_hold_input_default(void)
{
  dwa_side_wall_hold_input_t input;
  input.now_sec = 0.0f;
  input.dt_sec = 0.10f;
  input.forward = false;
  input.perception_fresh = false;
  input.left_clearance = INFINITY;
  input.right_clearance = INFINITY;
  input.requested_yawrate = 0.0f;
  return input;
}

typedef struct {
  bool active;
  bool left_latched;
  bool right_latched;
  bool left_active;
  bool right_active;
  dwa_side_wall_hold_side_t side;
  float correction_yawrate;
  float output_yawrate;
  float left_clearance;
  float right_clearance;
  float left_predicted_clearance;
  float right_predicted_clearance;
  float left_closing_rate;
  float right_closing_rate;
  bool left_clearance_held;
  bool right_clearance_held;
} dwa_side_wall_hold_result_t;

/* SideWallHoldResult{} 默认值直译。 */
static inline dwa_side_wall_hold_result_t dwa_side_wall_hold_result_default(void)
{
  dwa_side_wall_hold_result_t result;
  result.active = false;
  result.left_latched = false;
  result.right_latched = false;
  result.left_active = false;
  result.right_active = false;
  result.side = DWA_SIDE_HOLD_NONE;
  result.correction_yawrate = 0.0f;
  result.output_yawrate = 0.0f;
  result.left_clearance = INFINITY;
  result.right_clearance = INFINITY;
  result.left_predicted_clearance = INFINITY;
  result.right_predicted_clearance = INFINITY;
  result.left_closing_rate = 0.0f;
  result.right_closing_rate = 0.0f;
  result.left_clearance_held = false;
  result.right_clearance_held = false;
  return result;
}

/* apply_side_wall_hold_yawrate(result, requested_yawrate) 直译。 */
float dwa_apply_side_wall_hold_yawrate(
  const dwa_side_wall_hold_result_t *result,
  float requested_yawrate);

/*
 * SideWallHoldController → dwa_side_wall_hold_controller_t。
 * SideClearanceState / SideClearanceEstimate 为私有内部结构，一并直译。
 */
typedef struct {
  float last_clearance;
  float last_update_sec;
  float closing_rate;
  bool initialized;
} dwa_side_clearance_state_t;

static inline dwa_side_clearance_state_t dwa_side_clearance_state_default(void)
{
  dwa_side_clearance_state_t state;
  state.last_clearance = INFINITY;
  state.last_update_sec = -1.0f;
  state.closing_rate = 0.0f;
  state.initialized = false;
  return state;
}

typedef struct {
  float clearance;
  float predicted_clearance;
  float closing_rate;
  bool held;
} dwa_side_clearance_estimate_t;

static inline dwa_side_clearance_estimate_t dwa_side_clearance_estimate_default(void)
{
  dwa_side_clearance_estimate_t estimate;
  estimate.clearance = INFINITY;
  estimate.predicted_clearance = INFINITY;
  estimate.closing_rate = 0.0f;
  estimate.held = false;
  return estimate;
}

typedef struct {
  dwa_side_wall_hold_config_t config;
  bool left_active;
  bool right_active;
  float left_clear_since_sec;
  float right_clear_since_sec;
  float correction_yawrate;
  dwa_side_clearance_state_t left_clearance_state;
  dwa_side_clearance_state_t right_clearance_state;
} dwa_side_wall_hold_controller_t;

/* 构造：config_ = config，其余成员按类内初始化器。 */
void dwa_side_wall_hold_controller_init(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_config_t *config);
/* 便捷：默认配置构造。 */
void dwa_side_wall_hold_controller_init_default(dwa_side_wall_hold_controller_t *controller);

/* set_config(config, reason) 直译（校验失败不改配置）。 */
bool dwa_side_wall_hold_controller_set_config(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_config_t *config,
  const char **reason);

/* reset() 直译。 */
void dwa_side_wall_hold_controller_reset(dwa_side_wall_hold_controller_t *controller);

/* update(input) 直译，结果按值返回。 */
dwa_side_wall_hold_result_t dwa_side_wall_hold_controller_update(
  dwa_side_wall_hold_controller_t *controller,
  const dwa_side_wall_hold_input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* DWA_SIDE_WALL_HOLD_CONTROL_H */
