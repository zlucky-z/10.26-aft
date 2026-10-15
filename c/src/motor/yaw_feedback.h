/* =============================================================================
 * yaw_feedback.h —— IMU 角速度闭环补偿（A4，10.09）
 * -----------------------------------------------------------------------------
 * 移植自基线工程 ltw_dwa 的 feedback_dwa_step 核心（实车验证版，10Hz 标定）。
 * 作用：DWA 接管期（MOVING_DWA）把 DWA 角速度指令 w 与 IMU 实测角速度
 * gz 闭环，补偿底板执行缺口（A3 实测：1 档强制下指令→实际等效比从 0.78
 * 跌到 0.57，通道内 selw 0.14 物理只转 ~0.06，弯道转不过来）。
 *
 * 结构（与 ltw 逐项对应）：
 *   1) 过转 P 阻尼（双模式）：跟随=顺向补滞后；回直=对抗残余旋转
 *   2) w 侧小积分：收稳态缺口（cmd/gz 比值收敛到 ~1）
 *   3) 非对称斜率限幅：建转慢（平滑入弯），回正/压过转快
 *   4) 尾巴归零：目标与输出都接近 0 时直接清零
 *
 * 本模块纯逻辑无 IO（I2C 读取在 robot_main 的 poll 层完成）；
 * IMU 不可用时调用方直接旁路（本模块不被调用，行为与原版一致）。
 * =========================================================================== */
#ifndef DWA_YAW_FEEDBACK_H
#define DWA_YAW_FEEDBACK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /* P 阻尼（跟随模式） */
  float gain_follow;      /* 0.40（ltw 实车标定） */
  float cap_follow;       /* 单拍限幅基准 0.15 rad/s（×w_scale） */
  /* P 阻尼（回直模式） */
  float gain_kill;        /* 0.50 */
  float cap_kill;         /* 单拍限幅基准 0.18 rad/s（×w_scale） */
  float fight_margin;     /* 0.08：|w_meas| 超出指令此值才算明显过转 */
  /* 小积分 */
  float integ_gain;       /* 0.27/拍（10Hz） */
  float integ_max_frac;   /* 限幅 = 0.35×w_max */
  float integ_release;    /* 无需求时泄漏 0.85/拍 */
  float integ_min_cmd;    /* 0.03 rad/s 以下视为无转向需求 */
  float integ_min_v;      /* 车速 m/s 低于此值不积分（防起步 windup）=0.10 */
  /* 斜率限幅（非对称） */
  float slew_acc;         /* 建转 0.06/拍（×w_scale） */
  float slew_brake;       /* 回正/压过转 0.20/拍（×w_scale） */
  float slew_fight_min;   /* 0.10：车体明显旋转且指令反向时反打用 BRAKE */
  float manual_slew_min_theta; /* 手柄豁免阈值 rad（10.09 方案①）：|θ_joy|
                                * ≥此值且建转朝手柄方向时走快率——慢斜坡
                                * 是给 DWA 自动弧平滑的，人主动打方向
                                * 不应被斜坡拦（实测避障期爬坡 0.12/拍
                                * 延迟 0.3~1.2s）。0=禁用豁免 */
  /* 归零 */
  float zero_tgt;         /* 目标 <0.01 且输出 <0.03 → 清零 */
  float zero_out;
  /* 基准 */
  float ref_w_max;        /* 阻尼/斜率权限缩放基准=0.35（ltw 1 档标定分母） */
} yaw_feedback_params_t;

typedef struct {
  float w_cmd_prev;   /* 上一拍最终输出 w（斜率限幅锚点） */
  float w_integ;      /* 积分器 */
  bool  valid;        /* 是否已初始化（首次直接接受指令） */
} yaw_feedback_state_t;

typedef struct {
  float w_out;     /* 补偿后输出 */
  float w_meas;    /* 使用的实测角速度（死区处理后） */
  float w_integ;   /* 当前积分值（诊断） */
  float corr;      /* 本拍 P 修正量（诊断） */
  int   fighting;  /* 本拍是否回直模式（诊断） */
} yaw_feedback_out_t;

/* 默认参数（ltw 实车验证值） */
yaw_feedback_params_t yaw_feedback_default_params(void);

/* 状态复位（重接管/接管丢失/IMU 失效切换时调用） */
void yaw_feedback_reset(yaw_feedback_state_t *st);

/* 单拍补偿（10Hz 调用；w_max=本拍角速度限幅，如 1.40）。
 * w_meas：IMU 实测角速度（rad/s，已含符号约定：左转>0）。
 * imu_ok：IMU 本拍是否有效；=0（轮差兜底期）时 P 阻尼与积分器
 * 均不投用（ltw 同款门控——轮差含标定失配，闭环它反而帮倒忙），
 * 仅保留斜率限幅与尾巴归零。
 * theta_joy：手柄转向角（rad）；非零且与积分器反号时立即清零积分
 * （09-28 ltw 教训：人为换向必须立刻生效）。 */
yaw_feedback_out_t yaw_feedback_step(
  const yaw_feedback_params_t *p,
  yaw_feedback_state_t *st,
  float w_cmd,
  float w_meas_raw,
  float v_meas,
  float w_max,
  float theta_joy,
  int imu_ok);

#ifdef __cplusplus
}
#endif
#endif /* DWA_YAW_FEEDBACK_H */
