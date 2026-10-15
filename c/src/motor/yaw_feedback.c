/* =============================================================================
 * yaw_feedback.c —— IMU 角速度闭环补偿实现（A4，10.09）
 * 逐项移植 ltw_dwa/robot_main.c 的 feedback_dwa_step（W_DAMP 系列、
 * W_INTEG 系列、W_SLEW 系列实车标定参数），原注释要点保留。
 * =========================================================================== */
#include "yaw_feedback.h"

#include <math.h>
#include <stddef.h>

#ifndef INFINITY
#include <float.h>
#define INFINITY DBL_MAX
#endif

/* 小信号死区：|w_meas| 低于此值视为静止，不做阻尼纠偏
 * （起步段 fbL/fbR=0、gyroz 仅 ±0.02 噪声时阻尼纠噪声 → w 指令抖） */
#define YF_MEAS_DEADBAND 0.02f

yaw_feedback_params_t yaw_feedback_default_params(void)
{
  yaw_feedback_params_t p;
  /* A4b(10.09) 首版过补后调参（ltw 原值括注，板端 yaml 可覆盖）：
   * 宽走廊拖曳=泄漏慢(0.85)+权限大；通道过转=积分上限 0.49 远超
   * 通道指令 ~0.14+w_scale=4.0 权限整体放大。 */
  p.gain_follow = 0.30f;      /* ltw 0.40：P 跟随增益，纠偏力度 */
  p.cap_follow = 0.15f;
  p.gain_kill = 0.50f;        /* ltw 0.50：压制增益不动（过转的 antidote） */
  p.cap_kill = 0.18f;
  p.fight_margin = 0.08f;
  p.integ_gain = 0.27f;
  p.integ_max_frac = 0.15f;   /* ltw 0.35：上限 0.49→0.21，防积分顶过头 */
  p.integ_release = 0.70f;    /* ltw 0.85：无需求退快一倍，走廊不拖曳 */
  p.integ_min_cmd = 0.03f;
  p.integ_min_v = 0.10f;      /* ltw 10cm/s，换算 m/s */
  p.slew_acc = 0.06f;
  p.slew_brake = 0.20f;
  p.slew_fight_min = 0.10f;
  p.manual_slew_min_theta = 0.10f; /* 10.09 方案①：手柄豁免快率阈值 */
  p.zero_tgt = 0.01f;
  p.zero_out = 0.03f;
  p.ref_w_max = 0.70f;       /* ltw 0.35：w_scale 1.40/0.35=4.0→1.40/0.70=2.0 */
  return p;
}

void yaw_feedback_reset(yaw_feedback_state_t *st)
{
  if (st == NULL) {
    return;
  }
  st->w_cmd_prev = 0.0f;
  st->w_integ = 0.0f;
  st->valid = false;
}

yaw_feedback_out_t yaw_feedback_step(
  const yaw_feedback_params_t *p,
  yaw_feedback_state_t *st,
  float w_cmd,
  float w_meas_raw,
  float v_meas,
  float w_max,
  float theta_joy,
  int imu_ok)
{
  yaw_feedback_out_t r;
  float w_meas;
  float w_scale;
  float w_req;
  float corr;
  float cap;
  float integ_max;
  float ierr;
  int no_demand;
  int fighting;
  float w_prev;
  float dw;

  if (p == NULL || st == NULL) {
    r.w_out = w_cmd;
    r.w_meas = 0.0f;
    r.w_integ = 0.0f;
    r.corr = 0.0f;
    r.fighting = 0;
    return r;
  }
  r.w_out = w_cmd;
  r.w_meas = 0.0f;
  r.w_integ = st->w_integ;
  r.corr = 0.0f;
  r.fighting = 0;

  /* 权限缩放：cap/斜率按 w_max/0.35 等比放大（ltw 2026-09-24 方案），
   * 钳位防异常表值爆表。P 增益作用在绝对误差上不缩放。 */
  w_scale = w_max / p->ref_w_max;
  if (w_scale < 1.0f) {
    w_scale = 1.0f;
  }
  if (w_scale > 6.0f) {
    w_scale = 6.0f;
  }

  /* 小信号死区 */
  w_meas = (fabsf(w_meas_raw) < YF_MEAS_DEADBAND) ? 0.0f : w_meas_raw;
  r.w_meas = w_meas;

  /* ---- 1) 过转 P 阻尼（双模式）+ 2) w 侧小积分 ----
   * 门控（ltw 同款）：IMU 无效（轮差兜底期）两者均不投用——轮差含
   * 标定失配，闭环它反而帮倒忙；仅保留斜率限幅与尾巴归零。
   * fighting=回直模式：实测与指令反号，或实测明显超出指令
   * （+fight_margin，0.05 太敏感会抖动，ltw 定 0.08）。
   * 积分冻结/泄漏：fighting 反打期间不积分；无转向需求泄漏；
   * 低速不积分（防起步 windup 猛甩）；**需求反向或手柄明确反向
   * 立即清零**（连续 S 弯换向点积分器不带旧方向——09-28 ltw 实车
   * 教训：慢泄漏顶着手柄新指令 1.4s，"人为介入控制不了"）。 */
  if (imu_ok) {
    fighting = (w_meas * w_cmd < 0.0f) ||
      (fabsf(w_meas) > fabsf(w_cmd) + p->fight_margin);
    r.fighting = fighting;
    corr = (fighting ? p->gain_kill : p->gain_follow) * (w_meas - w_cmd);
    cap = (fighting ? p->cap_kill : p->cap_follow) * w_scale;
    if (corr > cap) {
      corr = cap;
    }
    if (corr < -cap) {
      corr = -cap;
    }
    w_req = w_cmd - corr;
    r.corr = corr;

    integ_max = p->integ_max_frac * w_max;
    ierr = w_req - w_meas;
    no_demand = fabsf(w_req) < p->integ_min_cmd;
    if ((fabsf(w_req) > p->integ_min_cmd &&
         st->w_integ * w_req < 0.0f) ||
        (fabsf(theta_joy) >= 0.30f &&
         st->w_integ * theta_joy < 0.0f)) {
      st->w_integ = 0.0f;
    }
    if (fighting || no_demand || v_meas < p->integ_min_v) {
      st->w_integ *= p->integ_release;
      if (fabsf(st->w_integ) < 0.01f) {
        st->w_integ = 0.0f;
      }
    } else if ((ierr > 0.0f && st->w_integ < integ_max) ||
               (ierr < 0.0f && st->w_integ > -integ_max)) {
      st->w_integ += p->integ_gain * ierr;
      if (st->w_integ > integ_max) {
        st->w_integ = integ_max;
      }
      if (st->w_integ < -integ_max) {
        st->w_integ = -integ_max;
      }
    }
    r.w_integ = st->w_integ;
    w_req += st->w_integ;

    if (w_req > w_max) {
      w_req = w_max;
    }
    if (w_req < -w_max) {
      w_req = -w_max;
    }
  } else {
    /* IMU 兜底期：指令原样进斜率限幅，积分器保持不动作 */
    w_req = w_cmd;
    r.w_integ = st->w_integ;
  }

  /* ---- 3) 非对称斜率限幅 ----
   * 建转慢（ACC×w_scale）平滑入弯；回正/压过转快（BRAKE×w_scale）。
   * 例外1：车体明显旋转（≥slew_fight_min）且本拍朝反向修正时，
   * 反打也用 BRAKE 上限——压制过转不慢爬（ltw logo3 教训）。
   * 例外2（10.09 方案① 手柄豁免）：骑手明确打杆（|θ_joy|≥阈值）
   * 且建转朝手柄方向时走快率 BRAKE×w_scale——慢斜坡只服务 DWA
   * 自动避障弧的平滑，人主动打方向被 0.12/拍斜坡拦是避障期转向
   * 延迟主因（085332 实测：非避障直通当拍响应，避障期 0.3~1.2s）。 */
  w_prev = st->w_cmd_prev;
  if (!st->valid) {
    /* 首拍直接接受（复位后无历史锚点，不斜坡起步延迟） */
    st->valid = true;
    st->w_cmd_prev = w_req;
    r.w_out = w_req;
    return r;
  }
  dw = w_req - w_prev;
  cap = (fabsf(w_req) < fabsf(w_prev)) ? p->slew_brake : p->slew_acc;
  cap *= w_scale;
  if (dw * w_meas < 0.0f && fabsf(w_meas) > p->slew_fight_min) {
    cap = p->slew_brake;
  }
  if (p->manual_slew_min_theta > 0.0f &&
      fabsf(theta_joy) >= p->manual_slew_min_theta &&
      dw * theta_joy > 0.0f) {
    const float fast = p->slew_brake * w_scale;
    if (fast > cap) {
      cap = fast;
    }
  }
  if (dw > cap) {
    dw = cap;
  }
  if (dw < -cap) {
    dw = -cap;
  }
  r.w_out = w_prev + dw;

  /* 尾巴归零：目标已很小且输出衰减到位后直接清零，消低幅摆动 */
  if (fabsf(w_cmd) < p->zero_tgt && fabsf(r.w_out) < p->zero_out) {
    r.w_out = 0.0f;
  }
  st->w_cmd_prev = r.w_out;
  return r;
}
