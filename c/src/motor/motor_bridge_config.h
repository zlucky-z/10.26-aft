/*
 * 模块功能：电机桥配置（对齐 9.20 Python parse_args 的参数清单）。
 * 直译自 motor_bridge_cpp/native/motor_xy_cmd_vel_node.cpp 的 Config 结构
 * + parse_config（argv 逐项解析 + 数值校验，错误消息逐字对齐）。
 */
#ifndef DWA_MOTOR_BRIDGE_CONFIG_H
#define DWA_MOTOR_BRIDGE_CONFIG_H

#include <stddef.h>

#include "motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MB_TOPIC_MAX 96
#define MB_GEAR_PEAK_SPEEDS_MAX 5
#define MB_STRING_MAX 32

typedef struct {
  char port[MB_TOPIC_MAX];             /* "/dev/ttyS6" */
  int baud;                            /* 9600 */
  mp_protocol_t protocol;              /* "2.0" */
  char protocol_name[MB_STRING_MAX];   /* 日志用字符串 */
  char cmd_topic[MB_TOPIC_MAX];        /* "/cmd_vel" */
  char controller_cmd_topic[MB_TOPIC_MAX];   /* "" */
  char feedback_topic[MB_TOPIC_MAX];         /* "" */
  char wheel_yaw_topic[MB_TOPIC_MAX];        /* "" */
  char hall_speed_control_topic[MB_TOPIC_MAX]; /* "/dwa/hall_speed_control" */
  char left_scan_topic[MB_TOPIC_MAX];   /* "/scan_left_safety" */
  char right_scan_topic[MB_TOPIC_MAX];  /* "/scan_right_safety" */
  char escape_cloud_topic[MB_TOPIC_MAX]; /* "/dwa/fused_scan_base" */
  double escape_stop_hold_sec;   /* 0.5 */
  double escape_stop_speed;      /* 0.03 */
  double escape_forward_deadband; /* 0.02 */
  double escape_turn_yawrate;    /* 0.5 */
  double escape_cloud_timeout;   /* 0.60 */
  double escape_half_width;      /* 0.27 */
  double radar_timeout;          /* 0.5 */
  char horn_topic[MB_TOPIC_MAX]; /* "/wheelchair/horn_cmd" */
  double horn_timeout;           /* 0.5 */
  int fallback_gear;             /* 1 */
  /* 按档位满推速度标尺(m/s)，空=全部用 max_forward_speed */
  double gear_peak_speeds[MB_GEAR_PEAK_SPEEDS_MAX];
  int gear_peak_speeds_count;
  char imu_topic[MB_TOPIC_MAX];   /* "/imu/data_raw" */
  char gyro_angle_source[MB_STRING_MAX]; /* "zero" */
  double max_forward_speed;  /* 0.50 */
  double max_reverse_speed;  /* 0.70 */
  double max_turn_speed;     /* 0.60 */
  int forward_output_deadzone_y; /* 120 */
  double linear_deadband;    /* 0.01 */
  double angular_deadband;   /* 0.02 */
  double cmd_timeout;        /* 0.5 */
  int interval_ms;           /* 200 */
  char output_mode[MB_STRING_MAX];  /* "continuous" */
  double intervention_linear_delta;  /* 0.02 */
  double intervention_angular_delta; /* 0.015 */
  int intervention_release_stop_repeats; /* 3 */
  double safe_stop_hold_sec;   /* 2.5 */
  double safe_stop_wheel_linear_threshold;  /* 0.03 */
  double safe_stop_wheel_angular_threshold; /* 0.05 */
  double intervention_linear_slew_rate; /* 0.6 */
  double intervention_angular_slew_rate; /* 0.9 */
  double wheel_track_m;        /* 0.56 */
  double wheel_feedback_timeout; /* 0.25 */
  double wheel_feedback_alpha;   /* 0.35 */
  double hall_speed_control_timeout; /* 0.25 */
  double hall_speed_y_gain;      /* 300.0 */
  double hall_speed_y_down_rate; /* 180.0 */
  double hall_speed_y_up_rate;   /* 80.0 */
  double hall_moving_hold_y_up_rate; /* 20.0 */
  double hall_moving_hold_max_y;     /* 150.0 */
  double hall_moving_hold_emergency_speed; /* 0.52 */
  double hall_moving_hold_release_margin;  /* 0.04 */
  double hall_speed_deadband;       /* 0.03 */
  double hall_speed_decel_predict_sec; /* 0.25 */
  int linear_profile_enabled;  /* 0 */
  double linear_profile_comfort_up_rate;     /* 15.0 */
  double linear_profile_comfort_down_rate;   /* 15.0 */
  double linear_profile_protective_up_rate;  /* 15.0 */
  double linear_profile_protective_down_rate; /* 60.0 */
  double wheel_yaw_max_dt;   /* 0.15 */
  char closed_loop_mode[MB_STRING_MAX];  /* "off" */
  double closed_loop_angular_kp;  /* 0.22 */
  double closed_loop_angular_ki;  /* 0.02 */
  double closed_loop_linear_kp;   /* 0.0 */
  double closed_loop_max_angular_correction; /* 0.04 */
  double closed_loop_max_linear_correction;  /* 0.0 */
  int pre_stop_repeats;    /* 5 */
  double pre_stop_settle;  /* 0.1 */
  int activate_stop_repeats;   /* 0 */
  double activate_stop_settle; /* 0.05 */
  int stop_repeats;        /* 5 */
  int stop_on_close;       /* 1 */
  char idle_stop_mode[MB_STRING_MAX];  /* "continuous" */
  int axis_lock;           /* 0 */
  int invert_linear;       /* 0 */
  int invert_angular;      /* 0 */
  int controller_center_x; /* kCenter */
  int controller_center_y; /* kCenter */
  int controller_dead_low;   /* 89 */
  int controller_dead_high;  /* 111 */
  int output_turn_dead_low;  /* 89 */
  int output_turn_dead_high; /* 111 */
  int controller_axis_lock;  /* 0 */
  int controller_invert_x;   /* 0 */
  int controller_invert_y;   /* 0 */
  int controller_require_neutral; /* 1 */
  int feedback_verbose;  /* 0 */
  int rx_debug;          /* 0 */
  int dry_run;           /* 0 */
  int verbose;           /* 0 */
} mb_config_t;

/* 默认值（Config 结构体成员默认值直译）。 */
void mb_config_defaults(mb_config_t *c);

/*
 * argv 解析（parse_config 直译）。返回 0 成功；-1 失败并写 err
 * （对应 C++ throw invalid_argument 的消息）。
 */
int mb_config_parse(int argc, char **argv, mb_config_t *c,
  char *err, size_t err_size);

#ifdef __cplusplus
}
#endif

#endif /* DWA_MOTOR_BRIDGE_CONFIG_H */
