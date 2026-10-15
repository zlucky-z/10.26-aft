/*
 * 模块功能：分级日志与节流日志。
 * 对应 C++ 侧 RCLCPP_INFO/WARN/ERROR/ERROR_THROTTLE/INFO_THROTTLE 系。
 * 直译语义：级别过滤 + THROTTLE 按 wall clock 节流窗口；输出到 stderr。
 * 影子模式（P8）可通过 hal_log_set_enabled 整体静默。
 */
#ifndef DWA_HAL_LOG_H
#define DWA_HAL_LOG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HAL_LOG_DEBUG = 0,
  HAL_LOG_INFO = 1,
  HAL_LOG_WARN = 2,
  HAL_LOG_ERROR = 3,
} hal_log_level_t;

/* 全局开关与最低级别（默认 INFO 起）。 */
void hal_log_set_enabled(bool enabled);
void hal_log_set_min_level(hal_log_level_t level);

/* 无节流日志：hal_log(HAL_LOG_INFO, "fmt", ...)。 */
void hal_log(hal_log_level_t level, const char *fmt, ...);

/*
 * 节流日志（RCLCPP_*_THROTTLE 语义）：
 * 同一 key 在 period_ms 窗口内只输出一次；clock 传 hal_steady_now_sec()。
 * key 用调用点的静态整数地址或字面量字符串地址（同一调用点恒定即可）。
 */
void hal_log_throttle(
  double clock_sec, int period_ms, const void *key,
  hal_log_level_t level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DWA_HAL_LOG_H */
