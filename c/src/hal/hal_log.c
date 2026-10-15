/* hal_log 实现。 */
#include "hal_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hal_time.h"

#define HAL_LOG_MAX_LINE 2048
#define HAL_LOG_THROTTLE_SLOTS 256

static bool g_enabled = true;
static hal_log_level_t g_min_level = HAL_LOG_INFO;

static const char *level_tag(hal_log_level_t level)
{
  switch (level) {
    case HAL_LOG_DEBUG: return "DEBUG";
    case HAL_LOG_INFO:  return "INFO";
    case HAL_LOG_WARN:  return "WARN";
    case HAL_LOG_ERROR: return "ERROR";
  }
  return "INFO";
}

void hal_log_set_enabled(bool enabled)
{
  g_enabled = enabled;
}

void hal_log_set_min_level(hal_log_level_t level)
{
  g_min_level = level;
}

void hal_log(hal_log_level_t level, const char *fmt, ...)
{
  char line[HAL_LOG_MAX_LINE];
  va_list ap;

  if (!g_enabled || level < g_min_level) return;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  fprintf(stderr, "[%s] %s\n", level_tag(level), line);
}

/* 节流槽：key 地址 → 上次输出时间。固定槽 + 线性探测（日志路径非热循环）。 */
typedef struct {
  const void *key;
  double last_sec;
} hal_log_throttle_slot_t;

static hal_log_throttle_slot_t g_slots[HAL_LOG_THROTTLE_SLOTS];

void hal_log_throttle(
  double clock_sec, int period_ms, const void *key,
  hal_log_level_t level, const char *fmt, ...)
{
  char line[HAL_LOG_MAX_LINE];
  va_list ap;
  hal_log_throttle_slot_t *slot = NULL;
  int i;
  const double period_sec = (double)period_ms * 1e-3;

  if (!g_enabled || level < g_min_level) return;

  for (i = 0; i < HAL_LOG_THROTTLE_SLOTS; ++i) {
    if (g_slots[i].key == key) { slot = &g_slots[i]; break; }
    if (g_slots[i].key == NULL && slot == NULL) slot = &g_slots[i];
  }
  if (slot == NULL) slot = &g_slots[0]; /* 极端溢出：复用 0 号槽 */

  if (slot->key == key) {
    if (clock_sec - slot->last_sec < period_sec) return;
  } else {
    slot->key = key;
  }
  slot->last_sec = clock_sec;

  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  fprintf(stderr, "[%s] %s\n", level_tag(level), line);
}
