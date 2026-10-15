/* hal_time 实现：POSIX / Windows 双路 + 回放虚拟墙钟。 */
#include "hal_time.h"

/* ---- 虚拟墙钟（P8 回放；非线程安全——回放为单线程事件循环） ---- */
static double g_virtual_wall_sec = -1.0;
static int g_virtual_wall_valid = 0;

void hal_wall_set_virtual(double t)
{
  if (t < 0.0) {
    g_virtual_wall_valid = 0;
    g_virtual_wall_sec = -1.0;
  } else {
    g_virtual_wall_valid = 1;
    g_virtual_wall_sec = t;
  }
}

#if defined(_WIN32)

#include <windows.h>

double hal_steady_now_sec(void)
{
  static LARGE_INTEGER freq;
  static int freq_ready = 0;
  LARGE_INTEGER counter;
  if (!freq_ready) {
    QueryPerformanceFrequency(&freq);
    freq_ready = 1;
  }
  QueryPerformanceCounter(&counter);
  return (double)counter.QuadPart / (double)freq.QuadPart;
}

double hal_wall_now_sec(void)
{
  FILETIME ft;
  ULARGE_INTEGER u;
  if (g_virtual_wall_valid) return g_virtual_wall_sec;
  GetSystemTimeAsFileTime(&ft);
  u.LowPart = ft.dwLowDateTime;
  u.HighPart = ft.dwHighDateTime;
  /* Windows epoch 1601-01-01 → Unix epoch 差 11644473600 秒，100ns 单位。 */
  return (double)(u.QuadPart - 116444736000000000ULL) * 1e-7;
}

#else /* POSIX */

#include <time.h>

double hal_steady_now_sec(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

double hal_wall_now_sec(void)
{
  struct timespec ts;
  if (g_virtual_wall_valid) return g_virtual_wall_sec;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

#endif
