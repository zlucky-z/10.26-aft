/*
 * 模块功能：SDK系统计时、微秒时间和延时辅助实现。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#include <assert.h>
#include <time.h>
#include "sys/time.h"

#include <inttypes.h>
#include "timer.h"
#include <unistd.h>

// 功能：获取getms供调用方使用。
uint32_t getms()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}

// 功能：获取gettimeofday、t供调用方使用。
int gettimeofday_t(struct timeval *tv, void * /*tzv*/)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    tv->tv_sec = ts.tv_sec;
    tv->tv_usec = ts.tv_nsec / 1000;
    return 0;
}

// 功能：获取get、时间供调用方使用。
uint64_t getTime()
{
#if HAS_CLOCK_GETTIME
    struct timespec tim;
    clock_gettime(CLOCK_REALTIME, &tim);
    return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday_t(&timeofday, NULL);
    return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
           static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
#endif
}

// 功能：执行delay相关的运动或时序辅助处理。
void delay(uint32_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  };

  if (ms != 0) {
    usleep(ms * 1000);
  }
}