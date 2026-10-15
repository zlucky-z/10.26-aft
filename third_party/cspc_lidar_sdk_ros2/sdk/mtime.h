/*
 * 模块功能：SDK时间统计辅助接口。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#ifndef __H_MTIME_H__
#define __H_MTIME_H__

#include <chrono>
#include <unistd.h>
#include <time.h>

enum TIME_PRECISION
{
	TIME_NANOSECOND = 0,	//纳秒
	TIME_MICROSECOND,		//微秒
	TIME_MILLISECOND,		//毫秒
	TIME_SECOND,
	TIME_MINUTE,
	TIME_HOUR
};

//计算用时
extern std::chrono::time_point<std::chrono::steady_clock> time_start;
// 功能：执行`timestart`，负责本模块对应的timestart处理。
inline void timestart()
{
	time_start = std::chrono::steady_clock::now();
}
// 功能：执行`timeused`，负责本模块对应的timeused处理。
inline int64_t timeused()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
}
// 功能：执行`timeused`，负责本模块对应的timeused处理。
inline int64_t timeused(std::chrono::time_point<std::chrono::steady_clock>& time_last)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_last).count();
}

//休眠多少毫秒
void sleep_ms(int ms);

//获取tm (只更新年月日时分秒)
// 返回指定时区偏移下的日历时间结构。
struct tm localdate(int time_zone = 8);

//获取时间[整数]
// 按 TIME_PRECISION 返回当前单调时钟的整数时间。
int64_t current_times(int precision = TIME_MILLISECOND);

//获取时间[字符串]
// 按给定格式把当前时间格式化为静态字符串。
char *time_str(const char *fmt = "%F %T");

#endif
