/*
 * 模块功能：单调时钟与墙钟时间接口。
 * 对应 C++ 侧 std::chrono::steady_clock（周期计时/新鲜度判定）与
 * rclcpp::Time::seconds()（数据时间戳）。
 * 可移植性：POSIX clock_gettime(CLOCK_MONOTONIC) / Windows QueryPerformanceCounter。
 */
#ifndef DWA_HAL_TIME_H
#define DWA_HAL_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

/* 单调时钟：自某个未指定起点的秒数（steady_clock 语义，不受系统改时间影响）。 */
double hal_steady_now_sec(void);

/* 墙钟 Unix 时间戳（秒，浮点）。数据时间戳（fused_cloud_time_ 等）用。 */
double hal_wall_now_sec(void);

/*
 * 虚拟墙钟（P8 回放模式专用）：设置后 hal_wall_now_sec 返回该值，
 * 使注入数据的 time 戳与 tick 的新鲜度判定复现录制时刻的相位。
 * 生产路径不调用（override 保持关闭，零开销）。
 * t < 0 恢复真实墙钟。
 */
void hal_wall_set_virtual(double t);

#ifdef __cplusplus
}
#endif

#endif /* DWA_HAL_TIME_H */
