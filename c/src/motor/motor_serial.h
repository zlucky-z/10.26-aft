/*
 * 模块功能：串口打开/配置、写帧、反馈读取与解析、端口锁。
 * 直译自 motor_bridge_cpp/native/motor_serial.hpp
 * （namespace hu_m40_motor_serial → ms_ 前缀）。
 *
 * 可移植性：真实串口路径仅 POSIX（termios/flock）；Windows 宿主机上
 * ms_serial_writer_open 恒失败（除非 dry_run），供宿主机冒烟/对拍使用。
 * 板端（aarch64 Linux）走完整 POSIX 路径。
 *
 * 时钟语义（bug-for-bug）：sample.time 用 CLOCK_MONOTONIC 原始值
 * （hal_steady_now_sec，开机起算），而 bridge 控制逻辑的 now 是进程起算
 * 的 steady_clock —— 与 C++ 原版两个时钟基一致（见 TRANSLATION_NOTES）。
 */
#ifndef DWA_MOTOR_SERIAL_H
#define DWA_MOTOR_SERIAL_H

#include <stddef.h>
#include <stdint.h>

#include "motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 反馈样本（对齐 Python parse_rx_buffer 返回的 dict） */
typedef enum {
  MS_SAMPLE_KIND_NONE = 0,
  MS_SAMPLE_KIND_DRIVE = 1,   /* "drive" */
  MS_SAMPLE_KIND_STATUS = 2   /* "status" */
} ms_sample_kind_t;

typedef struct {
  ms_sample_kind_t kind;
  int x;                /* -1 表示无（Python 用 None 语义） */
  int y;
  int left_raw;
  int right_raw;
  double left_mps;
  double right_mps;
  int gear;             /* -1 表示无 */
  int gear_valid;
  const char *protocol; /* 字面量："2.3" / "controller_feedback_gear" /
                           "2.1" / "2.0" / "legacy_feedback" / "" */
  uint8_t frame[MP_FRAME_MAX + 256]; /* 反馈帧最长 5+255，一般 ≤ 14 */
  size_t frame_len;
  double time;
} ms_sample_t;

typedef struct {
  long bytes;
  long frames;
  long drive;
  long status;
  long other;
  long bad_crc;
  long bad_version;
  long dropped;
} ms_rx_stats_t;

/* rx 环节缓冲上限：200ms 轮询 × 9600 波特 ≈ 240B/轮，64KB 远超需求。
 * C++ vector 无界增长；超限时本实现停止追加（仅病态输入下可见差异）。 */
#define MS_RX_BUF_MAX 65536

/* 一次 read_samples 最多返回的样本数（C++ vector 无界；实测每轮 ≤ 32）。 */
#define MS_SAMPLE_MAX 64

typedef struct {
  char port[128];
  int baud;
  int dry_run;
  int fd;
  /* PortLock（对齐 Python PortLock，flock） */
  char lock_path[192];
  int lock_fd;
  /* rx 状态 */
  uint8_t rx_buf[MS_RX_BUF_MAX];
  size_t rx_len;
  ms_rx_stats_t rx_stats;
  double last_rx_time;
  uint8_t last_rx_preview[32];
  size_t last_rx_preview_len;
} ms_serial_writer_t;

/*
 * 打开串口（SerialWriter 构造函数直译）。dry_run=1 时不开串口直接成功。
 * 失败返回 -1 并写 err 缓冲（对应 C++ throw runtime_error 的消息）。
 */
int ms_serial_writer_open(
  ms_serial_writer_t *w, const char *port, int baud, int dry_run,
  char *err, size_t err_size);

/* 写帧（dry_run 直接返回）。 */
void ms_serial_writer_write(ms_serial_writer_t *w, const mp_frame_t *frame);

/* 读取并解析反馈样本；返回样本数（dry_run / 未打开 → 0）。 */
int ms_serial_writer_read_samples(
  ms_serial_writer_t *w, ms_sample_t out[MS_SAMPLE_MAX]);

/* 关闭串口与端口锁。 */
void ms_serial_writer_close(ms_serial_writer_t *w);

/* CLOCK_MONOTONIC 原始秒（对齐 C++ SerialWriter::now_monotonic）。 */
double ms_now_monotonic(void);

#ifdef __cplusplus
}
#endif

#endif /* DWA_MOTOR_SERIAL_H */
