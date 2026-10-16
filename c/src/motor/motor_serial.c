/*
 * 模块功能：motor_serial.h 对应实现（motor_serial.hpp 直译）。
 */
#include "motor_serial.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "../hal/hal_time.h"

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#else
#include <windows.h>
#endif

double ms_now_monotonic(void)
{
  return hal_steady_now_sec();
}

#if !defined(_WIN32)

/* 波特率映射（对齐 Python configure_serial 的 baud_rates） */
static int baud_to_termios(int baud, speed_t *out)
{
  switch (baud) {
    case 9600: *out = B9600; return 0;
    case 19200: *out = B19200; return 0;
    case 38400: *out = B38400; return 0;
    case 57600: *out = B57600; return 0;
    case 115200: *out = B115200; return 0;
    default: return -1;
  }
}

static int configure_serial(int fd, int baud, char *err, size_t err_size)
{
  speed_t rate;
  struct termios attrs;
  if (baud_to_termios(baud, &rate) != 0) {
    snprintf(err, err_size, "unsupported baud rate");
    return -1;
  }
  if (tcgetattr(fd, &attrs) != 0) {
    snprintf(err, err_size, "tcgetattr failed: %s", strerror(errno));
    return -1;
  }
  attrs.c_iflag = 0;
  attrs.c_oflag = 0;
  attrs.c_cflag = rate | CS8 | CLOCAL | CREAD;
  attrs.c_lflag = 0;
  cfsetispeed(&attrs, rate);
  cfsetospeed(&attrs, rate);
  attrs.c_cc[VMIN] = 0;
  attrs.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &attrs) != 0) {
    snprintf(err, err_size, "tcsetattr failed: %s", strerror(errno));
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return 0;
}

/* 端口锁（对齐 Python PortLock，用 flock） */
static int port_lock_acquire(ms_serial_writer_t *w, char *err, size_t err_size)
{
  /* Python: "/tmp/hu_m40_motor_%s.lock" % basename(port).replace("/", "_")
   * —— replace 只作用于 basename（不含 '/'，实为 no-op），锁文件固定在
   * /tmp。此前实现误对整串做 '/'→'_' 替换，得到相对路径
   * "_tmp_hu_m40_motor_ttyS6.lock"：cwd 只读时锁文件创建失败，且不同
   * cwd 的进程会各锁各的文件、失去互斥意义（P9 修正）。 */
  const char *slash = strrchr(w->port, '/');
  const char *base = slash != NULL ? slash + 1 : w->port;
  snprintf(w->lock_path, sizeof(w->lock_path), "/tmp/hu_m40_motor_%s.lock", base);
  w->lock_fd = open(w->lock_path, O_CREAT | O_RDWR, 0644);
  if (w->lock_fd < 0) {
    snprintf(err, err_size, "cannot open lock file: %s", strerror(errno));
    return -1;
  }
  if (flock(w->lock_fd, LOCK_EX | LOCK_NB) != 0) {
    snprintf(err, err_size,
      "motor serial port is already in use; stop the other motor process first");
    return -1;
  }
  return 0;
}

static void port_lock_release(ms_serial_writer_t *w)
{
  if (w->lock_fd < 0) return;
  flock(w->lock_fd, LOCK_UN);
  close(w->lock_fd);
  w->lock_fd = -1;
}

#else /* _WIN32 */

static int port_lock_acquire(ms_serial_writer_t *w, char *err, size_t err_size)
{
  (void)w;
  snprintf(err, err_size, "serial port not supported on this host (dry-run only)");
  return -1;
}

static void port_lock_release(ms_serial_writer_t *w)
{
  (void)w;
}

#endif /* !_WIN32 */

int ms_serial_writer_open(
  ms_serial_writer_t *w, const char *port, int baud, int dry_run,
  char *err, size_t err_size)
{
  memset(w, 0, sizeof(*w));
  strncpy(w->port, port, sizeof(w->port) - 1);
  w->baud = baud;
  w->dry_run = dry_run;
  w->fd = -1;
  w->lock_fd = -1;
  w->rx_len = 0;
  w->last_rx_time = 0.0;
  w->last_rx_preview_len = 0;
  err[0] = '\0';
  if (!dry_run) {
#if !defined(_WIN32)
    if (port_lock_acquire(w, err, err_size) != 0) return -1;
    w->fd = open(w->port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (w->fd < 0) {
      snprintf(err, err_size, "cannot open serial port: %s", strerror(errno));
      port_lock_release(w);
      return -1;
    }
    if (configure_serial(w->fd, baud, err, err_size) != 0) {
      close(w->fd);
      w->fd = -1;
      port_lock_release(w);
      return -1;
    }
#else
    (void)port_lock_acquire(w, err, err_size); /* 恒失败路径 */
    return -1;
#endif
  }
  return 0;
}

long ms_serial_writer_write(ms_serial_writer_t *w, const mp_frame_t *frame,
  int *error_out)
{
  if (error_out != NULL) *error_out = 0;
  if (w->dry_run) return -2;
#if !defined(_WIN32)
  {
    const ssize_t written = write(w->fd, frame->bytes, frame->len);
    if (written < 0 && error_out != NULL) *error_out = errno;
    return (long)written;
  }
#else
  (void)frame;
  return -2;
#endif
}

/*
 * parse_rx_buffer：对齐 C++ 的 while 循环（SOP 搜索、帧提取、CRC/版本
 * 校验、drive/status 分支）。返回本次解析出的样本数。
 */
static int ms_parse_rx_buffer(ms_serial_writer_t *w, ms_sample_t out[MS_SAMPLE_MAX])
{
  int count = 0;
  while (1) {
    uint8_t version, cmd, length, got_crc, calc_crc;
    size_t total;
    const uint8_t *frame, *data;

    if (w->rx_len < 5) break;
    if (w->rx_buf[0] != (uint8_t)MP_SOP) {
      memmove(w->rx_buf, w->rx_buf + 1, --w->rx_len);
      w->rx_stats.dropped += 1;
      continue;
    }
    version = w->rx_buf[1];
    cmd = w->rx_buf[2];
    length = w->rx_buf[3];
    total = 5 + (size_t)length;
    if (w->rx_len < total) break;

    frame = w->rx_buf;
    data = w->rx_buf + 5;
    got_crc = frame[4];
    calc_crc = mp_crc8(data, (size_t)length);
    w->rx_stats.frames += 1;

    if (version != (uint8_t)MP_PACKET_VERSION) {
      w->rx_stats.bad_version += 1;
      memmove(w->rx_buf, w->rx_buf + total, w->rx_len - total);
      w->rx_len -= total;
      continue;
    }
    if (got_crc != calc_crc) {
      w->rx_stats.bad_crc += 1;
      memmove(w->rx_buf, w->rx_buf + total, w->rx_len - total);
      w->rx_len -= total;
      continue;
    }
    if (cmd == (uint8_t)MP_COMMAND_DRIVE && length == 2) {
      ms_sample_t *s;
      if (count < MS_SAMPLE_MAX) {
        s = &out[count];
        memset(s, 0, sizeof(*s));
        s->kind = MS_SAMPLE_KIND_DRIVE;
        s->x = data[0];
        s->y = data[1];
        s->gear = -1;
        s->protocol = "";
        memcpy(s->frame, frame, total);
        s->frame_len = total;
        s->time = ms_now_monotonic();
        count += 1;
      }
      w->rx_stats.drive += 1;
      memmove(w->rx_buf, w->rx_buf + total, w->rx_len - total);
      w->rx_len -= total;
      continue;
    }

    if (cmd == (uint8_t)MP_COMMAND_STATUS && (length == 2 || length == 4 ||
                                              length == 6 || length == 7 ||
                                              length == 9)) {
      ms_sample_t *s;
      double speed_scale_mps = MP_LEGACY_SPEED_SCALE_MPS;
      if (count < MS_SAMPLE_MAX) {
        s = &out[count];
        memset(s, 0, sizeof(*s));
        s->kind = MS_SAMPLE_KIND_STATUS;
        s->x = -1;
        s->y = -1;
        s->gear = -1;
        s->protocol = "";
        if (length == 9) {
          s->x = mp_protocol_coord_to_logical(data[0], data[1]);
          s->y = mp_protocol_coord_to_logical(data[2], data[3]);
          s->left_raw = mp_signed_int16_be(data[4], data[5]);
          s->right_raw = mp_signed_int16_be(data[6], data[7]);
          speed_scale_mps = MP_V21_SPEED_SCALE_MPS;
          s->gear = mp_valid_gear((int)data[8]) ? (int)data[8] : -1;
          s->protocol = "2.3";
        } else if (length == 6 || length == 7) {
          s->x = data[0];
          s->y = data[1];
          s->left_raw = mp_signed_int16_be(data[2], data[3]);
          s->right_raw = mp_signed_int16_be(data[4], data[5]);
          speed_scale_mps = MP_V21_SPEED_SCALE_MPS;
          if (length == 7 && mp_valid_gear((int)data[6])) {
            s->gear = (int)data[6];
            s->protocol = "controller_feedback_gear";
          } else {
            s->protocol = "2.1";
          }
        } else if (length == 4) {
          s->x = data[0];
          s->y = data[1];
          s->left_raw = mp_signed_int8(data[2]);
          s->right_raw = mp_signed_int8(data[3]);
          speed_scale_mps = MP_LEGACY_SPEED_SCALE_MPS;
          s->protocol = "2.0";
        } else {
          s->x = -1;
          s->y = -1;
          s->left_raw = mp_signed_int8(data[0]);
          s->right_raw = mp_signed_int8(data[1]);
          speed_scale_mps = MP_LEGACY_SPEED_SCALE_MPS;
          s->protocol = "legacy_feedback";
        }
        s->gear_valid = (s->gear >= 1 && s->gear <= 5);
        s->left_mps = (double)s->left_raw * speed_scale_mps;
        s->right_mps = (double)s->right_raw * speed_scale_mps;
        memcpy(s->frame, frame, total);
        s->frame_len = total;
        s->time = ms_now_monotonic();
        count += 1;
      }
      w->rx_stats.status += 1;
      memmove(w->rx_buf, w->rx_buf + total, w->rx_len - total);
      w->rx_len -= total;
      continue;
    }

    w->rx_stats.other += 1;
    memmove(w->rx_buf, w->rx_buf + total, w->rx_len - total);
    w->rx_len -= total;
  }
  return count;
}

int ms_serial_writer_read_samples(
  ms_serial_writer_t *w, ms_sample_t out[MS_SAMPLE_MAX])
{
  if (w->dry_run || w->fd < 0) return 0;
#if !defined(_WIN32)
  /* 非阻塞读取（对齐 Python 的 os.read + BlockingIOError break） */
  for (;;) {
    uint8_t chunk[512];
    const ssize_t n = read(w->fd, chunk, sizeof(chunk));
    if (n < 0) {
      break; /* EAGAIN / EWOULDBLOCK / 其他错误均 break（对齐 C++） */
    }
    if (n == 0) break;
    w->rx_stats.bytes += (long)n;
    w->last_rx_time = ms_now_monotonic();
    {
      const size_t keep = (size_t)n > 32 ? (size_t)n - 32 : 0;
      const size_t preview_len = (size_t)n - keep;
      memcpy(w->last_rx_preview, chunk + keep, preview_len);
      w->last_rx_preview_len = preview_len;
    }
    if (w->rx_len + (size_t)n <= MS_RX_BUF_MAX) {
      memcpy(w->rx_buf + w->rx_len, chunk, (size_t)n);
      w->rx_len += (size_t)n;
    }
    /* 缓冲满时丢弃新数据（C++ vector 无界；病态输入下才有差异，见头注释） */
  }
#endif
  return ms_parse_rx_buffer(w, out);
}

void ms_serial_writer_close(ms_serial_writer_t *w)
{
#if !defined(_WIN32)
  if (w->fd >= 0) {
    close(w->fd);
    w->fd = -1;
  }
#else
  w->fd = -1;
#endif
  port_lock_release(w);
}
