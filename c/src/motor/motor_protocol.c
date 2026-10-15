/*
 * 模块功能：motor_protocol.h 对应实现（motor_protocol.hpp 直译）。
 */
#include "motor_protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* —— CRC8（对齐 Python crc8）—— */
uint8_t mp_crc8(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xFF;
  size_t i;
  int bit;
  for (i = 0; i < size; ++i) {
    const uint8_t value = data[i];
    crc ^= value;
    for (bit = 0; bit < 8; ++bit) {
      crc = (uint16_t)((crc & 0x80) ? ((crc << 1) ^ 0x11D) : (crc << 1));
      crc &= 0x1FF;
    }
  }
  return (uint8_t)((~crc) & 0xFF);
}

/* —— 银行家舍入（复刻 Python round()，round half to even）—— */
long mp_py_round(double value)
{
  const double floor_value = floor(value);
  const double frac = value - floor_value;
  if (frac < 0.5) return (long)floor_value;
  if (frac > 0.5) return (long)floor_value + 1;
  const long lower = (long)floor_value;
  return (lower % 2 == 0) ? lower : lower + 1;
}

int mp_clamp_coord(double value)
{
  const long rounded = mp_py_round(value);
  if (rounded < 0) return 0;
  if (rounded > 200) return 200;
  return (int)rounded;
}

double mp_clamp_double(double value, double low, double high)
{
  /* std::max(low, std::min(high, value)) */
  double v = value;
  if (v > high) v = high;
  if (v < low) v = low;
  return v;
}

int mp_logical_coord_to_protocol(double value)
{
  const int logical = mp_clamp_coord(value);
  const double ratio = (double)logical / 200.0;
  return (int)mp_py_round(
    (double)MP_PROTOCOL_AXIS_MIN +
    ratio * (double)(MP_PROTOCOL_AXIS_MAX - MP_PROTOCOL_AXIS_MIN));
}

int mp_protocol_coord_to_logical(uint8_t high, uint8_t low)
{
  const int raw = ((int)high << 8) | (int)low;
  const double ratio = (double)(raw - MP_PROTOCOL_AXIS_MIN) /
    (double)(MP_PROTOCOL_AXIS_MAX - MP_PROTOCOL_AXIS_MIN);
  return mp_clamp_coord(ratio * 200.0);
}

int mp_signed_int8(uint8_t value)
{
  return value >= 128 ? (int)value - 256 : (int)value;
}

int mp_signed_int16_be(uint8_t high, uint8_t low)
{
  const int value = ((int)high << 8) | (int)low;
  return (value & 0x8000) ? value - 0x10000 : value;
}

int mp_valid_gear(int gear)
{
  return gear >= 1 && gear <= 5;
}

uint8_t mp_pack_gear_horn(int gear, int horn)
{
  const int safe_gear = mp_valid_gear(gear) ? gear : 1;
  return (uint8_t)(((safe_gear & 0x0F) << 4) | (horn ? 0x01 : 0x00));
}

/* append_uint16_be：对齐 Python，写两个字节 {高, 低}。 */
static void append_uint16_be(mp_frame_t *out, int value)
{
  int clamped = value;
  if (clamped < 0) clamped = 0;
  if (clamped > 0xFFFF) clamped = 0xFFFF;
  out->bytes[out->len++] = (uint8_t)((clamped >> 8) & 0xFF);
  out->bytes[out->len++] = (uint8_t)(clamped & 0xFF);
}

mp_frame_t mp_build_frame(
  int x,
  int y,
  int radar_status,
  int gyro_angle,
  int gear,
  int horn,
  mp_protocol_t protocol)
{
  mp_frame_t data;
  mp_frame_t frame;
  data.len = 0;

  if (protocol == MP_PROTOCOL_23) {
    append_uint16_be(&data, mp_logical_coord_to_protocol((double)x));
    append_uint16_be(&data, mp_logical_coord_to_protocol((double)y));
    data.bytes[data.len++] = (uint8_t)(radar_status & 0x03);
    data.bytes[data.len++] = (uint8_t)(gyro_angle & 0xFF);
    data.bytes[data.len++] = mp_pack_gear_horn(gear, 0);
  } else {
    data.bytes[data.len++] = (uint8_t)mp_clamp_coord((double)x);
    data.bytes[data.len++] = (uint8_t)mp_clamp_coord((double)y);
    {
      const int extended =
        (protocol == MP_PROTOCOL_21 || protocol == MP_PROTOCOL_GEAR_HORN);
      if (extended) {
        data.bytes[data.len++] = (uint8_t)(radar_status & 0x03);
        data.bytes[data.len++] = (uint8_t)(gyro_angle & 0xFF);
      }
      if (protocol == MP_PROTOCOL_GEAR_HORN) {
        data.bytes[data.len++] = mp_pack_gear_horn(gear, horn);
      }
    }
  }

  frame.len = 0;
  frame.bytes[frame.len++] = (uint8_t)MP_SOP;
  frame.bytes[frame.len++] = (uint8_t)MP_PACKET_VERSION;
  frame.bytes[frame.len++] = (uint8_t)MP_COMMAND_DRIVE;
  frame.bytes[frame.len++] = (uint8_t)data.len;
  frame.bytes[frame.len++] = mp_crc8(data.bytes, data.len);
  memcpy(frame.bytes + frame.len, data.bytes, data.len);
  frame.len += data.len;
  return frame;
}

void mp_frame_hex(const mp_frame_t *frame, char out[MP_FRAME_HEX_MAX])
{
  static const char hex[] = "0123456789ABCDEF";
  size_t i;
  size_t pos = 0;
  for (i = 0; i < frame->len; ++i) {
    const uint8_t b = frame->bytes[i];
    if (i != 0) out[pos++] = ' ';
    out[pos++] = hex[(b >> 4) & 0x0F];
    out[pos++] = hex[b & 0x0F];
  }
  out[pos] = '\0';
}

void mp_bytes_hex(
  const uint8_t *data, size_t size, size_t limit, char out[MP_FRAME_HEX_MAX])
{
  static const char hex[] = "0123456789ABCDEF";
  size_t i;
  size_t pos = 0;
  const size_t n = limit < size ? limit : size;
  for (i = 0; i < n; ++i) {
    const uint8_t b = data[i];
    if (i != 0) out[pos++] = ' ';
    out[pos++] = hex[(b >> 4) & 0x0F];
    out[pos++] = hex[b & 0x0F];
  }
  out[pos] = '\0';
}

int mp_frame_equal(const mp_frame_t *a, const mp_frame_t *b)
{
  if (a->len != b->len) return 0;
  return memcmp(a->bytes, b->bytes, a->len) == 0;
}
