/*
 * 模块功能：电机串口协议常量、CRC8、银行家舍入、坐标换算、帧封装。
 * 直译自 motor_bridge_cpp/native/motor_protocol.hpp（namespace
 * hu_m40_motor_protocol → mp_ 前缀函数）。保持 double 运算：
 * P7 验收标准是"相同 cmd 序列输出相同串口帧字节"，py_round/clamp_coord
 * 的 0.5 刀锋对精度敏感，不做 float 化（见 TRANSLATION_NOTES P7 记录）。
 */
#ifndef DWA_MOTOR_PROTOCOL_H
#define DWA_MOTOR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* —— 协议常量（对齐 Python motor_protocol.py）—— */
#define MP_SOP 0xA5
#define MP_PACKET_VERSION 0x01
#define MP_COMMAND_DRIVE 0x81
#define MP_COMMAND_STATUS 0x88
#define MP_CENTER 100
#define MP_LEGACY_SPEED_SCALE_MPS (100.0 / 3600.0)
/* Field calibration: 2.3 feedback was about 2x too low with 1/3600 at
 * verified 5th-gear speed. */
#define MP_V21_SPEED_SCALE_MPS (1.0 / 3600.0)
#define MP_PROTOCOL_AXIS_MIN 250
#define MP_PROTOCOL_AXIS_CENTER 2048
#define MP_PROTOCOL_AXIS_MAX 3846

/* 霍尔控制模式（对齐 Python 常量） */
#define MP_HALL_MODE_MOVING_PREBRAKE 1
#define MP_HALL_MODE_MOVING_HOLD 2
#define MP_HALL_MODE_MOVING_DWA 3
#define MP_HALL_MODE_STATIC_DYNAMIC_DWA 4
#define MP_HALL_MODE_SIDE_CAP 5

/* Drive-frame sensor status byte (2.1/2.3/gear_horn):
 * bit0=left radar stale, bit1=right radar stale, bit2=IMU unhealthy. */
#define MP_SENSOR_STATUS_LEFT_RADAR_FAULT 0x01
#define MP_SENSOR_STATUS_RIGHT_RADAR_FAULT 0x02
#define MP_SENSOR_STATUS_IMU_FAULT 0x04
#define MP_SENSOR_STATUS_MASK 0x07

/* 协议版本（C++ 侧用字符串匹配，此处枚举；config 解析时字符串→枚举） */
typedef enum {
  MP_PROTOCOL_20 = 0,      /* "2.0" */
  MP_PROTOCOL_21 = 1,      /* "2.1" */
  MP_PROTOCOL_23 = 2,      /* "2.3" */
  MP_PROTOCOL_GEAR_HORN = 3 /* "gear_horn" */
} mp_protocol_t;

/* 帧最大长度：header 5 + data 最大 7（2.3：2+2+1+1+1）＝ 12 */
#define MP_FRAME_MAX 16

typedef struct {
  uint8_t bytes[MP_FRAME_MAX];
  size_t len;
} mp_frame_t;

/* 帧十六进制文本上限："XX "×16 + NUL ≈ 50 */
#define MP_FRAME_HEX_MAX 64

/* —— CRC8（对齐 Python crc8）—— */
uint8_t mp_crc8(const uint8_t *data, size_t size);

/* —— 银行家舍入（复刻 Python round()，round half to even）—— */
long mp_py_round(double value);

/* clamp_coord：对齐 Python `max(0, min(200, int(round(value))))`。 */
int mp_clamp_coord(double value);

/* clamp：对齐 Python clamp(value, low, high)。 */
double mp_clamp_double(double value, double low, double high);

/* logical_coord_to_protocol：0..200 -> 250..3846（银行家舍入）。 */
int mp_logical_coord_to_protocol(double value);

/* protocol_coord_to_logical：250..3846 -> 0..200（银行家舍入）。 */
int mp_protocol_coord_to_logical(uint8_t high, uint8_t low);

/* int8：无符号字节 -> 有符号 8 位。 */
int mp_signed_int8(uint8_t value);

/* int16_be：大端两字节 -> 有符号 16 位。 */
int mp_signed_int16_be(uint8_t high, uint8_t low);

/* valid_gear：1..5。 */
int mp_valid_gear(int gear);

/* pack_gear_horn：对齐 Python `((safe_gear & 0x0F) << 4) | (horn ? 1 : 0)`。 */
uint8_t mp_pack_gear_horn(int gear, int horn);

/*
 * build_frame：对齐 Python build_frame(x,y,radar_status,gyro_angle,gear,horn,
 * protocol)。返回完整帧（含 5 字节头）。等价于 C++ 版按字符串匹配协议
 * 分支的枚举直译。
 */
mp_frame_t mp_build_frame(
  int x,
  int y,
  int radar_status,
  int gyro_angle,
  int gear,
  int horn,
  mp_protocol_t protocol);

/* frame_hex：帧 → "A5 01 81 ..." 大写十六进制文本（日志格式化）。 */
void mp_frame_hex(const mp_frame_t *frame, char out[MP_FRAME_HEX_MAX]);

/* bytes_hex：字节前缀 → 十六进制文本（诊断用）。 */
void mp_bytes_hex(
  const uint8_t *data, size_t size, size_t limit, char out[MP_FRAME_HEX_MAX]);

/* 帧相等（对应 C++ vector<uint8_t> 的 ==）。 */
int mp_frame_equal(const mp_frame_t *a, const mp_frame_t *b);

#ifdef __cplusplus
}
#endif

#endif /* DWA_MOTOR_PROTOCOL_H */
