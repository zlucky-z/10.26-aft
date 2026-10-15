/*
 * 模块功能：POSIX串口和超时控制接口声明。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#ifndef NODE_SERIAL_H
#define NODE_SERIAL_H

#include "lidar_information.h"
#include <stdint.h>
#include <vector>

#include <string>
#include <atomic>
#include <limits>
#include <termios.h>

using namespace std;

class MillisecondTimer
{
public:
  // 创建一个以当前时间为起点、持续指定毫秒数的计时器。
  explicit MillisecondTimer(const uint64_t millis);
  // 返回计时器剩余毫秒数，超时后返回非正值。
  int64_t remaining();

private:
  // 读取单调时钟，用于计算剩余时间。
  static timespec timespec_now();
  timespec expiry;
};

struct Timeout
{
  // 功能：执行`max`，负责本模块对应的max处理。
  static uint32_t max()
  {
    return std::numeric_limits<uint32_t>::max();
  }
  
  // 功能：执行`simpleTimeout`，负责本模块对应的simple、timeout处理。
  static Timeout simpleTimeout(uint32_t timeout_t)
  {
    return Timeout(max(), timeout_t, 0, timeout_t, 0);
  }

  uint32_t inter_byte_timeout;
  uint32_t read_timeout_constant;

  uint32_t read_timeout_multiplier;
  uint32_t write_timeout_constant;
 
  uint32_t write_timeout_multiplier;

  // 功能：执行`Timeout`，负责本模块对应的timeout处理。
  explicit Timeout(uint32_t inter_byte_timeout_ = 0,
                   uint32_t read_timeout_constant_ = 0,
                   uint32_t read_timeout_multiplier_ = 0,
                   uint32_t write_timeout_constant_ = 0,
                   uint32_t write_timeout_multiplier_ = 0)
      : inter_byte_timeout(inter_byte_timeout_),
        read_timeout_constant(read_timeout_constant_),
        read_timeout_multiplier(read_timeout_multiplier_),
        write_timeout_constant(write_timeout_constant_),
        write_timeout_multiplier(write_timeout_multiplier_)
  {
  }
};


typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

class Serial_Port
{
private:
  string port_;
  
  unsigned long baudrate_;    // Baudrate
  int fd_;
  pid_t pid;
  bool is_open_ = false;
  uint32_t byte_time_ns_; // Nanoseconds to transmit/receive a single byte

  Timeout timeout_;           // Timeout for read operations

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

public:
  // 配置串口路径、波特率、超时、数据位、校验位和停止位。
  Serial_Port(const std::string &port = "",
                  uint32_t baudrate = 115200,
                  Timeout timeout = Timeout(),
                  bytesize_t bytesize = eightbits,
                  parity_t parity = parity_none,
                  stopbits_t stopbits = stopbits_one,
                  flowcontrol_t flowcontrol = flowcontrol_none);

  // 关闭串口并释放相关系统资源。
  ~Serial_Port();
  // 打开串口并应用当前通信参数。
  bool open();
  // 关闭已打开的串口。
  void close();
  // 读取当前串口 termios 配置。
  bool getTermios(termios *tio);

  /*设置数据位数*/
  void set_databits(termios *tio, bytesize_t databits);

  /*设置奇偶性*/
  void set_parity(termios *tio, parity_t parity);

  /*设置停止位*/
  void set_stopbits(termios *tio, stopbits_t stopbits);

  /*流量控制*/
  void set_flowcontrol(termios *tio,flowcontrol_t flowcontrol);

  // 设置原始模式、流控和本地连接等通用串口属性。
  void set_common_props(termios *tio);

  
  // 从串口读取指定数量的数据。
  result_t read_data(uint8_t *buf, size_t size);
  // 向串口写入完整数据帧。
  size_t write_data(const uint8_t *data, size_t length);
  // 等待串口可读并返回实际可用字节数。
  result_t waitForData(size_t data_count, uint64_t timeout, size_t *returned_size);
  // 返回当前输入缓冲区中可读取的字节数。
  size_t available();
  // 设置或清除串口DTR信号线。
  bool setDTR(bool level);

  /*获取串口传输一个byte时间*/
  uint32_t getByteTime();

  // 修改串口波特率并同步更新每字节传输时间。
  bool setBaudrate(unsigned long baudrate);
  // 写入完整 termios 配置并刷新串口状态。
  bool setTermios(const termios *tio);
  // 设置系统标准表之外的自定义波特率。
  bool setCustomBaudRate(unsigned long baudrate);
  // 在指定超时时间内等待串口变为可读。
  bool waitReadable(uint32_t timeout_t);



};

#endif
