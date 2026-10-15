/*
 * 模块功能：影子回放 spool 文件格式（P8）。
 * shadow_recorder（板端 ROS2 旁路录制）写；dwa_main --replay 读。
 * 格式（本机小端，录放同架构 aarch64/x86_64 均为小端）：
 *   文件头 16 字节：magic[8]="DWARPL01" + double base_wall_sec（录制起点墙钟，仅参考）
 *   记录 = 头 16 字节 + payload：
 *     头：double rel_t（相对录制起点秒，steady 单调）；uint8 topic；uint8 pad[3]；uint32 payload_len
 *   payload 按 topic：
 *     1 fused_cloud     : uint32 count; count × {float x; float y; uint8 is_left; uint8 pad[3];}
 *     2 cmd_vel_raw     : double linear_x; double angular_z
 *     3 motor_feedback  : uint32 n; n × float
 *     4 odom            : double linear_x; double angular_z
 *     10 out_cmd        : double linear_x; double angular_z（C++ 原版输出，对拍基准）
 *     11 out_hall       : uint32 n; n × float（C++ 原版 hall 输出，对拍基准）
 */
#ifndef SHADOW_SPOOL_H
#define SHADOW_SPOOL_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHADOW_SPOOL_MAGIC "DWARPL01"
#define SHADOW_SPOOL_MAGIC_LEN 8

enum {
  SHADOW_TOPIC_FUSED_CLOUD = 1,
  SHADOW_TOPIC_CMD_VEL_RAW = 2,
  SHADOW_TOPIC_MOTOR_FEEDBACK = 3,
  SHADOW_TOPIC_ODOM = 4,
  SHADOW_TOPIC_OUT_CMD = 10,
  SHADOW_TOPIC_OUT_HALL = 11
};

/* 16 字节记录头（磁盘布局，勿改字段顺序）。 */
#pragma pack(push, 1)
typedef struct {
  double rel_t;
  uint8_t topic;
  uint8_t pad[3];
  uint32_t payload_len;
} shadow_rec_hdr_t;
#pragma pack(pop)

/* 12 字节点云点（磁盘布局）。 */
#pragma pack(push, 1)
typedef struct {
  float x;
  float y;
  uint8_t is_left;
  uint8_t pad[3];
} shadow_point_t;
#pragma pack(pop)

/* 文件头 16 字节。 */
#pragma pack(push, 1)
typedef struct {
  char magic[8];
  double base_wall_sec;
} shadow_file_hdr_t;
#pragma pack(pop)

/* 内存中的记录视图（payload 指向整包缓冲内部，不单独分配）。 */
typedef struct {
  double rel_t;
  uint8_t topic;
  const uint8_t *payload;
  uint32_t payload_len;
} shadow_rec_t;

/*
 * 读整个 spool 到内存。返回记录数组（*out_recs，malloc 分配，调用方 free），
 * payload 全部落在一个整包缓冲（*out_buf，调用方 free）。
 * 返回记录数；失败返回 -1（*out_recs、*out_buf 置 NULL）。
 */
long shadow_spool_load(
  const char *path,
  shadow_rec_t **out_recs,
  uint8_t **out_buf,
  size_t *out_buf_len,
  double *out_base_wall_sec);

/*
 * 流式跟随读取（live 影子，P8）：spool 由 recorder 持续追加时增量读取。
 * open：打开并校验文件头（文件须已存在且含 16 字节头）。
 * poll：读入追加字节，解析新增完整记录，返回本次新增条数（0=暂无新数据）。
 *   注意：recs 视图的 payload 指向下一次 poll 后可能失效（缓冲 realloc 搬迁），
 *   调用方须在下次 poll 前消费完新增记录。
 * close：释放全部资源。
 */
typedef struct {
  FILE *file;         /* 保持打开（共享读，不阻塞 recorder 追加写） */
  uint8_t *buf;       /* 文件头之后的全部已读字节 */
  size_t buf_len;
  size_t parse_pos;   /* 下一条未解析记录起点 */
  shadow_rec_t *recs; /* 已解析记录视图（append-only） */
  long count;
  long cap;
  double base_wall_sec;
} shadow_spool_stream_t;

int shadow_spool_stream_open(const char *path, shadow_spool_stream_t *st);
long shadow_spool_stream_poll(shadow_spool_stream_t *st);
void shadow_spool_stream_close(shadow_spool_stream_t *st);

#ifdef __cplusplus
}
#endif

#endif /* SHADOW_SPOOL_H */
