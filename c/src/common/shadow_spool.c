/* shadow_spool 实现：spool 文件加载（P8 回放读取侧）。 */
#include "shadow_spool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

long shadow_spool_load(
  const char *path,
  shadow_rec_t **out_recs,
  uint8_t **out_buf,
  size_t *out_buf_len,
  double *out_base_wall_sec)
{
  FILE *f;
  shadow_file_hdr_t fhdr;
  uint8_t *buf = NULL;
  shadow_rec_t *recs = NULL;
  size_t len, pos;
  long count = 0;

  *out_recs = NULL;
  *out_buf = NULL;
  if (out_buf_len != NULL) *out_buf_len = 0;

  f = fopen(path, "rb");
  if (f == NULL) return -1;
  if (fread(&fhdr, sizeof(fhdr), 1, f) != 1 ||
      memcmp(fhdr.magic, SHADOW_SPOOL_MAGIC, SHADOW_SPOOL_MAGIC_LEN) != 0) {
    fclose(f);
    return -1;
  }
  if (out_base_wall_sec != NULL) *out_base_wall_sec = fhdr.base_wall_sec;

  /* 其余整包读入。 */
  {
    long file_size;
    long hdr_size = (long)sizeof(fhdr);
    fseek(f, 0, SEEK_END);
    file_size = ftell(f);
    fseek(f, hdr_size, SEEK_SET);
    if (file_size < hdr_size) {
      fclose(f);
      return -1;
    }
    len = (size_t)(file_size - hdr_size);
    buf = (uint8_t *)malloc(len > 0 ? len : 1);
    if (buf == NULL || (len > 0 && fread(buf, 1, len, f) != len)) {
      free(buf);
      fclose(f);
      return -1;
    }
  }
  fclose(f);

  /* 第一遍：数记录数。 */
  pos = 0;
  while (pos + sizeof(shadow_rec_hdr_t) <= len) {
    shadow_rec_hdr_t hdr;
    memcpy(&hdr, buf + pos, sizeof(hdr));
    pos += sizeof(hdr);
    if (pos + hdr.payload_len > len) break; /* 截尾：容忍录制被杀 */
    pos += hdr.payload_len;
    ++count;
  }

  recs = (shadow_rec_t *)malloc(sizeof(shadow_rec_t) * (size_t)(count > 0 ? count : 1));
  if (recs == NULL) {
    free(buf);
    return -1;
  }

  /* 第二遍：填视图。 */
  {
    long i = 0;
    pos = 0;
    while (pos + sizeof(shadow_rec_hdr_t) <= len && i < count) {
      shadow_rec_hdr_t hdr;
      memcpy(&hdr, buf + pos, sizeof(hdr));
      pos += sizeof(hdr);
      if (pos + hdr.payload_len > len) break;
      recs[i].rel_t = hdr.rel_t;
      recs[i].topic = hdr.topic;
      recs[i].payload = buf + pos;
      recs[i].payload_len = hdr.payload_len;
      pos += hdr.payload_len;
      ++i;
    }
    count = i;
  }

  *out_recs = recs;
  *out_buf = buf;
  if (out_buf_len != NULL) *out_buf_len = len;
  return count;
}

/* ---- 流式跟随读取（live 影子） ---- */

int shadow_spool_stream_open(const char *path, shadow_spool_stream_t *st)
{
  shadow_file_hdr_t fhdr;

  memset(st, 0, sizeof(*st));
  st->file = fopen(path, "rb");
  if (st->file == NULL) return -1;
  if (fread(&fhdr, sizeof(fhdr), 1, st->file) != 1 ||
      memcmp(fhdr.magic, SHADOW_SPOOL_MAGIC, SHADOW_SPOOL_MAGIC_LEN) != 0) {
    fclose(st->file);
    st->file = NULL;
    return -1;
  }
  st->base_wall_sec = fhdr.base_wall_sec;
  return 0;
}

long shadow_spool_stream_poll(shadow_spool_stream_t *st)
{
  uint8_t chunk[65536];
  long added = 0;

  /* 增量读入追加字节（读到当前 EOF 为止）。 */
  clearerr(st->file);
  for (;;) {
    const size_t got = fread(chunk, 1, sizeof(chunk), st->file);
    if (got > 0) {
      uint8_t *grown = (uint8_t *)realloc(st->buf, st->buf_len + got);
      if (grown == NULL) {
        /* OOM：回退文件偏移，本批放弃（下次 poll 重读）。 */
        fseek(st->file, -(long)got, SEEK_CUR);
        break;
      }
      st->buf = grown;
      memcpy(st->buf + st->buf_len, chunk, got);
      st->buf_len += got;
    }
    if (got < sizeof(chunk)) break;
  }

  /* 解析新增完整记录（尾部不完整记录留待下次）。 */
  while (st->parse_pos + sizeof(shadow_rec_hdr_t) <= st->buf_len) {
    shadow_rec_hdr_t hdr;
    memcpy(&hdr, st->buf + st->parse_pos, sizeof(hdr));
    if ((size_t)hdr.payload_len >
        st->buf_len - st->parse_pos - sizeof(shadow_rec_hdr_t)) {
      break;
    }
    if (st->count == st->cap) {
      const long next = st->cap > 0 ? st->cap * 2 : 256;
      shadow_rec_t *grown = (shadow_rec_t *)realloc(
        st->recs, (size_t)next * sizeof(*grown));
      if (grown == NULL) break;
      st->recs = grown;
      st->cap = next;
    }
    st->recs[st->count].rel_t = hdr.rel_t;
    st->recs[st->count].topic = hdr.topic;
    st->recs[st->count].payload = st->buf + st->parse_pos + sizeof(shadow_rec_hdr_t);
    st->recs[st->count].payload_len = hdr.payload_len;
    ++st->count;
    st->parse_pos += sizeof(shadow_rec_hdr_t) + (size_t)hdr.payload_len;
    ++added;
  }
  return added;
}

void shadow_spool_stream_close(shadow_spool_stream_t *st)
{
  if (st->file != NULL) fclose(st->file);
  free(st->buf);
  free(st->recs);
  memset(st, 0, sizeof(*st));
}
