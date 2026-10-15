/*
 * 模块功能：YAML 子集解析器。
 * 只支持本项目 config/dwa_planner.yaml 的结构：
 *   <node_name>:
 *     ros__parameters:
 *       key: value            # 注释
 *       key: [a, b, c]        # 标量流式列表
 * 两层缩进 key: value 标量；'#' 后为注释（含 " #" 行内注释）。
 * 字段名与 yaml 完全一致。
 */
#ifndef DWA_YAML_H
#define DWA_YAML_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DWA_YAML_MAX_ENTRIES 512
#define DWA_YAML_MAX_KEY 96
#define DWA_YAML_MAX_VALUE 256

typedef struct {
  char key[DWA_YAML_MAX_KEY];
  char value[DWA_YAML_MAX_VALUE];
} dwa_yaml_entry_t;

typedef struct {
  dwa_yaml_entry_t entries[DWA_YAML_MAX_ENTRIES];
  size_t count;
  char section[DWA_YAML_MAX_KEY]; /* 最近一次命中的顶层节点名，未命中为空 */
} dwa_yaml_doc_t;

/*
 * 解析文件中 target_node 节点 ros__parameters 下的 key: value。
 * 返回解析到的条目数；文件打不开返回 -1。
 * 未列在 target_node 下的其他顶层节点被忽略。
 */
int dwa_yaml_load_file(const char *path, const char *target_node, dwa_yaml_doc_t *doc);

/* 查找 key；找不到返回 NULL。 */
const char *dwa_yaml_get(const dwa_yaml_doc_t *doc, const char *key);

/* 带默认值的类型化读取：key 存在则解析覆盖 default。 */
double dwa_yaml_get_double(const dwa_yaml_doc_t *doc, const char *key, double def);
int dwa_yaml_get_int(const dwa_yaml_doc_t *doc, const char *key, int def);
bool dwa_yaml_get_bool(const dwa_yaml_doc_t *doc, const char *key, bool def);
/* 字符串拷贝到 out（容量 cap），返回是否命中。 */
bool dwa_yaml_get_string(
  const dwa_yaml_doc_t *doc, const char *key, char *out, size_t cap);
/* 解析 "[a, b, c]" 浮点列表；返回写入个数（最多 max_out）。 */
int dwa_yaml_get_double_array(
  const dwa_yaml_doc_t *doc, const char *key, double *out, int max_out);

#ifdef __cplusplus
}
#endif

#endif /* DWA_YAML_H */
