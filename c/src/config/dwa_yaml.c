/* YAML 子集解析器实现：见 dwa_yaml.h。 */
#include "dwa_yaml.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static void trim(char *s)
{
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' ||
    s[len - 1] == '\r' || s[len - 1] == '\n')) {
    s[--len] = '\0';
  }
  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\t') ++start;
  if (start > 0) memmove(s, s + start, len - start + 1);
}

static void strip_inline_comment(char *s)
{
  /* 只剥 " #"（yaml 行内注释前必有空格），避免误伤值中的 '#'。 */
  for (char *p = s; *p != '\0'; ++p) {
    if (p[0] == ' ' && (p[1] == '#' || p[1] == '\t')) {
      *p = '\0';
      return;
    }
  }
}

int dwa_yaml_load_file(const char *path, const char *target_node, dwa_yaml_doc_t *doc)
{
  memset(doc, 0, sizeof(*doc));
  FILE *fp = fopen(path, "rb");
  if (fp == NULL) return -1;

  char line[1024];
  bool in_target = false;
  bool in_params = false;
  while (fgets(line, sizeof(line), fp) != NULL) {
    char work[1024];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    /* 去行内注释（'#' 顶格整行注释也覆盖） */
    strip_inline_comment(work);
    trim(work);
    if (work[0] == '\0') continue;
    if (work[0] == '#') continue;

    const size_t indent = (line[0] == ' ') ? 1 : 0;
    if (indent == 0) {
      /* 顶层 "node_name:" */
      in_target = false;
      in_params = false;
      size_t n = strlen(work);
      if (n > 0 && work[n - 1] == ':') {
        work[n - 1] = '\0';
        if (strcmp(work, target_node) == 0) {
          in_target = true;
          strncpy(doc->section, work, DWA_YAML_MAX_KEY - 1);
        }
      }
      continue;
    }
    if (!in_target) continue;

    if (strncmp(work, "ros__parameters:", 16) == 0) {
      in_params = true;
      continue;
    }
    if (!in_params) continue;

    /* "key: value" */
    char *colon = strchr(work, ':');
    if (colon == NULL) continue;
    *colon = '\0';
    char key[DWA_YAML_MAX_KEY];
    strncpy(key, work, DWA_YAML_MAX_KEY - 1);
    key[DWA_YAML_MAX_KEY - 1] = '\0';
    trim(key);
    char value[DWA_YAML_MAX_VALUE];
    strncpy(value, colon + 1, DWA_YAML_MAX_VALUE - 1);
    value[DWA_YAML_MAX_VALUE - 1] = '\0';
    trim(value);
    /* 去掉成对引号（若将来 yaml 使用） */
    size_t vlen = strlen(value);
    if (vlen >= 2 && ((value[0] == '"' && value[vlen - 1] == '"') ||
      (value[0] == '\'' && value[vlen - 1] == '\''))) {
      value[vlen - 1] = '\0';
      memmove(value, value + 1, strlen(value) + 1);
    }

    if (key[0] == '\0') continue;
    if (doc->count >= DWA_YAML_MAX_ENTRIES) break;
    strncpy(doc->entries[doc->count].key, key, DWA_YAML_MAX_KEY - 1);
    doc->entries[doc->count].key[DWA_YAML_MAX_KEY - 1] = '\0';
    strncpy(doc->entries[doc->count].value, value, DWA_YAML_MAX_VALUE - 1);
    doc->entries[doc->count].value[DWA_YAML_MAX_VALUE - 1] = '\0';
    ++doc->count;
  }
  fclose(fp);
  return (int)doc->count;
}

const char *dwa_yaml_get(const dwa_yaml_doc_t *doc, const char *key)
{
  for (size_t i = 0; i < doc->count; ++i) {
    if (strcmp(doc->entries[i].key, key) == 0) return doc->entries[i].value;
  }
  return NULL;
}

double dwa_yaml_get_double(const dwa_yaml_doc_t *doc, const char *key, double def)
{
  const char *v = dwa_yaml_get(doc, key);
  if (v == NULL) return def;
  return strtod(v, NULL);
}

int dwa_yaml_get_int(const dwa_yaml_doc_t *doc, const char *key, int def)
{
  const char *v = dwa_yaml_get(doc, key);
  if (v == NULL) return def;
  return (int)strtol(v, NULL, 10);
}

bool dwa_yaml_get_bool(const dwa_yaml_doc_t *doc, const char *key, bool def)
{
  const char *v = dwa_yaml_get(doc, key);
  if (v == NULL) return def;
  if (strcmp(v, "true") == 0) return true;
  if (strcmp(v, "false") == 0) return false;
  return def;
}

bool dwa_yaml_get_string(
  const dwa_yaml_doc_t *doc, const char *key, char *out, size_t cap)
{
  const char *v = dwa_yaml_get(doc, key);
  if (v == NULL || cap == 0) return false;
  strncpy(out, v, cap - 1);
  out[cap - 1] = '\0';
  return true;
}

int dwa_yaml_get_double_array(
  const dwa_yaml_doc_t *doc, const char *key, double *out, int max_out)
{
  const char *v = dwa_yaml_get(doc, key);
  if (v == NULL) return 0;
  const char *p = v;
  if (*p == '[') ++p;
  int count = 0;
  while (*p != '\0' && *p != ']' && count < max_out) {
    char *end = NULL;
    double value = strtod(p, &end);
    if (end == p) {
      ++p;
      continue;
    }
    out[count++] = value;
    p = end;
    while (*p == ',' || *p == ' ' || *p == '\t') ++p;
  }
  return count;
}
