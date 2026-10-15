#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P4.5 全库 double -> float 机械转换工具。

规则（铁律：逐行直译语义下的类型系统变换，不改逻辑）：
  1. double -> float（字面直译）
  2. 数学函数加 f 后缀（sin->sinf、atan2->atan2f、fabs->fabsf ...）
  3. DBL_EPSILON->FLT_EPSILON、DBL_MAX->FLT_MAX、DBL_MIN->FLT_MIN、HUGE_VAL->HUGE_VALF
  4. (double) 强转 -> (float)
  5. 浮点字面量加 f 后缀（杜绝隐式提升；跳过注释与字符串字面量）
  6. config/dwa_yaml.{c,h} 排除：YAML 文本解析是 I/O 边界，保持 double 精度，
     最终赋值到 float 参数字段时才舍入（与 C++ 基线 q() 量化路径一致）。
  7. c/tests/、c/tools/ 不处理：harness 的 LCG 与场景生成保持 double，
     在赋值进 float 字段时隐式舍入（= C++ 侧 q() 显式量化）。

用法: python floatify_p45.py   （直接作用于 ../src）
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # c/tools -> c
SRC = os.path.join(ROOT, "src")
EXCLUDE = {"dwa_yaml.c", "dwa_yaml.h"}

MATH_FUNCS = sorted(
    ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'sinh', 'cosh', 'tanh',
     'exp', 'log', 'log10', 'pow', 'sqrt', 'cbrt', 'hypot', 'fmod', 'fabs',
     'floor', 'ceil', 'round', 'trunc', 'fmin', 'fmax', 'fdim', 'copysign'],
    key=len, reverse=True)
MATH_RE = re.compile(r'\b(' + '|'.join(MATH_FUNCS) + r')\s*\(')

# 仅匹配带小数点或指数的字面量；纯整数不后缀（保持 int 语义，如数组大小/计数字段）
LIT_RE = re.compile(
    r'(?<![\w.])('
    r'\d+\.\d*(?:[eE][+-]?\d+)?'
    r'|\.\d+(?:[eE][+-]?\d+)?'
    r'|\d+[eE][+-]?\d+'
    r')(?![\w.])')


def transform_code(code):
    code = re.sub(r'\bdouble\b', 'float', code)
    code = MATH_RE.sub(lambda m: m.group(1) + 'f(', code)
    for old, new in (('DBL_EPSILON', 'FLT_EPSILON'), ('DBL_MAX', 'FLT_MAX'),
                     ('DBL_MIN', 'FLT_MIN'), ('HUGE_VAL', 'HUGE_VALF')):
        code = re.sub(r'\b' + old + r'\b', new, code)
    code = code.replace('(double)', '(float)')
    code = LIT_RE.sub(lambda m: m.group(1) + 'f', code)
    return code


def process(text):
    """词法级变换：注释与字符串字面量原样保留，仅处理代码段。"""
    out = []
    i, n = 0, len(text)
    while i < n:
        two = text[i:i + 2]
        c = text[i]
        if two == '//':
            j = text.find('\n', i)
            j = n if j < 0 else j
            out.append(text[i:j])
            i = j
        elif two == '/*':
            j = text.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(text[i:j])
            i = j
        elif c == '"' or c == "'":
            j = i + 1
            while j < n and text[j] != c:
                if text[j] == '\\':
                    j += 1
                j += 1
            j = min(j + 1, n)
            out.append(text[i:j])
            i = j
        else:
            j = i
            while j < n:
                ch = text[j]
                if ch == '"' or ch == "'":
                    break
                if ch == '/' and text[j:j + 2] in ('//', '/*'):
                    break
                j += 1
            out.append(transform_code(text[i:j]))
            i = j
    return ''.join(out)


def main():
    changed = []
    for dirpath, _, files in os.walk(SRC):
        for fn in files:
            if fn in EXCLUDE or not (fn.endswith('.c') or fn.endswith('.h')):
                continue
            path = os.path.join(dirpath, fn)
            with open(path, 'r', encoding='utf-8', newline='') as f:
                original = f.read()
            converted = process(original)
            if converted != original:
                with open(path, 'w', encoding='utf-8', newline='') as f:
                    f.write(converted)
                diff_lines = sum(1 for a, b in
                                 zip(original.splitlines(), converted.splitlines())
                                 if a != b)
                changed.append((path.replace(SRC + os.sep, ''), diff_lines))
    total = 0
    for rel, cnt in sorted(changed):
        print('%5d  %s' % (cnt, rel))
        total += cnt
    print('files changed: %d, lines changed: %d' % (len(changed), total))


if __name__ == '__main__':
    main()
