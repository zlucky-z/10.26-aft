#!/bin/sh
# C 版单进程整机启动：双雷达 + 滤波/融合 + DWA + C 电机桥。
# 本脚本不启动旧 ROS/C++ 链路，也不负责远程部署。
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=${PROJECT_DIR:-$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)}
BIN=${ROBOT_MAIN:-${PROJECT_DIR}/bin/robot_main}
PLANNER_YAML=${PLANNER_YAML:-${PROJECT_DIR}/config/dwa_planner.yaml}
FILTER_YAML=${FILTER_YAML:-${PROJECT_DIR}/config/lidar_filter.yaml}
LOG_DIR=${LOG_DIR:-${PROJECT_DIR}/logs}

mkdir -p "${LOG_DIR}"
LOG="${LOG_DIR}/robot_$(date +%Y%m%d_%H%M%S).log"

if [ ! -x "${BIN}" ]; then
  echo "missing executable: ${BIN}" >&2
  exit 1
fi
if [ ! -f "${PLANNER_YAML}" ]; then
  echo "missing planner config: ${PLANNER_YAML}" >&2
  exit 1
fi
if [ ! -f "${FILTER_YAML}" ]; then
  echo "missing filter config: ${FILTER_YAML}" >&2
  exit 1
fi

exec "${BIN}" "${PLANNER_YAML}" --filter-yaml "${FILTER_YAML}" 2>&1 | tee "${LOG}"
