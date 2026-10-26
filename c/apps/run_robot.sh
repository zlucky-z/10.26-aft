#!/bin/sh
# C 版单进程整机启动：双雷达 + 滤波/融合 + DWA + C 电机桥。
# 本脚本不启动旧 ROS/C++ 链路，也不负责远程部署。
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -z "${PROJECT_DIR:-}" ]; then
  if [ -x "${SCRIPT_DIR}/robot_main" ]; then
    # Flat board deployment: robot_main, config, and logs share one directory.
    PROJECT_DIR=${SCRIPT_DIR}
  else
    # Source-tree layout: c/apps/run_robot.sh -> project root.
    PROJECT_DIR=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
  fi
fi
BIN=${ROBOT_MAIN:-${PROJECT_DIR}/robot_main}
PLANNER_YAML=${PLANNER_YAML:-${PROJECT_DIR}/config/dwa_planner.yaml}
FILTER_YAML=${FILTER_YAML:-${PROJECT_DIR}/config/lidar_filter.yaml}
LOG_DIR=${LOG_DIR:-${PROJECT_DIR}/logs}

mkdir -p "${LOG_DIR}"
LOG="${LOG_DIR}/robot_$(date +%Y%m%d_%H%M%S).log"
TRACE_STEM=${LOG%.log}
export RM_DEBUG_SPEED=${RM_DEBUG_SPEED:-1}
export RM_CONTROL_TRACE=${RM_CONTROL_TRACE:-${TRACE_STEM}.control.csv}
export RM_POINTCLOUD_TRACE=${RM_POINTCLOUD_TRACE:-${TRACE_STEM}.fused_cloud.bin}
export RM_POINTCLOUD_TRACE_MAX_MB=${RM_POINTCLOUD_TRACE_MAX_MB:-32}

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
