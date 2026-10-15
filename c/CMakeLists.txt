# DWA 纯 C 重构 —— 顶层构建
cmake_minimum_required(VERSION 3.10)
project(dwa_c C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# POSIX 板端的雷达供应商 SDK 和 C 适配层需要 C++；规划、感知、
# 安全、电机桥和应用逻辑仍全部由 C 源码实现。
if(NOT WIN32)
  enable_language(CXX)
endif()

# 浮点求值稳定，便于回归测试复现。
add_compile_options(-O2 -ffloat-store -Wall -Wextra -Werror=format)

add_subdirectory(src/common)
add_subdirectory(src/config)
add_subdirectory(src/planning)
add_subdirectory(src/safety)
add_subdirectory(src/guidance)
add_subdirectory(src/fusion)
add_subdirectory(src/perception)
add_subdirectory(src/hal)
add_subdirectory(src/runtime)
add_subdirectory(src/motor)
add_subdirectory(src/drivers)

# 宿主机 C 版回归测试；板端交叉编译时关闭。
option(DWA_C_ENABLE_HOST_TOOLS "Build C-only host regression tests" ON)
if(DWA_C_ENABLE_HOST_TOOLS)
  enable_testing()
  add_subdirectory(tests)
endif()
add_subdirectory(apps)
