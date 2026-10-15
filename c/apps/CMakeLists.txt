add_executable(dwa_main dwa_main.c)
target_link_libraries(dwa_main PRIVATE dwa_runtime m)

# P7：电机桥入口（Windows 宿主机可构建：串口路径运行时才需要）
# m：POSIX 下数学库须显式链接（Windows MinGW 自动链，曾掩盖此问题）
add_executable(bridge_main bridge_main.c)
target_link_libraries(bridge_main PRIVATE dwa_motor m)

# P9：单进程整机入口（雷达×2 + 滤波×2 + 融合 + DWA + 电机桥）。
# 仅 POSIX（cspc SDK 双实例 + termios 串口 + pthread）。
if(NOT WIN32)
  add_executable(robot_main robot_main.c)
  target_link_libraries(robot_main PRIVATE
    dwa_runtime
    dwa_perception
    dwa_config
    dwa_motor
    dwa_drivers
    pthread
    m)
endif()
