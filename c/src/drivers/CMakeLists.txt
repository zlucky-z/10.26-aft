# c/src/drivers —— 外设驱动适配层（P7）
# cspc_lidar SDK core 为 C++（方案允许的唯一例外）；仅 POSIX 构建
# （serial_port 依赖 termios），Windows 宿主机跳过。
if(NOT WIN32)
  add_library(cspc_sdk STATIC
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/calibration.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/lidar_data_processing.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/lidar_information.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/mtime.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/node_lidar.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/point_cloud_optimize.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/serial_port.cpp
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk/timer.cpp
  )
  target_include_directories(cspc_sdk PUBLIC
    ${CMAKE_SOURCE_DIR}/../third_party/cspc_lidar_sdk_ros2/sdk
  )
  # SDK 为旧式 C++（默认成员初始化 + STL），需 C++11；不强加其余工程选项
  target_compile_features(cspc_sdk PRIVATE cxx_std_11)
  # 第三方 SDK 不受工程警告策略约束（gcc 9.4 交叉工具链触发
  # -Wmaybe-uninitialized 等旧代码警告；-w 压制全部警告含 -Werror=format）
  target_compile_options(cspc_sdk PRIVATE -w)

  add_library(dwa_drivers STATIC
    cspc_lidar_adapter.cpp
    cspc_lidar_pair.cpp
    bmi270.c
  )
  # pair 内部以命名空间复制 SDK 源码（预包含技巧，见 cspc_lidar_pair.cpp
  # 注释），同样按第三方警告策略 -w 压制（gcc 9.4 旧代码警告）。
  set_source_files_properties(cspc_lidar_pair.cpp PROPERTIES COMPILE_OPTIONS "-w")
  target_include_directories(dwa_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/hal
  )
  target_link_libraries(dwa_drivers PUBLIC cspc_sdk dwa_hal)
endif()
