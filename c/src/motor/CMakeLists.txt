# c/src/motor —— 电机桥（P7）
add_library(dwa_motor STATIC
  motor_protocol.c
  motor_control_profiles.c
  stop_escape_compensator.c
  motor_serial.c
  motor_bridge_config.c
  motor_bridge.c
  yaw_feedback.c
)
target_include_directories(dwa_motor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/hal
)
target_link_libraries(dwa_motor PUBLIC dwa_hal)
