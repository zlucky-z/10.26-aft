add_library(dwa_safety STATIC
  side_wall_hold_control.c
  escape_controller.c
  front_safety.c
  side_safety.c
)
target_include_directories(dwa_safety PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/runtime
)
target_link_libraries(dwa_safety PUBLIC dwa_common)
