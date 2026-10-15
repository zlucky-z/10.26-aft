add_library(dwa_runtime STATIC
  dwa_node.c
  dwa_node_tick.c
  dwa_node_safety.c
  dwa_node_perception.c
)
target_include_directories(dwa_runtime PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/config
  ${CMAKE_SOURCE_DIR}/src/planning
  ${CMAKE_SOURCE_DIR}/src/safety
  ${CMAKE_SOURCE_DIR}/src/guidance
  ${CMAKE_SOURCE_DIR}/src/fusion
  ${CMAKE_SOURCE_DIR}/src/perception
  ${CMAKE_SOURCE_DIR}/src/hal
)
target_link_libraries(dwa_runtime PUBLIC
  dwa_common
  dwa_config
  dwa_planning
  dwa_safety
  dwa_guidance
  dwa_fusion
  dwa_perception
  dwa_hal
  m
)
