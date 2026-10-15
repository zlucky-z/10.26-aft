add_library(dwa_fusion STATIC
  dynamic_velocity_fusion.c
)
target_include_directories(dwa_fusion PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
