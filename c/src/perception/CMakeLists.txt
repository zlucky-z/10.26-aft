add_library(dwa_perception STATIC
  laser_filter_core.c
  laser_scan_fusion.c
  local_occupancy_grid.c
)
target_include_directories(dwa_perception PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
)
