add_library(dwa_common STATIC
  dwa_types.c
  dwa_geometry.c
  shadow_spool.c
)
target_include_directories(dwa_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
