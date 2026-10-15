add_library(dwa_config STATIC
  dwa_yaml.c
  dwa_params.c
  perception_params.c
)
target_include_directories(dwa_config PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
)
target_link_libraries(dwa_config PUBLIC dwa_common)
