add_library(dwa_guidance STATIC
  corridor_guide.c
  dwa_geometric_selector.c
)
target_include_directories(dwa_guidance PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/planning
)
target_link_libraries(dwa_guidance PUBLIC dwa_planning)
