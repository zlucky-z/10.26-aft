add_library(dwa_planning STATIC
  dwa_trajectory.c
  dwa_collision.c
  dwa_scoring.c
  dwa_sampling.c
)
target_include_directories(dwa_planning PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src/common
)
target_link_libraries(dwa_planning PUBLIC dwa_common)
# dwa_sampling.c 恢复 pthread fork-join 并行；静态库符号解析要求
# -lpthread 排在使用方链接行的 libdwa_planning.a 之后，挂在库本身最稳。
target_link_libraries(dwa_planning PUBLIC pthread)
