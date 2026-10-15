add_library(dwa_hal STATIC
  hal_time.c
  hal_log.c
  hal_mutex.c
)
target_include_directories(dwa_hal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
