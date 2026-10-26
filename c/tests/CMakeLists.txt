add_executable(test_config test_config.c)
target_link_libraries(test_config dwa_config dwa_common)
add_test(NAME config COMMAND test_config
  ${CMAKE_SOURCE_DIR}/../config/dwa_planner.yaml)

add_executable(test_planning test_planning.c)
target_link_libraries(test_planning dwa_planning dwa_common m)
add_test(NAME planning COMMAND test_planning)

add_executable(test_front_turn_bias test_front_turn_bias.c)
target_link_libraries(test_front_turn_bias dwa_planning dwa_common m)
add_test(NAME front_turn_bias COMMAND test_front_turn_bias)

# C 版融合、通道、安全状态序列回归。
add_executable(test_p3p4 test_p3p4.c)
target_link_libraries(test_p3p4 dwa_fusion dwa_guidance dwa_safety dwa_planning dwa_common m)
add_test(NAME p3p4 COMMAND test_p3p4)

# C 版感知回归。
add_executable(test_p5 test_p5.c)
target_link_libraries(test_p5 dwa_perception dwa_planning dwa_common m)
add_test(NAME p5 COMMAND test_p5)

# C 版电机桥回归。
add_executable(test_p7 test_p7.c)
target_link_libraries(test_p7 dwa_motor m)
add_test(NAME p7 COMMAND test_p7)

# 前向安全参考面和通道侧墙回归。
add_executable(test_front_safety test_front_safety.c)
target_link_libraries(test_front_safety dwa_safety dwa_common m)
add_test(NAME front_safety COMMAND test_front_safety)

add_executable(test_front_collision_margin test_front_collision_margin.c)
target_link_libraries(test_front_collision_margin dwa_planning dwa_common m)
add_test(NAME front_collision_margin COMMAND test_front_collision_margin)

# 自车过滤不能删除脚尖前缘外部障碍回波。
add_executable(test_self_filter_toe_boundary test_self_filter_toe_boundary.c)
target_link_libraries(test_self_filter_toe_boundary dwa_common m)
add_test(NAME self_filter_toe_boundary COMMAND test_self_filter_toe_boundary)

# Unified escape gating and bridge-clock wheel-feedback freshness regressions.
add_executable(test_unified_escape test_unified_escape.c)
target_link_libraries(test_unified_escape dwa_motor m)
add_test(NAME unified_escape COMMAND test_unified_escape)

add_executable(test_side_triangle_stop test_side_triangle_stop.c)
target_link_libraries(test_side_triangle_stop dwa_safety m)
add_test(NAME side_triangle_stop COMMAND test_side_triangle_stop)

# Front-corner returns must not masquerade as a close physical side wall.
add_executable(test_side_clearance_sector test_side_clearance_sector.c)
target_link_libraries(test_side_clearance_sector dwa_runtime)
add_test(NAME side_clearance_sector COMMAND test_side_clearance_sector)

add_executable(test_stationary_turn_speed_limiter test_stationary_turn_speed_limiter.c)
target_link_libraries(test_stationary_turn_speed_limiter dwa_safety m)
add_test(NAME stationary_turn_speed_limiter COMMAND test_stationary_turn_speed_limiter)

