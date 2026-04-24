cmake_minimum_required(VERSION 3.10)
project(HydroMotionLib C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(HYDRO_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matiec/lib/C
)

# ==================================================================
# 核心运动控制库 (HydroMotionLib) - 始终构建
# ==================================================================
file(GLOB_RECURSE HYDRO_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

# Exclude simulator subdirectory — simulator sources live in src/sim/
# and are built separately as HydroSimLib.
list(FILTER HYDRO_CORE_SRCS EXCLUDE REGEX ".*/sim/.*\\.c$")

add_library(HydroMotionLib STATIC ${HYDRO_CORE_SRCS})
target_include_directories(HydroMotionLib PUBLIC ${HYDRO_INCLUDE_DIRS})
target_link_libraries(HydroMotionLib PUBLIC m)

# ==================================================================
# 独立仿真器库 (HydroSimLib) - 仅用于集成测试
# 仿真器源文件位于 src/sim/ 子目录，与核心库分离。
# ==================================================================
file(GLOB HYDRO_SIM_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sim/*.c
)
add_library(HydroSimLib STATIC ${HYDRO_SIM_SRCS})
target_include_directories(HydroSimLib PUBLIC ${HYDRO_INCLUDE_DIRS})
target_link_libraries(HydroSimLib PUBLIC m)

add_executable(test_motion_planner tests/test_motion_planner.c)
target_link_libraries(test_motion_planner PRIVATE HydroMotionLib)

add_executable(test_motion_control tests/test_motion_control.c)
target_link_libraries(test_motion_control PRIVATE HydroMotionLib)

add_executable(test_recipe_validator tests/test_recipe_validator.c)
target_link_libraries(test_recipe_validator PRIVATE HydroMotionLib)

add_executable(test_pressure_controller tests/test_pressure_controller.c)
target_link_libraries(test_pressure_controller PRIVATE HydroMotionLib)

add_executable(test_pump_converter tests/test_pump_converter.c)
target_link_libraries(test_pump_converter PRIVATE HydroMotionLib)

add_executable(segment_completion_test tests/segment_completion_test.c)
target_link_libraries(segment_completion_test PRIVATE HydroMotionLib)

add_executable(rbf_pid_test tests/rbf_pid_test.c)
target_link_libraries(rbf_pid_test PRIVATE HydroMotionLib)

add_executable(ramp_controller_test tests/ramp_controller_test.c)
target_link_libraries(ramp_controller_test PRIVATE HydroMotionLib)

add_executable(test_scenario_matrix tests/test_scenario_matrix.c)
target_link_libraries(test_scenario_matrix PRIVATE HydroMotionLib)

add_executable(test_diagnostic_monitor tests/test_diagnostic_monitor.c)
target_link_libraries(test_diagnostic_monitor PRIVATE HydroMotionLib)

add_executable(test_diagnostic_criteria tests/test_diagnostic_criteria.c)
target_link_libraries(test_diagnostic_criteria PRIVATE HydroMotionLib)

add_executable(test_sprint3_integration tests/test_sprint3_integration.c)
target_link_libraries(test_sprint3_integration PRIVATE HydroMotionLib)

add_executable(test_direct_mode tests/test_direct_mode.c)
target_link_libraries(test_direct_mode PRIVATE HydroMotionLib)

add_executable(test_direct_mode_simple tests/test_direct_mode_simple.c)
target_link_libraries(test_direct_mode_simple PRIVATE HydroMotionLib)

# ==================================================================
# 集成测试 - 需要同时链接核心库和仿真器库
# ==================================================================
add_executable(test_motion_sim_integration tests/test_motion_sim_integration.c)
target_link_libraries(test_motion_sim_integration PRIVATE HydroMotionLib HydroSimLib)

add_executable(test_hydro_sim_fb tests/test_hydro_sim_fb.c)
target_link_libraries(test_hydro_sim_fb PRIVATE HydroSimLib)

add_executable(test_ramp_controller tests/test_ramp_controller.c)
target_link_libraries(test_ramp_controller PRIVATE HydroMotionLib)

add_executable(main tests/main.c)
target_link_libraries(main PRIVATE HydroMotionLib)

# Performance benchmark executable
add_executable(benchmark_performance tests/benchmark_performance.c)
target_link_libraries(benchmark_performance PRIVATE HydroMotionLib)

enable_testing()
add_test(NAME test_motion_planner COMMAND test_motion_planner)
add_test(NAME test_motion_control COMMAND test_motion_control)
add_test(NAME test_recipe_validator COMMAND test_recipe_validator)
add_test(NAME test_pressure_controller COMMAND test_pressure_controller)
add_test(NAME test_pump_converter COMMAND test_pump_converter)
add_test(NAME test_segment_completion COMMAND segment_completion_test)
add_test(NAME test_rbf_pid COMMAND rbf_pid_test)
add_test(NAME test_ramp_controller COMMAND ramp_controller_test)
add_test(NAME test_scenario_matrix COMMAND test_scenario_matrix)
add_test(NAME test_diagnostic_monitor COMMAND test_diagnostic_monitor)
add_test(NAME test_diagnostic_criteria COMMAND test_diagnostic_criteria)
add_test(NAME test_sprint3_integration COMMAND test_sprint3_integration)
add_test(NAME test_direct_mode COMMAND test_direct_mode)
add_test(NAME test_motion_sim_integration COMMAND test_motion_sim_integration)
add_test(NAME test_hydro_sim_fb COMMAND test_hydro_sim_fb)
add_test(NAME test_ramp_controller_detailed COMMAND test_ramp_controller)
