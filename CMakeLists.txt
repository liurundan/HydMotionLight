cmake_minimum_required(VERSION 3.10)
project(HydroMotionLib C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(HYDRO_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 自动搜索 src 目录下所有 .c 源文件（推荐）
file(GLOB_RECURSE HYDRO_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(HydroMotionLib STATIC ${HYDRO_CORE_SRCS})
target_include_directories(HydroMotionLib PUBLIC ${HYDRO_INCLUDE_DIRS})
target_link_libraries(HydroMotionLib PUBLIC m)

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

add_executable(main tests/main.c)
target_link_libraries(main PRIVATE HydroMotionLib)

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
