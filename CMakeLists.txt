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

add_executable(test_recipe_validator tests/test_recipe_validator.c)
target_link_libraries(test_recipe_validator PRIVATE HydroMotionLib)

add_executable(test_pressure_controller tests/test_pressure_controller.c)
target_link_libraries(test_pressure_controller PRIVATE HydroMotionLib)

add_executable(test_ripple_compensator tests/test_ripple_compensator.c)
target_link_libraries(test_ripple_compensator PRIVATE HydroMotionLib)

add_executable(test_pump_converter tests/test_pump_converter.c)
target_link_libraries(test_pump_converter PRIVATE HydroMotionLib)

add_executable(test_output_limiter tests/test_output_limiter.c)
target_link_libraries(test_output_limiter PRIVATE HydroMotionLib)

add_executable(test_velocity_controller tests/test_velocity_controller.c)
target_link_libraries(test_velocity_controller PRIVATE HydroMotionLib)

add_executable(test_action_profile tests/test_action_profile.c)
target_link_libraries(test_action_profile PRIVATE HydroMotionLib)

add_executable(test_vp_transfer tests/test_vp_transfer.c)
target_link_libraries(test_vp_transfer PRIVATE HydroMotionLib)

add_executable(segment_completion_test tests/segment_completion_test.c)
target_link_libraries(segment_completion_test PRIVATE HydroMotionLib)

add_executable(rbf_pid_test tests/rbf_pid_test.c)
target_link_libraries(rbf_pid_test PRIVATE HydroMotionLib)

add_executable(test_negative_flow tests/test_negative_flow.c)
target_link_libraries(test_negative_flow PRIVATE HydroMotionLib)

add_executable(test_diagnostic_monitor tests/test_diagnostic_monitor.c)
target_link_libraries(test_diagnostic_monitor PRIVATE HydroMotionLib)

add_executable(test_diagnostic_criteria tests/test_diagnostic_criteria.c)
target_link_libraries(test_diagnostic_criteria PRIVATE HydroMotionLib)

add_executable(test_sprint3_integration tests/test_sprint3_integration.c)
target_link_libraries(test_sprint3_integration PRIVATE HydroMotionLib)

add_executable(test_sprint_b_integration tests/test_sprint_b_integration.c)
target_link_libraries(test_sprint_b_integration PRIVATE HydroMotionLib)

add_executable(test_direct_mode tests/test_direct_mode.c)
target_link_libraries(test_direct_mode PRIVATE HydroMotionLib)

add_executable(test_direct_mode_simple tests/test_direct_mode_simple.c)
target_link_libraries(test_direct_mode_simple PRIVATE HydroMotionLib)
# add_test registration for test_direct_mode_simple lives in the test
# registration block below (after enable_testing()).

add_executable(test_parameter_access tests/test_parameter_access.c)
target_link_libraries(test_parameter_access PRIVATE HydroMotionLib)

add_executable(test_parameter_iec tests/test_parameter_iec.c)
target_link_libraries(test_parameter_iec PRIVATE HydroMotionLib)

# ==================================================================
# 仿真相关测试
# ==================================================================
add_executable(test_hydro_sim_fb tests/test_hydro_sim_fb.c)
target_link_libraries(test_hydro_sim_fb PRIVATE HydroSimLib)

add_executable(test_pressure_model tests/test_pressure_model.c)
target_link_libraries(test_pressure_model PRIVATE HydroMotionLib HydroSimLib)

add_executable(pressure_model_replay tests/pressure_model_replay.c)
target_link_libraries(pressure_model_replay PRIVATE HydroSimLib)

add_executable(test_pressure_hold_diagnosis tests/test_pressure_hold_diagnosis.c)
target_link_libraries(test_pressure_hold_diagnosis PRIVATE HydroMotionLib HydroSimLib)

# ==================================================================
# IEC FB接口层测试
# ==================================================================
add_executable(test_motion_interface_unit tests/test_motion_interface_unit.c)
target_link_libraries(test_motion_interface_unit PRIVATE HydroMotionLib)

add_executable(test_motion_interface_arbitration tests/test_motion_interface_arbitration.c)
target_link_libraries(test_motion_interface_arbitration PRIVATE HydroMotionLib)

add_executable(test_motion_interface_done_signals tests/test_motion_interface_done_signals.c)
target_link_libraries(test_motion_interface_done_signals PRIVATE HydroMotionLib)

add_executable(test_movevelocity_stop_reverse_restart
               tests/test_movevelocity_stop_reverse_restart.c)
target_link_libraries(test_movevelocity_stop_reverse_restart PRIVATE HydroMotionLib)

add_executable(test_movecontinuousabsolute_integration
               tests/test_movecontinuousabsolute_integration.c)
target_link_libraries(test_movecontinuousabsolute_integration PRIVATE HydroMotionLib)

add_executable(test_movecontinuousabsolute_external_sim_integration
               tests/test_movecontinuousabsolute_external_sim_integration.c)
target_link_libraries(test_movecontinuousabsolute_external_sim_integration
                      PRIVATE HydroMotionLib HydroSimLib)

add_executable(test_moveabsolute_stop_integration tests/test_moveabsolute_stop_integration.c)
target_link_libraries(test_moveabsolute_stop_integration PRIVATE HydroMotionLib)

add_executable(test_moveabsolute_blending_done tests/test_moveabsolute_blending_done.c)
target_link_libraries(test_moveabsolute_blending_done PRIVATE HydroMotionLib)

add_executable(test_stop_immediate_done tests/test_stop_immediate_done.c)
target_link_libraries(test_stop_immediate_done PRIVATE HydroMotionLib)

add_executable(test_ramp_controller tests/test_ramp_controller.c)
target_link_libraries(test_ramp_controller PRIVATE HydroMotionLib)

add_executable(test_fault_recovery tests/test_fault_recovery.c)
target_link_libraries(test_fault_recovery PRIVATE HydroMotionLib)

add_executable(test_stop_diagnostics_retained tests/test_stop_diagnostics_retained.c)
target_link_libraries(test_stop_diagnostics_retained PRIVATE HydroMotionLib)

add_executable(test_state_reporter tests/test_state_reporter.c)
target_link_libraries(test_state_reporter PRIVATE HydroMotionLib)

add_executable(test_safety_state_manager tests/test_safety_state_manager.c)
target_link_libraries(test_safety_state_manager PRIVATE HydroMotionLib)

add_executable(test_diagnostics tests/test_diagnostics.c)
target_link_libraries(test_diagnostics PRIVATE HydroMotionLib)

add_executable(test_axismotion_setpoint_isolation tests/test_axismotion_setpoint_isolation.c)
target_link_libraries(test_axismotion_setpoint_isolation PRIVATE HydroMotionLib)

add_executable(test_recipe_multi_segment_ownership tests/test_recipe_multi_segment_ownership.c)
target_link_libraries(test_recipe_multi_segment_ownership PRIVATE HydroMotionLib)

add_executable(test_mold_protect tests/test_mold_protect.c)
target_link_libraries(test_mold_protect PRIVATE HydroMotionLib)

add_executable(test_rbf_pid_hil tests/test_rbf_pid_hil.c)
target_link_libraries(test_rbf_pid_hil PRIVATE HydroMotionLib HydroSimLib)

add_executable(test_pump_direction_conflict tests/test_pump_direction_conflict.c)
target_link_libraries(test_pump_direction_conflict PRIVATE HydroMotionLib)

add_executable(test_vp_bumpless_reverse tests/test_vp_bumpless_reverse.c)
target_link_libraries(test_vp_bumpless_reverse PRIVATE HydroMotionLib)

add_executable(test_pump_cylinder_config tests/test_pump_cylinder_config.c)
target_link_libraries(test_pump_cylinder_config PRIVATE HydroMotionLib m)
add_test(NAME test_pump_cylinder_config COMMAND test_pump_cylinder_config)

add_executable(test_toggle_kinematics tests/test_toggle_kinematics.c)
target_link_libraries(test_toggle_kinematics PRIVATE HydroMotionLib m)

add_executable(test_toggle_mechanism_pool tests/test_toggle_mechanism_pool.c)
target_link_libraries(test_toggle_mechanism_pool PRIVATE HydroMotionLib m)

add_executable(test_actuation_mapper tests/test_actuation_mapper.c)
target_link_libraries(test_actuation_mapper PRIVATE HydroMotionLib m)

add_executable(test_toggle_motion_interface tests/test_toggle_motion_interface.c)
target_link_libraries(test_toggle_motion_interface PRIVATE HydroMotionLib m)

add_executable(test_toggle_motion_integration tests/test_toggle_motion_integration.c)
target_link_libraries(test_toggle_motion_integration PRIVATE HydroMotionLib m)

add_executable(test_toggle_hydraulic_sim tests/test_toggle_hydraulic_sim.c)
target_link_libraries(test_toggle_hydraulic_sim PRIVATE HydroMotionLib HydroSimLib m)

add_executable(main tests/main.c)
target_link_libraries(main PRIVATE HydroMotionLib)

# Performance benchmark executable
add_executable(benchmark_performance tests/benchmark_performance.c)
target_link_libraries(benchmark_performance PRIVATE HydroMotionLib)
# benchmark_performance intentionally NOT registered with add_test():
# performance benchmarks should be run in a separate, non-blocking CI job
# to avoid flakiness in the main test suite.

enable_testing()
add_test(NAME test_motion_planner COMMAND test_motion_planner)
add_test(NAME test_recipe_validator COMMAND test_recipe_validator)
add_test(NAME test_pressure_controller COMMAND test_pressure_controller)
add_test(NAME test_ripple_compensator COMMAND test_ripple_compensator)
add_test(NAME test_pump_converter COMMAND test_pump_converter)
add_test(NAME test_output_limiter COMMAND test_output_limiter)
add_test(NAME test_velocity_controller COMMAND test_velocity_controller)
add_test(NAME test_action_profile COMMAND test_action_profile)
add_test(NAME test_vp_transfer COMMAND test_vp_transfer)
add_test(NAME test_segment_completion COMMAND segment_completion_test)
add_test(NAME test_rbf_pid COMMAND rbf_pid_test)
add_test(NAME test_diagnostic_monitor COMMAND test_diagnostic_monitor)
add_test(NAME test_diagnostic_criteria COMMAND test_diagnostic_criteria)
add_test(NAME test_sprint3_integration COMMAND test_sprint3_integration)
add_test(NAME test_sprint_b_integration COMMAND test_sprint_b_integration)
add_test(NAME test_direct_mode COMMAND test_direct_mode)
add_test(NAME test_direct_mode_simple
         COMMAND test_direct_mode_simple
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_parameter_access COMMAND test_parameter_access)
add_test(NAME test_parameter_iec COMMAND test_parameter_iec)
add_test(NAME test_hydro_sim_fb COMMAND test_hydro_sim_fb)
add_test(NAME test_pressure_model COMMAND test_pressure_model)
add_test(NAME test_pressure_hold_diagnosis COMMAND test_pressure_hold_diagnosis)
add_test(NAME test_motion_interface_unit COMMAND test_motion_interface_unit)
add_test(NAME test_motion_interface_arbitration COMMAND test_motion_interface_arbitration)
add_test(NAME test_motion_interface_done_signals COMMAND test_motion_interface_done_signals)
add_test(NAME test_movevelocity_stop_reverse_restart
         COMMAND test_movevelocity_stop_reverse_restart)
add_test(NAME test_movecontinuousabsolute_integration
         COMMAND test_movecontinuousabsolute_integration)
add_test(NAME test_movecontinuousabsolute_external_sim_integration
         COMMAND test_movecontinuousabsolute_external_sim_integration)
add_test(NAME test_moveabsolute_stop_integration COMMAND test_moveabsolute_stop_integration)
add_test(NAME test_moveabsolute_blending_done COMMAND test_moveabsolute_blending_done)
add_test(NAME test_stop_immediate_done COMMAND test_stop_immediate_done)
add_test(NAME test_ramp_controller_detailed COMMAND test_ramp_controller)
add_test(NAME test_fault_recovery
         COMMAND test_fault_recovery
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_stop_diagnostics_retained
         COMMAND test_stop_diagnostics_retained
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_state_reporter
         COMMAND test_state_reporter
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_safety_state_manager
         COMMAND test_safety_state_manager
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_diagnostics
         COMMAND test_diagnostics
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_axismotion_setpoint_isolation
         COMMAND test_axismotion_setpoint_isolation
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_recipe_multi_segment_ownership
         COMMAND test_recipe_multi_segment_ownership
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_mold_protect
         COMMAND test_mold_protect
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_rbf_pid_hil
         COMMAND test_rbf_pid_hil
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_pump_direction_conflict
         COMMAND test_pump_direction_conflict
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_vp_bumpless_reverse
         COMMAND test_vp_bumpless_reverse
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME test_interface_layout_consistency
         COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
                 python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_interface_layout_consistency.py)

add_test(NAME test_negative_flow COMMAND test_negative_flow)
add_test(NAME test_toggle_kinematics COMMAND test_toggle_kinematics)
add_test(NAME test_toggle_mechanism_pool COMMAND test_toggle_mechanism_pool)
add_test(NAME test_actuation_mapper COMMAND test_actuation_mapper)
add_test(NAME test_toggle_motion_interface COMMAND test_toggle_motion_interface)
add_test(NAME test_toggle_motion_integration COMMAND test_toggle_motion_integration)
add_test(NAME test_toggle_hydraulic_sim COMMAND test_toggle_hydraulic_sim)

add_executable(test_motion_interface_negative_speed tests/test_motion_interface_negative_speed.c)
target_link_libraries(test_motion_interface_negative_speed PRIVATE HydroMotionLib)
add_test(NAME test_motion_interface_negative_speed COMMAND test_motion_interface_negative_speed)

add_executable(test_pump_feedback_chain tests/test_pump_feedback_chain.c)
target_link_libraries(test_pump_feedback_chain PRIVATE HydroMotionLib HydroSimLib)
add_test(NAME test_pump_feedback_chain COMMAND test_pump_feedback_chain)

add_test(NAME test_ripple_tools
         COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ripple_tools.py
                 $<TARGET_FILE:pressure_model_replay>)
