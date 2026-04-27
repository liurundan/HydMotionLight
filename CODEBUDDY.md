# CODEBUDDY.md This file provides guidance to CodeBuddy when working with code in this repository.

## Commands

Commands assume the current directory is the repository root.

- Configure: `cmake --preset unixgcc`
  Generates the build tree in `out/build/unixgcc` with `cmake/unixgcc_toolchain.cmake`. Re-run this after adding new `src/*.c` files because the project uses `file(GLOB_RECURSE ...)` and new sources may not appear until CMake is configured again.

- Build everything: `cmake --build --preset unixgcc`
  Builds the static library `HydroMotionLib` plus the simulator library `HydroSimLib` and the current executables: `test_motion_planner`, `test_motion_control`, `test_recipe_validator`, `test_pressure_controller`, `test_pump_converter`, `segment_completion_test`, `rbf_pid_test`, `ramp_controller_test`, `test_scenario_matrix`, `test_diagnostic_monitor`, `test_diagnostic_criteria`, `test_sprint3_integration`, `test_direct_mode`, `test_hydro_sim_fb`, and `main`. There is no separate lint target; this build is also the compiler-warning check because the toolchain enables `-Wall -g -O0`.

- Run all registered tests: `ctest --test-dir out/build/unixgcc --output-on-failure`
  Runs the full automated regression baseline registered in `CMakeLists.txt`, including unit tests, module-integration tests, `rbf_pid_test`, and the end-to-end `test_scenario_matrix` injection-machine scenario suite.

- Run one registered test: `ctest --test-dir out/build/unixgcc -R '^test_scenario_matrix$' --output-on-failure`
  Uses CTest name matching to run one test executable. Swap the regex for any of the registered tests such as `^test_motion_control$` or `^test_rbf_pid$` when iterating.

- Build one target: `cmake --build out/build/unixgcc --target test_scenario_matrix`
  Useful for fast iteration when only one test executable needs rebuilding.

- Run the standalone `RBF_PID` check directly: `./out/build/unixgcc/rbf_pid_test`
  This is optional now because the executable is also registered with CTest, but it remains convenient when working only on `rbf_pid.c` or `rbf_pid.h`.

- Run the integration-style example: `./out/build/unixgcc/main`
  Executes the simulated multi-segment recipe in `tests/main.c` and prints pump-speed commands, flow, pressure, and segment transitions. Use it as the quickest manual end-to-end behavior check.

## High-level architecture

This repository is a small C99 motion-control library for hydraulic / injection-molding workflows. The key architectural boundary comes from `项目需求与设计说明书.md`: the external process layer owns machine sequencing, valve logic, and segment-switch decisions, while this library owns only motion math, pressure/flow planning, pump-speed conversion, and diagnostics. Future changes should preserve that split. If a feature sounds like machine mechanism logic, it probably belongs outside this library.

The shared domain model lives in `include/common_types.h`. That header defines the fixed-size, embedded-friendly types used everywhere: `HDY_AxisRef` for live feedback, `HDY_MotionSegment` for one recipe segment, `HDY_DiagnosticInfo` for alarms/errors, and `HDY_MotionState` for current execution state. The model is intentionally static: `HDY_MAX_SEGMENTS` keeps memory bounded, and the main control path does not use dynamic allocation. Segment naming uses an opaque `segmentTag` (uint8_t) instead of char arrays, saving RAM on embedded targets. The diagnostic history has been simplified from a ring buffer to a single-snapshot model (`lastSnapshot + totalRecorded + hasRecord`), with `HDY_DIAG_HISTORY_DEPTH` kept at 1 for config-export compatibility only. This is consistent with the project's embedded target and the repository rule to stay in pure C99.

The top-level integration point is the PLCopen-style function block `HDY_MotionControlFB` in `include/motion_control.h`. It mixes input fields, output fields, and internal state in one struct, then exposes command-style entry points: `Init`, `LoadRecipe`, `StartSegment`, `NextSegment`, `Abort`, and cyclic `Execute`. The intended caller pattern is:

1. Initialize the function block.
2. Fill gains/limits and load a `RECIPE` array.
3. Start the desired segment explicitly, or set `START_SEGMENT` before a cycle.
4. Update `AXIS_REF` every cycle.
5. Call `HDY_MotionControlFB_Execute()`.
6. Read `PUMP_SPEED`, `STATE`, `DIAGNOSTIC`, `SEGMENT_COMPLETED`, and `SEGMENT_CHANGED`.
7. Let the external process layer decide when to move to the next segment.

`src/motion_control.c` is the orchestrator and best place to understand runtime behavior. `Execute()` handles `EN` and `RESET`, honors the `START_SEGMENT` trigger, guards inactive/finished states, computes elapsed segment time, runs the pressure ramp helper, invokes the planner, copies planned outputs into `STATE`, evaluates segment completion, and populates diagnostics. In other words, it is the glue layer that turns the current feedback plus one active recipe segment into the next pump-speed command.

The actual motion math is isolated in `src/motion_planner.c`. `HDY_MotionPlanner_Execute()` takes the current feedback, the active segment, elapsed time, and the ramped pressure reference bundle. It then follows the non-pressure planning path only:

- `HDY_MODE_POSITION` computes signed motion using either the position-based braking rule `sqrt(2*a*s)` or a time-ramp buildup with braking protection;
- `HDY_MODE_SPEED_RAMP` builds velocity magnitude from `a*t` and, when the end condition is position-based, limits that ramp with the same braking envelope near targetPosition;
- both motion modes convert velocity magnitude to pump-side flow magnitude via `segment->velocityToFlowGain` and `segment->targetFlow` / `segment->maxFlow` caps.

Pressure closed-loop control no longer lives in the planner. It is handled by `src/pressure_controller.c`, which provides P / PI / PID strategy execution, anti-windup, bumpless tracking, measurement filtering, and derivative-rate filtering. Pump-speed conversion is then handled by `src/pump_converter.c`. Together these modules form the core "calculator" role of the library.

`src/ramp_controller.c` is a small stateful helper used only to smooth pressure-target changes before the planner sees them. `HDY_RampController` stores the previous ramped pressure and timestamp. Each cycle, it advances the internal pressure toward the requested target at `rampRate * deltaTime`, or jumps immediately if the rate is zero. `motion_control.c` initializes this ramp state when a segment starts, using the current measured pressure as the starting point.

`src/segment_completion.c` centralizes end-condition evaluation for position, time, pressure, flow, and manual-stop segments. The top-level function block calls `HDY_SegmentCompletion_CheckWithContext()` directly, so completion semantics now have a single source of truth shared by unit tests and runtime orchestration.

`src/segment_limits.c` provides segment parameter resolution helpers (tolerances, timeout limits) and the unified direction-resolution function `HDY_Segment_ResolveDirection()`. This function consolidates the direction-resolution logic that was previously duplicated in `segment_completion.c` and `motion_planner.c`. If the segment declares an explicit direction (EXTEND/RETRACT/HOLD), that direction is returned directly; otherwise it is inferred from the signed delta between `targetPosition` and the current axis position, gated by the segment's position tolerance.

`src/rbf_pid.c` and `include/rbf_pid.h` are architecturally separate from the main flow. They implement an embedded-oriented adaptive `RBF_PID` pressure controller with internal learning state, parameter limits, and feedforward terms. This module is built and testable, but it is not integrated into `HDY_MotionControlFB` or `HDY_MotionPlanner_Execute()`. The main path still uses the dedicated `pressure_controller.c` module for P/PI/PID strategies. Treat `RBF_PID` as a standalone or future integration point, not as current production behavior.

The most useful executable for understanding the intended call flow is `tests/main.c`. It acts like a simulation harness: it creates a multi-segment recipe, loads it into `HDY_MotionControlFB`, starts segment 0, updates `AXIS_REF` in a loop, calls `Execute()`, prints state, and advances segments when `SEGMENT_COMPLETED` becomes true. It also fakes plant feedback by deriving position, flow, and pressure from the planner outputs. When you need to understand how the library is expected to be driven by a higher layer, start there.

The test layout mirrors the module split and is now wired into CTest end to end. Module-focused tests cover planner, motion-control orchestration, recipe validation, pressure control, pump conversion, segment completion, ramp control, and standalone `RBF_PID` behavior. `tests/test_scenario_matrix.c` adds a higher-level regression baseline for typical injection-machine action chains plus long-run and max-segment boundary checks. `tests/test_sprint3_integration.c` validates the diagnostic pipeline end-to-end, including fault escalation from WARNING to FAULT.

The simulator source files live in `src/sim/` (separate from the core library sources in `src/`). They are built as `HydroSimLib` and only linked into integration test targets that need simulated plant feedback. The core `HydroMotionLib` is automatically excluded from the simulator subdirectory.

A few repository-specific constraints matter during edits. First, the planner still clamps flow magnitude and pump speed to nonnegative pump-side values even though it now supports signed velocity and explicit retract-direction planning. Second, `RESET` is strong: `HDY_MotionControlFB_Init()` does a full `memset`, so reset clears gains, recipe contents, runtime state, and flags. Reload configuration after reset. Third, `LoadRecipe()` keeps `ACTIVE=false`; meaningful execution still depends on explicitly starting a segment. Fourth, keep the README and CMake instructions aligned because the repository now uses the README as a lightweight integration handoff in addition to build guidance. Fifth, gain fields (`pressureKp`, `pressureKi`, `pressureKd`) in `HDY_MotionSegment` use zero as "not configured" — `HDY_ResolvePositiveOrDefault` replaces zero/negative values with legacy defaults (e.g., Kp=0 falls back to 1.5). If you need "no proportional control," set `pressureController = HDY_PRESSURE_CONTROLLER_NONE` rather than setting `pressureKp = 0.0`. Sixth, `HDY_DiagnosticCriteria_CheckFaultEscalation()` requires the `result` parameter to carry `severity == HDY_DIAG_SEVERITY_WARNING` and `triggered == true` from a prior `CheckPressure`/`CheckFlow`/etc. call — do not pass an uninitialized result struct.

Finally, keep the coding conventions from `.github/copilot-instructions.md`: stay in C99, use the `HDY_` prefix for the main library symbols, follow the PLCopen function-block style, keep data structures static and bounded, and preserve the separation between process-layer decisions and motion-layer calculations. Also keep `.codebuddy/` intact; it contains repository-specific agent configuration, including the `reviewcode` prompt used for architecture-oriented reviews.
