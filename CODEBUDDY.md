# CODEBUDDY.md This file provides guidance to CodeBuddy when working with code in this repository.

## Commands

Commands assume the current directory is the repository root.

- Configure: `cmake --preset unixgcc`
  Generates the build tree in `out/build/unixgcc` with `cmake/unixgcc_toolchain.cmake`. Re-run this after adding new `src/*.c` files because the project uses `file(GLOB_RECURSE ...)` and new sources may not appear until CMake is configured again.

- Build everything: `cmake --build --preset unixgcc`
  Builds the static library `HydroMotionLib` and the current executables: `test_motion_planner`, `ramp_controller_test`, `rbf_pid_test`, and `main`. There is no separate lint target; this build is also the compiler-warning check because the toolchain enables `-Wall -g -O0`.

- Run all registered tests: `ctest --test-dir out/build/unixgcc --output-on-failure`
  Runs only tests registered in `CMakeLists.txt`. At the moment that means `test_motion_planner` and `test_ramp_controller`.

- Run one registered test: `ctest --test-dir out/build/unixgcc -R '^test_motion_planner$' --output-on-failure`
  Uses CTest name matching to run one test executable. Swap the regex to `^test_ramp_controller$` when working on the ramp controller.

- Build one target: `cmake --build out/build/unixgcc --target test_motion_planner`
  Useful for fast iteration when only one test executable needs rebuilding.

- Run the standalone `RBF_PID` check: `./out/build/unixgcc/rbf_pid_test`
  This executable is built by CMake but is not registered with CTest, so it must be run directly when editing `rbf_pid.c` or `rbf_pid.h`.

- Run the integration-style example: `./out/build/unixgcc/main`
  Executes the simulated multi-segment recipe in `tests/main.c` and prints pump-speed commands, flow, pressure, and segment transitions. Use it as the quickest end-to-end behavior check.

Note: `tests/segment_completion_test.c` exists but is not added to `CMakeLists.txt`, so there is currently no repository-native build command for it.

## High-level architecture

This repository is a small C99 motion-control library for hydraulic / injection-molding workflows. The key architectural boundary comes from `项目需求与设计说明书.md`: the external process layer owns machine sequencing, valve logic, and segment-switch decisions, while this library owns only motion math, pressure/flow planning, pump-speed conversion, and diagnostics. Future changes should preserve that split. If a feature sounds like machine mechanism logic, it probably belongs outside this library.

The shared domain model lives in `include/common_types.h`. That header defines the fixed-size, embedded-friendly types used everywhere: `HDY_AxisRef` for live feedback, `HDY_MotionSegment` for one recipe segment, `HDY_DiagnosticInfo` for alarms/errors, and `HDY_MotionState` for current execution state. The model is intentionally static: `HDY_MAX_SEGMENTS`, `HDY_NAME_MAX`, and `HDY_MESSAGE_MAX` keep memory bounded, and the main control path does not use dynamic allocation. This is consistent with the project’s embedded target and the repository rule to stay in pure C99.

The top-level integration point is the PLCopen-style function block `HDY_MotionControlFB` in `include/motion_control.h`. It mixes input fields, output fields, and internal state in one struct, then exposes command-style entry points: `Init`, `LoadRecipe`, `StartSegment`, `NextSegment`, `Abort`, and cyclic `Execute`. The intended caller pattern is:

1. Initialize the function block.
2. Fill gains/limits and load a `RECIPE` array.
3. Start the desired segment explicitly, or set `START_SEGMENT` before a cycle.
4. Update `AXIS_REF` every cycle.
5. Call `HDY_MotionControlFB_Execute()`.
6. Read `PUMP_SPEED`, `STATE`, `DIAGNOSTIC`, `SEGMENT_COMPLETED`, and `SEGMENT_CHANGED`.
7. Let the external process layer decide when to move to the next segment.

`src/motion_control.c` is the orchestrator and best place to understand runtime behavior. `Execute()` handles `EN` and `RESET`, honors the `START_SEGMENT` trigger, guards inactive/finished states, computes elapsed segment time, runs the pressure ramp helper, invokes the planner, copies planned outputs into `STATE`, evaluates segment completion, and populates diagnostics. In other words, it is the glue layer that turns the current feedback plus one active recipe segment into the next pump-speed command.

The actual motion math is isolated in `src/motion_planner.c`. `HDY_MotionPlanner_Execute()` takes the current feedback, the active segment, elapsed time, the global flow-to-pump conversion gain, the pump-speed clamp, and the ramped pressure target. It then follows one of three modes:

- non-pressure modes compute a target velocity using either the position-based rule `sqrt(2*a*s)` or the time-based rule `a*t`;
- those modes convert velocity to target flow using `segment->velocityToFlowGain`;
- pressure closed-loop mode skips velocity planning and instead computes target flow from `targetFlow + 1.5 * pressureError` using the ramped pressure target.

Finally, the planner converts flow to pump speed with the function block’s global gain and clamp. This is the core “calculator” role of the library.

`src/ramp_controller.c` is a small stateful helper used only to smooth pressure-target changes before the planner sees them. `HDY_RampController` stores the previous ramped pressure and timestamp. Each cycle, it advances the internal pressure toward the requested target at `rampRate * deltaTime`, or jumps immediately if the rate is zero. `motion_control.c` initializes this ramp state when a segment starts, using the current measured pressure as the starting point.

`src/segment_completion.c` is meant to centralize end-condition evaluation for position, time, pressure, flow, and manual-stop segments. However, the current top-level function block does not call this helper. `motion_control.c` duplicates the same completion logic in a local static function. That means completion rules currently have two sources of truth. If you change end-condition semantics, update both places or refactor the function block to use `HDY_SegmentCompletion_Check()` directly.

`src/rbf_pid.c` and `include/rbf_pid.h` are architecturally separate from the main flow. They implement an embedded-oriented adaptive `RBF_PID` pressure controller with internal learning state, parameter limits, and feedforward terms. This module is built and testable, but it is not integrated into `HDY_MotionControlFB` or `HDY_MotionPlanner_Execute()`. The main path still uses the simpler proportional pressure correction in `motion_planner.c`. Treat `RBF_PID` as a standalone or future integration point, not as current production behavior.

The most useful executable for understanding the intended call flow is `tests/main.c`. It acts like a simulation harness: it creates a multi-segment recipe, loads it into `HDY_MotionControlFB`, starts segment 0, updates `AXIS_REF` in a loop, calls `Execute()`, prints state, and advances segments when `SEGMENT_COMPLETED` becomes true. It also fakes plant feedback by deriving position, flow, and pressure from the planner outputs. When you need to understand how the library is expected to be driven by a higher layer, start there.

The test layout mirrors the module split, but the CMake wiring is incomplete. `test_motion_planner` and `ramp_controller_test` are registered with CTest. `rbf_pid_test` is built but must be run directly. `segment_completion_test.c` exists but is not added to the build at all. If you touch segment-completion behavior, expect to either wire that test into `CMakeLists.txt` or verify it another way.

A few repository-specific constraints matter during edits. First, prefer the CMake files over `README.md` for build/run instructions: the README still references old paths such as `src/motion_control.h` and `src/main.c`, while the real public headers are in `include/` and the runnable example is `tests/main.c`. Second, the planner currently clamps velocity, flow, and pump speed to nonnegative values and forces negative remaining distance to zero, so reverse motion is not really modeled even though the enums cover broader machine scenarios. Third, `RESET` is strong: `HDY_MotionControlFB_Init()` does a full `memset`, so reset clears gains, recipe contents, runtime state, and flags. Reload configuration after reset. Fourth, `LoadRecipe()` sets `ACTIVE` when a recipe exists, but meaningful execution still depends on explicitly starting a segment.

Finally, keep the coding conventions from `.github/copilot-instructions.md`: stay in C99, use the `HDY_` prefix for the main library symbols, follow the PLCopen function-block style, keep data structures static and bounded, and preserve the separation between process-layer decisions and motion-layer calculations. Also keep `.codebuddy/` intact; it contains repository-specific agent configuration, including the `reviewcode` prompt used for architecture-oriented reviews.
