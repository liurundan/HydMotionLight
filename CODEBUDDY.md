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

The shared domain model lives in `include/common_types.h`. That header defines the fixed-size, embedded-friendly types used everywhere: `HYD_AxisRef` for live feedback, `HYD_MotionSegment` for one recipe segment, `HYD_DiagnosticInfo` for alarms/errors, and `HYD_MotionState` for current execution state. The model is intentionally static: `HYD_MAX_SEGMENTS` keeps memory bounded, and the main control path does not use dynamic allocation. Segment naming uses an opaque `segmentTag` (uint8_t) instead of char arrays, saving RAM on embedded targets. The diagnostic history has been simplified from a ring buffer to a single-snapshot model (`lastSnapshot + totalRecorded + hasRecord`), with `HYD_DIAG_HISTORY_DEPTH` kept at 1 for config-export compatibility only. This is consistent with the project's embedded target and the repository rule to stay in pure C99.

The top-level integration point is the PLCopen-style function block `HYD_MotionControlFB` in `include/motion_control.h`. It mixes input fields, output fields, and internal state in one struct, then exposes command-style entry points: `Init`, `LoadRecipe`, `StartSegment`, `NextSegment`, `Abort`, and cyclic `Execute`. The intended caller pattern is:

1. Initialize the function block.
2. Fill gains/limits and load a `RECIPE` array.
3. Start the desired segment explicitly, or set `START_SEGMENT` before a cycle.
4. Update `AXIS_REF` every cycle.
5. Call `HYD_MotionControlFB_Execute()`.
6. Read `PUMP_SPEED`, `STATE`, `DIAGNOSTIC`, `SEGMENT_COMPLETED`, and `SEGMENT_CHANGED`.
7. Let the external process layer decide when to move to the next segment.

`src/motion_control.c` is the orchestrator and best place to understand runtime behavior. `Execute()` handles `EN` and `RESET`, honors the `START_SEGMENT` trigger, guards inactive/finished states, computes elapsed segment time, runs the pressure ramp helper, invokes the planner, copies planned outputs into `STATE`, evaluates segment completion, and populates diagnostics. In other words, it is the glue layer that turns the current feedback plus one active recipe segment into the next pump-speed command.

The actual motion math is isolated in `src/motion_planner.c`. `HYD_MotionPlanner_Execute()` takes the current feedback, the active segment, elapsed time, and the ramped pressure reference bundle. It then follows the non-pressure planning path only:

- `HYD_MODE_POSITION` computes signed motion using either the position-based braking rule `sqrt(2*a*s)` or a time-ramp buildup with braking protection;
- `HYD_MODE_SPEED_RAMP` builds velocity magnitude from `a*t` and, when the end condition is position-based, limits that ramp with the same braking envelope near targetPosition;
- both motion modes convert velocity magnitude to pump-side flow magnitude via `segment->velocityToFlowGain` and `segment->targetFlow` / `segment->maxFlow` caps.

Pressure closed-loop control no longer lives in the planner. It is handled by `src/pressure_controller.c`, which provides P / PI / PID strategy execution, anti-windup, bumpless tracking, measurement filtering, and derivative-rate filtering. Pump-speed conversion is then handled by `src/pump_converter.c`. Together these modules form the core "calculator" role of the library.

`src/ramp_controller.c` is a small stateful helper used only to smooth pressure-target changes before the planner sees them. `HYD_RampController` stores the previous ramped pressure and timestamp. Each cycle, it advances the internal pressure toward the requested target at `rampRate * deltaTime`, or jumps immediately if the rate is zero. `motion_control.c` initializes this ramp state when a segment starts, using the current measured pressure as the starting point.

`src/segment_completion.c` centralizes end-condition evaluation for position, time, pressure, flow, and manual-stop segments. The top-level function block calls `HYD_SegmentCompletion_CheckWithContext()` directly, so completion semantics now have a single source of truth shared by unit tests and runtime orchestration.

`src/segment_limits.c` provides segment parameter resolution helpers (tolerances, timeout limits) and the unified direction-resolution function `HYD_Segment_ResolveDirection()`. This function consolidates the direction-resolution logic that was previously duplicated in `segment_completion.c` and `motion_planner.c`. If the segment declares an explicit direction (EXTEND/RETRACT/HOLD), that direction is returned directly; otherwise it is inferred from the signed delta between `targetPosition` and the current axis position, gated by the segment's position tolerance.

`src/rbf_pid.c` and `include/rbf_pid.h` are architecturally separate from the main flow. They implement an embedded-oriented adaptive `RBF_PID` pressure controller with internal learning state, parameter limits, and feedforward terms. This module is built and testable, but it is not integrated into `HYD_MotionControlFB` or `HYD_MotionPlanner_Execute()`. The main path still uses the dedicated `pressure_controller.c` module for P/PI/PID strategies. Treat `RBF_PID` as a standalone or future integration point, not as current production behavior.

The most useful executable for understanding the intended call flow is `tests/main.c`. It acts like a simulation harness: it creates a multi-segment recipe, loads it into `HYD_MotionControlFB`, starts segment 0, updates `AXIS_REF` in a loop, calls `Execute()`, prints state, and advances segments when `SEGMENT_COMPLETED` becomes true. It also fakes plant feedback by deriving position, flow, and pressure from the planner outputs. When you need to understand how the library is expected to be driven by a higher layer, start there.

The test layout mirrors the module split and is now wired into CTest end to end. Module-focused tests cover planner, motion-control orchestration, recipe validation, pressure control, pump conversion, segment completion, ramp control, and standalone `RBF_PID` behavior. `tests/test_scenario_matrix.c` adds a higher-level regression baseline for typical injection-machine action chains plus long-run and max-segment boundary checks. `tests/test_sprint3_integration.c` validates the diagnostic pipeline end-to-end, including fault escalation from WARNING to FAULT.

## IEC / PLCopen Interface Adapter Layer

`src/motion_interface.c` and `include/motion_interface.h` form the bridge between IEC61131-3 PLC programs and the core `HYD_MotionControlFB`. This layer provides standard PLCopen function blocks that PLC code calls directly.

**FB instance pool**: `HYD_MotionControlFB_inst[HYD_MAX_AXIS_MOTION]` (max 20 axes, configured in `hyd_config.h`). Instances are allocated on first use via `allocMotionControlFB()` and accessed by AXISINDEX.

**PLCopen function blocks defined in `motion_interface.h`**:

| FB | Purpose | Mode |
|---|---|---|
| `HYD_MOVEPROFILE` | Multi-segment recipe-driven motion | Recipe mode (`USE_RECIPE=true`) |
| `HYD_MOVEABSOLUTE` | Single-segment position control | Direct mode |
| `HYD_MOVEVELOCITY` | Continuous velocity control (manual stop) | Direct mode |
| `HYD_PRESSUREHANDLE` | Closed-loop pressure control | Direct mode |
| `HYD_STOP` | Immediate abort of active motion | N/A |
| `HYD_RESET` | Soft-reset FB (clear faults, keep config) | N/A |

Each FB uses matiec IEC types (`__DECLARE_VAR` macros), and the `__mcl_cmd_*` functions implement the PLC command interface. FB input/output signals follow IEC61131-3 conventions: `EN`/`ENO`, `EXECUTE` (rising-edge triggered), `BUSY`, `DONE`, `ACTIVE`, `ERROR`/`ERRORID`, `COMMANDABORTED`.

**Command arbitration**: `motion_interface.c` implements axis ownership with generation counters:
- `takeAxisOwnership()` increments the generation counter and records the active command type
- Each FB instance remembers the generation at which it started
- When a new command takes over, previous commands detect the mismatch and raise `COMMANDABORTED`
- Compatible with multi-FB-per-axis scenarios where one FB can preempt another

**Framework lifecycle**: `__HydMotion_framework_Init/Cleanup/Retrieve/Publish` manage the shared FB pool. `Publish()` calls `HYD_MotionControlFB_Scan()` on all allocated instances each cycle.

**`HYD_AXISMOTION` struct**: Maps IEC-level motion parameters (`SETPOSITION`, `SETVELOCITY`, `SETFLOW`, `SETPRESSURE`, `ACTPOSITION`, etc.) bidirectionally — input side feeds `AXIS_REF`, output side reflects the active segment's current parameters. Used by `HYD_MOVEPROFILE` as a parameter-passing channel.

**matiec IEC type system**: The library uses the matiec IEC61131-3 type infrastructure from `include/matiec/lib/C/`:
- `accessor.h`: `__DECLARE_VAR(type,name)` declares IEC-typed variables with force-flag support. `__GET_VAR`/`__SET_VAR` access them with force-flag protection.
- `iec_types_all.h`: Defines all IEC61131-3 base types (`IEC_BOOL`, `IEC_SINT`, `IEC_REAL`, `IEC_WORD`, etc.) and their tagged wrappers.
- Custom struct types like `HYD_AXISMOTION` are declared via `__DECLARE_STRUCT_TYPE`.
- This type system is what makes the library callable from IEC61131-3 PLC programs (e.g., compiled via matiec/Beremiz).

## Simulator Two-Layer Architecture

The simulator lives in `src/sim/` and `include/` (headers: `hydro_sim.h`, `hydro_sim_fb.h`, `hydro_interfaces.h`, `hydro_hardware.h`). It is built as `HydroSimLib` (separate from `HydroMotionLib`) and only linked into integration test targets.

**L1 — PLC Adapter Layer** (`src/sim/hydro_sim_fb.c`, `include/hydro_sim_fb.h`):

Provides IEC-callable function blocks that mirror the real hardware interface:
- `HYD_CREATESIMAXIS` / `__mcl_cmd_createSimAxis`: Allocates an axis in the shared sim environment, maps AXISID → axis slot, configures cylinder type (CLAMP or INJECT)
- `HYD_MOVESIMAXIS` / `__mcl_cmd_moveSimAxis`: Writes enable/cmd_rpm/direction to an axis, sets the single-pump owner
- `HYD_READSIMAXIS` / `__mcl_cmd_readSimAxis`: Reads per-axis feedback snapshot (position, velocity, pressure) by AXISID
- `__HydSimulator_framework_Publish()`: Steps the shared simulation once per scan cycle, then copies feedback to all allocated handles

Key constraint: the shared `HydraulicSimEnv` is stepped exactly once per `Publish()` call — if multiple FBs publish in the same scan, the physics only advances one tick.

**L2 — Physics Simulation Kernel** (`src/sim/hydro_sim.c`, `include/hydro_sim.h`):

Pure C99 physics engine with:
- `HydraulicSimEnv`: Shared system state (pump displacement, volumetric efficiency, melt stiffness, obstacle model)
- `SimAxisState[HYD_MAX_HYDRAULIC_SIM_FB]`: Each axis has a cylinder model (areas, stroke, friction), valve commands, feedback injection (for fault simulation), and an `ISensorBackend` dependency-injection interface
- Single-pump flow scheduling: only the `pump_owner_axis_id` axis receives flow. `HydraulicSim_Step()` computes `flow = displacement * rpm * efficiency`, converts to velocity via `flow / cylinder_area`, integrates position, and computes pressure from load forces (tie-bar stiffness for clamp, melt resistance for injection)
- Fault injection API: `HydraulicSim_SetPressureSensorStuck/Invalid/Bias/Scale`, `HydraulicSim_SetAxisServoReady/Interlock/MotionStall`
- `ISensorBackend` (`hydro_interfaces.h`): Dependency-injection abstraction with `read_feedback`/`write_valves`/`write_pump` function pointers — allows the same axis model to work with both simulated and real hardware backends

A few repository-specific constraints matter during edits. First, the planner and output limiter now support **negative flow/speed for quick pressure relief**: when `allowNegativeFlow=true` (in `HYD_OutputLimiterInput`) or `ALLOW_NEGATIVE=true` (in `HYD_GETPUMPREQUEST` IEC FB), negative pump speeds are allowed in the range `[-pumpSpeedLimit × HYD_PUMP_NEGATIVE_SPEED_RATIO, 0]` (default `RATIO=0.05`, max 5% reverse). This enables active pressure relief by reversing the pump slightly to drain oil from the pressure line. The negative-speed lower bound is defined by the `HYD_PUMP_NEGATIVE_SPEED_RATIO` macro in `hyd_config.h` and is shared across `output_limiter.c`, `pump_converter.c`, and `motion_interface.c`. Second, `RESET` is strong: `HYD_MotionControlFB_Init()` does a full `memset`, so reset clears gains, recipe contents, runtime state, and flags. Reload configuration after reset. Third, `LoadRecipe()` keeps `ACTIVE=false`; meaningful execution still depends on explicitly starting a segment. Fourth, keep the README and CMake instructions aligned because the repository now uses the README as a lightweight integration handoff in addition to build guidance. Fifth, gain fields (`pressureKp`, `pressureKi`, `pressureKd`) in `HYD_MotionSegment` use zero as "not configured" — `HYD_ResolvePositiveOrDefault` replaces zero/negative values with legacy defaults (e.g., Kp=0 falls back to 1.5). If you need "no proportional control," set `pressureController = HYD_PRESSURE_CONTROLLER_NONE` rather than setting `pressureKp = 0.0`. Sixth, `HYD_DiagnosticCriteria_CheckFaultEscalation()` requires the `result` parameter to carry `severity == HYD_DIAG_SEVERITY_WARNING` and `triggered == true` from a prior `CheckPressure`/`CheckFlow`/etc. call — do not pass an uninitialized result struct.

Finally, keep the coding conventions from `.github/copilot-instructions.md`: stay in C99, use the `HYD_` prefix for the main library symbols, follow the PLCopen function-block style, keep data structures static and bounded, and preserve the separation between process-layer decisions and motion-layer calculations. Also keep `.codebuddy/` intact; it contains repository-specific agent configuration, including the `reviewcode` prompt used for architecture-oriented reviews.
