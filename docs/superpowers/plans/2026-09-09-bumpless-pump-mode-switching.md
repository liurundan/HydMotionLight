# Bumpless Pump Mode Switching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Keep servo-pump speed continuous during Position/Pressure, Pressure/Position, Speed/Pressure, and Pressure/Speed transitions while preserving immediate zero-output behavior for STOP and FAULT.

**Architecture:** Keep mode handover in `src/motion_control.c`. Capture the last published flow/speed before a normal direct-mode takeover, restore it after the existing reset, and re-prime the successor controller. Add a stateless pump-speed slew helper in `src/pump_converter.c`, using the per-axis published `PUMP_SPEED` as the previous sample. No heap allocation or global mutable converter state.

**Tech Stack:** C99, CMake, assert-based C tests, existing HydroMotionLib APIs.

---

### Task 1: Pump slew helper contract

**Files:** `include/pump_converter.h`, `src/pump_converter.c`, `tests/test_pump_converter.c`

- [x] Add failing tests for `HYD_PumpConverter_ApplySlewLimit(...)`: 1 ms/1000 rpm/s limits a 100 -> 2000 rpm request to 101 rpm; reversal approaches zero without crossing it; invalid input remains safe.
- [x] Run `cmake --build --preset unixgcc --target test_pump_converter` and confirm the expected missing-symbol compile failure.
- [x] Add the declaration and implement the helper. It validates inputs, clamps to the configured pump limit, uses independent acceleration/deceleration rates, prevents one-sample sign changes, and recalculates `commandFlow` from limited speed without changing `HYD_PumpConverter_Execute()` semantics.
- [x] Run the focused converter build and ctest; all converter tests pass.

### Task 2: Normal direct-mode handover continuity

**Files:** `src/motion_control.c`, `tests/test_mode_switch_bumpless.c`, `CMakeLists.txt`

- [x] Create a focused test covering Position -> Pressure, Speed -> Pressure, Pressure -> Position, and Pressure -> Speed with `HYD_BUFFER_MODE_ABORT`, plus explicit Abort zero-output behavior.
- [x] Build/run the focused test and confirm it failed because `HYD_AbortNow()` cleared `_lastCommandedFlow` before successor priming.
- [x] Extend `HYD_DirectContinuityState` with last commanded flow and restore it with planner/output state.
- [x] Add continuity-capable mode and direction-compatibility predicates; direction flips and safety paths remain fresh/zeroing.
- [x] Capture before `HYD_AbortNow()`, restore after `HYD_BeginSegment()`, and re-prime with flow carry-over.
- [x] Run the focused test; all four handovers and explicit Abort pass.

### Task 3: Final pump-boundary rpm slew

**Files:** `src/motion_control.c`, `tests/test_mode_switch_bumpless.c`

- [x] Add a focused bounded-rpm assertion and reversal coverage.
- [x] Add finite acceleration/deceleration rpm/s resolution, deriving Position/Speed rates and using conservative pressure defaults.
- [x] Apply the slew helper after normal output protection, bypassing STOP/FAULT and pressure/soft-limit derate paths for immediate protection response.
- [x] Run the focused test; no zero-rpm gap, bounded normal step, and safety reversal behavior pass.

### Task 4: Regression verification

**Files:** no additional production files

- [x] Build with `cmake --build --preset unixgcc -j2`.
- [x] Run focused regressions:

```bash
ctest --test-dir out/build/unixgcc -R '^(test_pump_converter|test_pressure_controller|test_motion_interface_unit|test_motion_interface_arbitration|test_movevelocity_stop_reverse_restart|test_vp_bumpless_reverse|test_direct_mode_simple|test_mode_switch_bumpless)$' --output-on-failure
```

- [x] Run the full ctest suite; existing pressure-model/continuous-absolute baseline failures are reported separately.
- [x] Confirm no heap allocation, no global mutable converter state, no extra recipe storage, and bounded scalar work per scan.
- [x] Run `git diff --check` and inspect the planned source, header, test, CMake, and plan-document changes.
