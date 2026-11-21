# HydAxis Time-Based Ramp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change motion acceleration/deceleration and pressure ramp inputs to millisecond transition times, implement smooth time-based motion/pressure behavior in the hydraulic library, and simplify `FB_HydAxis` without adding unbounded embedded state.

**Architecture:** Preserve the existing fixed-size segment and planner-state layouts where practical. Treat `maxAcceleration`, `maxDeceleration`, and `pressureRampRate` as time-valued fields at the library boundary, with shared normalization helpers converting them to seconds/rates only inside the planner and ramp controller. Reuse existing planner, stop, ramp, and ownership state; add only the smallest output/state needed to expose position reached separately from command completion.

**Tech Stack:** C99, IEC Structured Text, CMake/CTest, existing standalone C regression tests and IEC source-contract tests.

---

## Task 1: Centralize millisecond normalization and update interface contracts

**Files:**
- Create: `include/motion_time.h`
- Modify: `include/common_types.h:330-365, 600-620`
- Modify: `include/motion_interface.h:25-55, 220-370`
- Modify: `src/motion_interface.c:153-315, 600-715, 1330-1385`
- Modify: `src/recipe_validator.c:170-200`
- Test: `tests/test_motion_time_contract.c`
- Modify: `CMakeLists.txt` to register the new test

- [ ] **Step 1: Write failing normalization tests**

Add tests that call the shared helpers and verify:

```c
ASSERT_NEAR(HYD_NormalizeMotionTimeMs(0.0), 100.0, 1e-6);
ASSERT_NEAR(HYD_NormalizeMotionTimeMs(99.0), 100.0, 1e-6);
ASSERT_NEAR(HYD_NormalizeMotionTimeMs(100.0), 100.0, 1e-6);
ASSERT_NEAR(HYD_NormalizePressureTimeMs(0.0), 20.0, 1e-6);
ASSERT_NEAR(HYD_NormalizePressureTimeMs(5.0), 5.0, 1e-6);
ASSERT_TRUE(!HYD_IsValidMotionTimeMs(-1.0));
ASSERT_TRUE(!HYD_IsValidMotionTimeMs(NAN));
```

Run `cmake --build --preset unixgcc --target test_motion_time_contract` and confirm the target fails because the helpers and target do not exist.

- [ ] **Step 2: Add zero-allocation shared helpers**

Create `include/motion_time.h` with `static inline` C99 helpers and constants:

```c
#define HYD_MIN_MOTION_TIME_MS 100.0f
#define HYD_DEFAULT_PRESSURE_TIME_MS 20.0f

static inline HYD_BOOL HYD_IsValidMotionTimeMs(HYD_REAL value) {
    return isfinite(value) && value >= 0.0f;
}

static inline HYD_REAL HYD_NormalizeMotionTimeMs(HYD_REAL value) {
    return (value < HYD_MIN_MOTION_TIME_MS) ? HYD_MIN_MOTION_TIME_MS : value;
}

static inline HYD_REAL HYD_NormalizePressureTimeMs(HYD_REAL value) {
    return (value == 0.0f) ? HYD_DEFAULT_PRESSURE_TIME_MS : value;
}

static inline HYD_TIME HYD_MotionTimeMsToSeconds(HYD_REAL value) {
    return HYD_NormalizeMotionTimeMs(value) * 0.001f;
}

static inline HYD_TIME HYD_PressureTimeMsToSeconds(HYD_REAL value) {
    return HYD_NormalizePressureTimeMs(value) * 0.001f;
}
```

Include `<math.h>` and `common_types.h`; keep these inline so no runtime allocation or extra instance state is introduced.

- [ ] **Step 3: Change comments and boundary builders to time semantics**

Document the existing fixed-size fields as millisecond transition times to avoid growing `HYD_MotionSegment` and `HYD_LiveUpdateRequest`. Include `motion_time.h` in `motion_interface.c`. Validate negative/non-finite values at direct-command and profile construction boundaries. In `buildPositionSegment`, `buildVelocitySegment`, `buildPressureSegment`, and `buildSegmentFromMotion`, store normalized millisecond times in the existing fields. Apply the same normalization in `applyMoveAbsoluteLiveUpdate`, `applyMoveVelocityLiveUpdate`, and `applyPressureHandleLiveUpdate` before calling `HYD_MotionControlFB_ApplyLiveUpdate`.

Remove the old “zero deceleration inherits acceleration” behavior for IEC inputs: zero becomes the 100 ms default; a supplied positive deceleration remains independent.

- [ ] **Step 4: Update validator/default parameter documentation**

Keep recipe validation as finite/non-negative validation, but update comments and default values to millisecond semantics. Set the library defaults to `500 ms` motion time and `20 ms` pressure time unless existing project configuration explicitly overrides them. Do not add new fields to `HYD_MotionSegment` or `HYD_LiveUpdateRequest`.

- [ ] **Step 5: Run the focused contract test and commit**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_time_contract
ctest --test-dir out/build/unixgcc -R '^test_motion_time_contract$' --output-on-failure
```

Expected: PASS. Commit only the shared helper, interface contract, validator, and focused test using the Lore commit format.

## Task 2: Make motion planning use transition times while preserving velocity continuity

**Files:**
- Modify: `include/motion_planner.h:4-45`
- Modify: `src/motion_planner.c:20-700`
- Modify: `src/motion_control.c:210-240, 820-1040, 2270-2410, 3220-3335`
- Test: `tests/test_motion_planner_time_ramp.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing planner tests**

Add deterministic `1 ms` cycle tests that construct a position and speed segment with `maxAcceleration=100` and `maxDeceleration=100` (milliseconds), then verify:

- a speed target changes by no more than the transition slope implied by the current speed delta and `100 ms`;
- changing the target while running does not change the first output discontinuously;
- a short position move lowers the internal target velocity before the braking boundary;
- a zero/short time segment is normalized by the helper and never produces a non-finite rate.

Run `ctest --test-dir out/build/unixgcc -R '^test_motion_planner_time_ramp$' --output-on-failure`; confirm failure against the current rate-based planner.

- [ ] **Step 2: Add planner-local time-to-rate helpers**

Use `HYD_MotionTimeMsToSeconds(segment->maxAcceleration)` and `HYD_MotionTimeMsToSeconds(segment->maxDeceleration)` in planner-local helpers. Compute a rate from the actual transition magnitude:

```c
static HYD_REAL HYD_ResolveTransitionRate(HYD_REAL startMagnitude,
                                          HYD_REAL targetMagnitude,
                                          HYD_TIME transitionTime) {
    HYD_REAL delta = fabs(targetMagnitude - startMagnitude);
    return (delta > HYD_REAL_EPSILON && transitionTime > 0.0)
        ? delta / transitionTime : 0.0;
}
```

Use current planner magnitude (falling back to finite feedback magnitude) as the start magnitude. For braking-distance calculations, use the current/entry speed and terminal speed rather than `maxVelocity / time`.

- [ ] **Step 3: Preserve phase state and rate-limit all planner outputs**

Extend only the existing `HYD_MotionPlannerState` with the minimum fixed fields needed to remember transition start/target/time, or reuse `lastTargetVelocity` and `lastTimestamp` when the current code can express the same behavior. At each phase boundary, establish the transition once. On live velocity/time update, rebase from the already emitted target velocity. Keep the final output path through `HYD_ApplyVelocityRateLimit` so signed velocity and flow remain continuous.

Update position-based safety and speed-ramp deceleration to use a dynamic braking rate. Keep the existing one-cycle response delay and direction-change behavior; do not add trajectory buffers or heap allocation.

- [ ] **Step 4: Update stop and continuous-absolute calculations**

Change the stop path in `HYD_RunRunningStateStopping` to convert `_stopDeceleration` from normalized milliseconds to seconds and derive the stop rate from the captured `_stopStartVel` to zero. Change continuous-absolute crossing/braking calculations to use the same time-derived rate and preserve the current direction. Keep the existing `adaptEndVelEnabled` policy; only replace fixed acceleration assumptions.

- [ ] **Step 5: Run focused planner, stop, and continuous-absolute tests**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner_time_ramp test_motion_planner test_stop_immediate_done test_movecontinuousabsolute_integration
ctest --test-dir out/build/unixgcc -R '^(test_motion_planner_time_ramp|test_motion_planner|test_stop_immediate_done|test_movecontinuousabsolute_integration)$' --output-on-failure
```

Expected: all pass after updating assertions that describe units. Commit the planner and stop slice.

## Task 3: Convert pressure ramp input to time and preserve setpoint continuity

**Files:**
- Modify: `include/ramp_controller.h:1-35`
- Modify: `src/ramp_controller.c:1-60`
- Modify: `src/motion_control.c:2290-2330`
- Modify: `src/motion_interface.c:283-315, 700-710`
- Test: `tests/test_pressure_ramp_time.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing pressure-ramp tests**

Add a fixed `1 ms` test that starts at `0 bar`, targets `100 bar`, and uses `20 ms`. Assert the internal setpoint advances monotonically without jumps and reaches exactly `100 bar` at the end of the transition. Change the target mid-ramp to `40 bar` and assert the next output starts from the previous internal setpoint. Add the `0 ms -> 20 ms` and positive `5 ms` cases.

Run the focused test and confirm failure because the current controller interprets the value as `bar/s`.

- [ ] **Step 2: Change ramp controller to time semantics with bounded fixed state**

Add only three fixed scalar fields to `HYD_RampController`: the latched target pressure, transition rate, and transition duration. Treat `HYD_RampControllerInput.rampRate` as a millisecond transition time at the call boundary, convert it with `HYD_PressureTimeMsToSeconds`, and compute the transition rate once whenever the target or time changes:

```c
HYD_TIME transitionTime = HYD_PressureTimeMsToSeconds(input->rampRate);
if (target_or_time_changed) {
    controller->transitionRate =
        (input->targetPressure - controller->rampedPressure) / transitionTime;
}
controller->rampedPressure += controller->transitionRate * deltaTime;
```

Clamp the result to the target. When the target changes, the existing `rampedPressure` is the new start automatically; do not add a previous-PLC-target history buffer.

- [ ] **Step 3: Update motion-control pressure input and defaults**

Pass the normalized pressure time to the ramp controller. Update initialization defaults from `10 bar/s` to `20 ms` and update comments/parameter names. Keep pressure controller feedback and output limiting unchanged. Verify the resulting controller object remains within the resource budget.

- [ ] **Step 4: Run pressure tests and commit**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_ramp_time test_pressure_controller test_pressure_model
ctest --test-dir out/build/unixgcc -R '^(test_pressure_ramp_time|test_pressure_controller|test_pressure_model)$' --output-on-failure
```

Expected: PASS. Commit the pressure slice using the Lore format.

## Task 4: Simplify `FB_HydAxis` and expose position-reached state

**Files:**
- Modify: `iec/FB_HydAxis.st:75-150, 200-240, 330-590`
- Modify: `include/motion_interface.h:230-255`
- Modify: `src/motion_interface.c:1890-2110`
- Modify: `tests/test_hydaxis_p0_contract.py`
- Test: `tests/test_hydaxis_time_contract.py`

- [ ] **Step 1: Write failing IEC source-contract tests**

Assert that `FB_HydAxis.st` contains no `CalcPresRampRate`, `rPresLastCycle`, or IEC-side `bar/s` conversion; `uiAcc/uiDec` are passed directly as milliseconds; the fixed cycle is a named `1 ms` constant; and the output includes `bPositionReached`.

- [ ] **Step 2: Remove IEC-side slope calculations**

Delete `CalcPresRampRate`, `rPressureRampRate`, `rPresLastCycle`, and the `rAcc/rDec/rStopDec` rate conversion. Pass `UINT_TO_REAL(uiAcc)` and `UINT_TO_REAL(uiDec)` to motion FBs as millisecond values, with default inputs initialized to `100 ms`. Pass `UINT_TO_REAL(uiPressureAccTime)` directly as pressure transition time, defaulting to `20 ms` when the input is zero. Keep `uiJerk` as a compatibility input but do not derive a rate from it.

- [ ] **Step 3: Use separate normal-stop and E-stop invocation paths**

Pass the normal `uiDec` time to `fbStop`. For `fbEStop`, use the existing dedicated safe-stop invocation/path rather than `99999.0` as a fake normal deceleration time. Keep the PLC scan period at a named `1 ms` constant.

- [ ] **Step 4: Add `POSITIONREACHED` to MoveAbsolute and map it to `FB_HydAxis`**

Extend `HYD_MOVEABSOLUTE` with `POSITIONREACHED` and its previous-state latch. Set it from the core's final settled-position result. Add `bPositionReached` to `FB_HydAxis` and map the active MoveAbsolute result. A normal command may finish with `DONE=TRUE` while `POSITIONREACHED=FALSE` after a smooth overshoot; do not set `ERROR` or `bAlarm` for that result.

- [ ] **Step 5: Run IEC contract and integration tests; commit**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_time_contract.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
cmake --build --preset unixgcc --target test_motion_interface_unit test_moveabsolute_stop_integration
ctest --test-dir out/build/unixgcc -R '^(test_motion_interface_unit|test_moveabsolute_stop_integration)$' --output-on-failure
```

Expected: PASS. Commit the IEC and MoveAbsolute output slice.

## Task 5: Full targeted regression and embedded resource check

**Files:**
- Modify: affected tests under `tests/` only when their assertions encode the old units
- Modify: `docs/superpowers/specs/2026-11-07-hydaxis-time-based-ramp-design.md` only if implementation constraints require clarification

- [ ] **Step 1: Run the complete CTest suite**

Run `ctest --test-dir out/build/unixgcc --output-on-failure`. Record failures by unit contract: motion time, planner, stop, pressure, IEC, or unrelated regression.

- [ ] **Step 2: Fix only unit-contract regressions**

Update old test inputs that explicitly use `mm/s²` or `bar/s` to millisecond values while preserving the test's physical scenario. Do not add compatibility shims or change unrelated controller tuning tests.

- [ ] **Step 3: Check static resource impact**

Run:

```bash
size out/build/unixgcc/libHydroSimLib.a 2>/dev/null || true
cmake --build --preset unixgcc --target hydro_sim
```

Compare `sizeof(HYD_MotionSegment)`, `sizeof(HYD_MotionPlannerState)`, and `sizeof(HYD_RampController)` before/after from a small compile-time probe or existing build diagnostics. Confirm no dynamic allocation or trajectory buffer was introduced.

- [ ] **Step 4: Run final source and whitespace checks**

Run:

```bash
git diff --check
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_time_contract.py
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all required tests pass, no known build errors remain, and the final report identifies any untested hardware-only behavior.

- [ ] **Step 5: Commit the final regression/test updates**

Use the Lore commit format and include exact verification commands and any remaining embedded-hardware validation gap.
