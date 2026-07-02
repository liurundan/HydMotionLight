# MoveContinuousAbsolute Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `HYD_MoveContinuousAbsolute` as a new IEC direct-motion FB that drives to an absolute position, keeps running with a programmed terminal velocity, and aligns its `PressureLimit` input with the existing global and per-segment pressure-limit chain.

**Architecture:** Keep `HYD_MoveAbsolute` unchanged. Implement `HYD_MoveContinuousAbsolute` as a new direct-command kind with one owner and two internal runtime phases: `APPROACH` and `SUSTAIN`. Reuse the current planner's nonzero terminal-velocity math by injecting a synthetic local blend context during `APPROACH`, then switch the same owner into an endless speed-ramp segment for `SUSTAIN`. Treat an active `HYD_MoveContinuousAbsolute` owner as endless for follower admission so later `BUFFER` / `BLENDING_*` submissions degrade to abort-takeover the same way endless `MoveVelocity` already does.

**Tech Stack:** C99, matiec-generated IEC structs/macros, HydroMotionLib direct-command runtime in `src/motion_control.c`, IEC adapter layer in `src/motion_interface.c`, XML/header layout consistency checks, CMake preset `unixgcc`, standalone C regression tests in `tests/`.

---

## File Structure

- Modify `include/motion_interface.h`
  - add `HYD_MOVECONTINUOUSABSOLUTE` with the approved public pin order
  - add `__mcl_cmd_MoveContinuousAbsolute(...)` prototype
- Modify `pousHydMotion.xml`
  - add the matching `HYD_MoveContinuousAbsolute` POU
  - keep XML pin order identical to `include/motion_interface.h`
- Modify `include/motion_control.h`
  - add the new direct-command kind
  - add a compact continuous-absolute runtime context and phase enum
  - extend the direct-start API so callers can pass an explicit direct kind plus optional continuous-absolute context
- Modify `src/motion_control.c`
  - plumb the explicit direct kind through immediate start, pending queue, and pending-slot promotion
  - store/copy/clear continuous-absolute context alongside the direct owner and pending slot
  - inject a synthetic planner blend target during `APPROACH`
  - switch from `APPROACH` to `SUSTAIN` without completing the owner
  - latch `PositionReached` and `InEndVelocity`
- Modify `src/motion_interface.c`
  - add the new IEC command wrapper
  - parse `EndVelocityDirection`
  - normalize `PressureLimit`
  - build the initial `APPROACH` segment and the continuous-absolute context
  - map runtime state back to `Busy`, `PositionReached`, `InEndVelocity`, `CommandAborted`, `Error`
- Modify `include/segment_completion.h`
  - expose a raw position-reached helper without the zero-velocity completion gate
- Modify `src/segment_completion.c`
  - implement the raw position-reached helper by reusing the existing directional position comparison logic
- Create `tests/test_movecontinuousabsolute_integration.c`
  - cover validation, same-direction sustain, reverse-direction sustain, pressure-limit interaction, and Stop takeover
- Modify `CMakeLists.txt`
  - register the new opt-in test target
  - wire the target into `ctest` only after the executable becomes buildable
- Modify `tests/test_motion_interface_arbitration.c`
  - once the adapter and runtime exist, lock the follower-admission rule for active continuous-absolute owners
  - verify later `BUFFER` / `BLENDING_*` commands degrade to abort takeover once the continuous-absolute owner is active

No planner API change is planned. `src/motion_planner.c` should stay untouched unless the local synthetic-blend approach proves impossible.

## Task 1: Expose the IEC surface and lock XML/header layout

**Files:**
- Modify: `include/motion_interface.h`
- Modify: `pousHydMotion.xml`
- Test: `tests/test_interface_layout_consistency.py`

- [ ] **Step 1: Add the new FB struct to `include/motion_interface.h`**

Insert this block immediately after `HYD_MOVEABSOLUTE` and before `HYD_MOVEVELOCITY` so the public motion-command section stays grouped:

```c
// FUNCTION_BLOCK HYD_MoveContinuousAbsolute
typedef struct {
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(REAL,POSITION)
  __DECLARE_VAR(REAL,VELOCITY)
  __DECLARE_VAR(REAL,ENDVELOCITY)
  __DECLARE_VAR(SINT,ENDVELOCITYDIRECTION)
  __DECLARE_VAR(REAL,ACCELERATION)
  __DECLARE_VAR(REAL,DECELERATION)
  __DECLARE_VAR(REAL,JERK)
  __DECLARE_VAR(SINT,DIRECTION)
  __DECLARE_VAR(BOOL,ADAPTENDVELTOAVOIDOVERSHOOT)
  __DECLARE_VAR(REAL,PRESSURELIMIT)
  __DECLARE_VAR(BOOL,INENDVELOCITY)
  __DECLARE_VAR(BOOL,POSITIONREACHED)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,INENDVELOCITY0)
  __DECLARE_VAR(BOOL,POSITIONREACHED0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)
} HYD_MOVECONTINUOUSABSOLUTE;
```

Also add the prototype near the other direct-command prototypes:

```c
extern void __mcl_cmd_MoveContinuousAbsolute(HYD_MOVECONTINUOUSABSOLUTE *data__);
```

- [ ] **Step 2: Add the matching POU to `pousHydMotion.xml`**

Insert this POU right after `HYD_MoveAbsolute` so the XML order matches the header order exactly:

```xml
<pou name="HYD_MoveContinuousAbsolute" pouType="functionBlock">
  <interface>
    <inputVars>
      <variable name="AXISID"><type><SINT /></type></variable>
      <variable name="EXECUTE"><type><BOOL /></type></variable>
      <variable name="POSITION"><type><REAL /></type></variable>
      <variable name="VELOCITY"><type><REAL /></type></variable>
      <variable name="ENDVELOCITY"><type><REAL /></type></variable>
      <variable name="ENDVELOCITYDIRECTION"><type><SINT /></type></variable>
      <variable name="ACCELERATION"><type><REAL /></type></variable>
      <variable name="DECELERATION"><type><REAL /></type></variable>
      <variable name="JERK"><type><REAL /></type></variable>
      <variable name="DIRECTION"><type><SINT /></type></variable>
      <variable name="ADAPTENDVELTOAVOIDOVERSHOOT"><type><BOOL /></type></variable>
      <variable name="PRESSURELIMIT"><type><REAL /></type></variable>
    </inputVars>
    <outputVars>
      <variable name="INENDVELOCITY"><type><BOOL /></type></variable>
      <variable name="POSITIONREACHED"><type><BOOL /></type></variable>
      <variable name="BUSY"><type><BOOL /></type></variable>
      <variable name="COMMANDABORTED"><type><BOOL /></type></variable>
      <variable name="ERROR"><type><BOOL /></type></variable>
      <variable name="ERRORID"><type><WORD /></type></variable>
    </outputVars>
    <localVars>
      <variable name="EXECUTE0"><type><BOOL /></type></variable>
      <variable name="INENDVELOCITY0"><type><BOOL /></type></variable>
      <variable name="POSITIONREACHED0"><type><BOOL /></type></variable>
      <variable name="_PENDING"><type><BOOL /></type></variable>
      <variable name="_EXEC_ID"><type><WORD /></type></variable>
    </localVars>
  </interface>
  <body>
    <ST><![CDATA[{{ extern void __mcl_cmd_MoveContinuousAbsolute(HYD_MOVECONTINUOUSABSOLUTE*); __mcl_cmd_MoveContinuousAbsolute(data__); }} ]]></ST>
  </body>
</pou>
```

- [ ] **Step 3: Run the layout consistency test**

Run:

```bash
python3 tests/test_interface_layout_consistency.py
```

Expected:

```text
interface layout consistency tests passed
```

- [ ] **Step 4: Sanity-check that the library still configures**

Run:

```bash
cmake --preset unixgcc
```

Expected:

```text
-- Configuring done
-- Generating done
-- Build files have been written to: .../out/build/unixgcc
```

- [ ] **Step 5: Commit the public-surface scaffold**

Run:

```bash
git add include/motion_interface.h pousHydMotion.xml
git commit -m "Expose the MoveContinuousAbsolute IEC surface" -m "Constraint: The new FB must match the approved public pin contract without altering existing MoveAbsolute semantics\nRejected: Extending HYD_MoveAbsolute pins in place | Would break the repository's existing field-order and lifecycle contract\nConfidence: high\nScope-risk: narrow\nDirective: Keep XML and header field order identical; run the layout consistency test on every edit\nTested: python3 tests/test_interface_layout_consistency.py; cmake --preset unixgcc\nNot-tested: No runtime implementation or command execution yet"
```

### Task 2: Add failing integration regressions

**Files:**
- Create: `tests/test_movecontinuousabsolute_integration.c`
- Modify: `CMakeLists.txt`
- Test: `tests/test_movecontinuousabsolute_integration.c`

- [ ] **Step 1: Create the dedicated integration test target**

Create `tests/test_movecontinuousabsolute_integration.c` with this starting test harness and the core direction/adaptation contract tests.

Important constraint for this task: keep these regressions black-box. Do not assert on future private runtime members such as `_directContinuousAbsolute`, do not redeclare non-header test hooks, and do not mutate `HYD_MotionControlFB` internals directly. Express expectations through public FB outputs, `HYD_READSIMFEEDBACK` / `HYD_READSTATUS`, `HYD_SETAXISFEEDBACK` where needed, and other public command FBs only, so the target compiles against today's headers and fails solely because the adapter/runtime implementation is missing.

Starting harness:

```c
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define MAX_SIM_STEPS 20000

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s [line %d]\n", msg, __LINE__); } \
} while (0)

static int create_sim_axis(void) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static void hold_true_scan(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    fb->EXECUTE0.value = true;
    __mcl_cmd_MoveContinuousAbsolute(fb);
}

static int run_until_position_reached(HYD_MOVECONTINUOUSABSOLUTE* fb, int maxSteps) {
    int step;
    for (step = 0; step < maxSteps; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb);
        if (IEC_VAL(fb->POSITIONREACHED)) {
            return step + 1;
        }
    }
    return -1;
}

static void test_rejects_invalid_end_velocity_direction(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 50.0f;
    IEC_VAL(mc.VELOCITY) = 20.0f;
    IEC_VAL(mc.ENDVELOCITY) = 10.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 0; /* shortest is unsupported here */
    IEC_VAL(mc.ACCELERATION) = 100.0f;
    IEC_VAL(mc.DIRECTION) = 1;

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    ASSERT_TRUE(IEC_VAL(mc.ERROR) == true,
               "MoveContinuousAbsolute should reject unsupported EndVelocityDirection values");
}

static void test_current_end_velocity_direction_uses_velocity_then_last_active_direction(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Current-direction test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = -5.0f;
    fb->_lastActiveDirection = HYD_DIRECTION_POSITIVE;

    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 40.0f;
    IEC_VAL(mc.VELOCITY) = 25.0f;
    IEC_VAL(mc.ENDVELOCITY) = 8.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 3;
    IEC_VAL(mc.ACCELERATION) = 80.0f;
    IEC_VAL(mc.DIRECTION) = 1;
    __mcl_cmd_MoveContinuousAbsolute(&mc);

    ASSERT_TRUE(fb->_directContinuousAbsolute.sustainDirection == HYD_DIRECTION_NEGATIVE,
               "EndVelocityDirection=current should follow measured velocity direction when the axis is already moving");

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Current-direction fallback test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = 0.0f;
    fb->_lastActiveDirection = HYD_DIRECTION_NEGATIVE;

    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 40.0f;
    IEC_VAL(mc.VELOCITY) = 25.0f;
    IEC_VAL(mc.ENDVELOCITY) = 8.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 3;
    IEC_VAL(mc.ACCELERATION) = 80.0f;
    IEC_VAL(mc.DIRECTION) = 1;
    __mcl_cmd_MoveContinuousAbsolute(&mc);

    ASSERT_TRUE(fb->_directContinuousAbsolute.sustainDirection == HYD_DIRECTION_NEGATIVE,
               "EndVelocityDirection=current should fall back to _lastActiveDirection when actual velocity is near zero");
}

static void test_same_direction_reaches_position_and_end_velocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;
    int reachedStep;
    int step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 80.0f;
    IEC_VAL(mc.VELOCITY) = 40.0f;
    IEC_VAL(mc.ENDVELOCITY) = 15.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 200.0f;
    IEC_VAL(mc.DIRECTION) = 1;

    __mcl_cmd_MoveContinuousAbsolute(&mc);
    reachedStep = run_until_position_reached(&mc, MAX_SIM_STEPS);

    ASSERT_TRUE(reachedStep > 0, "PositionReached should go true");

    for (step = 0; step < 2000; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
        if (IEC_VAL(mc.INENDVELOCITY)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(mc.POSITIONREACHED) == true,
               "PositionReached should stay latched after target reach");
    ASSERT_TRUE(IEC_VAL(mc.INENDVELOCITY) == true,
               "Same-direction case should eventually reach end velocity");
    ASSERT_TRUE(IEC_VAL(mc.BUSY) == true,
               "Busy should stay true while the sustain phase owns the axis");
    ASSERT_TRUE(fb != NULL && fabs(fb->AXIS_REF.velocity) > 1.0f,
               "The sustain phase should keep nonzero velocity");
}

static void test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Adapt-down test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->AXIS_REF.position = 0.0f;
    fb->AXIS_REF.velocity = 0.0f;

    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 1.0f;
    IEC_VAL(mc.VELOCITY) = 50.0f;
    IEC_VAL(mc.ENDVELOCITY) = 20.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 10.0f;
    IEC_VAL(mc.DECELERATION) = 10.0f;
    IEC_VAL(mc.DIRECTION) = 1;
    IEC_VAL(mc.ADAPTENDVELTOAVOIDOVERSHOOT) = true;

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    ASSERT_TRUE(fb->_directContinuousAbsolute.crossingVelocity < 20.0f,
               "Adapt=true should lower crossing velocity when distance is too short to accelerate");
}

static void test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Adapt-up test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->AXIS_REF.position = 0.0f;
    fb->AXIS_REF.velocity = 40.0f;

    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 2.0f;
    IEC_VAL(mc.VELOCITY) = 50.0f;
    IEC_VAL(mc.ENDVELOCITY) = 5.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 20.0f;
    IEC_VAL(mc.DECELERATION) = 10.0f;
    IEC_VAL(mc.DIRECTION) = 1;
    IEC_VAL(mc.ADAPTENDVELTOAVOIDOVERSHOOT) = true;

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    ASSERT_TRUE(fb->_directContinuousAbsolute.crossingVelocity > 5.0f,
               "Adapt=true should raise crossing velocity when distance is too short to decelerate");
}

static void test_reverse_sustain_delays_inendvelocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;
    int reachedStep;
    int step;
    HYD_BOOL sawDelayedInEndVelocity = false;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 60.0f;
    IEC_VAL(mc.VELOCITY) = 30.0f;
    IEC_VAL(mc.ENDVELOCITY) = 10.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 2;
    IEC_VAL(mc.ACCELERATION) = 120.0f;
    IEC_VAL(mc.DECELERATION) = 120.0f;
    IEC_VAL(mc.DIRECTION) = 1;

    __mcl_cmd_MoveContinuousAbsolute(&mc);
    reachedStep = run_until_position_reached(&mc, MAX_SIM_STEPS);

    ASSERT_TRUE(reachedStep > 0, "Reverse-sustain case should still reach target position");
    if (IEC_VAL(mc.POSITIONREACHED) && !IEC_VAL(mc.INENDVELOCITY)) {
        sawDelayedInEndVelocity = true;
    }

    for (step = 0; step < 4000; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
        if (IEC_VAL(mc.INENDVELOCITY)) {
            break;
        }
        if (IEC_VAL(mc.POSITIONREACHED) && !IEC_VAL(mc.INENDVELOCITY)) {
            sawDelayedInEndVelocity = true;
        }
    }

    ASSERT_TRUE(sawDelayedInEndVelocity == true,
               "Reverse sustain should show PositionReached before InEndVelocity");
    ASSERT_TRUE(IEC_VAL(mc.INENDVELOCITY) == true,
               "Reverse sustain should eventually reach the sustained velocity");
    ASSERT_TRUE(fb != NULL && fb->AXIS_REF.velocity < -1.0f,
               "Reverse sustain should end up moving in the negative direction");
}

int main(void) {
    printf("=== MoveContinuousAbsolute integration ===\n");
    test_rejects_invalid_end_velocity_direction();
    test_current_end_velocity_direction_uses_velocity_then_last_active_direction();
    test_same_direction_reaches_position_and_end_velocity();
    test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate();
    test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate();
    test_reverse_sustain_delays_inendvelocity();
    printf("Passed %d / %d assertions\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: Add the pressure-limit and Stop takeover regressions**

Append these tests to the same file:

```c
static void test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;
    int reachedStep;
    int step;
    HYD_BOOL observedLimitedGap = false;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Pressure-limit test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->PRESSURE_LIMIT = 2.0f;

    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 40.0f;
    IEC_VAL(mc.VELOCITY) = 35.0f;
    IEC_VAL(mc.ENDVELOCITY) = 20.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 150.0f;
    IEC_VAL(mc.DIRECTION) = 1;
    IEC_VAL(mc.PRESSURELIMIT) = 0.0f; /* force fallback to fb->PRESSURE_LIMIT */

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    for (step = 0; step < 1000; step++) {
        fb->AXIS_REF.pressure = 20.0f;
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
    }

    reachedStep = run_until_position_reached(&mc, MAX_SIM_STEPS);
    ASSERT_TRUE(reachedStep > 0, "Pressure-limited command should still reach target position");

    for (step = 0; step < 200; step++) {
        fb->AXIS_REF.pressure = 20.0f;
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
        if (IEC_VAL(mc.POSITIONREACHED) && !IEC_VAL(mc.INENDVELOCITY)) {
            observedLimitedGap = true;
            break;
        }
    }

    ASSERT_TRUE(observedLimitedGap == true,
               "Under limiting, PositionReached should be possible while InEndVelocity stays false");
}

static void test_stop_takes_over_and_sets_commandaborted(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_STOP stop;
    int axisId;
    int step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 120.0f;
    IEC_VAL(mc.VELOCITY) = 50.0f;
    IEC_VAL(mc.ENDVELOCITY) = 20.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 200.0f;
    IEC_VAL(mc.DECELERATION) = 200.0f;
    IEC_VAL(mc.DIRECTION) = 1;

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    for (step = 0; step < 50; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
    }

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.DECELERATION) = 80.0f;
    __mcl_cmd_Stop(&stop);

    for (step = 0; step < 4000; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(mc.COMMANDABORTED)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(mc.COMMANDABORTED) == true,
               "Stop takeover should surface COMMANDABORTED on the continuous command");
    ASSERT_TRUE(IEC_VAL(mc.BUSY) == false,
               "Stop takeover should clear Busy on the continuous command");
}
```

And register them in `main()`:

```c
    test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false();
    test_stop_takes_over_and_sets_commandaborted();
```

- [ ] **Step 3: Register the new test target in `CMakeLists.txt`**

Add the executable near the other IEC interface tests, but keep it out of the default build until the adapter/runtime exists:

```cmake
add_executable(test_movecontinuousabsolute_integration EXCLUDE_FROM_ALL tests/test_movecontinuousabsolute_integration.c)
target_link_libraries(test_movecontinuousabsolute_integration PRIVATE HydroMotionLib)
```

Do not add an `add_test(...)` entry yet. Wire it into `ctest` only in the finalization task once the executable builds successfully.

- [ ] **Step 4: Build the new target and confirm the expected failure**

Run:

```bash
cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration
```

Expected before implementation:

```text
the target fails to build or link because the runtime state and/or adapter support does not exist yet
```

The failure should come from missing `MoveContinuousAbsolute` implementation surfaces, not from unrelated syntax errors in the new test file.

- [ ] **Step 5: Commit the failing regression scaffold**

Run:

```bash
git add CMakeLists.txt tests/test_movecontinuousabsolute_integration.c
git commit -m "Add MoveContinuousAbsolute regression targets" -m "Constraint: The new command must be test-driven across validation, lifecycle, and pressure-limit behavior before runtime edits begin\nRejected: Hiding the new coverage inside existing MoveAbsolute tests | Would blur the new contract and make failures harder to interpret\nConfidence: high\nScope-risk: narrow\nDirective: Keep the dedicated integration target focused on public signal semantics; add interface-level arbitration coverage only after the adapter symbol exists\nTested: cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration\nNot-tested: Expected build/link failure because the new command runtime state and adapter are not implemented yet"
```

### Task 3: Plumb explicit direct kinds and continuous-absolute context through the core

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add the new direct kind, phase enum, and runtime context to `include/motion_control.h`**

Update the direct-kind enum:

```c
typedef enum {
    HYD_DIRECT_CMD_NONE = 0,
    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
    HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
    HYD_DIRECT_CMD_MOVE_VELOCITY,
    HYD_DIRECT_CMD_PRESSURE_HANDLE,
    HYD_DIRECT_CMD_STOP
} HYD_DirectCommandKind;
```

Add a private phase enum and context struct near the direct-command declarations:

```c
typedef enum {
    HYD_CONTABS_PHASE_NONE = 0,
    HYD_CONTABS_PHASE_APPROACH,
    HYD_CONTABS_PHASE_SUSTAIN
} HYD_ContinuousAbsolutePhase;

typedef struct {
    HYD_BOOL valid;
    uint16_t ownerTicket;
    HYD_ContinuousAbsolutePhase phase;
    HYD_REAL targetPosition;
    HYD_REAL crossingVelocity;
    HYD_REAL sustainVelocity;
    HYD_REAL effectivePressureLimit;
    HYD_BOOL adaptEndVelEnabled;
    HYD_MotionDirection approachDirection;
    HYD_MotionDirection sustainDirection;
    HYD_BOOL positionReachedLatched;
    HYD_BOOL inEndVelocityLatched;
} HYD_ContinuousAbsoluteContext;
```

Store one active and one pending copy on the FB:

```c
    HYD_ContinuousAbsoluteContext _directContinuousAbsolute;
    HYD_ContinuousAbsoluteContext _directPendingContinuousAbsolute;
```

- [ ] **Step 2: Extend the direct-start API to accept the explicit kind and optional context**

Change the declaration in `include/motion_control.h` to:

```c
HYD_DirectStartResult HYD_MotionControlFB_StartDirectCommand(
    HYD_MotionControlFB* fb,
    HYD_DirectCommandKind kind,
    const HYD_MotionSegment* segment,
    const HYD_ContinuousAbsoluteContext* continuousAbsolute,
    HYD_BufferMode bufferMode,
    HYD_TIME timestamp);
```

Then update the function definition in `src/motion_control.c` to match the new signature exactly.

- [ ] **Step 3: Update the legacy direct-start wrapper so existing FBs still compile**

In `src/motion_interface.c`, add a tiny adapter-local inference helper next to `startDirectSegmentExecution(...)`:

```c
static HYD_DirectCommandKind inferAdapterDirectKind(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DIRECT_CMD_NONE;
    }
    if (segment->mode == HYD_MODE_POSITION &&
        segment->endCondition == HYD_END_POSITION) {
        return HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    }
    if (segment->mode == HYD_MODE_SPEED_RAMP) {
        return HYD_DIRECT_CMD_MOVE_VELOCITY;
    }
    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        return HYD_DIRECT_CMD_PRESSURE_HANDLE;
    }
    return HYD_DIRECT_CMD_NONE;
}
```

Then update the existing wrapper to call the new signature:

```c
    result = HYD_MotionControlFB_StartDirectCommand(fb,
                                                    inferAdapterDirectKind(segment),
                                                    segment,
                                                    NULL,
                                                    (HYD_BufferMode)bufferMode,
                                                    fb->AXIS_REF.timestamp);
```

- [ ] **Step 4: Add clear/copy helpers and wire them into pending-slot lifecycle**

Add these helpers near the existing direct-slot helpers in `src/motion_control.c`:

```c
static void HYD_ClearContinuousAbsoluteContext(HYD_ContinuousAbsoluteContext* ctx) {
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
        ctx->phase = HYD_CONTABS_PHASE_NONE;
    }
}

static void HYD_CopyContinuousAbsoluteContext(HYD_ContinuousAbsoluteContext* dst,
                                              const HYD_ContinuousAbsoluteContext* src) {
    if (dst == NULL) {
        return;
    }
    if (src == NULL || !src->valid) {
        HYD_ClearContinuousAbsoluteContext(dst);
        return;
    }
    *dst = *src;
}
```

Use them in:
- `HYD_ClearDirectPendingSlot(...)`
- direct-command reset/init paths
- abort/takeover cleanup

Specifically extend `HYD_ClearDirectPendingSlot(...)` with:

```c
    HYD_ClearContinuousAbsoluteContext(&fb->_directPendingContinuousAbsolute);
```

- [ ] **Step 5: Preserve the explicit kind and pending continuous context through start and promotion**

In the immediate-start branch of `HYD_MotionControlFB_StartDirectCommand(...)`, after `HYD_BeginSegment(...)`, overwrite the legacy inferred kind:

```c
        fb->_directOwnerKind = kind;
        HYD_CopyContinuousAbsoluteContext(&fb->_directContinuousAbsolute, continuousAbsolute);
        fb->_directContinuousAbsolute.ownerTicket = fb->_directOwnerTicket;
```

In the buffered branch, store both the explicit kind and the pending context:

```c
        fb->_directPendingSegment = *segment;
        fb->_directPendingKind = kind;
        HYD_CopyContinuousAbsoluteContext(&fb->_directPendingContinuousAbsolute, continuousAbsolute);
        fb->_directPendingBufferMode = bufferMode;
        fb->_directPendingValid = true;
```

In `HYD_StartPendingDirectSlot(...)`, save the pending context before `HYD_ClearDirectPendingSlot(...)` wipes it, then restore it to the active owner after `HYD_BeginSegment(...)`:

```c
    HYD_ContinuousAbsoluteContext pendingContinuousAbsolute = fb->_directPendingContinuousAbsolute;
    HYD_DirectCommandKind pendingKind = fb->_directPendingKind;
```

and after the begin succeeds:

```c
    fb->_directOwnerKind = pendingKind;
    HYD_CopyContinuousAbsoluteContext(&fb->_directContinuousAbsolute, &pendingContinuousAbsolute);
    fb->_directContinuousAbsolute.ownerTicket = fb->_directOwnerTicket;
```

- [ ] **Step 6: Build the core and existing arbitration tests before touching the new IEC adapter**

Run:

```bash
cmake --build --preset unixgcc --target HydroMotionLib test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:
- existing arbitration target still passes
- the new `test_movecontinuousabsolute_integration` target is still not linkable because the IEC adapter function does not exist yet

- [ ] **Step 7: Commit the direct-kind plumbing**

Run:

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c
git commit -m "Plumb explicit direct kinds for MoveContinuousAbsolute" -m "Constraint: The new command's approach segment looks like MoveAbsolute at the segment level, so kind inference alone is not sufficient\nRejected: Encoding the new FB as a fake MoveAbsolute kind | Would collapse ownership, takeover, and test semantics back into the old contract\nConfidence: high\nScope-risk: moderate\nDirective: Keep the explicit direct kind authoritative through immediate start, pending storage, and pending-slot promotion\nTested: cmake --build --preset unixgcc --target HydroMotionLib test_motion_interface_arbitration; ./out/build/unixgcc/test_motion_interface_arbitration\nNot-tested: The new IEC command target still lacks its adapter implementation"
```

### Task 4: Implement the IEC adapter start path and public signal skeleton

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_movecontinuousabsolute_integration.c`

- [ ] **Step 1: Add helpers for end-velocity direction and the new start wrapper**

Also add this include near the top of `src/motion_interface.c` with the other project headers:

```c
#include "segment_limits.h"
```

Insert these helpers near `mapPlcOpenDirection(...)` and `startDirectSegmentExecution(...)` in `src/motion_interface.c`:

```c
static HYD_MotionDirection mapContinuousEndVelocityDirectionRequest(IEC_SINT direction,
                                                                    HYD_BOOL* ok) {
    if (ok != NULL) {
        *ok = true;
    }

    switch ((int)direction) {
        case 1: return HYD_DIRECTION_POSITIVE;
        case 2: return HYD_DIRECTION_NEGATIVE;
        case 3: return HYD_DIRECTION_CURRENT;
        default:
            if (ok != NULL) {
                *ok = false;
            }
            return HYD_DIRECTION_HOLD;
    }
}

static HYD_MotionDirection resolveContinuousEndVelocityDirection(
    const HYD_MotionControlFB* fb,
    HYD_MotionDirection requestedDirection,
    HYD_MotionDirection approachDirection) {
    const HYD_REAL directionVelocityThreshold = 0.01f;

    if (requestedDirection != HYD_DIRECTION_CURRENT) {
        return requestedDirection;
    }
    if (fb != NULL) {
        if (fb->AXIS_REF.velocity > directionVelocityThreshold) {
            return HYD_DIRECTION_POSITIVE;
        }
        if (fb->AXIS_REF.velocity < -directionVelocityThreshold) {
            return HYD_DIRECTION_NEGATIVE;
        }
        if (fb->_lastActiveDirection == HYD_DIRECTION_POSITIVE ||
            fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE) {
            return fb->_lastActiveDirection;
        }
    }
    return approachDirection;
}

static HYD_DirectStartResult startDirectSegmentExecutionWithKind(
    HYD_MotionControlFB* fb,
    HYD_DirectCommandKind kind,
    HYD_BufferMode bufferMode,
    const HYD_MotionSegment* segment,
    const HYD_ContinuousAbsoluteContext* continuousAbsolute,
    IEC_WORD* errorId) {
    HYD_DirectStartResult result;

    result = HYD_MotionControlFB_StartDirectCommand(fb,
                                                    kind,
                                                    segment,
                                                    continuousAbsolute,
                                                    bufferMode,
                                                    fb->AXIS_REF.timestamp);
    if (result == HYD_DIRECT_START_REJECTED && errorId != NULL) {
        *errorId = commandFailureErrorId(fb);
    }
    return result;
}
```

- [ ] **Step 2: Add segment/context builders for the new command**

Insert these helpers near `buildPositionSegment(...)` / `buildVelocitySegment(...)`:

```c
static HYD_MotionSegment buildContinuousAbsoluteApproachSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    HYD_REAL pressureLimit,
    const HYD_MotionControlFB* fb) {
    HYD_MotionSegment seg = buildPositionSegment(targetPosition,
                                                 velocity,
                                                 acceleration,
                                                 deceleration,
                                                 direction,
                                                 fb);
    seg.maxPressure = pressureLimit;
    seg.velocityTolerance = fb->_params.velocityTolerance;
    return seg;
}

static HYD_ContinuousAbsoluteContext buildContinuousAbsoluteContext(
    HYD_REAL targetPosition,
    HYD_REAL endVelocityMagnitude,
    HYD_MotionDirection approachDirection,
    HYD_MotionDirection sustainDirection,
    HYD_REAL effectivePressureLimit,
    HYD_BOOL adaptEnabled) {
    HYD_ContinuousAbsoluteContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.valid = true;
    ctx.phase = HYD_CONTABS_PHASE_APPROACH;
    ctx.targetPosition = targetPosition;
    ctx.sustainVelocity = (sustainDirection == HYD_DIRECTION_NEGATIVE)
        ? -endVelocityMagnitude
        : endVelocityMagnitude;
    ctx.effectivePressureLimit = effectivePressureLimit;
    ctx.adaptEndVelEnabled = adaptEnabled;
    ctx.approachDirection = approachDirection;
    ctx.sustainDirection = sustainDirection;
    return ctx;
}
```

- [ ] **Step 3: Implement `__mcl_cmd_MoveContinuousAbsolute(...)`**

Add the new function beside `__mcl_cmd_MoveAbsolute(...)` and `__mcl_cmd_MoveVelocity(...)`:

```c
void __mcl_cmd_MoveContinuousAbsolute(HYD_MOVECONTINUOUSABSOLUTE *data__) {
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB* fb;
    IEC_WORD myExecId;
    IEC_BOOL isPending;

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB) {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    fb = &HYD_MotionControlFB_inst[axisIndex];
    myExecId = __GET_VAR(data__->_EXEC_ID);
    isPending = __GET_VAR(data__->_PENDING);

    if (!execute) {
        __SET_VAR(data__->, INENDVELOCITY, , false);
        __SET_VAR(data__->, POSITIONREACHED, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising) {
        IEC_WORD errorId = 0;
        HYD_BOOL endDirOk = false;
        HYD_MotionDirection configuredDirection;
        HYD_MotionDirection approachDirection;
        HYD_MotionDirection requestedEndDirection;
        HYD_MotionDirection sustainDirection;
        HYD_REAL requestedVelocity;
        HYD_REAL rawEndVelocity;
        HYD_REAL requestedEndVelocity;
        HYD_REAL pressureLimit;
        HYD_MotionSegment approachSegment;
        HYD_ContinuousAbsoluteContext context;
        HYD_DirectStartResult startResult;

        if (!validateUnsupportedMotionOptions(__GET_VAR(data__->JERK), &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        configuredDirection = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        requestedEndDirection = mapContinuousEndVelocityDirectionRequest(
            __GET_VAR(data__->ENDVELOCITYDIRECTION),
            &endDirOk);
        requestedVelocity = (HYD_REAL)fabs((double)__GET_VAR(data__->VELOCITY));
        rawEndVelocity = __GET_VAR(data__->ENDVELOCITY);
        requestedEndVelocity = (HYD_REAL)fabs((double)rawEndVelocity);
        pressureLimit = __GET_VAR(data__->PRESSURELIMIT);
        if (pressureLimit <= 0.0f) {
            pressureLimit = fb->PRESSURE_LIMIT;
        }
        sustainDirection = resolveContinuousEndVelocityDirection(fb,
                                                                 requestedEndDirection,
                                                                 approachDirection);

        if (!endDirOk || requestedVelocity <= 0.0f || rawEndVelocity < 0.0f ||
            !isfinite(requestedVelocity) || !isfinite(requestedEndVelocity) ||
            !isfinite(__GET_VAR(data__->ACCELERATION)) || __GET_VAR(data__->ACCELERATION) <= 0.0f ||
            !isfinite(pressureLimit)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        approachSegment = buildContinuousAbsoluteApproachSegment(__GET_VAR(data__->POSITION),
                                                                 requestedVelocity,
                                                                 __GET_VAR(data__->ACCELERATION),
                                                                 __GET_VAR(data__->DECELERATION),
                                                                 configuredDirection,
                                                                 pressureLimit,
                                                                 fb);
        approachDirection = HYD_Segment_ResolveDirection(&approachSegment,
                                                         &fb->AXIS_REF,
                                                         fb->_lastActiveDirection);
        context = buildContinuousAbsoluteContext(__GET_VAR(data__->POSITION),
                                                 requestedEndVelocity,
                                                 approachDirection,
                                                 sustainDirection,
                                                 pressureLimit,
                                                 __GET_VAR(data__->ADAPTENDVELTOAVOIDOVERSHOOT));

        startResult = startDirectSegmentExecutionWithKind(fb,
                                                          HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                          HYD_BUFFER_MODE_ABORT,
                                                          &approachSegment,
                                                          &context,
                                                          &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , startResult != HYD_DIRECT_START_STARTED);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb));
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, INENDVELOCITY, , false);
        __SET_VAR(data__->, POSITIONREACHED, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }
}
```

This step intentionally builds only the rising-edge skeleton. Do not try to finish steady-state output mapping until the runtime phase logic exists.

- [ ] **Step 4: Build the new target and confirm it links but still fails at runtime**

Run:

```bash
cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration
./out/build/unixgcc/test_movecontinuousabsolute_integration
```

Expected after this step:
- the target links successfully
- one or more runtime assertions fail because `PositionReached`, `InEndVelocity`, takeover, and pressure-limit semantics are not fully implemented yet

- [ ] **Step 5: Commit the adapter skeleton**

Run:

```bash
git add src/motion_interface.c
git commit -m "Add the MoveContinuousAbsolute IEC adapter skeleton" -m "Constraint: The new command must follow the repository's direct-command adapter style while exposing its own outputs instead of Done/Active\nRejected: Reusing MoveAbsolute's DONE-based mapping | The approved command stays Busy after target reach\nConfidence: medium\nScope-risk: moderate\nDirective: Keep public validation in the adapter and push motion math into the runtime core\nTested: cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration; ./out/build/unixgcc/test_movecontinuousabsolute_integration\nNot-tested: Phase switching, latching, and pressure-limit behavior are still incomplete"
```

### Task 5: Implement approach-to-sustain switching and signal latches in the runtime

**Files:**
- Modify: `include/segment_completion.h`
- Modify: `src/segment_completion.c`
- Modify: `src/motion_control.c`
- Test: `tests/test_movecontinuousabsolute_integration.c`
- Modify: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Expose a raw position-reached helper**

Add this declaration to `include/segment_completion.h`:

```c
HYD_BOOL HYD_SegmentCompletion_IsPositionReachedRaw(const HYD_MotionSegment* segment,
                                                    const HYD_AxisRef* axisRef,
                                                    HYD_REAL positionTolerance);
```

In `src/segment_completion.c`, rename the current private helper to the new public name and remove `static`:

```c
HYD_BOOL HYD_SegmentCompletion_IsPositionReachedRaw(
    const HYD_MotionSegment* segment,
    const HYD_AxisRef* axisRef,
    HYD_REAL positionTolerance) {
    HYD_MotionDirection direction;

    direction = HYD_Segment_ResolveDirection(segment, axisRef,
                                             HYD_DIRECTION_POSITIVE);
    if (direction == HYD_DIRECTION_POSITIVE) {
        return axisRef->position >= segment->targetPosition - positionTolerance;
    }
    if (direction == HYD_DIRECTION_NEGATIVE) {
        return axisRef->position <= segment->targetPosition + positionTolerance;
    }
    return fabs(axisRef->position - segment->targetPosition) <= positionTolerance;
}
```

Then update the existing completion path to call the new public helper:

```c
    rawComplete =
        HYD_SegmentCompletion_IsPositionReachedRaw(segment, axisRef, positionTolerance) &&
        HYD_SegmentCompletion_IsPositionVelocitySettled(segment, axisRef, references);
```

- [ ] **Step 2: Add the continuous-absolute math and tolerance helpers to `src/motion_control.c`**

Insert these helpers near the direct-command helper section:

```c
static HYD_REAL HYD_ResolveContinuousAbsoluteCrossingVelocity(
    const HYD_MotionControlFB* fb,
    const HYD_ContinuousAbsoluteContext* ctx,
    const HYD_MotionSegment* segment) {
    HYD_REAL remainingDistance;
    HYD_REAL projectedVelocity;
    HYD_REAL requested;
    HYD_REAL decel;

    if (fb == NULL || ctx == NULL || segment == NULL) {
        return 0.0f;
    }

    requested = fabs(ctx->sustainVelocity);
    decel = (segment->maxDeceleration > 0.0f) ? segment->maxDeceleration : segment->maxAcceleration;
    remainingDistance = fabs(segment->targetPosition - fb->AXIS_REF.position);
    projectedVelocity = fabs(fb->AXIS_REF.velocity);

    if (ctx->sustainDirection != ctx->approachDirection) {
        return 0.0f;
    }

    if (!ctx->adaptEndVelEnabled) {
        return requested;
    }

    if (projectedVelocity < requested) {
        HYD_REAL reachable = sqrt(projectedVelocity * projectedVelocity +
                                  2.0f * segment->maxAcceleration * remainingDistance);
        return HYD_ClampReal(reachable, 0.0f, requested);
    }

    if (projectedVelocity > requested) {
        HYD_REAL floorValue = projectedVelocity * projectedVelocity -
                              2.0f * decel * remainingDistance;
        HYD_REAL reachable = (floorValue > 0.0f) ? sqrt(floorValue) : 0.0f;
        if (reachable < requested) {
            return requested;
        }
        return reachable;
    }

    return requested;
}

static HYD_BOOL HYD_IsContinuousAbsoluteVelocityReached(const HYD_MotionSegment* segment,
                                                        HYD_REAL actualVelocity,
                                                        HYD_REAL targetVelocity) {
    HYD_REAL configuredTolerance = 0.0f;
    HYD_REAL tolerance;

    if (segment != NULL && segment->velocityTolerance > 0.0f) {
        configuredTolerance = segment->velocityTolerance;
    }

    tolerance = (configuredTolerance > 0.0f)
        ? configuredTolerance
        : ((fabs(targetVelocity) * 0.05f) > 0.01f
            ? (fabs(targetVelocity) * 0.05f)
            : 0.01f);

    return fabs(actualVelocity - targetVelocity) <= tolerance;
}

static HYD_MotionSegment HYD_BuildContinuousAbsoluteSustainSegment(
    const HYD_MotionControlFB* fb,
    const HYD_ContinuousAbsoluteContext* ctx,
    const HYD_MotionSegment* approachSegment) {
    HYD_MotionSegment seg;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = ctx->sustainDirection;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.maxVelocity = fabs(ctx->sustainVelocity);
    seg.maxAcceleration = approachSegment->maxAcceleration;
    seg.maxDeceleration = approachSegment->maxDeceleration;
    seg.maxFlow = (seg.maxVelocity > 0.0f)
        ? seg.maxVelocity * fb->_params.velocityToFlowGain
        : fb->_params.maxFlow;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;
    seg.velocityTolerance = approachSegment->velocityTolerance;
    seg.maxPressure = ctx->effectivePressureLimit;
    return seg;
}
```

- [ ] **Step 3: Reuse planner nonzero-terminal math by injecting a local blend context during `APPROACH`**

Inside `HYD_ExecuteActiveSegmentControl(...)`, before `plannerInput.blend` is assigned, add:

```c
    HYD_MotionBlendContext localContinuousBlend;
    const HYD_MotionBlendContext* selectedBlend = fb->_directBlendContext.active
        ? &fb->_directBlendContext
        : NULL;

    memset(&localContinuousBlend, 0, sizeof(localContinuousBlend));
    if (fb->_directOwnerKind == HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE &&
        fb->_directContinuousAbsolute.valid &&
        fb->_directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH) {
        localContinuousBlend.active = true;
        localContinuousBlend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
        localContinuousBlend.blendVelocity = fb->_directContinuousAbsolute.crossingVelocity;
        localContinuousBlend.switchPosition = fb->_directContinuousAbsolute.targetPosition;
        localContinuousBlend.switchTolerance = HYD_Segment_GetPositionTolerance(segment);
        selectedBlend = &localContinuousBlend;
    }
```

Then pass `selectedBlend` into the planner input:

```c
    plannerInput.blend = selectedBlend;
```

This keeps `src/motion_planner.c` unchanged.

- [ ] **Step 4: Switch to `SUSTAIN` before generic completion and latch both outputs**

In `HYD_MotionControlFB_RunRunningState(...)`, after `HYD_RunRunningStateBlendCutover(...)` and before `HYD_RunRunningStateCompletion(...)`, add:

```c
    if (fb->_directOwnerKind == HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE &&
        fb->_directContinuousAbsolute.valid &&
        fb->_directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH &&
        HYD_SegmentCompletion_IsPositionReachedRaw(segment,
                                                   &fb->AXIS_REF,
                                                   HYD_Segment_GetPositionTolerance(segment))) {
        HYD_MotionSegment sustainSegment =
            HYD_BuildContinuousAbsoluteSustainSegment(fb,
                                                      &fb->_directContinuousAbsolute,
                                                      segment);

        fb->_directContinuousAbsolute.positionReachedLatched = true;
        fb->_directContinuousAbsolute.phase = HYD_CONTABS_PHASE_SUSTAIN;
        if (fb->_directContinuousAbsolute.crossingVelocity == fb->_directContinuousAbsolute.sustainVelocity) {
            fb->_directContinuousAbsolute.inEndVelocityLatched = true;
        }

        fb->_previousSegmentMode = fb->_activeSegment.mode;
        HYD_SafetyStateManager_ResetRuntimeActuation(fb);
        HYD_StateReporter_ApplySafeOutputs(fb);
        HYD_StateReporter_ResetTransitionFlags(fb);
        fb->_activeSegment = sustainSegment;
        fb->_activeSegmentValid = true;
        fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
        fb->_segmentStartTime = fb->AXIS_REF.timestamp;
        HYD_PrimeSegmentControllers(fb, &fb->_activeSegment, fb->AXIS_REF.timestamp, false);
        HYD_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
        HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_STARTING);
        return;
    }
```

Then, after `HYD_ExecuteActiveSegmentControl(...)` and before diagnostics are published, latch `InEndVelocity` in the sustain phase:

```c
    if (fb->_directOwnerKind == HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE &&
        fb->_directContinuousAbsolute.valid &&
        fb->_directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN &&
        !fb->_directContinuousAbsolute.inEndVelocityLatched &&
        HYD_IsContinuousAbsoluteVelocityReached(segment,
                                                fb->AXIS_REF.velocity,
                                                fb->_directContinuousAbsolute.sustainVelocity)) {
        fb->_directContinuousAbsolute.inEndVelocityLatched = true;
    }
```

- [ ] **Step 5: Add the public-FB arbitration regression once the adapter symbol and runtime owner kind exist**

Append this test to `tests/test_motion_interface_arbitration.c` near the existing endless-owner takeover regressions:

```c
static void test_movecontinuousabsolute_active_owner_degrades_buffered_follower_to_abort_takeover(void) {
    HYD_MOVECONTINUOUSABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ENDVELOCITY) = 8.0f;
    IEC_VAL(first.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    __mcl_cmd_MoveContinuousAbsolute(&first);
    __HydMotion_framework_Publish();

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.POSITION) = 20.0f;
    IEC_VAL(second.VELOCITY) = 15.0f;
    IEC_VAL(second.ACCELERATION) = 80.0f;
    IEC_VAL(second.DECELERATION) = 80.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BUFFER;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.BUSY) == true,
               "Follower behind an active MoveContinuousAbsolute should be accepted by immediate takeover, not pending");
    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == false,
               "The front command reports COMMANDABORTED on the next scan, not the submission call");

    __HydMotion_framework_Publish();
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveContinuousAbsolute(&first);

    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == true,
               "The active MoveContinuousAbsolute should report COMMANDABORTED after the buffered follower degrades to takeover");
}
```

Register it in `main()`:

```c
    test_movecontinuousabsolute_active_owner_degrades_buffered_follower_to_abort_takeover();
```

- [ ] **Step 6: Run the focused integration and arbitration tests**

Run:

```bash
cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration test_motion_interface_arbitration
./out/build/unixgcc/test_movecontinuousabsolute_integration
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after this step:
- same-direction, adapt-down, adapt-up, and reverse-direction integration tests pass
- takeover and follower-admission arbitration passes
- pressure-limit integration may still need one more adapter/runtime pass if output mapping is incomplete

- [ ] **Step 7: Commit the phase-switch implementation**

Run:

```bash
git add include/segment_completion.h src/segment_completion.c src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "Teach MoveContinuousAbsolute to switch from approach to sustain" -m "Constraint: The command must reach target position without reusing MoveAbsolute's zero-velocity completion contract\nRejected: Changing src/motion_planner.c for a one-off API | The existing nonzero-terminal planner math is sufficient when driven by a local blend context\nConfidence: medium\nScope-risk: moderate\nDirective: Keep target reach and sustain latches in the runtime core; do not synthesize them in the IEC adapter\nTested: cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration test_motion_interface_arbitration; ./out/build/unixgcc/test_movecontinuousabsolute_integration; ./out/build/unixgcc/test_motion_interface_arbitration\nNot-tested: Full suite still pending, especially existing done-signal and limiter regressions"
```

### Task 6: Finalize adapter output mapping, pressure-limit propagation, and regression sweep

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `src/motion_interface.c`
- Modify: `src/motion_control.c`
- Modify: `tests/test_movecontinuousabsolute_integration.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_output_limiter.c`

- [ ] **Step 1: Finish per-command pressure-limit fallback and segment propagation**

In `src/motion_interface.c`, keep the fallback rule explicit on the rising-edge path:

```c
        pressureLimit = __GET_VAR(data__->PRESSURELIMIT);
        if (pressureLimit <= 0.0f) {
            pressureLimit = fb->PRESSURE_LIMIT;
        }
```

In `src/motion_control.c`, when building the sustain segment, preserve the same effective limit:

```c
        sustainSegment.maxPressure = fb->_directContinuousAbsolute.effectivePressureLimit;
```

Do not map the new input to `pressureCeiling` or any windowed mold-protect field.

- [ ] **Step 2: Finish non-rising-edge output mapping in `__mcl_cmd_MoveContinuousAbsolute(...)`**

Extend the non-rising-edge path so it mirrors the approved contract:

```c
    if (isPending) {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);
        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, POSITIONREACHED, , false);
            __SET_VAR(data__->, INENDVELOCITY, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
    }

    if (myExecId != 0) {
        if (directExecutionWasPreempted(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE) ||
            directExecutionLostOwnership(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, POSITIONREACHED, , false);
            __SET_VAR(data__->, INENDVELOCITY, , false);
        } else if (directExecutionIsCurrentOwner(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE)) {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, POSITIONREACHED, , fb->_directContinuousAbsolute.positionReachedLatched ? true : false);
            __SET_VAR(data__->, INENDVELOCITY, , fb->_directContinuousAbsolute.inEndVelocityLatched ? true : false);
            __SET_VAR(data__->, COMMANDABORTED, , false);
            if (HYD_MotionControlFB_IsError(fb)) {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
            }
        }
    }
```

- [ ] **Step 3: Tighten the pressure-limit test with explicit fallback and fault assertions**

Extend `test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false()` with:

```c
    ASSERT_TRUE(fb->_directContinuousAbsolute.effectivePressureLimit == 2.0f,
               "PressureLimit input 0 should fall back to fb->PRESSURE_LIMIT");
```

Add a fault-escalation test to the same file:

```c
static void test_pressure_limit_fault_surfaces_as_error(void) {
    HYD_MOVECONTINUOUSABSOLUTE mc;
    HYD_MotionControlFB* fb;
    int axisId;
    int step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Fault-escalation test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->PRESSURE_LIMIT = 1.0f;
    memset(&mc, 0, sizeof(mc));
    IEC_VAL(mc.EN) = true;
    IEC_VAL(mc.AXISID) = axisId;
    IEC_VAL(mc.EXECUTE) = true;
    mc.EXECUTE0.value = false;
    IEC_VAL(mc.POSITION) = 30.0f;
    IEC_VAL(mc.VELOCITY) = 20.0f;
    IEC_VAL(mc.ENDVELOCITY) = 10.0f;
    IEC_VAL(mc.ENDVELOCITYDIRECTION) = 1;
    IEC_VAL(mc.ACCELERATION) = 120.0f;
    IEC_VAL(mc.DIRECTION) = 1;

    __mcl_cmd_MoveContinuousAbsolute(&mc);

    for (step = 0; step < 3000; step++) {
        fb->AXIS_REF.pressure = 40.0f;
        __HydMotion_framework_Publish();
        hold_true_scan(&mc);
        if (IEC_VAL(mc.ERROR)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(mc.ERROR) == true,
               "Fault-level pressure limiting should surface ERROR on the command");
    ASSERT_TRUE(IEC_VAL(mc.BUSY) == false,
               "Fault-level pressure limiting should clear Busy on the command");
}
```

Register it in `main()`:

```c
    test_pressure_limit_fault_surfaces_as_error();
```

- [ ] **Step 4: Re-enable the dedicated integration target in the normal test graph**

Once `test_movecontinuousabsolute_integration` builds cleanly, update `CMakeLists.txt`:

```cmake
add_test(NAME test_movecontinuousabsolute_integration
         COMMAND test_movecontinuousabsolute_integration)
```

Leave the target itself as-is if explicit-only build is still useful during iteration, or remove `EXCLUDE_FROM_ALL` in the same step if the default preset build should now include it without failing.

- [ ] **Step 5: Run the focused regression bundle**

Run:

```bash
cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration test_motion_interface_arbitration test_motion_interface_done_signals test_output_limiter test_motion_interface_unit
./out/build/unixgcc/test_movecontinuousabsolute_integration
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_output_limiter
./out/build/unixgcc/test_motion_interface_unit
```

Expected:

```text
all focused targets exit 0
```

- [ ] **Step 6: Run the full suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected:

```text
100% tests passed, 0 tests failed out of ...
```

- [ ] **Step 7: Commit the completed feature**

Run:

```bash
git add CMakeLists.txt include/motion_control.h include/motion_interface.h include/segment_completion.h pousHydMotion.xml src/motion_control.c src/motion_interface.c src/segment_completion.c tests/test_movecontinuousabsolute_integration.c tests/test_motion_interface_arbitration.c
git commit -m "Add MoveContinuousAbsolute with sustained end velocity" -m "Constraint: The new command must preserve MoveAbsolute semantics while aligning PressureLimit with the repository's existing max-pressure chain\nRejected: Implementing the feature as MoveAbsolute plus user-side MoveVelocity chaining | Would split one approved lifecycle into two unrelated ownership contracts\nConfidence: medium\nScope-risk: broad\nDirective: Keep MoveContinuousAbsolute's target-reach and end-velocity latches runtime-owned, and keep pressure-limit handling on the existing maxPressure path only\nTested: cmake --build --preset unixgcc --target test_movecontinuousabsolute_integration test_motion_interface_arbitration test_motion_interface_done_signals test_output_limiter test_motion_interface_unit; ./out/build/unixgcc/test_movecontinuousabsolute_integration; ./out/build/unixgcc/test_motion_interface_arbitration; ./out/build/unixgcc/test_motion_interface_done_signals; ./out/build/unixgcc/test_output_limiter; ./out/build/unixgcc/test_motion_interface_unit; ctest --test-dir out/build/unixgcc --output-on-failure\nNot-tested: No PLC hardware validation; no recipe-side MoveProfile integration for continuous absolute"
```

## Self-review checklist

- Spec coverage:
  - public pin contract -> Task 1
  - `EndVelocityDirection=current` resolution order -> Tasks 2 and 4
  - same-owner two-phase runtime -> Tasks 3, 4, 5
  - overshoot adaptation and reverse sustain -> Tasks 2 and 5
  - pressure-limit fallback and nonconflicting max-pressure alignment -> Tasks 4 and 6
  - takeover semantics -> Tasks 5 and 6
- Placeholder scan:
  - no placeholder markers or "similar to above" shortcuts remain
- Type consistency:
  - `HYD_MOVECONTINUOUSABSOLUTE`
  - `HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE`
  - `HYD_ContinuousAbsoluteContext`
  - `HYD_CONTABS_PHASE_APPROACH`
  - `HYD_CONTABS_PHASE_SUSTAIN`
  - `HYD_SegmentCompletion_IsPositionReachedRaw`
  - `__mcl_cmd_MoveContinuousAbsolute`

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-02-movecontinuousabsolute.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
