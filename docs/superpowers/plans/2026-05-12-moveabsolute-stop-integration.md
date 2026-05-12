# MoveAbsolute + Stop Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `MoveAbsolute` and `Stop` work together in direct mode so that `Stop` preempts active positioning, decelerates to zero over published simulation cycles, reports `DONE` only after zero velocity, and allows the next command loop to restart cleanly after `EXECUTE` falls.

**Architecture:** Introduce a small direct-command session layer in the motion core, reuse `_executionId` as the command owner identity, and move stop completion semantics into `motion_control.c`. Keep `motion_interface.c` limited to PLCopen edge handling and output mapping. Preserve the existing explicit `__HydMotion_framework_Publish()` simulation contract.

**Tech Stack:** C99, existing HydroMotionLib core, IEC FB interface layer, CMake/CTest, direct-mode simulation via `__HydMotion_framework_Publish()`.

---

## File Map

- Modify: `include/motion_control.h`
  Add minimal direct-command session enums, session metadata fields, and session query APIs used by IEC mapping.
- Modify: `src/motion_control.c`
  Implement `Stop` as a first-class direct command, own stop deceleration lifecycle, and own preemption bookkeeping.
- Modify: `src/motion_interface.c`
  Replace abort-as-stop behavior with real stop request handling and map `MoveAbsolute`/`Stop` outputs from session facts instead of shared axis state guesses.
- Modify: `tests/test_motion_interface_done_signals.c`
  Update the existing stop-during-motion assertions to the corrected decelerating-stop timing semantics.
- Modify: `tests/test_motion_interface_arbitration.c`
  Update stop-success assertions so `Stop` completes after deceleration, not on the next cycle.
- Create: `tests/test_moveabsolute_stop_integration.c`
  Add the dedicated looped integration scenarios described in the spec.
- Modify: `CMakeLists.txt`
  Register the new integration test target and add it to `ctest`.

## Task 1: Add the Failing Dedicated Integration Test

**Files:**
- Create: `tests/test_moveabsolute_stop_integration.c`
- Modify: `CMakeLists.txt`
- Test: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Write the failing integration test file**

```c
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define CYCLE_PERIOD 0.001f

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
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

static void test_moveabsolute_stop_loop(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    HYD_MotionControlFB* fb;
    int axisId, step;
    int stopDoneCycle = 0;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 400.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 0;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 80; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    CHECK(fabs(fb->AXIS_REF.velocity) > 1.0f, "MoveAbsolute should build non-zero velocity before Stop");

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 50.0f;
    __mcl_cmd_Stop(&stop);

    CHECK(IEC_VAL(stop.DONE) == false, "Stop should not be done on the trigger call");

    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();

        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);

        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            stopDoneCycle = step + 1;
            break;
        }
    }

    CHECK(stopDoneCycle > 5, "Stop should require multiple publish cycles to decelerate");
    CHECK(IEC_VAL(ma.COMMANDABORTED) == true, "MoveAbsolute should report COMMANDABORTED after Stop takeover");
    CHECK(fabs(fb->AXIS_REF.velocity) < 0.01f, "Velocity should be near zero when Stop.DONE becomes true");
}

int main(void) {
    printf("=== MoveAbsolute + Stop Integration ===\n");
    test_moveabsolute_stop_loop();
    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: Register the new test target in `CMakeLists.txt`**

```cmake
add_executable(test_moveabsolute_stop_integration tests/test_moveabsolute_stop_integration.c)
target_link_libraries(test_moveabsolute_stop_integration PRIVATE HydroMotionLib)
```

```cmake
add_test(NAME test_moveabsolute_stop_integration COMMAND test_moveabsolute_stop_integration)
```

- [ ] **Step 3: Run the new test to verify it fails**

Run: `cmake --build out/build/unixgcc --target test_moveabsolute_stop_integration && ./out/build/unixgcc/test_moveabsolute_stop_integration`

Expected:
- build succeeds
- test fails because current `Stop` is still implemented as abort/immediate completion behavior
- failure text should mention either:
  - `Stop should require multiple publish cycles to decelerate`
  - `MoveAbsolute should report COMMANDABORTED after Stop takeover`
  - `Velocity should be near zero when Stop.DONE becomes true`

- [ ] **Step 4: Commit the red test**

```bash
git add CMakeLists.txt tests/test_moveabsolute_stop_integration.c
git commit -m "test: add failing moveabsolute stop integration coverage"
```

## Task 2: Add Minimal Direct-Command Session Metadata in the Core

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Add minimal direct-command enums and metadata to `include/motion_control.h`**

```c
typedef enum {
    HYD_DIRECT_CMD_NONE = 0,
    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
    HYD_DIRECT_CMD_STOP
} HYD_DirectCommandKind;

typedef enum {
    HYD_DIRECT_SESSION_IDLE = 0,
    HYD_DIRECT_SESSION_RUNNING,
    HYD_DIRECT_SESSION_STOPPING,
    HYD_DIRECT_SESSION_DONE,
    HYD_DIRECT_SESSION_ABORTED,
    HYD_DIRECT_SESSION_FAULT
} HYD_DirectSessionState;
```

```c
    HYD_DirectCommandKind _directOwnerKind;
    HYD_DirectSessionState _directSessionState;
    uint16_t _directOwnerExecutionId;
    uint16_t _lastPreemptedExecutionId;
    HYD_DirectCommandKind _lastPreemptedKind;
    HYD_BOOL _isStopping;
    HYD_TIME _stopStartTime;
    HYD_REAL _stopStartVel;
    HYD_REAL _stopDeceleration;
```

```c
HYD_DirectCommandKind HYD_MotionControlFB_GetDirectOwnerKind(const HYD_MotionControlFB* fb);
HYD_DirectSessionState HYD_MotionControlFB_GetDirectSessionState(const HYD_MotionControlFB* fb);
uint16_t HYD_MotionControlFB_GetDirectOwnerExecutionId(const HYD_MotionControlFB* fb);
HYD_BOOL HYD_MotionControlFB_WasExecutionPreempted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind);
```

- [ ] **Step 2: Initialize and reset the new metadata in `src/motion_control.c`**

```c
fb->_directOwnerKind = HYD_DIRECT_CMD_NONE;
fb->_directSessionState = HYD_DIRECT_SESSION_IDLE;
fb->_directOwnerExecutionId = 0U;
fb->_lastPreemptedExecutionId = 0U;
fb->_lastPreemptedKind = HYD_DIRECT_CMD_NONE;
fb->_isStopping = false;
fb->_stopStartTime = 0.0;
fb->_stopStartVel = 0.0;
fb->_stopDeceleration = 0.0;
```

- [ ] **Step 3: Add the small query helpers in `src/motion_control.c`**

```c
HYD_DirectCommandKind HYD_MotionControlFB_GetDirectOwnerKind(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directOwnerKind : HYD_DIRECT_CMD_NONE;
}

HYD_DirectSessionState HYD_MotionControlFB_GetDirectSessionState(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directSessionState : HYD_DIRECT_SESSION_IDLE;
}

uint16_t HYD_MotionControlFB_GetDirectOwnerExecutionId(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directOwnerExecutionId : 0U;
}

HYD_BOOL HYD_MotionControlFB_WasExecutionPreempted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind) {
    return (fb != NULL) &&
           fb->_lastPreemptedExecutionId == executionId &&
           fb->_lastPreemptedKind == kind;
}
```

- [ ] **Step 4: Rebuild and rerun the integration test**

Run: `cmake --build out/build/unixgcc --target test_moveabsolute_stop_integration && ./out/build/unixgcc/test_moveabsolute_stop_integration`

Expected:
- build succeeds
- test still fails
- failure should now be behavioral, not due to missing symbols or compile errors

- [ ] **Step 5: Commit the metadata scaffolding**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "refactor: add direct command session scaffolding"
```

## Task 3: Implement Core Stop Session Lifecycle

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Add the public stop request API to `include/motion_control.h`**

```c
HYD_BOOL HYD_MotionControlFB_Stop(HYD_MotionControlFB* fb,
                                  HYD_TIME timestamp,
                                  HYD_REAL deceleration);
```

- [ ] **Step 2: Add stop request queuing and owner takeover bookkeeping in `src/motion_control.c`**

```c
static HYD_BOOL HYD_RequestStopCommand(HYD_MotionControlFB* fb,
                                       HYD_TIME timestamp,
                                       HYD_REAL deceleration) {
    if (fb == NULL || fb->STATE.faultActive) {
        return false;
    }

    fb->_stopDeceleration = (deceleration > 0.0f) ? deceleration : 0.0f;
    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_STOP,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}
```

```c
HYD_BOOL HYD_MotionControlFB_Stop(HYD_MotionControlFB* fb,
                                  HYD_TIME timestamp,
                                  HYD_REAL deceleration) {
    return HYD_RequestStopCommand(fb, timestamp, deceleration);
}
```

```c
case HYD_CMD_STOP:
    fb->_lastPreemptedExecutionId = fb->_directOwnerExecutionId;
    fb->_lastPreemptedKind = fb->_directOwnerKind;
    fb->_executionId++;
    fb->_directOwnerExecutionId = fb->_executionId;
    fb->_directOwnerKind = HYD_DIRECT_CMD_STOP;
    fb->_directSessionState = HYD_DIRECT_SESSION_STOPPING;
    fb->_isStopping = true;
    fb->_stopStartTime = timestamp;
    fb->_stopStartVel = fb->AXIS_REF.velocity;
    if (fabs(fb->_stopStartVel) < 0.001f) {
        fb->_stopStartVel = fb->STATE.plannedVelocity;
    }
    return true;
```

- [ ] **Step 3: Implement the decelerating stop path in `HYD_MotionControlFB_RunRunningState`**

```c
if (fb->_isStopping) {
    HYD_REAL stopElapsed = fb->AXIS_REF.timestamp - fb->_stopStartTime;
    HYD_REAL stopMag = fabs(fb->_stopStartVel);
    HYD_REAL stopSign = (fb->_stopStartVel >= 0.0f) ? 1.0f : -1.0f;
    HYD_REAL stopDeceleration = (fb->_stopDeceleration > 0.0f) ?
        fb->_stopDeceleration : segment->maxAcceleration;
    HYD_REAL decelMag = stopMag - stopDeceleration * stopElapsed;

    if (decelMag < 0.0f) {
        decelMag = 0.0f;
    }

    plannerOutput.targetVelocity = decelMag * stopSign;
    plannerOutput.targetFlow = HYD_ClampReal(decelMag * segment->velocityToFlowGain,
                                             0.0f,
                                             segment->maxFlow);
    pumpOutput.commandFlow = plannerOutput.targetFlow;
    pumpOutput.pumpSpeed = HYD_ClampReal(plannerOutput.targetFlow * fb->FLOW_TO_PUMP_SPEED_GAIN,
                                         0.0f,
                                         fb->PUMP_SPEED_LIMIT);

    HYD_StateReporter_ReportExecution(fb,
                                      &plannerOutput,
                                      &pumpOutput,
                                      &executionReference,
                                      pressureOutput.appliedStrategy,
                                      &pressureOutput,
                                      &fb->DIAGNOSTIC);

    fb->_simFeedback.targetPosition = segment->targetPosition;
    fb->_simFeedback.targetVelocity = plannerOutput.targetVelocity;
    fb->_simFeedback.targetFlow = pumpOutput.commandFlow;
    fb->_simFeedback.targetPressure = executionReference.pressureReference;
    fb->_simFeedback.valid = true;

    if (decelMag < 0.001f && fabs(fb->AXIS_REF.velocity) < 0.01f) {
        fb->_isStopping = false;
        fb->_stopStartVel = 0.0f;
        fb->_stopDeceleration = 0.0f;
        fb->_directSessionState = HYD_DIRECT_SESSION_DONE;
        HYD_ProtectionManager_ApplyIdleState(fb, true, false);
        HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_DONE);
    }
    return;
}
```

- [ ] **Step 4: Keep direct-session state synchronized with normal and fault terminals**

```c
fb->_directSessionState = HYD_DIRECT_SESSION_RUNNING;
fb->_directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
fb->_directOwnerExecutionId = fb->_executionId;
```

```c
fb->_directSessionState = HYD_DIRECT_SESSION_ABORTED;
```

```c
fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
```

- [ ] **Step 5: Run the dedicated integration test**

Run: `cmake --build out/build/unixgcc --target test_moveabsolute_stop_integration && ./out/build/unixgcc/test_moveabsolute_stop_integration`

Expected:
- `Stop` now requires multiple cycles
- velocity is near zero when `Stop.DONE` becomes true
- test may still fail on IEC mapping or latch clearing until Task 4 is complete

- [ ] **Step 6: Commit the core stop lifecycle**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "feat: add decelerating stop session lifecycle"
```

## Task 4: Refactor IEC MoveAbsolute/Stop Output Mapping

**Files:**
- Modify: `src/motion_interface.c`
- Modify: `tests/test_motion_interface_done_signals.c`
- Modify: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_moveabsolute_stop_integration.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Replace abort-as-stop behavior in `__mcl_cmd_Stop`**

```c
if (execRising)
{
    HYD_MotionControlFB_Scan(fb);

    if (fb->FB_STATE != HYD_FB_STATE_STARTING &&
        fb->FB_STATE != HYD_FB_STATE_RUNNING &&
        fb->FB_STATE != HYD_FB_STATE_HOLD) {
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB_Stop(fb,
                             fb->AXIS_REF.timestamp,
                             __GET_VAR(data__->DECELERATION));
    HYD_MotionControlFB_Scan(fb);
    __SET_VAR(data__->, _PENDING, , true);
    __SET_VAR(data__->, BUSY, , true);
    __SET_VAR(data__->, DONE, , false);
    __SET_VAR(data__->, COMMANDABORTED, , false);
    __SET_VAR(data__->, EXECUTE0, , execute);
    return;
}
```

```c
if (isPending) {
    if (HYD_MotionControlFB_IsError(fb)) {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
    } else if (HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_STOP &&
               HYD_MotionControlFB_GetDirectSessionState(fb) == HYD_DIRECT_SESSION_DONE) {
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
    } else {
        __SET_VAR(data__->, BUSY, , true);
    }
}
```

- [ ] **Step 2: Map `MoveAbsolute` terminal state from execution ownership, not shared FB state guesses**

```c
if (myExecId != 0)
{
    if (HYD_MotionControlFB_WasExecutionPreempted(fb,
                                                  (uint16_t)myExecId,
                                                  HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, DONE, , false);
    } else if (myExecId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
               HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
        if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);
        }
    }
}
```

- [ ] **Step 3: Restore falling-edge latch clearing for `MoveAbsolute` and `Stop`**

```c
if (!execute) {
    __SET_VAR(data__->, DONE, , false);
    __SET_VAR(data__->, BUSY, , false);
    __SET_VAR(data__->, ACTIVE, , false);
    __SET_VAR(data__->, COMMANDABORTED, , false);
    __SET_VAR(data__->, ERROR, , false);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
    __SET_VAR(data__->, _PENDING, , false);
    __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
}
```

```c
if (!isPending && !execRising && !execute) {
    __SET_VAR(data__->, DONE, , false);
    __SET_VAR(data__->, BUSY, , false);
    __SET_VAR(data__->, COMMANDABORTED, , false);
    __SET_VAR(data__->, ERROR, , false);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
}
```

- [ ] **Step 4: Update the existing stop-related tests**

```c
ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
           "Stop should reach DONE after deceleration during active motion");
ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
           "MoveAbsolute should report COMMANDABORTED after Stop takeover");
ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
           "MoveAbsolute should not report DONE after Stop takeover");
```

```c
ASSERT_TRUE(stopDoneStep > 1,
           "Stop should not complete on the next cycle while decelerating");
```

- [ ] **Step 5: Run the targeted IEC tests**

Run: `cmake --build out/build/unixgcc --target test_moveabsolute_stop_integration test_motion_interface_done_signals test_motion_interface_arbitration`

Run: `./out/build/unixgcc/test_moveabsolute_stop_integration`

Run: `./out/build/unixgcc/test_motion_interface_done_signals`

Run: `./out/build/unixgcc/test_motion_interface_arbitration`

Expected:
- all three targeted binaries pass
- `Stop` no longer completes immediately during active motion
- `MoveAbsolute` reports `COMMANDABORTED` after stop takeover

- [ ] **Step 6: Commit the IEC mapping refactor**

```bash
git add src/motion_interface.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "refactor: map moveabsolute and stop from direct session state"
```

## Task 5: Full Regression and Final Verification

**Files:**
- Test: `out/build/unixgcc/*`

- [ ] **Step 1: Run the dedicated integration binary again**

Run: `./out/build/unixgcc/test_moveabsolute_stop_integration`

Expected: PASS

- [ ] **Step 2: Run the full CTest suite**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure`

Expected:
- all registered tests pass
- zero failures

- [ ] **Step 3: Review the working tree for only intended implementation changes**

Run: `git status --short`

Expected:
- only the planned source, test, and CMake files are modified
- no accidental generated files outside ignored build directories

- [ ] **Step 4: Commit the final verified integration pass**

```bash
git add CMakeLists.txt include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_moveabsolute_stop_integration.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "feat: unify moveabsolute and stop direct command lifecycle"
```
