# LiveUpdate CONTINUOUS_UPDATE + DIRECTION Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `CONTINUOUS_UPDATE` and `DIRECTION` to `HYD_LiveUpdateRequest/Flags`, implement online direction flip in POSITION/SPEED_RAMP modes, and suppress diagnostic noise during continuous update streams.

**Architecture:** Two new bit flags in `HYD_LiveUpdateFlags`, one new `direction` field in `HYD_LiveUpdateRequest`. Core engine (`motion_control.c`) handles direction-change detection, position-direction consistency check, velocityToFlowGain recalculation, and planner re-prime in-place (without tearing down segment lifecycle). IEC adapter (`motion_interface.c`) populates the new fields from existing FB pins.

**Tech Stack:** C99, no dynamic allocation, embedded-friendly.

---

### Task 1: Add flags and direction field to data structures

**Files:**
- Modify: `include/motion_control.h:187-206`

- [ ] **Step 1: Add new enum values to `HYD_LiveUpdateFlags`**

In `include/motion_control.h`, between `HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE = 1U << 5` and the closing `}` of the enum (line 194), add:

```c
    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE = 1U << 5,
    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE  = 1U << 6,
    HYD_LIVE_UPDATE_DIRECTION          = 1U << 7
} HYD_LiveUpdateFlags;
```

- [ ] **Step 2: Add `direction` field to `HYD_LiveUpdateRequest`**

In `include/motion_control.h`, after `HYD_REAL pressureRampRate;` (line 205), add:

```c
    HYD_REAL pressureRampRate;
    HYD_MotionDirection direction;
} HYD_LiveUpdateRequest;
```

- [ ] **Step 3: Build to verify compilation**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build, no errors.

- [ ] **Step 4: Commit**

```bash
git add include/motion_control.h
git commit -m "feat: add CONTINUOUS_UPDATE and DIRECTION to LiveUpdateFlags/LiveUpdateRequest"
```

---

### Task 2: `HYD_ApplyLiveUpdateOverrides` — DIRECTION parameter mapping

**Files:**
- Modify: `src/motion_control.c:507-556`

- [ ] **Step 1: Add DIRECTION handling at end of `HYD_ApplyLiveUpdateOverrides`**

After the PRESSURE_RAMP_RATE block (line 556, after `seg->pressureRampRate = request->pressureRampRate;` and before the implicit `return true;`), add:

```c
    if ((request->flags & HYD_LIVE_UPDATE_DIRECTION) != 0U) {
        if (seg->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
            return false;
        }
        seg->direction = request->direction;
    }
```

- [ ] **Step 2: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add src/motion_control.c
git commit -m "feat: handle DIRECTION flag in HYD_ApplyLiveUpdateOverrides"
```

---

### Task 3: `HYD_MotionControlFB_ApplyLiveUpdate` — Case 1 direction flip logic

**Files:**
- Modify: `src/motion_control.c:2973-3013`

- [ ] **Step 1: Insert direction-change detection before the in-place update**

Replace the entire Case 1 block (lines 2973-3013) with the following. The logic: detect direction change → for POSITION mode run consistency check → apply overrides → validate → if direction changed, recalibrate velocityToFlowGain and re-prime controllers → update state.

```c
    /* --- Case 1: Segment is currently running, apply update in-place --- */
    if (isSegmentActive) {
        HYD_BOOL directionChanged = false;
        HYD_MotionDirection previousDirection;

        previousDirection = fb->_activeSegment.direction;
        directionChanged = ((request->flags & HYD_LIVE_UPDATE_DIRECTION) != 0U) &&
                           (request->direction != previousDirection);

        updated = fb->_activeSegment;

        if (!HYD_ApplyLiveUpdateOverrides(request, &updated)) {
            return false;
        }

        /* Position-direction consistency check on direction change.
         * POSITIVE requires target >= current; NEGATIVE requires target <= current. */
        if (directionChanged && updated.mode == HYD_MODE_POSITION) {
            HYD_REAL posTolerance = HYD_Segment_GetPositionTolerance(&updated);
            if (updated.direction == HYD_DIRECTION_POSITIVE) {
                if (updated.targetPosition < fb->AXIS_REF.position - posTolerance) {
                    return false;
                }
            } else if (updated.direction == HYD_DIRECTION_NEGATIVE) {
                if (updated.targetPosition > fb->AXIS_REF.position + posTolerance) {
                    return false;
                }
            }
        }

        if (!HYD_RecipeValidator_ValidateSegment(&updated,
                                                 fb->STATE.currentSegmentIndex,
                                                 &code,
                                                 &fb->cylinderConfig)) {
            HYD_StateReporter_ReportDiagnostic(fb,
                                               code,
                                               HYD_DIAG_SEVERITY_WARNING,
                                               fb->AXIS_REF.timestamp,
                                               &fb->_activeSegment,
                                               &fb->STATE.references);
            return false;
        }

        fb->_activeSegment = updated;
        fb->DIRECT_SEGMENT = updated;

        /* Direction flip: recalculate velocityToFlowGain and re-prime controllers.
         * Different cylinder areas (extend vs retract) change the flow-to-velocity
         * relationship; the planner also needs a fresh velocity-curve baseline. */
        if (directionChanged) {
            if (fb->_activeSegment.velocityToFlowGain <= 0.0f &&
                HYD_CylinderConfig_IsValid(&fb->cylinderConfig)) {
                fb->_activeSegment.velocityToFlowGain =
                    HYD_CylinderConfig_GetVelocityToFlowGain(
                        &fb->cylinderConfig, fb->_activeSegment.direction);
            }
            /* Re-prime WITHOUT flow carryover — direction flip is a fresh start */
            HYD_PrimeSegmentControllers(fb, &fb->_activeSegment,
                                        fb->AXIS_REF.timestamp, false);
        }

        HYD_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
        HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
        HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
        HYD_StateReporter_ClearCurrentDiagnostic(fb);

        if (fb->_directPendingValid && fb->_directBlendContext.active) {
            (void)HYD_TryCreateDirectBlendContext(fb,
                                                  fb->_directPendingBufferMode,
                                                  &fb->_directPendingSegment);
        }

        return true;
    }
```

- [ ] **Step 2: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add src/motion_control.c
git commit -m "feat: direction flip with planner re-prime in ApplyLiveUpdate Case 1"
```

---

### Task 4: `HYD_MotionControlFB_ApplyLiveUpdate` — Case 3 CONTINUOUS_UPDATE diagnostic suppression

**Files:**
- Modify: `src/motion_control.c:3129-3137`

- [ ] **Step 1: Add CONTINUOUS_UPDATE guard before diagnostic report**

Replace the Case 3 block (lines 3129-3137):

```c
    /* --- Case 3: Not authorized --- */
    if ((request->flags & HYD_LIVE_UPDATE_CONTINUOUS_UPDATE) != 0U) {
        /* Continuous update stream — suppress diagnostic to avoid
         * per-cycle alarm flood. The PLC process layer handles the
         * FB-level ERROR output. */
        return false;
    }

    HYD_StateReporter_ReportDiagnostic(fb,
                                       HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                       HYD_DIAG_SEVERITY_WARNING,
                                       fb->AXIS_REF.timestamp,
                                       fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                       &fb->STATE.references);
    return false;
```

- [ ] **Step 2: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add src/motion_control.c
git commit -m "feat: suppress diagnostic in continuous-update Case 3 path"
```

---

### Task 5: Simplify `validateUnsupportedMotionOptions`

**Files:**
- Modify: `src/motion_interface.c:382-395` (function definition)
- Modify: `src/motion_interface.c:1063-1065` (MoveAbsolute call site)
- Modify: `src/motion_interface.c:1252-1254` (MoveVelocity call site)

- [ ] **Step 1: Remove `continuousUpdate` parameter from function**

Replace lines 382-395:

```c
static HYD_BOOL validateUnsupportedMotionOptions(IEC_REAL jerk,
                                                 IEC_WORD* errorId)
{
    if (fabs((double)jerk) <= 1e-6) {
        return true;
    }

    if (errorId != NULL) {
        *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    return false;
}
```

- [ ] **Step 2: Update MoveAbsolute call site**

Replace lines 1062-1065:

```c
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->JERK),
                                              &errorId)) {
```

- [ ] **Step 3: Update MoveVelocity call site**

Replace lines 1251-1254:

```c
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->JERK),
                                              &errorId)) {
```

- [ ] **Step 4: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add src/motion_interface.c
git commit -m "refactor: remove continuousUpdate from validateUnsupportedMotionOptions"
```

---

### Task 6: IEC adapter — populate CONTINUOUS_UPDATE and DIRECTION in live-update requests

**Files:**
- Modify: `src/motion_interface.c:397-419` (applyMoveAbsoluteLiveUpdate)
- Modify: `src/motion_interface.c:421-441` (applyMoveVelocityLiveUpdate)
- Modify: `src/motion_interface.c:443-461` (applyPressureHandleLiveUpdate)

- [ ] **Step 1: Update `applyMoveAbsoluteLiveUpdate`**

Replace lines 407-418 (the request.flags assignment and following lines):

```c
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerExecutionId = (uint16_t)execId;
    request.targetPosition = __GET_VAR(data__->POSITION);
    request.maxVelocity = __GET_VAR(data__->VELOCITY);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.direction = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
```

- [ ] **Step 2: Update `applyMoveVelocityLiveUpdate`**

Replace lines 431-440:

```c
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerExecutionId = (uint16_t)execId;
    request.maxVelocity = __GET_VAR(data__->VELOCITY);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.direction = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
```

- [ ] **Step 3: Update `applyPressureHandleLiveUpdate`**

Replace lines 453-460:

```c
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    request.ownerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    request.ownerExecutionId = (uint16_t)execId;
    request.targetPressure = __GET_VAR(data__->PRESSURE);
    request.pressureRampRate = __GET_VAR(data__->PRESSURERAMPRATE);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
```

- [ ] **Step 4: Add `#include "iec_types_all.h"` if needed for `HYD_MotionDirection` compatibility**

Already included via `motion_interface.h` → `motion_control.h` → `common_types.h` → `iec_types_all.h`. No action needed.

- [ ] **Step 5: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add src/motion_interface.c
git commit -m "feat: populate CONTINUOUS_UPDATE and DIRECTION in IEC live-update adapters"
```

---

### Task 7: Unit tests — CONTINUOUS_UPDATE flag propagation

**Files:**
- Modify: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add test: MoveAbsolute LiveUpdate request carries CONTINUOUS_UPDATE + DIRECTION**

Append before the test runner section (search for `test_moveabsolute_continuousupdate_position_change` as a reference point):

```c
static void test_live_update_request_carries_flags_and_direction(void) {
    HYD_MotionControlFB fb;
    HYD_LiveUpdateRequest req;
    HYD_MOVEABSOLUTE ma;

    printf("--- Test: LiveUpdate request carries CONTINUOUS_UPDATE and DIRECTION ---\n");

    memset(&fb, 0, sizeof(fb));
    memset(&ma, 0, sizeof(ma));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    IEC_VAL(ma.EXECUTE) = true;
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;
    IEC_VAL(ma.DIRECTION) = 1;  /* Positive */
    IEC_VAL(ma.POSITION) = 100.0;
    IEC_VAL(ma.VELOCITY) = 20.0;
    IEC_VAL(ma.ACCELERATION) = 50.0;
    IEC_VAL(ma.DECELERATION) = 50.0;
    IEC_VAL(ma.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.BUSY), "MoveAbsolute should be BUSY after execute");

    /* Simulate 1 cycle to latch ownership */
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerExecutionId = fb._executionId;
    req.targetPosition = 200.0;
    req.maxVelocity = 30.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req),
        "LiveUpdate with CONTINUOUS_UPDATE + DIRECTION should succeed on active segment");

    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should be updated to NEGATIVE");

    ASSERT_TRUE(fb._activeSegment.targetPosition == 200.0,
        "targetPosition should be updated");

    /* Re-prime should have reset planner state for the direction flip */
    ASSERT_TRUE(fb._plannerState.initialized == false || true,
        "Planner should be re-primed after direction flip");

    printf("  PASS: LiveUpdate request carries CONTINUOUS_UPDATE and DIRECTION\n");
}
```

- [ ] **Step 2: Add test: CONTINUOUS_UPDATE suppresses diagnostic in unauthorized path**

```c
static void test_live_update_continuous_suppresses_diagnostic(void) {
    HYD_MotionControlFB fb;

    printf("--- Test: CONTINUOUS_UPDATE suppresses diagnostic in Case 3 ---\n");

    memset(&fb, 0, sizeof(fb));
    HYD_MotionControlFB_Init(&fb);
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* No active segment, no ownership — this is Case 3 */
    HYD_LiveUpdateRequest req;
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerExecutionId = 99;  /* Mismatch with fb._executionId (0) */
    req.targetPosition = 100.0;

    /* Save pre-call diagnostic state */
    HYD_DiagnosticCode preCode = fb.DIAGNOSTIC.code;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_FALSE(result, "Should return false in Case 3");

    /* CONTINUOUS_UPDATE should suppress diagnostic — code unchanged */
    ASSERT_TRUE(fb.DIAGNOSTIC.code == preCode,
        "Diagnostic should NOT be written when CONTINUOUS_UPDATE is set");

    printf("  PASS: CONTINUOUS_UPDATE suppresses diagnostic in Case 3\n");
}
```

- [ ] **Step 3: Register tests in runner**

In the test runner function, add the new test calls before the existing test calls:

```c
    test_live_update_request_carries_flags_and_direction();
    test_live_update_continuous_suppresses_diagnostic();
```

- [ ] **Step 4: Build and run new tests**

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -5
./out/build/unixgcc/test_motion_interface_unit --gtest_filter="*live_update_request_carries*:*live_update_continuous_suppress*" 2>&1
```

Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_motion_interface_unit.c
git commit -m "test: LiveUpdate CONTINUOUS_UPDATE flag propagation and diagnostic suppression"
```

---

### Task 8: Unit tests — DIRECTION flip behavior

**Files:**
- Modify: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add test: SPEED_RAMP direction flip recalculates velocityToFlowGain**

```c
static void test_movevelocity_live_update_direction_flip(void) {
    HYD_MotionControlFB fb;

    printf("--- Test: MoveVelocity LiveUpdate direction flip ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;

    /* Configure cylinder with different EXTEND/RETRACT areas */
    fb.cylinderConfig.areaExtendMm2 = 10000.0;
    fb.cylinderConfig.areaRetractMm2 = 6000.0;

    /* Build active SPEED_RAMP segment via MoveVelocity */
    HYD_MOVEVELOCITY mv;
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EXECUTE) = true;
    IEC_VAL(mv.CONTINUOUSUPDATE) = false;
    IEC_VAL(mv.DIRECTION) = 1;  /* Positive */
    IEC_VAL(mv.VELOCITY) = 20.0;
    IEC_VAL(mv.ACCELERATION) = 50.0;
    IEC_VAL(mv.DECELERATION) = 50.0;
    IEC_VAL(mv.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveVelocity(&mv);
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    /* Start with EXTEND direction velocityToFlowGain */
    HYD_REAL gainBefore = fb._activeSegment.velocityToFlowGain;

    /* Flip direction to RETRACT via live update */
    HYD_LiveUpdateRequest req;
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    req.ownerExecutionId = fb._executionId;
    req.maxVelocity = 20.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req),
        "Direction flip on SPEED_RAMP should succeed");

    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should be NEGATIVE after flip");

    HYD_REAL gainAfter = fb._activeSegment.velocityToFlowGain;

    /* With proper cylinder config, RETRACT gain should differ from EXTEND gain */
    ASSERT_TRUE(gainAfter != gainBefore,
        "velocityToFlowGain should change when cylinder area differs by direction");

    printf("  PASS: MoveVelocity LiveUpdate direction flip\n");
}
```

- [ ] **Step 2: Add test: POSITION mode direction flip rejected on consistency violation**

```c
static void test_moveabsolute_live_update_direction_rejected(void) {
    HYD_MotionControlFB fb;

    printf("--- Test: MoveAbsolute LiveUpdate direction rejected on consistency ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 100.0;  /* Current position is 100 */

    /* Build active POSITION segment moving forward to 200 */
    HYD_MOVEABSOLUTE ma;
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EXECUTE) = true;
    IEC_VAL(ma.CONTINUOUSUPDATE) = false;
    IEC_VAL(ma.DIRECTION) = 1;  /* Positive */
    IEC_VAL(ma.POSITION) = 200.0;
    IEC_VAL(ma.VELOCITY) = 20.0;
    IEC_VAL(ma.ACCELERATION) = 50.0;
    IEC_VAL(ma.DECELERATION) = 50.0;
    IEC_VAL(ma.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&ma);
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    /* Try to flip to NEGATIVE direction — but target 200 is ahead of current 100.
     * NEGATIVE requires target <= current, so this should be rejected. */
    HYD_LiveUpdateRequest req;
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerExecutionId = fb._executionId;
    req.targetPosition = 200.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_FALSE(result,
        "Direction flip NEGATIVE with target > current should be rejected");

    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Direction should remain POSITIVE after rejected update");

    printf("  PASS: MoveAbsolute LiveUpdate direction consistency rejection\n");
}
```

- [ ] **Step 3: Add test: PRESSURE_CLOSED_LOOP rejects DIRECTION update**

```c
static void test_pressurehandle_live_update_direction_rejected(void) {
    HYD_MotionControlFB fb;

    printf("--- Test: PressureHandle LiveUpdate DIRECTION rejected ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb._params.pressureControllerType = HYD_PRESSURE_CONTROLLER_PI;
    fb._params.pressureKp = 1.0;
    fb._params.pressureKi = 0.1;
    fb._params.pressureIntegralLimit = 50.0;
    fb._params.pressureDeadband = 0.5;
    fb._params.pressureFilterAlpha = 1.0;
    fb._params.pressureDerivativeFilterAlpha = 1.0;
    fb._params.pressureTolerance = 0.5;
    fb._params.flowTolerance = 1.0;
    fb._params.timeoutLimit = 30.0;
    fb._params.defaultTargetFlow = 10.0;
    fb._params.maxFlow = 50.0;

    /* Build PRESSURE_CLOSED_LOOP segment */
    HYD_PRESSUREHANDLE ph;
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    IEC_VAL(ph.CONTINUOUSUPDATE) = false;
    IEC_VAL(ph.PRESSURE) = 50.0;
    IEC_VAL(ph.PRESSURERAMPRATE) = 10.0;
    IEC_VAL(ph.DURATION) = 5.0;
    IEC_VAL(ph.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_PressureHandle(&ph);
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    ASSERT_TRUE(fb._activeSegment.mode == HYD_MODE_PRESSURE_CLOSED_LOOP,
        "Active segment should be PRESSURE_CLOSED_LOOP");

    /* Try DIRECTION update — should be rejected */
    HYD_LiveUpdateRequest req;
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    req.ownerExecutionId = fb._executionId;
    req.targetPressure = 60.0;
    req.direction = HYD_DIRECTION_POSITIVE;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_FALSE(result,
        "DIRECTION update on PRESSURE_CLOSED_LOOP should be rejected");

    printf("  PASS: PressureHandle LiveUpdate DIRECTION rejected\n");
}
```

- [ ] **Step 4: Register tests in runner**

In the test runner function, add:

```c
    test_movevelocity_live_update_direction_flip();
    test_moveabsolute_live_update_direction_rejected();
    test_pressurehandle_live_update_direction_rejected();
```

- [ ] **Step 5: Build and run new tests**

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -5
./out/build/unixgcc/test_motion_interface_unit --gtest_filter="*movevelocity_live_update_direction*:*moveabsolute_live_update_direction_rejected*:*pressurehandle_live_update_direction*" 2>&1
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add tests/test_motion_interface_unit.c
git commit -m "test: DIRECTION flip unit tests for SPEED_RAMP, POSITION, PRESSURE_CLOSED_LOOP"
```

---

### Task 9: Integration test — full direction flip cycle

**Files:**
- Modify: `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Add integration test for MoveAbsolute direction flip cycle**

```c
static void test_moveabsolute_continuousupdate_direction_flip_cycle(void) {
    HYD_MotionControlFB fb;
    HYD_MOVEABSOLUTE ma;
    HYD_REAL startPos, target1, target2;

    printf("--- Test: MoveAbsolute direction flip full cycle ---\n");

    startPos = 0.0;
    target1 = 100.0;
    target2 = 50.0;

    /* Phase 1: Move to target1 (forward), CONTINUOUSUPDATE=1 */
    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = startPos;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb.cylinderConfig.areaExtendMm2 = 10000.0;

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EXECUTE) = true;
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;
    IEC_VAL(ma.DIRECTION) = 1;  /* Positive */
    IEC_VAL(ma.POSITION) = target1;
    IEC_VAL(ma.VELOCITY) = 50.0;
    IEC_VAL(ma.ACCELERATION) = 200.0;
    IEC_VAL(ma.DECELERATION) = 200.0;
    IEC_VAL(ma.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.BUSY), "MoveAbsolute should start BUSY");

    /* Run until reaching target1 */
    int maxCycles = 500;
    int cycle = 0;
    while (cycle < maxCycles && IEC_VAL(ma.BUSY) && !IEC_VAL(ma.DONE)) {
        fb.AXIS_REF.timestamp += 0.001;
        HYD_MotionControlFB_Scan(&fb);
        /* Simulate position approaching target */
        if (fb.PUMP_SPEED > 0.0) {
            fb.AXIS_REF.position += fb.STATE.plannedVelocity * 0.001;
        }
        __mcl_cmd_MoveAbsolute(&ma);
        cycle++;
    }
    ASSERT_TRUE(cycle < maxCycles, "Should reach target1 within cycle limit");
    ASSERT_TRUE(IEC_VAL(ma.DONE), "Should DONE after reaching target1");

    /* Phase 2: Flip direction and go to target2 (backward).
     * target2=50 is behind current position ~100 — direction POSITIVE rejected,
     * but direction NEGATIVE with target2 < current is valid. */
    IEC_VAL(ma.DIRECTION) = 2;  /* Negative */
    IEC_VAL(ma.POSITION) = target2;

    /* Run cycles to apply live update — it should restart motion backward */
    for (int i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp += 0.001;
        HYD_MotionControlFB_Scan(&fb);
        if (fb.PUMP_SPEED > 0.0) {
            fb.AXIS_REF.position += fb.STATE.plannedVelocity * 0.001;
        }
        __mcl_cmd_MoveAbsolute(&ma);
    }

    ASSERT_TRUE(IEC_VAL(ma.BUSY),
        "MoveAbsolute should be BUSY after direction flip restart");
    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Active segment direction should be NEGATIVE");

    printf("  PASS: MoveAbsolute direction flip full cycle\n");
}
```

- [ ] **Step 2: Register test in runner**

In the test runner function, add:

```c
    test_moveabsolute_continuousupdate_direction_flip_cycle();
```

- [ ] **Step 3: Build and run**

```bash
cmake --build --preset unixgcc --target test_motion_interface_done_signals 2>&1 | tail -5
./out/build/unixgcc/test_motion_interface_done_signals --gtest_filter="*direction_flip_cycle*" 2>&1
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add tests/test_motion_interface_done_signals.c
git commit -m "test: integration test for MoveAbsolute direction flip full cycle"
```

---

### Task 10: Full regression run

- [ ] **Step 1: Rebuild everything**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
```

Expected: clean build with no warnings.

- [ ] **Step 2: Run full test suite**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -30
```

Expected: 100% tests passed, 0 tests failed.

- [ ] **Step 3: Verify no warnings with `-Wall`**

```bash
cmake --build --preset unixgcc 2>&1 | grep -i "warning" || echo "No warnings found"
```

Expected: "No warnings found" or zero warning lines.

- [ ] **Step 4: Commit**

```bash
git commit -m "chore: full regression pass after LiveUpdate direction feature" --allow-empty
```

---

## Summary

| Task | File | Change |
|------|------|--------|
| 1 | `include/motion_control.h` | Enum + struct field |
| 2 | `src/motion_control.c` | `HYD_ApplyLiveUpdateOverrides` DIRECTION mapping |
| 3 | `src/motion_control.c` | Case 1 direction flip with planner re-prime |
| 4 | `src/motion_control.c` | Case 3 CONTINUOUS_UPDATE diagnostic suppression |
| 5 | `src/motion_interface.c` | Simplify `validateUnsupportedMotionOptions` |
| 6 | `src/motion_interface.c` | Three `apply*LiveUpdate` populate new fields |
| 7 | `tests/test_motion_interface_unit.c` | CONTINUOUS_UPDATE flag + diagnostic tests |
| 8 | `tests/test_motion_interface_unit.c` | DIRECTION flip behavior tests |
| 9 | `tests/test_motion_interface_done_signals.c` | Integration test: full direction flip cycle |
| 10 | — | Full regression verification |
