# MoveAbsolute Buffered Blending Activation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reproduce and fix the direct `MoveAbsolute -> MoveAbsolute` buffered blending bug so a pending second FB cannot report false `ACTIVE` or push the first segment to the second segment's full velocity before the cutover region.

**Architecture:** Keep the current one-active-one-pending direct buffer model and the existing direct-ticket lifecycle. Extend the direct start path so the IEC adapter can distinguish immediate ownership from pending acceptance, then delay planner participation for buffered blends until an explicit arming window near the cutover. The work stays within the current `motion_interface.c` / `motion_control.c` split and does not redesign planner math or public IEC pin surfaces.

**Tech Stack:** C99, matiec IEC structs/macros, CMake preset `unixgcc`, C unit tests under `tests/`, static bounded memory, no heap allocation.

---

## File Structure

- Modify `tests/test_moveabsolute_blending_done.c`
  - add the failing early-trigger regressions for `BlendingHigh` and `BlendingLow`
  - validate that `FB2.ACTIVE` stays false and the first segment velocity is not overwritten before cutover
- Modify `tests/test_motion_interface_arbitration.c`
  - add pending-state contract coverage (`BUSY=true`, `ACTIVE=false`)
  - add queue-depth enforcement for third same-axis `MoveAbsolute`
- Modify `include/motion_control.h`
  - add a small direct-start result enum so callers can distinguish `started now` from `queued pending`
- Modify `src/motion_interface.c`
  - propagate the new direct-start result through `startDirectSegmentExecution(...)`
  - map pending acceptance to `BUSY=true`, `ACTIVE=false`
- Modify `src/motion_control.c`
  - return `started vs queued` from direct submission
  - keep recording direct blend context on pending acceptance
  - add an explicit "blend armed" gate before the planner consumes the pending blend

No `motion_interface.h`, XML POU layout, or public FB pin changes are needed.

### Task 1: Reproduce the early-trigger buffered blend bug first

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Add helpers that trigger the second FB from the first FB's `ACTIVE` state**

Insert these helpers below `rising_edge(...)` in `tests/test_moveabsolute_blending_done.c`:

```c
static void hold_true_scan(HYD_MOVEABSOLUTE* ma) {
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);
}

static int trigger_fb2_when_fb1_active(HYD_MOVEABSOLUTE* fb1,
                                       HYD_MOVEABSOLUTE* fb2,
                                       int maxWaitScans) {
    for (int step = 0; step < maxWaitScans; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb1);
        if (IEC_VAL(fb1->ACTIVE)) {
            rising_edge(fb2);
            return step + 1;
        }
    }
    return -1;
}

static HYD_BOOL velocity_overwritten_before_cutover(HYD_MotionControlFB* core,
                                                    HYD_REAL switchPosition,
                                                    HYD_REAL forbiddenVelocity,
                                                    HYD_REAL tolerance) {
    if (core == NULL) {
        return false;
    }
    if (core->AXIS_REF.position >= switchPosition - 5.0f) {
        return false;
    }
    return fabs(core->_plannerState.lastTargetVelocity - forbiddenVelocity) <= tolerance;
}
```

- [ ] **Step 2: Add the failing `BlendingHigh` early-trigger regression**

Insert this test above `main()` in `tests/test_moveabsolute_blending_done.c`:

```c
static void test_blending_high_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    HYD_MOVEABSOLUTE fb1, fb2;
    HYD_MotionControlFB* core;
    int axisId;
    int triggerScan;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");

    init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    triggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(triggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    ASSERT_TRUE(IEC_VAL(fb2.BUSY) == true,
               "Buffered FB2 should report BUSY immediately after acceptance");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
               "Buffered FB2 must not report ACTIVE before cutover");

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
                   "FB2 should stay inactive while FB1 is still far from the cutover");
        ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
                   "FB1 should not be aborted by buffered BlendingHigh submission");
        ASSERT_TRUE(!velocity_overwritten_before_cutover(core, 100.0f, 20.0f, 0.25f),
                   "Before cutover, planner velocity must not jump to FB2 max velocity");

        if (core->AXIS_REF.position >= 95.0f) {
            break;
        }
    }
}
```

- [ ] **Step 3: Add the same failing regression for `BlendingLow`**

Insert this test below the previous one:

```c
static void test_blending_low_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    HYD_MOVEABSOLUTE fb1, fb2;
    HYD_MotionControlFB* core;
    int axisId;
    int triggerScan;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");

    init_ma(&fb1, axisId, 100.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 8.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_LOW);
    triggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(triggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    ASSERT_TRUE(IEC_VAL(fb2.BUSY) == true,
               "Buffered FB2 should report BUSY immediately after acceptance");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
               "Buffered FB2 must not report ACTIVE before cutover");

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
                   "FB2 should stay inactive while FB1 is still far from the cutover");
        ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
                   "FB1 should not be aborted by buffered BlendingLow submission");
        ASSERT_TRUE(!velocity_overwritten_before_cutover(core, 100.0f, 8.0f, 0.25f),
                   "Before cutover, planner velocity must not collapse to FB2 max velocity");

        if (core->AXIS_REF.position >= 95.0f) {
            break;
        }
    }
}
```

- [ ] **Step 4: Register the new regressions in `main()`**

Update `main()` in `tests/test_moveabsolute_blending_done.c` to:

```c
int main(void) {
    printf("=== test_moveabsolute_blending_done ===\n\n");
    test_blending_high_pending_does_not_take_active_or_overwrite_velocity_early();
    test_blending_low_pending_does_not_take_active_or_overwrite_velocity_early();
    test_blending_high_two_moveabsolute_cycles();
    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_failed > 0) ? 1 : 0;
}
```

- [ ] **Step 5: Run the new regression target and verify it fails before any runtime change**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected before implementation:

- at least one new assertion fails
- likely failures are:
  - `Buffered FB2 must not report ACTIVE before cutover`
  - `Before cutover, planner velocity must not jump to FB2 max velocity`

- [ ] **Step 6: Commit the failing reproduction**

Run:

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Capture early buffered MoveAbsolute blend regression" -m "Constraint: Reproduce the PLC-side early-trigger blending bug before changing runtime behavior\nRejected: Fix-first workflow without a failing reproduction | Makes the acceptance target ambiguous\nConfidence: high\nScope-risk: narrow\nDirective: Keep these regressions failing until pending ACTIVE and blend timing are corrected together\nTested: ./out/build/unixgcc/test_moveabsolute_blending_done (expected failure)\nNot-tested: No runtime code changed yet"
```

### Task 2: Make direct submission report `started` vs `queued` and map IEC state correctly

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`
- Modify: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add a direct-start result enum to `include/motion_control.h`**

Insert this enum near `HYD_DirectTicketRecord` in `include/motion_control.h`:

```c
typedef enum {
    HYD_DIRECT_START_REJECTED = 0,
    HYD_DIRECT_START_STARTED,
    HYD_DIRECT_START_QUEUED
} HYD_DirectStartResult;
```

Change the direct-start declaration in `include/motion_control.h` from:

```c
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp);
```

to:

```c
HYD_DirectStartResult HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                             const HYD_MotionSegment* segment,
                                                             HYD_BufferMode bufferMode,
                                                             HYD_TIME timestamp);
```

- [ ] **Step 2: Thread the new result through `startDirectSegmentExecution(...)`**

Replace the helper in `src/motion_interface.c`:

```c
static HYD_BOOL startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                            IEC_INT bufferMode,
                                            const HYD_MotionSegment* segment,
                                            IEC_WORD* errorId)
```

with:

```c
static HYD_DirectStartResult startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                                         IEC_INT bufferMode,
                                                         const HYD_MotionSegment* segment,
                                                         IEC_WORD* errorId)
{
    HYD_DirectStartResult result;

    if (errorId != NULL) {
        *errorId = (IEC_WORD)0;
    }

    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR;
        }
        return HYD_DIRECT_START_REJECTED;
    }

    result = HYD_MotionControlFB_StartDirectCommand(fb,
                                                    segment,
                                                    (HYD_BufferMode)bufferMode,
                                                    fb->AXIS_REF.timestamp);
    if (result == HYD_DIRECT_START_REJECTED) {
        if (errorId != NULL) {
            *errorId = commandFailureErrorId(fb);
        }
    }

    return result;
}
```

- [ ] **Step 3: Update `__mcl_cmd_MoveAbsolute(...)` to map pending acceptance to `ACTIVE=false`**

Inside the `execRising` block in `src/motion_interface.c`, replace:

```c
        if (!startDirectSegmentExecution(fb, bufferMode, &segment, &errorId))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
```

with:

```c
        HYD_DirectStartResult startResult =
            startDirectSegmentExecution(fb, bufferMode, &segment, &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , startResult == HYD_DIRECT_START_QUEUED);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , startResult == HYD_DIRECT_START_STARTED);
```

Keep:

```c
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
        __SET_VAR(data__->, EXECUTE0, , execute);
```

- [ ] **Step 4: Return `STARTED` vs `QUEUED` from `HYD_MotionControlFB_StartDirectCommand(...)`**

In `src/motion_control.c`, change the signature from:

```c
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp) {
```

to:

```c
HYD_DirectStartResult HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                             const HYD_MotionSegment* segment,
                                                             HYD_BufferMode bufferMode,
                                                             HYD_TIME timestamp) {
```

Apply these return rules:

```c
    if (fb == NULL || segment == NULL) {
        return HYD_DIRECT_START_REJECTED;
    }
```

```c
    if (!HYD_RecipeValidator_ValidateSegment(segment, HYD_MAX_SEGMENTS, &code, &fb->cylinderConfig)) {
        HYD_StateReporter_ReportDiagnostic(...);
        return HYD_DIRECT_START_REJECTED;
    }
```

```c
    if (fb->_isStopping && bufferMode != HYD_BUFFER_MODE_ABORT) {
        HYD_StateReporter_ReportDiagnostic(...);
        return HYD_DIRECT_START_REJECTED;
    }
```

Abort/immediate takeover path:

```c
        return HYD_DIRECT_START_STARTED;
```

Buffered/blended queue path:

```c
        return HYD_DIRECT_START_QUEUED;
```

Immediate idle start path:

```c
    if (!HYD_MotionControlFB_StartSegment(fb, 0U, timestamp)) {
        fb->USE_RECIPE = savedUseRecipe;
        return HYD_DIRECT_START_REJECTED;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return HYD_DIRECT_START_STARTED;
```

- [ ] **Step 5: Add arbitration coverage for the pending state contract**

Insert this test into `tests/test_motion_interface_arbitration.c` above `main()`:

```c
static void test_buffered_moveabsolute_reports_busy_but_not_active_while_waiting(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) || IEC_VAL(first.BUSY),
               "First MoveAbsolute should be running before the buffered follower starts");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.BUSY) == true,
               "Buffered MoveAbsolute should be BUSY while waiting");
    ASSERT_TRUE(IEC_VAL(second.ACTIVE) == false,
               "Buffered MoveAbsolute must not be ACTIVE while another MoveAbsolute owns the axis");
    ASSERT_TRUE(IEC_VAL(second.DONE) == false,
               "Buffered MoveAbsolute should not report DONE while waiting");
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Buffered MoveAbsolute should not report COMMANDABORTED while waiting");
}
```

Register it in `main()`:

```c
    test_buffered_moveabsolute_reports_busy_but_not_active_while_waiting();
```

- [ ] **Step 6: Add queue-depth enforcement for the third same-axis command**

Insert this test below the pending-state contract test:

```c
static void test_third_same_axis_moveabsolute_is_rejected_when_one_active_and_one_pending_exist(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MOVEABSOLUTE third;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.ERROR) == false,
               "Second MoveAbsolute should occupy the single pending slot");

    memset(&third, 0, sizeof(third));
    IEC_VAL(third.EN) = true;
    IEC_VAL(third.EXECUTE) = true;
    third.EXECUTE0.value = false;
    IEC_VAL(third.AXISID) = 0;
    IEC_VAL(third.POSITION) = 300.0f;
    IEC_VAL(third.VELOCITY) = 60.0f;
    IEC_VAL(third.ACCELERATION) = 100.0f;
    IEC_VAL(third.DECELERATION) = 100.0f;
    IEC_VAL(third.DIRECTION) = 1;
    IEC_VAL(third.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_LOW;
    __mcl_cmd_MoveAbsolute(&third);

    ASSERT_TRUE(IEC_VAL(third.ERROR) == true,
               "Third same-axis MoveAbsolute should be rejected when one active and one pending command already exist");
    ASSERT_TRUE(IEC_VAL(third.BUSY) == false,
               "Rejected third MoveAbsolute should not enter BUSY");
    ASSERT_TRUE(IEC_VAL(third.ACTIVE) == false,
               "Rejected third MoveAbsolute should not enter ACTIVE");
}
```

Register it in `main()`:

```c
    test_third_same_axis_moveabsolute_is_rejected_when_one_active_and_one_pending_exist();
```

- [ ] **Step 7: Run the focused tests and verify the state contract now passes**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_arbitration test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected after this task:

- `test_motion_interface_arbitration` passes the new pending-state check
- `test_motion_interface_arbitration` passes the third-command rejection check
- `test_moveabsolute_blending_done` may still fail on early velocity overwrite, which is expected until Task 3

- [ ] **Step 8: Commit the direct-start result and IEC state fix**

Run:

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_arbitration.c
git commit -m "Separate queued MoveAbsolute from active ownership" -m "Constraint: ACTIVE must reflect current direct ownership, not mere acceptance into the pending slot\nRejected: Keeping boolean start-result semantics | Leaves the IEC layer unable to distinguish queued acceptance from immediate ownership\nConfidence: high\nScope-risk: narrow\nDirective: Keep pending direct commands BUSY but inactive until their ticket becomes the direct owner\nTested: ./out/build/unixgcc/test_motion_interface_arbitration; ./out/build/unixgcc/test_moveabsolute_blending_done (early velocity regression may still fail)\nNot-tested: Full ctest suite"
```

### Task 3: Arm direct blending only near cutover and close the regressions

**Files:**
- Modify: `src/motion_control.c`
- Modify: `tests/test_motion_interface_arbitration.c`
- Modify: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Add a helper that decides when a recorded blend is armed**

Insert this helper near `HYD_ShouldCutoverDirectBlend(...)` in `src/motion_control.c`:

```c
static HYD_BOOL HYD_IsDirectBlendArmed(const HYD_MotionControlFB* fb,
                                       const HYD_MotionSegment* segment) {
    HYD_REAL selectedVelocity;
    HYD_REAL previousMagnitude;
    HYD_REAL brakingAcceleration;
    HYD_REAL tolerance;
    HYD_REAL remainingDistance;
    HYD_REAL brakingDistance;
    HYD_MotionDirection direction;

    if (fb == NULL || segment == NULL ||
        !fb->_directBlendContext.active ||
        !fb->_directPendingValid ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
        return false;
    }

    direction = HYD_Segment_ResolveDirection(segment, &fb->AXIS_REF, fb->_lastActiveDirection);
    if (direction != HYD_DIRECTION_EXTEND && direction != HYD_DIRECTION_RETRACT) {
        return false;
    }

    selectedVelocity = fb->_directBlendContext.blendVelocity;
    previousMagnitude = fabs(fb->_plannerState.lastTargetVelocity);
    brakingAcceleration = HYD_ResolveSegmentBrakingAccelerationForBlend(segment);
    tolerance = fb->_directBlendContext.switchTolerance;
    if (tolerance <= 0.0f) {
        tolerance = HYD_Segment_GetPositionTolerance(segment);
    }

    remainingDistance = HYD_ComputeRemainingDistanceToPosition(fb->_directBlendContext.switchPosition,
                                                               &fb->AXIS_REF,
                                                               direction);
    if (remainingDistance <= tolerance) {
        return true;
    }
    if (brakingAcceleration <= 0.0f || previousMagnitude <= selectedVelocity) {
        return false;
    }

    brakingDistance = ((previousMagnitude * previousMagnitude) -
                       (selectedVelocity * selectedVelocity)) /
                      (2.0f * brakingAcceleration);
    if (brakingDistance < 0.0f) {
        brakingDistance = 0.0f;
    }

    return remainingDistance <= brakingDistance + tolerance;
}
```

- [ ] **Step 2: Feed the planner only when the direct blend is armed**

In `src/motion_control.c`, replace:

```c
        plannerInput.state = &fb->_plannerState;
        plannerInput.blend = fb->_directBlendContext.active
            ? &fb->_directBlendContext
            : NULL;
        plannerInput.lastActiveDirection = fb->_lastActiveDirection;
```

with:

```c
        plannerInput.state = &fb->_plannerState;
        plannerInput.blend = HYD_IsDirectBlendArmed(fb, segment)
            ? &fb->_directBlendContext
            : NULL;
        plannerInput.lastActiveDirection = fb->_lastActiveDirection;
```

- [ ] **Step 3: Add a regression that proves the planner ignores recorded blend context while far from cutover**

Insert this test in `tests/test_motion_interface_arbitration.c` near the other blend tests:

```c
static void test_recorded_blend_context_does_not_drive_velocity_while_far_from_cutover(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_HIGH, 5.0f, 20.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for far-from-cutover blend test");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be recorded when the pending command is accepted");

    fb->AXIS_REF.position = 10.0f;
    fb->AXIS_REF.velocity = 5.0f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 5.0f;
    fb->AXIS_REF.timestamp += 0.1f;

    __HydMotion_framework_Publish();

    ASSERT_TRUE(fb->STATE.references.velocityReference < 10.0f,
               "Far from cutover, the active segment should not jump to the pending segment high velocity");
}
```

Register it in `main()`:

```c
    test_recorded_blend_context_does_not_drive_velocity_while_far_from_cutover();
```

- [ ] **Step 4: Re-run the full buffered-blending targets and verify they all pass**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_arbitration test_motion_planner test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_planner
./out/build/unixgcc/test_moveabsolute_blending_done
ctest --test-dir out/build/unixgcc --output-on-failure -R "test_motion_interface_arbitration|test_motion_planner|test_moveabsolute_blending_done"
```

Expected:

- the new early-trigger regressions pass
- `Buffered FB2 must not report ACTIVE before cutover` no longer fails
- far-from-cutover velocity does not jump to the pending segment value
- existing cutover continuity tests remain green

- [ ] **Step 5: Commit the blend-arming fix**

Run:

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c tests/test_moveabsolute_blending_done.c
git commit -m "Delay MoveAbsolute blend influence until cutover window" -m "Constraint: Buffered blending must preserve TwinCAT-style one-active-one-pending semantics and only shape cutover velocity near the first target position\nRejected: Immediate planner participation for any accepted pending blend | Causes early velocity takeover before cutover\nConfidence: high\nScope-risk: moderate\nDirective: Keep recorded blend metadata separate from armed planner participation; do not let pending acceptance imply early active-segment takeover\nTested: ./out/build/unixgcc/test_motion_interface_arbitration; ./out/build/unixgcc/test_motion_planner; ./out/build/unixgcc/test_moveabsolute_blending_done; ctest --test-dir out/build/unixgcc --output-on-failure -R \"test_motion_interface_arbitration|test_motion_planner|test_moveabsolute_blending_done\"\nNot-tested: Full ctest suite outside the blending/arbitration subset"
```

## Self-Review

### Spec coverage

- `1 running + 1 pending` queue depth:
  - Task 2 Step 5 adds pending-state contract coverage
  - Task 3 Step 3 plus existing direct-slot logic preserve one-slot buffering
- `FB2` pending while `FB1` remains active:
  - Task 1 Steps 2-3
  - Task 2 Steps 3-6
- no early overwrite of first-segment velocity:
  - Task 1 Steps 2-5 reproduce it
  - Task 3 Steps 1-4 fix and verify it
- valid blended cutover still yields `FB1.DONE` and `FB2.ACTIVE`:
  - existing `test_blending_high_two_moveabsolute_cycles`
  - existing `test_blended_cutover_preserves_planner_state`
  - Task 3 Step 4 reruns them
- reproduce-before-fix validation order:
  - Task 1 Step 5 is the required failing proof before runtime edits

### Placeholder scan

- No `TODO`, `TBD`, or "similar to Task N" references remain.
- Every code-touching step names exact files and includes the code shape to add or replace.
- Every verification step includes explicit commands and expected outcomes.

### Type consistency

- `HYD_DirectStartResult` is introduced once in `include/motion_control.h` and reused consistently in `src/motion_interface.c` and `src/motion_control.c`.
- The plan keeps the existing `HYD_LiveUpdateRequest.ownerTicket` and current direct-ticket API; it does not reintroduce the older `_executionId` contract from the earlier June 27 plan.
- Blend gating uses the existing `HYD_MotionBlendContext` fields: `active`, `blendVelocity`, `switchPosition`, `switchTolerance`.
