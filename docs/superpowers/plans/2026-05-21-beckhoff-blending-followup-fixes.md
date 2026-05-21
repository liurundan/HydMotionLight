# Beckhoff Blending Curves Follow-up Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the four Important / Minor gaps surfaced by the 2026-05-21 code review of the Beckhoff blending curves work, so the spec contract "Stop, Abort, Reset, or Fault clears the blend context and pending slot" is honored, online target/buffer updates remain consistent with the active blend, and pending blend submissions during Stopping are rejected up front.

**Architecture:** Keep all blend lifecycle ownership in `motion_control.c`. Reuse the existing `HYD_ClearDirectPendingSlot` and `HYD_TryCreateDirectBlendContext` helpers — do not introduce new public surface. Touch only the runtime branches and the live-update path; do not change the planner curve, IEC POU surface, or the one-slot direct buffer contract.

**Tech Stack:** C99 runtime library, matiec-style IEC structs/macros, CMake, CTest, project C unit tests.

---

## File Structure

- Modify `src/motion_control.c`: add `HYD_ClearDirectPendingSlot` calls into the Stop completion branch and the runtime fault-stop branch; reject blend pending submissions while `_isStopping`; refresh `_directBlendContext` after a successful `HYD_MotionControlFB_ApplyLiveUpdate`.
- Modify `tests/test_motion_interface_arbitration.c`: add runtime coverage for Stop / Fault teardown, blend rejection during Stopping, live-update blend-context refresh, and reverse-direction pending fallback completion.

No header, planner, IEC surface, or XML POU changes are required.

## Task 1: Clear Pending + Blend On Stop Completion

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for Stop teardown**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_stop_completion_clears_blend_pending_slot(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for stop-teardown test");
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending blend command should be present before Stop");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before Stop");

    ASSERT_TRUE(HYD_MotionControlFB_Stop(fb, fb->AXIS_REF.timestamp, 200.0),
               "Stop request should be accepted while blended pending is queued");

    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.timestamp += 1.0f;
    __HydMotion_framework_Publish();
    fb->AXIS_REF.timestamp += 1.0f;
    __HydMotion_framework_Publish();

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_DONE,
               "Stop completion should reach DONE");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Stop completion should clear pending direct slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Stop completion should clear blend context");
}
```

Add this call in `main()`:

```c
    test_stop_completion_clears_blend_pending_slot();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation: the assertions on `_directPendingValid` and `_directBlendContext.active` fail because the Stop completion branch (`src/motion_control.c:1641-1648`) does not call `HYD_ClearDirectPendingSlot`.

- [ ] **Step 3: Clear pending + blend at Stop completion**

In `src/motion_control.c`, update the Stop completion branch inside `HYD_MotionControlFB_RunRunningState` so it clears the pending direct slot (which also clears the blend context):

```c
        if (decelMag < 0.001f && fabs(fb->AXIS_REF.velocity) < 0.01f) {
            fb->_isStopping = false;
            fb->_stopStartVel = 0.0f;
            fb->_stopDeceleration = 0.0f;
            fb->_directSessionState = HYD_DIRECT_SESSION_DONE;
            HYD_ClearDirectPendingSlot(fb);
            HYD_ProtectionManager_ApplyIdleState(fb, true, false);
            HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_DONE);
        }
```

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation: the new test passes and all existing arbitration tests still pass.

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "fix: clear blend pending on stop completion"
```

## Task 2: Clear Pending + Blend On Runtime Fault Entry

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for fault teardown**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_runtime_fault_clears_blend_pending_slot(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for fault-teardown test");
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending blend command should be present before runtime fault");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before runtime fault");

    fb->DIAGNOSTIC.protectionAction = HYD_PROTECTION_ACTION_STOP;
    __HydMotion_framework_Publish();

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_FAULT,
               "Runtime fault path should enter FAULT state");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Runtime fault entry should clear pending direct slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Runtime fault entry should clear blend context");
}
```

Add this call in `main()`:

```c
    test_runtime_fault_clears_blend_pending_slot();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation: the assertions on `_directPendingValid` and `_directBlendContext.active` fail because the protection-action stop branch (`src/motion_control.c:1652-1656`) does not call `HYD_ClearDirectPendingSlot`.

- [ ] **Step 3: Clear pending + blend at fault entry**

In `src/motion_control.c`, update the protection-action stop branch inside `HYD_MotionControlFB_RunRunningState` so it clears the pending direct slot before transitioning to FAULT:

```c
    if (fb->DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP) {
        fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
        HYD_ClearDirectPendingSlot(fb);
        HYD_ProtectionManager_EnterFaultStop(fb);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
        return;
    }
```

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation: the new test passes and existing tests still pass.

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "fix: clear blend pending on runtime fault entry"
```

## Task 3: Reject Blend Pending Submissions While Stopping

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for blend rejection during Stop**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_blend_pending_rejected_while_stopping(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for stopping-reject test");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    ASSERT_TRUE(HYD_MotionControlFB_Stop(fb, fb->AXIS_REF.timestamp, 100.0),
               "Stop request should be accepted while a direct MoveAbsolute is active");
    __HydMotion_framework_Publish();
    ASSERT_TRUE(fb->_isStopping,
               "FB should report stopping before blend submission attempt");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.ERROR) == true,
               "Blend MoveAbsolute should report ERROR when submitted during Stopping");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Blend MoveAbsolute should not occupy pending slot during Stopping");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should remain cleared when submission is rejected during Stopping");
}
```

Add this call in `main()`:

```c
    test_blend_pending_rejected_while_stopping();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation: the submission is accepted into the pending slot because `HYD_MotionControlFB_StartDirectCommand` (`src/motion_control.c:2110`) does not gate on `_isStopping`.

- [ ] **Step 3: Reject non-abort submissions while stopping**

In `src/motion_control.c`, immediately after the existing `HYD_RecipeValidator_ValidateSegment` early-return block inside `HYD_MotionControlFB_StartDirectCommand`, insert a stopping gate so blend / buffered submissions are refused while a Stop profile is running. Replace the lines that begin `activeDirect = fb->STATE.active &&` with this block (the new block is exactly the existing `activeDirect` / `shouldAbort` setup with one new conditional inserted in front of it):

```c
    if (fb->_isStopping && bufferMode != HYD_BUFFER_MODE_ABORT) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                           &fb->STATE.references);
        return false;
    }

    activeDirect = fb->STATE.active &&
                   fb->_activeSegmentValid &&
                   fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT;
```

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation: the new test passes and existing tests still pass. Abort submissions during Stopping are still accepted because they bypass the new gate.

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "fix: reject blend submissions while stopping"
```

## Task 4: Refresh Blend Context After Live Update

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for live-update blend refresh**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_live_update_refreshes_blend_context(void) {
    HYD_MotionControlFB* fb;
    HYD_LiveUpdateRequest request;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_PREVIOUS, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for live-update refresh test");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before live update");
    ASSERT_TRUE(fabs(fb->_directBlendContext.switchPosition - 100.0f) < 0.001f,
               "Initial blend switch position should match active target");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "Initial blend velocity should match BlendingPrevious selection");

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerExecutionId = fb->_directOwnerExecutionId;
    request.targetPosition = 140.0f;
    request.maxVelocity = 12.0f;
    request.maxAcceleration = 100.0f;
    request.maxDeceleration = 100.0f;

    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(fb, &request),
               "Live update should succeed for active direct MoveAbsolute");

    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should remain active after live update");
    ASSERT_TRUE(fabs(fb->_directBlendContext.switchPosition - 140.0f) < 0.001f,
               "Blend switch position should follow updated target position");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 12.0f) < 0.001f,
               "BlendingPrevious blend velocity should follow updated active maxVelocity");
}
```

Add this call in `main()`:

```c
    test_live_update_refreshes_blend_context();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation: the `switchPosition` and `blendVelocity` assertions fail because `HYD_MotionControlFB_ApplyLiveUpdate` (`src/motion_control.c:2445-2538`) leaves `_directBlendContext` untouched.

- [ ] **Step 3: Refresh blend context inside ApplyLiveUpdate**

In `src/motion_control.c`, update `HYD_MotionControlFB_ApplyLiveUpdate` so that a successful update also refreshes `_directBlendContext` when a blended pending exists. Replace the block that begins `fb->_activeSegment = updated;` and ends with `return true;` with:

```c
    fb->_activeSegment = updated;
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
```

`HYD_TryCreateDirectBlendContext` already clears the context first, re-checks every eligibility condition, recomputes the through-velocity for the latest active/pending pair, and writes the new `switchPosition` from `_activeSegment.targetPosition`. If the update made the pair ineligible (for example by reducing `maxVelocity` to zero), the helper will leave the context inactive, which is the correct fallback to plain buffered completion.

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
```

Expected after implementation: the new test passes and existing arbitration / interface unit tests still pass.

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "fix: refresh blend context on live update"
```

## Task 5: Reverse-Direction Pending Falls Back To Buffered Completion

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add reverse-direction completion test**

The existing `test_blend_context_requires_compatible_moveabsolute_direction` already asserts that the blend context is not created. This task adds the missing end-to-end assertion that the reverse pending still completes through the normal buffered path after the front segment reports DONE.

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_reverse_blend_pending_completes_as_buffered(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE second;

    fb = start_active_moveabsolute_for_blend_fallback_test();
    queue_pending_moveabsolute_for_blend_fallback_test(0.0f, 2);

    ASSERT_TRUE(fb->_directPendingValid,
               "Reverse MoveAbsolute should still occupy the pending slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Reverse MoveAbsolute should not create a blend context");

    fb->AXIS_REF.position = 100.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.timestamp += 0.5f;
    __HydMotion_framework_Publish();

    ASSERT_TRUE(!fb->_directPendingValid,
               "Reverse pending should be consumed via buffered completion path");
    ASSERT_TRUE(fb->_activeSegmentValid,
               "Active segment should be the reverse MoveAbsolute after buffered cutover");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 0.0f) < 0.001f,
               "Pending reverse MoveAbsolute should become active segment");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should remain inactive across reverse buffered completion");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 0.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 2;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Reverse buffered completion should not raise COMMANDABORTED for the second command");
}
```

Add this call in `main()`:

```c
    test_reverse_blend_pending_completes_as_buffered();
```

- [ ] **Step 2: Run tests**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected: the new test passes against the already-existing buffered-completion logic in `HYD_MotionControlFB_RunRunningState`. If it does not pass, do not modify the runtime — instead stop and reconcile with the existing buffered-direct lifecycle, because that path is shared with non-blend traffic.

- [ ] **Step 3: Commit**

```bash
git add tests/test_motion_interface_arbitration.c
git commit -m "test: cover reverse blend pending buffered completion"
```

## Task 6: Full Verification

**Files:**
- Verify: all changed files

- [ ] **Step 1: Run targeted binaries**

Run:

```bash
cmake --build out/build/unixgcc -j2
./out/build/unixgcc/test_motion_planner
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: all four binaries pass.

- [ ] **Step 2: Run layout consistency**

Run:

```bash
python3 tests/test_interface_layout_consistency.py
```

Expected: script passes. This plan does not change the IEC POU layout, so any failure indicates an unrelated drift that must be investigated before continuing.

- [ ] **Step 3: Run full CTest**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected:

```text
100% tests passed
```

- [ ] **Step 4: Skip commit if no diff**

If Task 5's commit captured the final test addition and no other file remains modified, this task creates no new commit. Run:

```bash
git status --short
```

Expected:

```text

```

## Self-Review

- Spec coverage: Tasks 1-2 satisfy the "Stop/Abort/Reset/Fault clears the blend context and pending slot" contract from the blending design doc (line 201); Task 3 closes the "pending command arrives after the active segment is already complete or stopping" fallback gap (design doc line 168); Task 4 keeps the blend context consistent with the buffer-live-update feature so online target / velocity updates do not produce stale switch positions; Task 5 makes the existing reverse-direction guard observable end-to-end.
- Out of scope: protection_manager fault-hold cycles (cycles after the FAULT-entry tick), `_executionId` width (`uint16_t` wrap risk), and the `switchTolerance` zero-fallback redundancy. These are pre-existing concerns unrelated to the blend cutover contract and should be handled in dedicated follow-ups if needed.
- No public surface change: helpers stay static in `motion_control.c`; the IEC POU XML and `tests/test_interface_layout_consistency.py` are not affected.
- Verification coverage: each task adds a targeted runtime assertion that fails before the fix and passes after, plus the final `ctest` gate.
