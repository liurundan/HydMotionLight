# MoveAbsolute Third-Command Buffer-Full Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the three-FB same-axis `MoveAbsolute` scenario obey strict `1 active + 1 pending` semantics so the third command is rejected with `Error` in the submission scan while the accepted `FB1 -> FB2` blended chain completes normally.

**Architecture:** Reuse the existing direct-ticket lifecycle, single pending slot, and direct blend context already present in `motion_control.c` / `motion_interface.c`. Add a field-order regression for the exact `FB1() -> FB2() -> FB3() -> Publish()` scenario, tighten the IEC/output assertions around the third-command rejection path, and only change runtime code if the new regression proves that the current slot-capacity rejection still leaks side effects or the wrong diagnostics.

**Tech Stack:** C99, matiec IEC structs/macros, CMake preset `unixgcc`, standalone C regression tests in `tests/`, static bounded-memory runtime in `src/motion_control.c` and `src/motion_interface.c`.

---

## File Structure

- Modify `tests/test_moveabsolute_blending_done.c`
  - add the exact three-FB same-axis field-order regression from the approved spec
  - verify `FB3` is rejected with `Error`, not `CommandAborted`
  - verify `FB3` rejection leaves `FB1` / `FB2` ownership, pending-slot state, and blend context intact
- Modify `tests/test_motion_interface_arbitration.c`
  - tighten the existing third-command rejection test to assert `_PENDING`, `ErrorID`, and no false abort semantics
  - add one direct core-state invariant check for `1 active + 1 pending` before and after the rejected third submission
- Optionally modify `src/motion_interface.c`
  - only if the new regression shows the direct `MoveAbsolute` reject path still leaves stale outputs or wrong diagnostics on the rejected third FB
- Optionally modify `src/motion_control.c`
  - only if the new regression shows the reject path still mutates the pending slot, blend context, or accepted direct-ticket bookkeeping
- Reuse `tests/probe_fb2_scenario.c`
  - treat as a local smoke probe during development if needed, but do not commit it unless the user explicitly asks to keep it

No new public IEC pins, no queue-depth expansion, and no planner redesign are in scope.

### Task 1: Add the exact three-FB field-order regression first

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Add a helper that drives three FBs in field order**

Insert this helper near the existing `hold_true_scan(...)` / `run_until_fb2_done(...)` helpers in `tests/test_moveabsolute_blending_done.c`:

```c
static void hold_three_true_scan(HYD_MOVEABSOLUTE* fb1,
                                 HYD_MOVEABSOLUTE* fb2,
                                 HYD_MOVEABSOLUTE* fb3) {
    hold_true_scan(fb1);
    hold_true_scan(fb2);
    hold_true_scan(fb3);
}
```

Keep the field-order contract explicit in the surrounding comment:

```c
/* Field scan order for the third-command regression:
 *   FB1() -> FB2() -> FB3() -> Publish() -> read outputs
 */
```

- [ ] **Step 2: Add the three-FB same-axis rejection regression**

Insert this test above `main()` in `tests/test_moveabsolute_blending_done.c`:

```c
static void test_third_same_axis_moveabsolute_is_rejected_without_disturbing_blended_pair(void) {
    HYD_MOVEABSOLUTE fb1, fb2, fb3;
    HYD_MotionControlFB* core;
    int axisId;
    int secondTriggerScan;
    int thirdTriggerScan = -1;
    int finishSteps;
    BlendRunResult runResult;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");
    if (axisId < 0) {
        return;
    }

    init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    secondTriggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(secondTriggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");
    if (secondTriggerScan <= 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    if (core == NULL) {
        return;
    }

    ASSERT_TRUE(IEC_VAL(fb2.BUSY) == true,
               "Buffered FB2 should be BUSY after pending acceptance");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
               "Buffered FB2 must remain inactive before cutover");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Pending slot should be occupied by FB2 before the third submission");

    init_ma(&fb3, axisId, 10.0f, 10.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);

    for (int step = 0; step < 5; step++) {
        IEC_VAL(fb1.EXECUTE) = true;
        fb1.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb1);

        IEC_VAL(fb2.EXECUTE) = true;
        fb2.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb2);

        if (step == 0) {
            IEC_VAL(fb3.EXECUTE) = true;
            fb3.EXECUTE0.value = false;
        } else {
            IEC_VAL(fb3.EXECUTE) = true;
            fb3.EXECUTE0.value = true;
        }
        __mcl_cmd_MoveAbsolute(&fb3);

        __HydMotion_framework_Publish();

        if (step == 0) {
            thirdTriggerScan = 1;
            ASSERT_TRUE(IEC_VAL(fb3.ERROR) == true,
                       "FB3 should report ERROR on the submission scan when one active and one pending command already exist");
            ASSERT_TRUE(IEC_VAL(fb3.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                       "FB3 should report COMMAND_NOT_ALLOWED for the full-slot rejection");
            ASSERT_TRUE(IEC_VAL(fb3.BUSY) == false,
                       "Rejected FB3 should not enter BUSY");
            ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
                       "Rejected FB3 should not enter ACTIVE");
            ASSERT_TRUE(IEC_VAL(fb3.DONE) == false,
                       "Rejected FB3 should not report DONE");
            ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
                       "Rejected FB3 should not report COMMANDABORTED");
            ASSERT_TRUE(core->_directPendingValid == true,
                       "Rejected FB3 should not clear the pending slot occupied by FB2");
            ASSERT_TRUE(fabs(core->_directPendingSegment.targetPosition - 200.0f) <= 0.001f,
                       "Rejected FB3 should not overwrite FB2's pending target");
            ASSERT_TRUE(core->_directBlendContext.active == true,
                       "Rejected FB3 should not clear the FB1 -> FB2 blend context");
            ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
                       "FB1 should not be aborted by the rejected third submission");
            ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
                       "FB2 should not be aborted by the rejected third submission");
        }
    }

    ASSERT_TRUE(thirdTriggerScan > 0,
               "FB3 should have been triggered in the field-order loop");

    finishSteps = run_until_fb2_done(&fb1, &fb2, &runResult);

    ASSERT_TRUE(finishSteps > 0,
               "FB1 and FB2 should still complete after the rejected third submission");
    ASSERT_TRUE(IEC_VAL(fb1.DONE) == true,
               "FB1 should report DONE at the blended cutover");
    ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
               "FB1 should not report COMMANDABORTED after the rejected third submission");
    ASSERT_TRUE(IEC_VAL(fb2.DONE) == true,
               "FB2 should report DONE at final completion");
    ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
               "FB2 should not report COMMANDABORTED after the rejected third submission");
    ASSERT_TRUE(runResult.vel_at_switch > 0.1f,
               "The FB1 -> FB2 cutover should keep nonzero velocity even when FB3 is rejected");
}
```

- [ ] **Step 3: Register the new regression in `main()`**

Update the `main()` function in `tests/test_moveabsolute_blending_done.c` so it includes:

```c
    test_third_same_axis_moveabsolute_is_rejected_without_disturbing_blended_pair();
```

Place it after the existing pending-early-takeover regressions and before the slot-reuse chain tests.

- [ ] **Step 4: Build and run just this target to verify the new regression reflects current behavior**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected before any implementation change:

- either the new three-FB regression already passes, proving the runtime contract is already correct for this exact scenario
- or one of these assertions fails:
  - `FB3 should report COMMAND_NOT_ALLOWED for the full-slot rejection`
  - `Rejected FB3 should not clear the pending slot occupied by FB2`
  - `Rejected FB3 should not clear the FB1 -> FB2 blend context`
  - `FB1 and FB2 should still complete after the rejected third submission`

- [ ] **Step 5: Commit the regression checkpoint**

Run:

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Capture third-command MoveAbsolute buffer-full regression" -m "Constraint: Same-axis direct commands must remain limited to one active owner plus one pending successor during blended MoveAbsolute execution\nRejected: Relying only on generic arbitration tests | Does not prove the exact three-FB field-order scenario from the approved spec\nConfidence: high\nScope-risk: narrow\nDirective: Keep the third-command scenario centered on submission-time Error semantics, not COMMANDABORTED\nTested: ./out/build/unixgcc/test_moveabsolute_blending_done\nNot-tested: No runtime code changed yet"
```

### Task 2: Tighten arbitration coverage around slot capacity and error semantics

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Extend the existing third-command rejection test with error-ID and slot-invariant assertions**

In `test_third_same_axis_moveabsolute_is_rejected_when_one_active_and_one_pending_exist()` inside `tests/test_motion_interface_arbitration.c`, after the existing `third` assertions, add:

```c
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for pending-slot invariant checks");
    if (fb != NULL) {
        ASSERT_TRUE(fb->_directPendingValid == true,
                   "Rejected third MoveAbsolute should leave the single pending slot occupied");
        ASSERT_TRUE(fabs(fb->_directPendingSegment.targetPosition - 200.0f) < 0.001f,
                   "Rejected third MoveAbsolute should not overwrite the existing pending target");
        ASSERT_TRUE(fb->_directBlendContext.active == true,
                   "Rejected third MoveAbsolute should not clear the active blend context");
    }

    ASSERT_TRUE(IEC_VAL(third.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Rejected third MoveAbsolute should report COMMAND_NOT_ALLOWED");
    ASSERT_TRUE(IEC_VAL(third.COMMANDABORTED) == false,
               "Rejected third MoveAbsolute should not report COMMANDABORTED");
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Buffered second MoveAbsolute should remain non-aborted after rejecting the third");
```

- [ ] **Step 2: Add one same-scan persistence check for the rejected third FB**

Below the existing third-command test, add:

```c
static void test_rejected_third_moveabsolute_stays_local_under_persistent_execute_high(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MOVEABSOLUTE third;
    HYD_MotionControlFB* fb;

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
    IEC_VAL(third.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&third);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for persistent-high rejection checks");
    if (fb == NULL) {
        return;
    }

    for (int step = 0; step < 3; step++) {
        IEC_VAL(first.EXECUTE) = true;
        first.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&first);

        IEC_VAL(second.EXECUTE) = true;
        second.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&second);

        IEC_VAL(third.EXECUTE) = true;
        third.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&third);

        ASSERT_TRUE(IEC_VAL(third.BUSY) == false,
                   "Rejected third MoveAbsolute should remain non-busy while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.ACTIVE) == false,
                   "Rejected third MoveAbsolute should remain inactive while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.COMMANDABORTED) == false,
                   "Rejected third MoveAbsolute should not mutate into COMMANDABORTED on later scans");
        ASSERT_TRUE(fb->_directPendingValid == true,
                   "Rejected third MoveAbsolute should not dislodge the accepted pending command on later scans");

        advance_non_sim_feedback(0, 0.01f);
        __HydMotion_framework_Publish();
    }
}
```

Register it in `main()` near the other direct-command arbitration tests.

- [ ] **Step 3: Run the arbitration target**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- if the current runtime is already correct, the new assertions pass immediately
- otherwise the failures identify whether the gap is in:
  - `ERRORID`
  - false `COMMANDABORTED`
  - pending-slot mutation
  - blend-context mutation under repeated `Execute = TRUE`

- [ ] **Step 4: Commit the expanded arbitration coverage**

Run:

```bash
git add tests/test_motion_interface_arbitration.c
git commit -m "Tighten MoveAbsolute third-command arbitration coverage" -m "Constraint: Submission-time rejection must stay distinct from post-acceptance abort in the IEC adapter outputs\nRejected: Treating the third-command case as a generic overflow without checking pending-slot invariants | Misses the approved behavior contract\nConfidence: high\nScope-risk: narrow\nDirective: Keep same-axis queue-depth enforcement expressed through Error + preserved accepted-command state\nTested: ./out/build/unixgcc/test_motion_interface_arbitration\nNot-tested: No runtime code changed yet"
```

### Task 3: Fix the runtime only if the new regressions prove a real gap

**Files:**
- Modify: `src/motion_interface.c`
- Modify: `src/motion_control.c`
- Test: `tests/test_moveabsolute_blending_done.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: If the failure is wrong `ERRORID`, normalize the direct reject path in `__mcl_cmd_MoveAbsolute()`**

Use the existing `startDirectSegmentExecution(...)` path in `src/motion_interface.c`. If the regression shows a non-`COMMAND_NOT_ALLOWED` error on the full-slot rejection, tighten the `execRising` reject block to keep the submission failure local:

```c
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
            __SET_VAR(data__->, COMMANDABORTED, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
```

This keeps the rejected third command outside the accepted-command lifecycle.

- [ ] **Step 2: If the failure is pending-slot or blend-context mutation, isolate the rejection branch in `HYD_MotionControlFB_StartDirectCommand(...)`**

In `src/motion_control.c`, keep the existing full-slot guard:

```c
        if (fb->_directPendingValid) {
            HYD_StateReporter_ReportDiagnostic(fb,
                                               HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                               HYD_DIAG_SEVERITY_WARNING,
                                               timestamp,
                                               &fb->_activeSegment,
                                               &fb->STATE.references);
            return HYD_DIRECT_START_REJECTED;
        }
```

If the regression shows side effects despite this branch, fix the surrounding code so the reject path does **not**:

- call `HYD_DiscardPendingDirectSlot(fb)`
- write `fb->_directPendingSegment`
- write `fb->_directPendingKind`
- write `fb->_directPendingBufferMode`
- write `fb->_directBlendContext`
- record preempted or completed tickets

Keep the implementation minimal: do not refactor the whole start function if the regression only needs one guarded write to move below the `fb->_directPendingValid` check.

- [ ] **Step 3: Re-run the targeted regressions until both are green**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done test_motion_interface_arbitration
./out/build/unixgcc/test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- `FB3` is rejected with `ERROR=true`, `ERRORID=HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`
- `FB3` never reports `BUSY`, `ACTIVE`, `DONE`, or `COMMANDABORTED`
- `FB1` still reports blended `DONE`
- `FB2` still reaches `ACTIVE` then final `DONE`

- [ ] **Step 4: Commit the runtime fix only if code changed**

Run:

```bash
git add src/motion_interface.c src/motion_control.c
git commit -m "Keep third MoveAbsolute rejection outside accepted lifecycle" -m "Constraint: Same-axis direct commands must remain capped at one active owner plus one pending successor without mutating accepted command state\nRejected: Folding submission-time rejection into COMMANDABORTED semantics | Breaks PLC-visible distinction between not accepted and accepted-then-aborted\nConfidence: medium\nScope-risk: narrow\nDirective: Preserve the existing ticket, pending-slot, and blend-context state for accepted commands when rejecting overflow submissions\nTested: ./out/build/unixgcc/test_moveabsolute_blending_done && ./out/build/unixgcc/test_motion_interface_arbitration\nNot-tested: Full ctest not run yet"
```

Skip this commit step entirely if the new regressions passed without code changes.

### Task 4: Run focused completion checks and optional local probe

**Files:**
- Test: `tests/test_motion_interface_done_signals.c`
- Optional local probe: `tests/probe_fb2_scenario.c`

- [ ] **Step 1: Run the direct done-signal regression that is closest to this contract**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected:

- existing direct `MoveAbsolute` done-signal cases remain green
- no new `COMMANDABORTED` regressions appear after the third-command rejection work

- [ ] **Step 2: Optionally use the local probe during implementation if diagnosis is still ambiguous**

If `FB2.Active` or `FB2.Done` timing becomes unclear during debugging, run the local probe without committing it:

```bash
gcc -Iinclude -Isrc -o /tmp/probe_fb2_scenario tests/probe_fb2_scenario.c out/build/unixgcc/libHydroMotionLib.a -lm
/tmp/probe_fb2_scenario
```

Expected:

- scenario A and B logs help confirm whether the issue is in same-scan triggering or delayed triggering
- the probe remains a local diagnostic artifact unless the user explicitly asks to keep it

- [ ] **Step 3: Commit any test-only adjustments needed for the done-signal target**

If `tests/test_motion_interface_done_signals.c` needed edits, commit them separately:

```bash
git add tests/test_motion_interface_done_signals.c
git commit -m "Guard MoveAbsolute done-signal regressions after buffer-full fix"
```

If no code changed in this task, skip the commit.

### Task 5: Full verification and cleanup

**Files:**
- Verify only; no file changes required unless a failure forces a fix

- [ ] **Step 1: Run the combined focused targets**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done test_motion_interface_arbitration test_motion_interface_done_signals
./out/build/unixgcc/test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected:

- all three targets pass
- the approved three-FB scenario is covered by a committed regression

- [ ] **Step 2: Run the relevant `ctest` slice**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure -R "test_moveabsolute_blending_done|test_motion_interface_arbitration|test_motion_interface_done_signals"
```

Expected:

- all matching tests pass

- [ ] **Step 3: Verify no unintended files are staged**

Run:

```bash
git status --short
```

Expected:

- only the intended runtime/test files for this work are modified
- leave `.claude/settings.local.json`, `.omx/metrics.json`, and the untracked `tests/probe_fb2_scenario.c` alone unless the user explicitly asks otherwise

- [ ] **Step 4: Final implementation commit if Task 3 or Task 4 changed code**

If there are still uncommitted implementation files after the targeted commits above, create one final narrow commit:

```bash
git add tests/test_moveabsolute_blending_done.c tests/test_motion_interface_arbitration.c tests/test_motion_interface_done_signals.c src/motion_interface.c src/motion_control.c
git commit -m "Align MoveAbsolute third-command rejection with buffered blending contract" -m "Constraint: The third same-axis MoveAbsolute must fail as a submission-time Error while the accepted blended pair continues unchanged\nRejected: Allowing implicit third-slot buffering | Violates the approved one-active-one-pending contract\nConfidence: high\nScope-risk: narrow\nDirective: Preserve Error vs CommandAborted separation for rejected direct commands in future IEC adapter changes\nTested: ./out/build/unixgcc/test_moveabsolute_blending_done && ./out/build/unixgcc/test_motion_interface_arbitration && ./out/build/unixgcc/test_motion_interface_done_signals && ctest --test-dir out/build/unixgcc --output-on-failure -R \"test_moveabsolute_blending_done|test_motion_interface_arbitration|test_motion_interface_done_signals\"\nNot-tested: Full repository-wide ctest not run"
```

Skip this step if there is no remaining diff.

## Self-Review Checklist

- Spec coverage:
  - three-FB field-order scenario: Task 1
  - strict `1 active + 1 pending` invariants: Task 2
  - `Error` vs `CommandAborted` separation: Tasks 1-3
  - accepted `FB1 -> FB2` completion integrity after `FB3` rejection: Tasks 1 and 3
  - persistent `Execute = TRUE` stability: Task 2
- Placeholder scan:
  - no `TODO`, `TBD`, or abstract “handle later” steps remain
- Type consistency:
  - uses existing `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`, `_directPendingValid`, `_directBlendContext`, and `BlendRunResult`

