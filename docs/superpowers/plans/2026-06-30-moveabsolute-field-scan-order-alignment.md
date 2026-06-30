# MoveAbsolute Field Scan Order Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `MoveAbsolute` publish correct `ACTIVE` / `DONE` outputs for buffered direct blending under the real PLC scan order `FB1() -> FB2() -> Publish() -> read outputs`, and align the main blending tests to that field timing model.

**Architecture:** Keep the existing direct-ticket ownership, blend cutover, and planner continuity logic in place. Fix only the IEC adapter timing gap in `__mcl_cmd_MoveAbsolute()` so a buffered command that acquires ownership does not lose one scan before owner-state outputs refresh, then realign the blending regression helpers and the most relevant post-scan assertions to the confirmed field scan order.

**Tech Stack:** C99, CMake, ctest, existing direct-command runtime in `src/motion_interface.c` / `src/motion_control.c`, standalone C regression tests in `tests/`

---

### Task 1: Lock the field-order reproducer before touching implementation

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c:81-225`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Write the failing field-order reproducer helpers**

Replace the current helper timing model so the main helpers reflect field scan order. Update the helper region around `trigger_fb2_when_fb1_active(...)` and `run_until_fb2_done(...)` with the following code:

```c
static int trigger_fb2_when_fb1_active(HYD_MOVEABSOLUTE* fb1,
                                       HYD_MOVEABSOLUTE* fb2,
                                       int maxWaitScans) {
    for (int step = 0; step < maxWaitScans; step++) {
        hold_true_scan(fb1);
        __HydMotion_framework_Publish();

        if (IEC_VAL(fb1->ACTIVE)) {
            rising_edge(fb2);
            __HydMotion_framework_Publish();
            return step + 1;
        }
    }
    return -1;
}

static int run_until_fb2_done(HYD_MOVEABSOLUTE* fb1, HYD_MOVEABSOLUTE* fb2,
                              float* vel_at_switch, float switch_pos) {
    *vel_at_switch = -1.0f;

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        IEC_VAL(fb1->EXECUTE) = true;
        fb1->EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(fb1);

        IEC_VAL(fb2->EXECUTE) = true;
        fb2->EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(fb2);

        __HydMotion_framework_Publish();

        if (*vel_at_switch < 0.0f && IEC_VAL(fb1->DONE)) {
            HYD_MotionControlFB* core = __MK_GetPublic_MotionControlFB(
                (int)IEC_VAL(fb1->AXISID));
            if (core != NULL) {
                *vel_at_switch = (float)fabs(core->_plannerState.lastTargetVelocity);
            }
        }

        if (IEC_VAL(fb2->DONE)) {
            return step + 1;
        }
        if (IEC_VAL(fb1->ERROR) || IEC_VAL(fb2->ERROR) ||
            IEC_VAL(fb1->COMMANDABORTED) || IEC_VAL(fb2->COMMANDABORTED)) {
            return -1;
        }
    }
    return -1;
}
```

Also update the file header comment so it explicitly says the test uses field scan order `FB1() -> FB2() -> Publish() -> read outputs`.

- [ ] **Step 2: Add a direct assertion that should fail with current implementation**

In `test_blending_high_two_moveabsolute_cycles()` immediately after triggering `fb2`, add a field-order ownership assertion that expects the second FB to become visible as `ACTIVE` on the first owner scan after cutover:

```c
ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
           "FB2 should still be inactive immediately after buffered acceptance under field scan order");
```

Then inside the scan loop used to observe cutover, add:

```c
HYD_BOOL observed_fb2_active_after_cutover = false;
...
if (IEC_VAL(fb1.DONE) == true && IEC_VAL(fb2.ACTIVE) == true) {
    observed_fb2_active_after_cutover = true;
}
...
ASSERT_TRUE(observed_fb2_active_after_cutover == true,
           "FB2 should become ACTIVE on the first post-cutover owner scan under field timing");
```

- [ ] **Step 3: Run the targeted test to verify it fails**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected: FAIL in the new field-order `FB2.ACTIVE` assertion because `__mcl_cmd_MoveAbsolute()` still returns early from the pending-acquired branch.

- [ ] **Step 4: Commit the failing test checkpoint**

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Expose MoveAbsolute field-order blending timing gap"
```

### Task 2: Fix `MoveAbsolute` pending-acquired state publication

**Files:**
- Modify: `src/motion_interface.c:1170-1187`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Change the pending-acquired branch to fall through**

Edit `src/motion_interface.c` inside `__mcl_cmd_MoveAbsolute()` so the `_PENDING` branch only returns on `WAITING` and `ABORTED`, not on `ACQUIRED`.

Use this replacement for the block beginning at `if (isPending)`:

```c
    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
            isPending = false;
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
            __SET_VAR(data__->, COMMANDABORTED, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
    }
```

This is the whole fix: buffered acquisition clears `_PENDING` and then immediately continues into the `myExecId != 0` owner-state mapping below.

- [ ] **Step 2: Re-run the focused blending test**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected: PASS for the new `FB2.ACTIVE` assertion and the pre-existing `FB1.DONE` / `FB2.DONE` checks.

- [ ] **Step 3: Commit the interface fix**

```bash
git add src/motion_interface.c tests/test_moveabsolute_blending_done.c
git commit -m "Fix MoveAbsolute buffered ownership state publication"
```

### Task 3: Convert the main blending progression helpers to field scan order

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c:108-185`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Update the early-takeover helper loop**

In `assert_pending_blend_does_not_take_over_early(...)`, change the loop body from:

```c
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
```

to:

```c
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
        __HydMotion_framework_Publish();
```

Keep the existing assertions, because they are post-scan field-visible assertions.

- [ ] **Step 2: Update the cutover velocity test loop**

In `test_blending_high_cutover_scan_keeps_nonzero_output_velocity()`, change the main loop from:

```c
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
```

to:

```c
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
        __HydMotion_framework_Publish();
```

and change the one-step post-cutover check from:

```c
    __HydMotion_framework_Publish();
    hold_true_scan(&fb2);
```

to:

```c
    hold_true_scan(&fb2);
    __HydMotion_framework_Publish();
```

- [ ] **Step 3: Update the cycle reset-to-origin loop**

In `test_blending_high_two_moveabsolute_cycles()`, change the reset loop from:

```c
        for (int s = 0; s < MAX_SIM_STEPS; s++) {
            __HydMotion_framework_Publish();
            IEC_VAL(fb1.EXECUTE) = true;
            fb1.EXECUTE0.value   = true;
            __mcl_cmd_MoveAbsolute(&fb1);
            if (IEC_VAL(fb1.DONE)) break;
        }
```

to:

```c
        for (int s = 0; s < MAX_SIM_STEPS; s++) {
            IEC_VAL(fb1.EXECUTE) = true;
            fb1.EXECUTE0.value   = true;
            __mcl_cmd_MoveAbsolute(&fb1);
            __HydMotion_framework_Publish();
            if (IEC_VAL(fb1.DONE)) break;
        }
```

- [ ] **Step 4: Run the full blending executable**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected: PASS, with all blending scenarios using field-order post-scan observation by default.

- [ ] **Step 5: Commit the helper realignment**

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Align MoveAbsolute blending helpers to field scan order"
```

### Task 4: Realign the chained buffered blending scenarios

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c:442-877`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Update the chained forward-blend loops**

In `test_three_segment_buffered_chain_reuses_pending_slot_without_early_takeover()`, change both loop shapes from:

```c
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
```

and

```c
        __HydMotion_framework_Publish();
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);
```

to:

```c
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
        __HydMotion_framework_Publish();
```

and

```c
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);
        __HydMotion_framework_Publish();
```

- [ ] **Step 2: Update the reverse-fallback loops**

In `test_reverse_direction_pending_falls_back_to_buffered_promotion_after_slot_reuse()` and `test_reverse_then_forward_reuses_pending_slot_without_stale_blend_context()`, change each post-scan assertion loop from `Publish()` first to FB calls first.

Use the same transformation pattern:

```c
        hold_true_scan(&fbX);
        hold_true_scan(&fbY);
        __HydMotion_framework_Publish();
```

Apply it consistently to:

- the `fb1/fb2` scan loop that waits for `fb2.ACTIVE`
- the `fb2/fb3` scan loop that waits for promotion
- the single-FB `fb3` progression loop
- the `fb3/fb4` pending-observation loop

- [ ] **Step 3: Preserve the documented exception points**

Do not change direct core-state tests that intentionally force raw runtime state before a `Publish()` step, such as tests that manually poke `AXIS_REF.position`, `_plannerState`, or `_directPendingValid` and then inspect one specific scan effect. Add one-line comments only where needed, for example:

```c
    /* Deliberately drives raw runtime state before Publish(); this test checks same-scan cutover math, not field polling order. */
```

- [ ] **Step 4: Run the blending executable again**

Run:

```bash
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected: PASS, including the chained pending-slot reuse and reverse-direction fallback scenarios.

- [ ] **Step 5: Commit the chained-scenario timing audit**

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Audit buffered blending regressions for field scan timing"
```

### Task 5: Inspect and align related post-scan arbitration assertions

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c:693-736, 827-920`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Adjust the buffered-wait ownership test if needed**

In `test_buffered_moveabsolute_waits_without_preempting_active_owner()`, keep the structure that accepts `second`, runs one `Publish()`, and then polls both FBs, but make the post-scan intent explicit with a comment:

```c
    /* Post-scan field-visible checks: both FBs have been called, then Publish() has advanced the runtime once. */
```

If any assertion is still sampling immediately after a command call but before the `Publish()` that should define the visible state, move that assertion after the `Publish()` step.

- [ ] **Step 2: Adjust the blended cutover ownership test if needed**

In `test_blended_cutover_preserves_planner_state()`, keep the raw-state setup before `__HydMotion_framework_Publish()` because this test intentionally validates same-scan cutover internals. Add a clarifying comment before the forced state block:

```c
    /* Intentional same-scan cutover setup: this test validates cutover internals, not the default PLC field polling order. */
```

Do not change the forced-state core assertions in this test beyond documenting why the timing differs.

- [ ] **Step 3: Run the arbitration executable**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected: PASS, with no regression in buffered waiting, cutover ownership, or completion marker consumption.

- [ ] **Step 4: Commit the arbitration timing audit**

```bash
git add tests/test_motion_interface_arbitration.c
git commit -m "Document arbitration timing assumptions"
```

### Task 6: Inspect direct `MoveAbsolute` done-signal loops for field-visible timing

**Files:**
- Modify: `tests/test_motion_interface_done_signals.c:94-118, 413-556`
- Test: `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Update the generic `run_moveabsolute_to_done(...)` helper**

Change the helper loop from:

```c
    __HydMotion_framework_Publish();
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);
```

to:

```c
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);
    __HydMotion_framework_Publish();
```

This helper exists to model repeated post-scan completion polling for direct `MoveAbsolute`, so it should follow field order by default.

- [ ] **Step 2: Update the multi-FB alternating `MoveAbsolute` test**

In `test_multi_fb_same_axis_cycle()` update the post-start and progression loops so visible-state assertions happen after `MoveAbsolute()` calls followed by `Publish()`, not before `Publish()`.

Use this pattern for the main progression loops:

```c
        IEC_VAL(maX.EXECUTE) = true;
        maX.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&maX);
        __HydMotion_framework_Publish();
```

Keep the preemption/takeover structure intact; only align the observation timing.

- [ ] **Step 3: Run the done-signals executable**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected: PASS, with direct `MoveAbsolute` done-signal checks aligned to field polling order.

- [ ] **Step 4: Commit the done-signal timing alignment**

```bash
git add tests/test_motion_interface_done_signals.c
git commit -m "Align MoveAbsolute done-signal polling to field scan order"
```

### Task 7: Full verification and final cleanup

**Files:**
- Modify: none expected
- Test: `tests/test_moveabsolute_blending_done.c`, `tests/test_motion_interface_arbitration.c`, `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Run the focused ctest set**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure -R "test_moveabsolute_blending_done|test_motion_interface_arbitration|test_motion_interface_done_signals"
```

Expected:

```text
100% tests passed, 0 tests failed
```

- [ ] **Step 2: Review the final diff**

Run:

```bash
git diff --stat HEAD~4..HEAD
git diff HEAD~4..HEAD -- src/motion_interface.c tests/test_moveabsolute_blending_done.c tests/test_motion_interface_arbitration.c tests/test_motion_interface_done_signals.c
```

Expected: one narrow runtime fix in `src/motion_interface.c`, field-order timing alignment in blending/done-signal tests, and timing-intent comments only where exceptions are deliberate.

- [ ] **Step 3: Create the final implementation commit if needed**

If the work from prior task commits is already clean and reviewable, do not squash automatically. If an extra finishing commit is needed for small final fixes:

```bash
git add src/motion_interface.c tests/test_moveabsolute_blending_done.c tests/test_motion_interface_arbitration.c tests/test_motion_interface_done_signals.c
git commit -m "Finalize MoveAbsolute field scan order alignment"
```

- [ ] **Step 4: Record verification output for handoff**

Capture these exact commands and results in the final handoff note:

```bash
./out/build/unixgcc/test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
ctest --test-dir out/build/unixgcc --output-on-failure -R "test_moveabsolute_blending_done|test_motion_interface_arbitration|test_motion_interface_done_signals"
```

Expected: all pass.
