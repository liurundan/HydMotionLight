# MoveAbsolute Three-FB Terminal State Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make same-axis three-FB `MoveAbsolute` field scans preserve `FB1.Done`, promote `FB2.Active` correctly after blend cutover, and keep `FB3.Error` latched while `FB3.Execute = TRUE`, without regressing other function-block combinations.

**Architecture:** Keep the existing direct-ticket ownership model, one-active-one-pending direct queue, and current blend cutover math intact. Add a small `MoveAbsolute`-local terminal latch state in the IEC struct, teach `__mcl_cmd_MoveAbsolute()` to consume runtime completion/preemption/rejection facts into that local state, and prove behavior with a field-order reproducer plus targeted done/arbitration regressions before running the broader direct-command subset.

**Tech Stack:** C99, matiec IEC struct macros, CMake preset `unixgcc`, standalone C regression executables under `tests/`, existing direct-command runtime in `src/motion_interface.c` and `src/motion_control.c`.

---

## File Structure

- Modify `include/motion_interface.h`
  - extend `HYD_MOVEABSOLUTE` private storage with one small local state field used only by the IEC adapter
- Modify `src/motion_interface.c`
  - add a tiny local enum + helper functions for `MoveAbsolute` terminal-state mapping
  - update `__mcl_cmd_MoveAbsolute()` to latch `DONE`, `ERROR`, and `COMMANDABORTED` until `Execute` falls
- Modify `tests/test_moveabsolute_blending_done.c`
  - tighten the exact field-order three-FB reproducer so it asserts sticky `FB3.Error` and preserved `FB1.Done`
- Modify `tests/test_motion_interface_done_signals.c`
  - add focused checks that `MoveAbsolute` keeps `Done` and `Error` latched while `Execute` remains high, and clears them only on the falling edge
- Reuse `tests/test_motion_interface_arbitration.c`
  - keep the existing full-slot and persistent-high arbitration regressions green
- Reuse `tests/test_moveabsolute_stop_integration.c`
  - keep the genuine preemption path green so `CommandAborted` semantics do not regress

No new public pins, no queue-depth changes, no planner changes, and no broad terminal-state redesign for other FBs belong in this plan.

### Task 1: Freeze the field-order three-FB behavior with a failing regression

**Files:**
- Modify: `tests/test_moveabsolute_blending_done.c`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Tighten the existing three-FB reproducer to assert sticky `FB3.Error`**

In `test_third_same_axis_moveabsolute_is_rejected_without_disturbing_blended_pair()` inside `tests/test_moveabsolute_blending_done.c`, keep the current setup and add these assertions inside the post-trigger loop after each `__mcl_cmd_MoveAbsolute(&fb3);` call and before `__HydMotion_framework_Publish();`:

```c
        ASSERT_TRUE(IEC_VAL(fb3.ERROR) == true,
                   "Rejected FB3 should keep ERROR latched while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(fb3.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                   "Rejected FB3 should keep COMMAND_NOT_ALLOWED latched while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(fb3.BUSY) == false,
                   "Rejected FB3 should remain non-busy while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
                   "Rejected FB3 should remain inactive while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(fb3.DONE) == false,
                   "Rejected FB3 must not mutate into DONE while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
                   "Rejected FB3 must not mutate into COMMANDABORTED while EXECUTE stays high");
```

Keep the current pending-slot and blend-context invariants around `core->_directPendingValid`, `core->_directPendingSegment.targetPosition`, and `core->_directBlendContext.active`.

- [ ] **Step 2: Add a latched `FB1.Done` observation before `FB2` completes**

In the same test, replace the single `run_until_fb2_done(...)` call section with an intermediate observation loop so the test proves `FB1.Done` survives while `FB2` is already active:

```c
    HYD_BOOL observed_fb1_done_while_fb2_active = false;

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);
        __HydMotion_framework_Publish();

        if (IEC_VAL(fb1.DONE) == true && IEC_VAL(fb2.ACTIVE) == true) {
            observed_fb1_done_while_fb2_active = true;
            ASSERT_TRUE(IEC_VAL(fb1.BUSY) == false,
                       "FB1 should be non-busy after blended completion");
            ASSERT_TRUE(IEC_VAL(fb1.ACTIVE) == false,
                       "FB1 should be inactive after blended completion");
            ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
                       "FB1 should not report COMMANDABORTED after legal blended completion");
            break;
        }

        if (IEC_VAL(fb1.ERROR) || IEC_VAL(fb2.ERROR) ||
            IEC_VAL(fb1.COMMANDABORTED) || IEC_VAL(fb2.COMMANDABORTED)) {
            break;
        }
    }

    ASSERT_TRUE(observed_fb1_done_while_fb2_active == true,
               "The field-order reproducer should observe FB1.DONE while FB2 is already ACTIVE");
```

After that loop, keep the existing `run_until_fb2_done(&fb1, &fb2, &runResult);` call so the test still proves the pair finishes cleanly.

- [ ] **Step 3: Build and run the focused blending executable to verify failure**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
./out/build/unixgcc/test_moveabsolute_blending_done
```

Expected before implementation:

- FAIL because `FB3.ERROR` currently clears on later scans while `Execute` stays high, or
- FAIL because `FB1.DONE` is not preserved long enough to observe it while `FB2.ACTIVE` is true

- [ ] **Step 4: Commit the failing regression checkpoint**

Run:

```bash
git add tests/test_moveabsolute_blending_done.c
git commit -m "Capture MoveAbsolute three-FB terminal-state regression"
```

### Task 2: Add explicit local latch state to `HYD_MOVEABSOLUTE`

**Files:**
- Modify: `include/motion_interface.h`
- Test: `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Extend the private IEC struct storage**

In `include/motion_interface.h`, update `HYD_MOVEABSOLUTE` so the private section becomes:

```c
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)
  __DECLARE_VAR(USINT,_LOCAL_STATE)
```

Use `USINT` to keep storage minimal and avoid introducing a new public enum type in the header.

- [ ] **Step 2: Add a focused compile-only assertion in the done-signals test**

In `tests/test_motion_interface_done_signals.c`, add this test above `main()`:

```c
static void test_moveabsolute_error_and_done_clear_only_on_execute_falling_edge(void) {
    HYD_MOVEABSOLUTE ma;
    int axisId;
    int steps;

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed for latch-reset test");
    if (axisId < 0) {
        return;
    }

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 40.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 100.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "MoveAbsolute should reach DONE for latch-reset test");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "DONE should be true after completion");

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true,
               "DONE should remain latched while EXECUTE stays high after completion");

    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
               "DONE should clear on the EXECUTE falling edge");
}
```

This test is expected to compile immediately after the header change and will become fully meaningful after the runtime change in Task 4.

- [ ] **Step 3: Build the done-signals target**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_done_signals
```

Expected: build succeeds with the new `USINT _LOCAL_STATE` field available in `HYD_MOVEABSOLUTE`.

- [ ] **Step 4: Commit the struct change checkpoint**

Run:

```bash
git add include/motion_interface.h tests/test_motion_interface_done_signals.c
git commit -m "Reserve local MoveAbsolute terminal-state storage"
```

### Task 3: Add tiny `MoveAbsolute`-local latch helpers in `motion_interface.c`

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_moveabsolute_blending_done.c`

- [ ] **Step 1: Insert a private local-state enum and output helpers**

Near the existing `HYD_DirectPendingStatus` enum in `src/motion_interface.c`, add:

```c
typedef enum {
    HYD_MOVEABS_STATE_IDLE = 0,
    HYD_MOVEABS_STATE_RUNNING_OWNER = 1,
    HYD_MOVEABS_STATE_RUNNING_PENDING = 2,
    HYD_MOVEABS_STATE_DONE_LATCHED = 3,
    HYD_MOVEABS_STATE_ERROR_LATCHED = 4,
    HYD_MOVEABS_STATE_ABORTED_LATCHED = 5
} HYD_MoveAbsoluteLocalState;

static HYD_MoveAbsoluteLocalState getMoveAbsoluteLocalState(const HYD_MOVEABSOLUTE* data__) {
    return (HYD_MoveAbsoluteLocalState)__GET_VAR(data__->_LOCAL_STATE);
}

static void setMoveAbsoluteLocalState(HYD_MOVEABSOLUTE* data__,
                                      HYD_MoveAbsoluteLocalState state) {
    __SET_VAR(data__->, _LOCAL_STATE, , (IEC_USINT)state);
}

static void setMoveAbsoluteOutputs(HYD_MOVEABSOLUTE* data__,
                                   IEC_BOOL busy,
                                   IEC_BOOL active,
                                   IEC_BOOL done,
                                   IEC_BOOL commandAborted,
                                   IEC_BOOL error,
                                   IEC_WORD errorId) {
    __SET_VAR(data__->, BUSY, , busy);
    __SET_VAR(data__->, ACTIVE, , active);
    __SET_VAR(data__->, DONE, , done);
    __SET_VAR(data__->, COMMANDABORTED, , commandAborted);
    __SET_VAR(data__->, ERROR, , error);
    __SET_VAR(data__->, ERRORID, , errorId);
}
```

- [ ] **Step 2: Add one reset helper for the falling-edge path**

Directly below the helper above, add:

```c
static void resetMoveAbsoluteLocalState(HYD_MOVEABSOLUTE* data__) {
    setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_IDLE);
    __SET_VAR(data__->, _PENDING, , false);
    __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
    setMoveAbsoluteOutputs(data__, false, false, false, false, false, (IEC_WORD)0);
}
```

- [ ] **Step 3: Rebuild the blending executable**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
```

Expected: build succeeds without behavior change yet.

- [ ] **Step 4: Commit the helper scaffold**

Run:

```bash
git add src/motion_interface.c
git commit -m "Add MoveAbsolute terminal-state helper scaffolding"
```

### Task 4: Rework `__mcl_cmd_MoveAbsolute()` to consume runtime facts into latched local state

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_moveabsolute_blending_done.c`
- Test: `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Replace the falling-edge branch with the local reset helper**

In `src/motion_interface.c` inside `__mcl_cmd_MoveAbsolute()`, replace the current `if (!execute)` block:

```c
    if (!execute)
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }
```

with:

```c
    if (!execute)
    {
        resetMoveAbsoluteLocalState(data__);
        __SET_VAR(data__->, ACTIVE0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }
```

- [ ] **Step 2: Latch accepted/rejected rising-edge outcomes**

In the `execRising` branch:

- on `HYD_DIRECT_START_REJECTED`, replace the direct output writes with:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_ERROR_LATCHED);
            setMoveAbsoluteOutputs(data__, false, false, false, false, true, errorId);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
            __SET_VAR(data__->, ACTIVE0, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
```

- on `HYD_DIRECT_START_STARTED`, set:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_OWNER);
```

- on `HYD_DIRECT_START_QUEUED`, set:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_PENDING);
```

- then replace the raw output writes at the end of the branch with:

```c
        setMoveAbsoluteOutputs(data__,
                               true,
                               startResult == HYD_DIRECT_START_STARTED,
                               false,
                               false,
                               false,
                               (IEC_WORD)0);
```

- [ ] **Step 3: Make pending-acquired and pending-aborted transitions update local state**

Inside the `if (isPending)` branch:

- on `HYD_DIRECT_PENDING_ACQUIRED`, after updating `_EXEC_ID` and `_PENDING`, add:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_OWNER);
```

- on `HYD_DIRECT_PENDING_ABORTED`, replace the raw output writes with:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_ABORTED_LATCHED);
            setMoveAbsoluteOutputs(data__, false, false, false, true, false, (IEC_WORD)0);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
```

- on `WAITING`, replace the bare `return` with:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_PENDING);
            setMoveAbsoluteOutputs(data__, true, false, false, false, false, (IEC_WORD)0);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
```

- [ ] **Step 4: Convert runtime completion/preemption/error facts into sticky local states**

Still inside `__mcl_cmd_MoveAbsolute()`, replace the direct writes in the `myExecId != 0` block with local-state transitions:

- on `directExecutionWasCompleted(...)` and the non-`CONTINUOUSUPDATE` path:

```c
                setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_DONE_LATCHED);
                setMoveAbsoluteOutputs(data__, false, false, true, false, false, (IEC_WORD)0);
                __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
```

- on `directExecutionWasPreempted(...)` and `directExecutionLostOwnership(...)`:

```c
            setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_ABORTED_LATCHED);
            setMoveAbsoluteOutputs(data__, false, false, false, true, false, (IEC_WORD)0);
```

- on live-update or owner-path errors:

```c
                setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_ERROR_LATCHED);
                setMoveAbsoluteOutputs(data__, false, false, false, false, true,
                                       commandFailureErrorId(fb));
```

- on normal owner-running scans:

```c
                setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_OWNER);
                setMoveAbsoluteOutputs(data__, true, true, false, false, false, (IEC_WORD)0);
```

- on owner hold scans:

```c
                setMoveAbsoluteLocalState(data__, HYD_MOVEABS_STATE_RUNNING_OWNER);
                setMoveAbsoluteOutputs(data__, true, false, false, false, false, (IEC_WORD)0);
```

Do not add any branch that clears a terminal state while `Execute` remains high.

- [ ] **Step 5: Add a final local-state fallback to preserve terminal outputs**

Just before the trailing `ACTIVE0` / `EXECUTE0` writes, add:

```c
    switch (getMoveAbsoluteLocalState(data__)) {
        case HYD_MOVEABS_STATE_DONE_LATCHED:
            setMoveAbsoluteOutputs(data__, false, false, true, false, false, __GET_VAR(data__->ERRORID));
            break;
        case HYD_MOVEABS_STATE_ERROR_LATCHED:
            setMoveAbsoluteOutputs(data__, false, false, false, false, true, __GET_VAR(data__->ERRORID));
            break;
        case HYD_MOVEABS_STATE_ABORTED_LATCHED:
            setMoveAbsoluteOutputs(data__, false, false, false, true, false, __GET_VAR(data__->ERRORID));
            break;
        default:
            break;
    }
```

This fallback is intentionally narrow: it only reasserts terminal outputs if no earlier branch already returned or refreshed them in the current call.

- [ ] **Step 6: Rebuild and run the two focused executables**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_blending_done
cmake --build --preset unixgcc --target test_motion_interface_done_signals
./out/build/unixgcc/test_moveabsolute_blending_done
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected:

- `test_moveabsolute_blending_done` passes the sticky `FB3.Error` and preserved `FB1.Done` assertions
- `test_motion_interface_done_signals` passes the new falling-edge latch-reset test

- [ ] **Step 7: Commit the runtime change**

Run:

```bash
git add include/motion_interface.h src/motion_interface.c tests/test_moveabsolute_blending_done.c tests/test_motion_interface_done_signals.c
git commit -m "Latch MoveAbsolute terminal states while execute stays high"
```

### Task 5: Prove full-slot rejection and genuine preemption semantics still hold

**Files:**
- Modify: `tests/test_motion_interface_done_signals.c`
- Reuse: `tests/test_motion_interface_arbitration.c`
- Reuse: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Add an explicit sticky-error reset test in `done_signals`**

In `tests/test_motion_interface_done_signals.c`, add this second focused latch test above `main()`:

```c
static void test_moveabsolute_rejected_error_stays_high_until_execute_falls(void) {
    HYD_MOVEABSOLUTE fb1, fb2, fb3;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed for sticky-error test");
    if (axisId < 0) {
        return;
    }

    memset(&fb1, 0, sizeof(fb1));
    memset(&fb2, 0, sizeof(fb2));
    memset(&fb3, 0, sizeof(fb3));

    IEC_VAL(fb1.EN) = IEC_VAL(fb2.EN) = IEC_VAL(fb3.EN) = true;
    IEC_VAL(fb1.AXISID) = IEC_VAL(fb2.AXISID) = IEC_VAL(fb3.AXISID) = axisId;

    IEC_VAL(fb1.POSITION) = 100.0f;
    IEC_VAL(fb1.VELOCITY) = 20.0f;
    IEC_VAL(fb1.ACCELERATION) = 100.0f;
    IEC_VAL(fb1.DIRECTION) = 1;
    IEC_VAL(fb1.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    IEC_VAL(fb1.EXECUTE) = true;
    fb1.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb1);
    __HydMotion_framework_Publish();
    IEC_VAL(fb1.EXECUTE) = true;
    fb1.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&fb1);

    IEC_VAL(fb2.POSITION) = 200.0f;
    IEC_VAL(fb2.VELOCITY) = 20.0f;
    IEC_VAL(fb2.ACCELERATION) = 100.0f;
    IEC_VAL(fb2.DIRECTION) = 1;
    IEC_VAL(fb2.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    IEC_VAL(fb2.EXECUTE) = true;
    fb2.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb2);

    IEC_VAL(fb3.POSITION) = 10.0f;
    IEC_VAL(fb3.VELOCITY) = 10.0f;
    IEC_VAL(fb3.ACCELERATION) = 100.0f;
    IEC_VAL(fb3.DIRECTION) = 1;
    IEC_VAL(fb3.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    IEC_VAL(fb3.EXECUTE) = true;
    fb3.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb3);

    ASSERT_TRUE(IEC_VAL(fb3.ERROR) == true, "Rejected third MoveAbsolute should set ERROR");

    for (int step = 0; step < 3; step++) {
        IEC_VAL(fb1.EXECUTE) = true;
        fb1.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb1);
        IEC_VAL(fb2.EXECUTE) = true;
        fb2.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb2);
        IEC_VAL(fb3.EXECUTE) = true;
        fb3.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb3);
        ASSERT_TRUE(IEC_VAL(fb3.ERROR) == true,
                   "Rejected third MoveAbsolute should keep ERROR latched while EXECUTE stays high");
        __HydMotion_framework_Publish();
    }

    IEC_VAL(fb3.EXECUTE) = false;
    fb3.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&fb3);
    ASSERT_TRUE(IEC_VAL(fb3.ERROR) == false,
               "Rejected third MoveAbsolute should clear ERROR on the EXECUTE falling edge");
}
```

- [ ] **Step 2: Build and run the three regression executables**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_interface_arbitration
cmake --build --preset unixgcc --target test_motion_interface_done_signals
cmake --build --preset unixgcc --target test_moveabsolute_stop_integration
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected:

- arbitration tests stay green, including the existing `test_rejected_third_moveabsolute_stays_local_under_persistent_execute_high()`
- done-signals tests stay green with both new latch tests
- stop integration still shows genuine `MoveAbsolute -> Stop` takeover as `COMMANDABORTED`

- [ ] **Step 3: Commit the verification-layer additions**

Run:

```bash
git add tests/test_motion_interface_done_signals.c
git commit -m "Verify MoveAbsolute terminal latches and reset semantics"
```

### Task 6: Run the required ctest subset and capture the completion evidence

**Files:**
- Modify: none
- Test: `tests/test_moveabsolute_blending_done.c`
- Test: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Run the required ctest subset**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure -R test_moveabsolute_blending_done
ctest --test-dir out/build/unixgcc --output-on-failure -R test_motion_interface_arbitration
ctest --test-dir out/build/unixgcc --output-on-failure -R test_motion_interface_done_signals
ctest --test-dir out/build/unixgcc --output-on-failure -R test_moveabsolute_stop_integration
```

Expected:

- all four selected tests pass
- no `MoveAbsolute` terminal-state regression remains
- no cross-FB preemption or stop regression appears

- [ ] **Step 2: Run one broader direct-command smoke subset**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure -R "test_moveabsolute_blending_done|test_motion_interface_arbitration|test_motion_interface_done_signals|test_moveabsolute_stop_integration"
```

Expected:

- the combined targeted subset remains green in one pass

- [ ] **Step 3: Commit the final verified branch state**

Run:

```bash
git status --short
```

Expected: only the intended implementation files are modified or staged.

Then run:

```bash
git add include/motion_interface.h src/motion_interface.c tests/test_moveabsolute_blending_done.c tests/test_motion_interface_done_signals.c
git commit -m "Preserve MoveAbsolute terminal states across three-FB field scans"
```

If the same files were already committed in earlier tasks and no new changes remain, skip this final commit and keep the earlier commits.

## Self-Review

- Spec coverage:
  - sticky `FB3.Error` while `Execute = TRUE` is covered by Task 1 and Task 5
  - preserved `FB1.Done` through `FB2` activation is covered by Task 1 and Task 4
  - falling-edge reset semantics are covered by Task 2 and Task 5
  - cross-FB no-regression requirements are covered by Task 5 and Task 6
- Placeholder scan:
  - no `TODO`, `TBD`, or “similar to above” shortcuts remain
  - every code-edit step includes the target code to add or replace
- Type consistency:
  - the private IEC field is consistently named `_LOCAL_STATE`
  - the runtime-local enum is consistently named `HYD_MoveAbsoluteLocalState`
  - latch states are consistently `IDLE`, `RUNNING_OWNER`, `RUNNING_PENDING`, `DONE_LATCHED`, `ERROR_LATCHED`, `ABORTED_LATCHED`

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-01-moveabsolute-three-fb-terminal-state.md`. Two execution options:

1. Subagent-Driven (recommended) - I dispatch a fresh subagent per task, review between tasks, fast iteration

2. Inline Execution - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
