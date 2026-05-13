# MoveVelocity + PressureHandle Direct Session Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the direct-command session architecture so `MoveVelocity` and `PressureHandle` use the same owner, preemption, completion, and latch-clearing semantics as `MoveAbsolute + Stop`.

**Architecture:** Reuse the current direct-session metadata in the core, add direct owner kinds for velocity and pressure commands, and refactor IEC mapping so `MoveVelocity` and `PressureHandle` consume owner-kind, owner-execution, and preemption facts instead of relying on raw `_executionId` mismatch alone. Preserve the explicit publish-driven simulation contract and avoid changing recipe-mode semantics.

**Tech Stack:** C99, HydroMotionLib core, IEC FB interface layer, CMake/CTest, explicit `__HydMotion_framework_Publish()` simulation.

---

## File Map

- Modify: `include/motion_control.h`
  Extend direct owner kinds to cover `MOVE_VELOCITY` and `PRESSURE_HANDLE`.
- Modify: `src/motion_control.c`
  Infer and store direct owner kind for velocity and pressure direct segments.
- Modify: `src/motion_interface.c`
  Refactor `__mcl_cmd_MoveVelocity` and `__mcl_cmd_PressureHandle` to use direct-session facts for `COMMANDABORTED`, `ERROR`, `INVELOCITY`, `INPRESSURE`, and duration-based completion.
- Modify: `tests/test_motion_interface_done_signals.c`
  Add or strengthen direct-session-based lifecycle assertions for `MoveVelocity` and `PressureHandle`.
- Modify: `tests/test_motion_interface_arbitration.c`
  Add or strengthen arbitration/preemption assertions for `MoveVelocity` and `PressureHandle`.

## Task 1: Add Failing Direct-Session Regression Coverage for MoveVelocity and PressureHandle

**Files:**
- Modify: `tests/test_motion_interface_done_signals.c`
- Modify: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add a failing `MoveVelocity` stop-takeover band-clearing assertion**

In `tests/test_motion_interface_done_signals.c`, strengthen `test_movevelocity_then_stop_done()` so it explicitly requires the in-band flag to clear after Stop takeover:

```c
ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
           "MoveVelocity INVELOCITY should clear after Stop takeover");
```

Place that assertion after:

```c
IEC_VAL(mv.EXECUTE) = true;
mv.EXECUTE0.value = true;
__mcl_cmd_MoveVelocity(&mv);
ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
           "MoveVelocity should get COMMANDABORTED when stopped");
```

- [ ] **Step 2: Add a failing `PressureHandle` stop-takeover band-clearing assertion**

In `tests/test_motion_interface_arbitration.c`, strengthen `test_pressurehandle_preempted_by_stop()` so it explicitly requires:

```c
ASSERT_TRUE(IEC_VAL(ph.INPRESSURE) == false,
           "PressureHandle INPRESSURE should clear after Stop takeover");
```

Place it after the existing stop preemption assertion block.

- [ ] **Step 3: Add a failing `MoveVelocity -> PressureHandle` preemption assertion**

In `tests/test_motion_interface_arbitration.c`, add a focused test function:

```c
static void test_movevelocity_preempted_by_pressurehandle(void) {
    HYD_MOVEVELOCITY mv;
    HYD_PRESSUREHANDLE ph;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 5.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 10.0f;
    IEC_VAL(ph.DURATION) = 0.5f;
    __mcl_cmd_PressureHandle(&ph);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should raise COMMANDABORTED when preempted by PressureHandle");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should clear after PressureHandle takeover");
}
```

Add the new test to `main()`.

- [ ] **Step 4: Run the targeted tests to verify they fail**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_done_signals test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- at least one of the newly added assertions fails
- failure should be behavioral, not a compile/link error

- [ ] **Step 5: Commit the red tests**

```bash
git add tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "test: add direct session coverage for velocity and pressure"
```

## Task 2: Extend Core Direct Owner Kinds

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Extend `HYD_DirectCommandKind` in `include/motion_control.h`**

Update the enum to:

```c
typedef enum {
    HYD_DIRECT_CMD_NONE = 0,
    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
    HYD_DIRECT_CMD_MOVE_VELOCITY,
    HYD_DIRECT_CMD_PRESSURE_HANDLE,
    HYD_DIRECT_CMD_STOP
} HYD_DirectCommandKind;
```

- [ ] **Step 2: Extend owner-kind inference in `src/motion_control.c`**

Update the existing helper that infers direct owner kind from a direct segment so it covers velocity and pressure segments:

```c
static HYD_DirectCommandKind HYD_InferDirectCommandKindFromSegment(const HYD_MotionSegment* segment) {
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

- [ ] **Step 3: Verify direct-start synchronization still compiles and tests remain red behaviorally**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_done_signals test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- build succeeds
- the new failing assertions from Task 1 are still failing until IEC mapping is updated

- [ ] **Step 4: Commit the core owner-kind extension**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "refactor: extend direct owner kinds for velocity and pressure"
```

## Task 3: Refactor MoveVelocity IEC Mapping to Use Direct-Session Facts

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Keep rising-edge and pending ownership acquisition behavior intact**

Retain the existing shape for:

- segment build/load
- `HYD_MotionControlFB_StartSegment(...)`
- `_PENDING` → `_EXEC_ID` acquisition after the first publish

Do not redesign these parts in this task.

- [ ] **Step 2: Refactor the owned execution branch in `__mcl_cmd_MoveVelocity`**

Replace the raw `_executionId` mismatch-only mapping with direct-session-driven logic in this order:

```c
if (myExecId != 0)
{
    if (HYD_MotionControlFB_WasExecutionPreempted(fb,
                                                  (uint16_t)myExecId,
                                                  HYD_DIRECT_CMD_MOVE_VELOCITY)) {
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INVELOCITY, , false);
    } else if (myExecId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
               HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_MOVE_VELOCITY) {
        if (HYD_MotionControlFB_IsError(fb)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            HYD_REAL velError = fb->AXIS_REF.velocity - targetVelocity;
            if (velError < 0.0f) velError = -velError;
            if (targetVelocity > 0.0f && velError < targetVelocity * 0.05f) {
                __SET_VAR(data__->, INVELOCITY, , true);
            } else {
                __SET_VAR(data__->, INVELOCITY, , false);
            }
        }
    } else if (myExecId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) ||
               HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_MOVE_VELOCITY) {
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INVELOCITY, , false);
    }
}
```

- [ ] **Step 3: Verify `EXECUTE = 0` latch clearing still clears `INVELOCITY`**

Keep/confirm the `!execute` branch clears:

```c
__SET_VAR(data__->, INVELOCITY, , false);
__SET_VAR(data__->, COMMANDABORTED, , false);
__SET_VAR(data__->, ERROR, , false);
__SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
__SET_VAR(data__->, BUSY, , false);
__SET_VAR(data__->, ACTIVE, , false);
```

- [ ] **Step 4: Run targeted tests**

Run:

```bash
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- the new `MoveVelocity`-specific direct-session assertions pass
- any remaining failures should now be limited to `PressureHandle`

- [ ] **Step 5: Commit the `MoveVelocity` IEC refactor**

```bash
git add src/motion_interface.c
git commit -m "refactor: map movevelocity from direct session state"
```

## Task 4: Refactor PressureHandle IEC Mapping to Use Direct-Session Facts

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Preserve existing duration-based completion behavior**

Do not change the current duration-based semantics. This task should only make completion contingent on owner/session correctness.

- [ ] **Step 2: Refactor the owned execution branch in `__mcl_cmd_PressureHandle`**

Use direct-session-driven logic in this order:

```c
if (myExecId != 0)
{
    if (HYD_MotionControlFB_WasExecutionPreempted(fb,
                                                  (uint16_t)myExecId,
                                                  HYD_DIRECT_CMD_PRESSURE_HANDLE)) {
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INPRESSURE, , false);
    } else if (myExecId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
               HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_PRESSURE_HANDLE) {
        if (HYD_MotionControlFB_IsError(fb)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        } else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            HYD_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
            if (pressError < 0.0f) pressError = -pressError;
            if (targetPressure > 0.0f && pressError < 0.5f) {
                __SET_VAR(data__->, INPRESSURE, , true);
            } else {
                __SET_VAR(data__->, INPRESSURE, , false);
            }
        }
    } else if (myExecId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) ||
               HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_PRESSURE_HANDLE) {
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INPRESSURE, , false);
    }
}
```

- [ ] **Step 3: Confirm latch clearing still resets `INPRESSURE` and `COMMANDABORTED`**

Keep/confirm the `!execute` branch clears the pressure-hold outputs and latches.

- [ ] **Step 4: Run targeted tests**

Run:

```bash
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- `PressureHandle` stop/preemption assertions pass
- `INPRESSURE` clears immediately on ownership loss

- [ ] **Step 5: Commit the `PressureHandle` IEC refactor**

```bash
git add src/motion_interface.c
git commit -m "refactor: map pressurehandle from direct session state"
```

## Task 5: Full Regression and Final Verification

**Files:**
- Test: `out/build/unixgcc/*`

- [ ] **Step 1: Run the direct-session focused binaries**

Run:

```bash
./out/build/unixgcc/test_moveabsolute_stop_integration
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
```

Expected:

- all four pass

- [ ] **Step 2: Run the full suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected:

- all registered tests pass
- zero failures

- [ ] **Step 3: Verify the worktree is clean and scoped**

Run:

```bash
git status --short
```

Expected:

- only intended files changed before final commit
- clean working tree after final commit

- [ ] **Step 4: Commit the verified final integration**

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "feat: unify velocity and pressure direct session semantics"
```
