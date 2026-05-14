# IEC Surface Contract Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the IEC-facing motion FB surface with the currently intended contract by explicitly rejecting unsupported PLCopen-style options, implementing the missing `HYD_LOADPROFILE` preload path, and tightening public comments so the code no longer over-promises behavior the runtime does not provide.

**Architecture:** Keep the existing middle-layer boundary intact. This plan does not add machine workflow logic, valve behavior, or new process semantics. It hardens the public surface so PLC callers get either real behavior or explicit rejection, and it completes the missing recipe/direct preload adapter path without starting execution.

**Tech Stack:** C99, HydroMotionLib runtime and IEC adapter (`src/motion_control.c`, `src/motion_interface.c`), shared types in `include/`, unit/integration tests via CMake/CTest.

---

## Scope Choice

The gap list contains several partially independent remediation tracks. This plan intentionally focuses on the first coherent slice:

1. unsupported or misleading IEC FB options
2. missing `HYD_LOADPROFILE`
3. comment/contract tightening for currently supported surface behavior

It does **not** include:

- recipe/direct takeover signal redesign
- adding IEC wrappers for `Hold`/`Resume`
- `segmentTag`/`segmentType` separation
- config source-of-truth cleanup

Those should follow in later plans once the public surface contract is no longer misleading.

## File Map

- Modify: `tests/test_motion_interface_unit.c`
  Add failing unit tests for `LoadProfile`, unsupported pin rejection, and unsupported `BUFFERMODE` rejection.
- Modify: `src/motion_interface.c`
  Implement `HYD_LOADPROFILE` and add explicit surface-validation helpers for unsupported options.
- Modify: `include/common_types.h`
  Tighten the `HYD_BufferMode` comment to make the supported subset explicit.
- Modify: `include/motion_control.h`
  Fix the command-legality comment so it matches actual runtime behavior for `ABORT`.
- Optional modify: `include/motion_interface.h`
  Add narrow comments clarifying that some exposed IEC pins are compatibility/reserved pins until later support is added.

## Task 1: Add Failing IEC Surface Regression Tests

**Files:**
- Modify: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add a failing recipe-axis `LoadProfile` preload test**

Append this test near the other IEC unit tests in `tests/test_motion_interface_unit.c`:

```c
static void test_loadprofile_preloads_single_recipe_segment(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 7;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 42.0f;
    motion.SETVELOCITY = 12.0f;
    motion.SETFLOW = 4.0f;
    motion.SETPRESSURE = 0.0f;
    motion.ACCELERATION = 55.0f;
    motion.DURATION = 0.0f;
    motion.PRESSURERAMPRATE = 0.0f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile recipe axis should resolve an FB");
    ASSERT_TRUE(IEC_VAL(lp.DONE) == true, "LoadProfile should set DONE after preload");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "LoadProfile should preload one recipe segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == false,
               "Recipe-axis LoadProfile should not populate DIRECT_SEGMENT");
    ASSERT_TRUE(fb->STATE.active == false,
               "LoadProfile should preload only, not start execution");
}
```

- [ ] **Step 2: Add a failing direct-axis `LoadProfile` preload test**

Append this test in the same file:

```c
static void test_loadprofile_preloads_direct_segment_on_direct_axis(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 8;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_SPEED_RAMP;
    motion.ENDCONDITION = HYD_END_TIME;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETVELOCITY = 15.0f;
    motion.SETFLOW = 6.0f;
    motion.ACCELERATION = 80.0f;
    motion.DURATION = 0.5f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile direct axis should resolve an FB");
    ASSERT_TRUE(IEC_VAL(lp.DONE) == true, "LoadProfile should set DONE on direct preload");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "Direct-axis LoadProfile should populate DIRECT_SEGMENT");
    ASSERT_TRUE(fb->RECIPE_SIZE == 0U,
               "Direct-axis LoadProfile should not populate RECIPE");
    ASSERT_TRUE(fb->STATE.active == false,
               "Direct-axis LoadProfile should preload only, not start execution");
}
```

- [ ] **Step 3: Add a failing unsupported-`JERK` rejection test**

Append this test:

```c
static void test_moveabsolute_rejects_nonzero_jerk_until_supported(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 50.0f;
    IEC_VAL(ma.JERK) = 1.0f;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute should reject unsupported nonzero JERK");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Unsupported JERK should surface COMMAND_NOT_ALLOWED");
}
```

- [ ] **Step 4: Add a failing unsupported `CONTINUOUSUPDATE` rejection test**

Append this test:

```c
static void test_movevelocity_rejects_continuousupdate_until_supported(void) {
    HYD_MOVEVELOCITY mv;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;

    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == true,
               "MoveVelocity should reject unsupported CONTINUOUSUPDATE");
    ASSERT_TRUE(IEC_VAL(mv.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Unsupported CONTINUOUSUPDATE should surface COMMAND_NOT_ALLOWED");
}
```

- [ ] **Step 5: Add a failing unsupported `BUFFERMODE` rejection test**

Append this test:

```c
static void test_moveabsolute_rejects_unsupported_buffer_mode_values(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 10.0f;
    IEC_VAL(ma.ACCELERATION) = 40.0f;
    IEC_VAL(ma.BUFFERMODE) = 2;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute should reject unsupported BUFFERMODE values");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Unsupported BUFFERMODE should surface COMMAND_NOT_ALLOWED");
}
```

- [ ] **Step 6: Register new tests in `main()`**

Add these calls in `main()`:

```c
    test_loadprofile_preloads_single_recipe_segment();
    test_loadprofile_preloads_direct_segment_on_direct_axis();
    test_moveabsolute_rejects_nonzero_jerk_until_supported();
    test_movevelocity_rejects_continuousupdate_until_supported();
    test_moveabsolute_rejects_unsupported_buffer_mode_values();
```

- [ ] **Step 7: Run the unit test target to verify the new tests fail**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_unit
out/build/unixgcc/test_motion_interface_unit
```

Expected:

- build succeeds
- at least the new `LoadProfile` and unsupported-option tests fail against current behavior

- [ ] **Step 8: Commit the red tests**

```bash
git add tests/test_motion_interface_unit.c
git commit -m "test: add IEC surface contract regression coverage"
```

## Task 2: Implement `HYD_LOADPROFILE` and Unsupported-Surface Validation

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add a shared helper for unsupported IEC-surface options**

Add these helpers near the other adapter helpers in `src/motion_interface.c`:

```c
static HYD_BOOL validateSupportedBufferMode(IEC_INT bufferMode, IEC_WORD* errorId)
{
    if (bufferMode == HYD_BUFFER_MODE_ABORT || bufferMode == HYD_BUFFER_MODE_BUFFER) {
        return true;
    }

    if (errorId != NULL) {
        *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    return false;
}

static HYD_BOOL validateUnsupportedMotionOptions(IEC_BOOL continuousUpdate,
                                                 IEC_REAL jerk,
                                                 IEC_WORD* errorId)
{
    if (!continuousUpdate && jerk == 0.0f) {
        return true;
    }

    if (errorId != NULL) {
        *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    return false;
}
```

- [ ] **Step 2: Make `MoveAbsolute` reject unsupported `BUFFERMODE`, `CONTINUOUSUPDATE`, and `JERK`**

In `__mcl_cmd_MoveAbsolute()`, add this early in the `execRising` path before segment construction:

```c
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->CONTINUOUSUPDATE),
                                              __GET_VAR(data__->JERK),
                                              &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        directBufferModeAbortIfRequested(fb, bufferMode);
```

- [ ] **Step 3: Make `MoveVelocity` reject unsupported `BUFFERMODE`, `CONTINUOUSUPDATE`, and `JERK`**

In `__mcl_cmd_MoveVelocity()`, add the same pattern:

```c
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->CONTINUOUSUPDATE),
                                              __GET_VAR(data__->JERK),
                                              &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        directBufferModeAbortIfRequested(fb, bufferMode);
```

- [ ] **Step 4: Make `PressureHandle` and `MoveProfile` reject unsupported `BUFFERMODE` values**

Add this near the top of their `execRising` paths:

```c
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
```

- [ ] **Step 5: Implement `__mcl_cmd_LoadProfile()` as a preload-only adapter**

Replace the stub with:

```c
void __mcl_cmd_LoadProfile(HYD_LOADPROFILE *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);

    if (fb == NULL) {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (!execute) {
        __SET_VAR(data__->, DONE,, false);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (execRising) {
        HYD_AXISMOTION motionData = __GET_VAR(data__->MOTION);
        HYD_MotionSegment segment = buildSegmentFromMotion(&motionData, fb);
        HYD_BOOL ok;

        if (fb->USE_RECIPE) {
            ok = HYD_MotionControlFB_LoadRecipe(fb, &segment, 1U);
        } else {
            ok = HYD_MotionControlFB_LoadDirectSegment(fb, &segment);
        }

        if (!ok) {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }

        __SET_VAR(data__->, DONE,, true);
        __SET_VAR(data__->, BUSY,, false);
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}
```

- [ ] **Step 6: Run the unit tests to verify they pass**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_unit
out/build/unixgcc/test_motion_interface_unit
```

Expected:

- `test_motion_interface_unit` passes with the new `LoadProfile` and unsupported-option behavior

- [ ] **Step 7: Commit the implementation**

```bash
git add src/motion_interface.c
git commit -m "feat: align IEC surface with supported motion semantics"
```

## Task 3: Tighten Public Comments and Supported-Surface Messaging

**Files:**
- Modify: `include/common_types.h`
- Modify: `include/motion_control.h`
- Optional modify: `include/motion_interface.h`

- [ ] **Step 1: Make the supported `BufferMode` subset explicit**

Update the `HYD_BufferMode` comment in `include/common_types.h` to:

```c
/* BufferMode: currently supported IEC buffering subset.
 * ABORT  (0): preempt current motion and execute immediately.
 * BUFFER (1): preserve current command if immediate takeover is not requested.
 * Values 2-5 are reserved and currently rejected by the IEC adapter layer. */
```

- [ ] **Step 2: Correct the `ABORT` legality comment in `include/motion_control.h`**

Update the command legality section so `ABORT` matches the implementation:

```c
 * - ABORT: IDLE / READY / STARTING / RUNNING / SEGMENT_COMPLETE / HOLD / DONE / ABORTED
```

- [ ] **Step 3: Add narrow compatibility comments for unsupported IEC pins**

Add short comments in `include/motion_interface.h` above the affected FB blocks:

```c
/* Note: JERK and CONTINUOUSUPDATE are currently reserved compatibility pins.
 * Non-default values are rejected by the IEC adapter until runtime support exists. */
```

Apply this above `HYD_MOVEABSOLUTE` and `HYD_MOVEVELOCITY`.

- [ ] **Step 4: Run formatting-neutral verification**

Run:

```bash
git diff --check
```

Expected:

- no whitespace or patch-format problems

- [ ] **Step 5: Commit the comment alignment**

```bash
git add include/common_types.h include/motion_control.h include/motion_interface.h
git commit -m "docs: tighten supported IEC surface comments"
```

## Task 4: Verify Surface Alignment Does Not Regress Existing Direct-Lifecycle Behavior

**Files:**
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Build all affected motion-interface targets**

Run:

```bash
cmake --build out/build/unixgcc --target \
  test_motion_interface_unit \
  test_motion_interface_done_signals \
  test_motion_interface_arbitration \
  test_moveabsolute_stop_integration
```

Expected:

- all four targets build successfully

- [ ] **Step 2: Run the key executables directly**

Run:

```bash
out/build/unixgcc/test_motion_interface_unit
out/build/unixgcc/test_motion_interface_done_signals
out/build/unixgcc/test_motion_interface_arbitration
out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected:

- all executables exit successfully
- no existing direct-session takeover behavior regresses

- [ ] **Step 3: Run targeted `ctest`**

Run:

```bash
ctest --test-dir out/build/unixgcc -R "test_motion_interface_unit|test_motion_interface_done_signals|test_motion_interface_arbitration|test_moveabsolute_stop_integration" --output-on-failure
```

Expected:

- all targeted tests pass

- [ ] **Step 4: Commit the verification checkpoint**

```bash
git commit --allow-empty -m "test: verify IEC surface alignment regressions"
```

## Follow-Up Plans (Out of Scope Here)

After this plan, create separate plans for:

1. recipe/direct lifecycle exposure alignment
2. `Hold` / `Resume` IEC surface design decision
3. `segmentTag` / `segmentType` separation
4. config source-of-truth cleanup

## Self-Review

Spec coverage:

- This plan covers the highest-value public-surface gaps:
  - unsupported IEC pins
  - missing `LoadProfile`
  - limited `BufferMode` subset
  - comment/contract drift
- It intentionally does not cover the larger lifecycle and structural cleanup items, which are better treated as follow-up plans.

Placeholder scan:

- No `TODO/TBD` placeholders are used as instructions.
- Every task includes exact files, code blocks, commands, and expected outcomes.

Type consistency:

- The plan uses the current codebase names exactly:
  - `HYD_LOADPROFILE`
  - `HYD_MOVEABSOLUTE`
  - `HYD_MOVEVELOCITY`
  - `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`
  - `HYD_BUFFER_MODE_ABORT`
  - `HYD_BUFFER_MODE_BUFFER`

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-14-iec-surface-contract-alignment.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
