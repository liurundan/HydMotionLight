# Recipe/Direct Lifecycle Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align recipe-mode and direct-mode lifecycle exposure so mixed PLC applications can reason about ownership loss, completion, and reset consistently without relying on undocumented differences between `MoveProfile` and direct FBs.

**Architecture:** This plan does not add machine workflow logic. It works inside the existing middle-layer boundary by tightening lifecycle observation in the IEC adapter and, where needed, the runtime-facing status surface. `HYD_MoveProfile` already has `COMMANDABORTED` in the current XML/C public surface, so this plan aligns recipe/direct lifecycle behavior directly against that exposed interface instead of treating recipe takeover as an ABI-expansion question.

**Tech Stack:** C99, HydroMotionLib runtime and IEC adapter (`src/motion_control.c`, `src/motion_interface.c`), PLC XML surface in `pousHydMotion.xml`, interface-layout consistency script, CMake/CTest.

---

## Scope Choice

This plan intentionally covers:

1. clearer recipe/direct lifecycle observation
2. reset/takeover visibility consistency for mixed recipe/direct use
3. explicit contract-level tests for recipe-side `COMMANDABORTED` semantics

It intentionally does **not** cover:

- `Hold` / `Resume` IEC wrapper design
- `segmentTag` / `segmentType` separation
- configuration source-of-truth cleanup

It does **not** include any further XML/ABI expansion beyond the already-present `MoveProfile.COMMANDABORTED` surface.

## File Map

- Modify: `tests/test_motion_interface_arbitration.c`
  Add recipe/direct takeover and reset-preemption regression coverage that reflects the current XML surface.
- Modify: `tests/test_motion_interface_done_signals.c`
  Add or strengthen mixed-lifecycle observable-behavior tests where useful.
- Modify: `src/motion_interface.c`
  Align recipe-side lifecycle observation with documented runtime-contract behavior, without adding machine workflow logic.
- Optional modify: `src/motion_control.c`
  Only if minimal exported runtime facts are needed for the adapter to express mixed ownership loss consistently.
- Reference: `pousHydMotion.xml`
  Already exposes `HYD_MoveProfile.COMMANDABORTED`; use it as an invariant in this plan.
- Reference: `include/motion_interface.h`
  Already exposes `HYD_MOVEPROFILE.COMMANDABORTED`; keep XML/C alignment intact.
- Test: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_moveabsolute_stop_integration.c`
- Test: `tests/test_interface_layout_consistency.py`

## Task 1: Add Failing Mixed-Lifecycle Regression Tests

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`
- Modify: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_done_signals.c`

- [ ] **Step 1: Add a failing recipe-started ownership-loss test that expects `MoveProfile.COMMANDABORTED`**

Append this test to `tests/test_motion_interface_arbitration.c`:

```c
static void test_moveprofile_loses_activity_when_direct_command_takes_over(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 40.0f;
    motion.SETFLOW = 10.0f;
    motion.ACCELERATION = 150.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) || IEC_VAL(mp.BUSY),
               "MoveProfile should be active before direct takeover");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 100.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
               "MoveProfile should raise COMMANDABORTED when direct motion takes over");
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
               "MoveProfile should clear ACTIVE after direct takeover");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
               "MoveProfile should clear BUSY after direct takeover");
    ASSERT_TRUE(IEC_VAL(mp.DONE) == false,
               "MoveProfile should not report DONE when displaced by takeover");
}
```

- [ ] **Step 2: Add a failing reset-preempts-recipe test**

Append this test in the same file:

```c
static void test_moveprofile_loses_activity_after_reset_takeover(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_RESET reset;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 80.0f;
    motion.SETVELOCITY = 30.0f;
    motion.SETFLOW = 8.0f;
    motion.ACCELERATION = 120.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = IEC_VAL(cm.AXISID);
    __mcl_cmd_Reset(&reset);
    ASSERT_TRUE(IEC_VAL(reset.DONE) == true,
               "Reset should complete immediately on the recipe axis");

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
               "MoveProfile should raise COMMANDABORTED after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
               "MoveProfile should clear ACTIVE after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
               "MoveProfile should clear BUSY after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.DONE) == false,
               "MoveProfile should not report DONE after reset takeover");
}
```

- [ ] **Step 3: Add a failing direct-after-recipe-done sanity test**

Append this test:

```c
static void test_direct_command_starts_cleanly_after_recipe_done(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MOVEABSOLUTE ma;
    int step;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 20.0f;
    motion.SETVELOCITY = 10.0f;
    motion.SETFLOW = 2.0f;
    motion.ACCELERATION = 50.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    for (step = 0; step < 4000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(mp.EXECUTE) = true;
        mp.EXECUTE0.value = true;
        __mcl_cmd_MoveProfile(&mp);
        if (IEC_VAL(mp.DONE)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(mp.DONE) == true,
               "MoveProfile should reach DONE on the single recipe segment");

    IEC_VAL(mp.EXECUTE) = false;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 0.0f;
    IEC_VAL(ma.VELOCITY) = 15.0f;
    IEC_VAL(ma.ACCELERATION) = 70.0f;
    IEC_VAL(ma.DIRECTION) = -1;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "Direct command should start cleanly after recipe DONE");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Direct command should not inherit an aborted lifecycle after recipe DONE");
}
```

- [ ] **Step 4: Register the new tests in `main()`**

Add:

```c
    test_moveprofile_loses_activity_when_direct_command_takes_over();
    test_moveprofile_loses_activity_after_reset_takeover();
    test_direct_command_starts_cleanly_after_recipe_done();
```

- [ ] **Step 5: Run targeted tests to verify at least one recipe/direct lifecycle assertion fails**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_arbitration test_motion_interface_done_signals
out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- build succeeds
- at least one of the new recipe/direct lifecycle assertions fails against current behavior

- [ ] **Step 6: Commit the red lifecycle tests**

```bash
git add tests/test_motion_interface_arbitration.c
git commit -m "test: add recipe direct lifecycle alignment coverage"
```

## Task 2: Align Recipe-Side Lifecycle Signals with the Existing Surface

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add a recipe-side ownership-loss helper**

Near the other adapter helpers in `src/motion_interface.c`, add:

```c
static HYD_BOOL recipeExecutionLostOwnership(const HYD_MotionControlFB* fb,
                                             IEC_WORD execId)
{
    return fb != NULL && execId != 0 && execId != (IEC_WORD)fb->_executionId;
}
```

- [ ] **Step 2: Make `MoveProfile` raise `COMMANDABORTED` and clear owned execution on ownership loss**

In the `myExecId != 0` branch of `__mcl_cmd_MoveProfile()`, replace the current mismatch handling with:

```c
    if (myExecId != 0) {
        if (recipeExecutionLostOwnership(fb, myExecId)) {
            __SET_VAR(data__->, COMMANDABORTED,, true);
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, ERROR,, false);
        } else {
            __SET_VAR(data__->, COMMANDABORTED,, false);
            __SET_VAR(data__->, ACTIVE,, fb->STATE.active ? true : false);
            __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb));
            __SET_VAR(data__->, DONE,, (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) ? true : false);
            __SET_VAR(data__->, ERROR,, HYD_MotionControlFB_IsError(fb) ? true : false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)fb->ERROR_ID);

            if (fb->_activeSegmentValid) {
                HYD_AXISMOTION motionOut = __GET_VAR(data__->MOTION);
                writeMotionFromSegment(&motionOut, fb);
                __SET_VAR(data__->, MOTION,, motionOut);
            }
        }
    }
```

The intent is to align recipe-side ownership loss with the already-exposed `COMMANDABORTED` surface, without introducing new machine workflow semantics.

- [ ] **Step 3: Run the arbitration test to verify the new recipe-side expectations pass**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_arbitration
out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- the new recipe/direct lifecycle tests pass
- existing direct-session takeover tests remain green

- [ ] **Step 4: Commit the adapter-side alignment**

```bash
git add src/motion_interface.c
git commit -m "fix: align recipe lifecycle observation with direct ownership model"
```

## Task 3: Verify XML/C Surface Consistency and No Regression in Existing Lifecycle Behavior

**Files:**
- Test: `tests/test_interface_layout_consistency.py`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_moveabsolute_stop_integration.c`
- Test: `tests/test_interface_layout_consistency.py`

- [ ] **Step 1: Build all affected targets**

Run:

```bash
cmake --build out/build/unixgcc --target \
  test_motion_interface_unit \
  test_motion_interface_done_signals \
  test_motion_interface_arbitration \
  test_moveabsolute_stop_integration
```

Expected:

- all targets build successfully

- [ ] **Step 2: Run the key executables**

Run:

```bash
out/build/unixgcc/test_motion_interface_unit
out/build/unixgcc/test_motion_interface_done_signals
out/build/unixgcc/test_motion_interface_arbitration
out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected:

- all executables pass

- [ ] **Step 3: Run targeted `ctest`**

Run:

```bash
ctest --test-dir out/build/unixgcc -R "test_motion_interface_unit|test_motion_interface_done_signals|test_motion_interface_arbitration|test_moveabsolute_stop_integration|test_interface_layout_consistency" --output-on-failure
```

Expected:

- all targeted tests pass

- [ ] **Step 4: Commit the verification checkpoint**

```bash
git commit --allow-empty -m "test: verify recipe direct lifecycle alignment regressions"
```

## Follow-Up Plans (Out of Scope Here)

After this plan, create separate plans for:

1. `Hold` / `Resume` IEC surface design
2. `segmentTag` / `segmentType` separation
3. configuration source-of-truth cleanup

## Self-Review

Spec coverage:

- This plan covers the next highest-value gap after IEC surface alignment:
  - recipe/direct lifecycle exposure alignment
- It explicitly avoids mixing in:
  - `Hold` / `Resume` wrappers
  - further XML/ABI expansion beyond the already-present `MoveProfile.COMMANDABORTED`
  - segment identity redesign
  - config cleanup

Placeholder scan:

- No `TODO/TBD` placeholders are used as instructions.
- Every task includes exact files, test code, commands, and expected outcomes.

Type consistency:

- The plan uses existing identifiers and surfaces:
  - `HYD_MOVEPROFILE`
  - `HYD_MoveProfile`
  - `DONE/BUSY/ACTIVE/ERROR`
  - `HYD_FB_STATE_SEGMENT_COMPLETE`
  - `HYD_LoadProfile`
  - `pousHydMotion.xml`

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-14-recipe-direct-lifecycle-alignment.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
