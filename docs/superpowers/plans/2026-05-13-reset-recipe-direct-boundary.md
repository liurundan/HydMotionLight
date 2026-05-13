# Reset + Recipe/Direct Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unify `Reset` behavior and recipe/direct ownership boundaries under a thin dispatcher model so recipe and direct commands use consistent `DONE`, `COMMANDABORTED`, and `ERROR` semantics.

**Architecture:** Add a small axis-level execution-source and owner abstraction in the core, keep `SoftReset` preserving configuration while clearing execution state, and refactor IEC mapping to consume explicit owner/preemption facts instead of mixing `USE_RECIPE`, `FB_STATE`, and raw `_executionId` heuristics. Reuse the existing tests where possible rather than introducing a large new test surface.

**Tech Stack:** C99, HydroMotionLib core, IEC FB interface layer, CMake/CTest, explicit `__HydMotion_framework_Publish()` simulation.

---

## File Map

- Modify: `include/motion_control.h`
  Add execution-source and dispatcher-owner enums plus the minimal query API needed by IEC mapping.
- Modify: `src/motion_control.c`
  Hold dispatcher truth: active execution source, active owner kind, last preempted owner, and reset takeover semantics.
- Modify: `src/motion_interface.c`
  Refactor `MoveProfile`, `Reset`, and mixed recipe/direct output mapping to use dispatcher facts.
- Modify: `tests/test_motion_interface_unit.c`
  Add focused reset and latch-behavior assertions.
- Modify: `tests/test_motion_interface_done_signals.c`
  Add or strengthen reset-on-active-command behavior checks.
- Modify: `tests/test_motion_interface_arbitration.c`
  Add recipe/direct takeover and reset-preemption checks.

## Task 1: Add Failing Boundary Tests for Reset and Recipe/Direct Takeover

**Files:**
- Modify: `tests/test_motion_interface_unit.c`
- Modify: `tests/test_motion_interface_done_signals.c`
- Modify: `tests/test_motion_interface_arbitration.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add a reset-preempts-direct assertion for MoveVelocity**

In `tests/test_motion_interface_done_signals.c`, add a new focused test:

```c
static void test_reset_preempts_movevelocity(void) {
    HYD_MOVEVELOCITY mv;
    HYD_RESET reset;
    int axisId, step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    for (step = 0; step < 5; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(mv.EXECUTE) = true;
        mv.EXECUTE0.value = true;
        __mcl_cmd_MoveVelocity(&mv);
    }

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = axisId;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(IEC_VAL(reset.DONE) == true,
               "Reset should complete immediately while preempting MoveVelocity");

    __HydMotion_framework_Publish();
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should report COMMANDABORTED after Reset");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should clear after Reset takeover");
}
```

Add the new test to `main()`.

- [ ] **Step 2: Add a reset-preserves-direct-config assertion**

In `tests/test_motion_interface_unit.c`, add a focused test:

```c
static void test_reset_preserves_direct_segment_configuration(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_RESET reset;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 123.0f;
    IEC_VAL(ma.VELOCITY) = 40.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB instance should exist");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "Direct segment should be loaded before reset");

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 0;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "SoftReset should preserve the direct segment");
    ASSERT_TRUE(fb->STATE.active == false,
               "SoftReset should clear active execution state");
}
```

- [ ] **Step 3: Add a recipe-direct takeover boundary test**

In `tests/test_motion_interface_arbitration.c`, add a focused test:

```c
static void test_direct_command_preempts_moveprofile(void) {
    HYD_MOVEPROFILE mp;
    HYD_MOVEABSOLUTE ma;
    HYD_CREATEMOTION cm;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(mp.POSITION) = 100.0f;
    IEC_VAL(mp.VELOCITY) = 40.0f;
    IEC_VAL(mp.ACCELERATION) = 150.0f;
    IEC_VAL(mp.DIRECTION) = 1;
    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 120.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
               "MoveProfile should report COMMANDABORTED when a direct command takes over");
}
```

Add the new test to `main()`.

- [ ] **Step 4: Run the targeted tests to verify they fail**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_unit test_motion_interface_done_signals test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- at least one new assertion fails
- failure is behavioral, not a compile/link error

- [ ] **Step 5: Commit the red boundary tests**

```bash
git add tests/test_motion_interface_unit.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "test: add reset and recipe direct boundary coverage"
```

## Task 2: Add Dispatcher Execution Source and Owner Metadata

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add dispatcher execution-source and owner enums in `include/motion_control.h`**

```c
typedef enum {
    HYD_EXEC_SRC_NONE = 0,
    HYD_EXEC_SRC_RECIPE,
    HYD_EXEC_SRC_DIRECT,
    HYD_EXEC_SRC_SYSTEM
} HYD_ExecutionSource;

typedef enum {
    HYD_OWNER_NONE = 0,
    HYD_OWNER_MOVE_PROFILE,
    HYD_OWNER_MOVE_ABSOLUTE,
    HYD_OWNER_MOVE_VELOCITY,
    HYD_OWNER_PRESSURE_HANDLE,
    HYD_OWNER_STOP,
    HYD_OWNER_RESET
} HYD_DispatchOwner;
```

- [ ] **Step 2: Add minimal dispatcher fields to `HYD_MotionControlFB`**

```c
    HYD_ExecutionSource _executionSource;
    HYD_DispatchOwner _dispatchOwner;
    HYD_DispatchOwner _lastPreemptedOwner;
```

- [ ] **Step 3: Add dispatcher query APIs**

```c
HYD_ExecutionSource HYD_MotionControlFB_GetExecutionSource(const HYD_MotionControlFB* fb);
HYD_DispatchOwner HYD_MotionControlFB_GetDispatchOwner(const HYD_MotionControlFB* fb);
HYD_DispatchOwner HYD_MotionControlFB_GetLastPreemptedOwner(const HYD_MotionControlFB* fb);
```

- [ ] **Step 4: Initialize/reset dispatcher fields in `Init` and `SoftReset`**

```c
fb->_executionSource = HYD_EXEC_SRC_NONE;
fb->_dispatchOwner = HYD_OWNER_NONE;
fb->_lastPreemptedOwner = HYD_OWNER_NONE;
```

- [ ] **Step 5: Synchronize dispatcher state on direct start and recipe start**

In direct-source `HYD_BeginSegment(...)`:

```c
fb->_executionSource = HYD_EXEC_SRC_DIRECT;
fb->_dispatchOwner = HYD_OWNER_MOVE_ABSOLUTE; /* or inferred direct owner */
```

In recipe start path:

```c
fb->_executionSource = HYD_EXEC_SRC_RECIPE;
fb->_dispatchOwner = HYD_OWNER_MOVE_PROFILE;
```

Use the smallest helper set necessary to infer the correct owner kind from the active segment and start source.

- [ ] **Step 6: Rebuild and rerun the boundary tests**

Run:

```bash
cmake --build out/build/unixgcc --target test_motion_interface_unit test_motion_interface_done_signals test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- build succeeds
- boundary tests remain behaviorally red until IEC mapping is updated

- [ ] **Step 7: Commit the dispatcher scaffolding**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "refactor: add reset and ownership dispatcher scaffolding"
```

## Task 3: Unify Reset Core Takeover Semantics

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Record previous owner/source before reset takeover**

In the core reset path, capture the displaced owner:

```c
fb->_lastPreemptedExecutionId = fb->_directOwnerExecutionId;
fb->_lastPreemptedKind = fb->_directOwnerKind;
fb->_lastPreemptedOwner = fb->_dispatchOwner;
```

Guard recipe/direct/source cases so stale direct-only metadata is not reported when recipe ownership is active.

- [ ] **Step 2: Mark reset as a system-level owner during takeover**

Before or during soft reset transition:

```c
fb->_executionSource = HYD_EXEC_SRC_SYSTEM;
fb->_dispatchOwner = HYD_OWNER_RESET;
```

Then ensure `SoftReset` returns the dispatcher to:

```c
fb->_executionSource = HYD_EXEC_SRC_NONE;
fb->_dispatchOwner = HYD_OWNER_NONE;
```

- [ ] **Step 3: Preserve configuration while clearing execution state**

Do not change the current persisted-configuration list in `HYD_MotionControlFB_SoftReset`, but verify and keep these preserved:

```c
fb->RECIPE
fb->RECIPE_SIZE
fb->DIRECT_SEGMENT
fb->DIRECT_SEGMENT_VALID
fb->_params
fb->USE_RECIPE
```

- [ ] **Step 4: Run targeted reset-boundary tests**

Run:

```bash
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- reset-related new tests may still fail in IEC mapping
- no compile/link regressions

- [ ] **Step 5: Commit the core reset takeover semantics**

```bash
git add src/motion_control.c
git commit -m "feat: add reset dispatcher takeover semantics"
```

## Task 4: Refactor IEC Mapping for Reset and Recipe/Direct Preemption

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_interface_done_signals.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Keep `__mcl_cmd_Reset` immediate completion behavior**

Preserve:

```c
if (execRising)
{
    HYD_MotionControlFB_SoftReset(fb);
    __SET_VAR(data__->, DONE, , true);
    __SET_VAR(data__->, BUSY, , false);
}
```

But make sure downstream owners can observe the resulting preemption fact after reset.

- [ ] **Step 2: Map direct owners displaced by reset to `COMMANDABORTED`**

Use dispatcher/preemption facts so:

- `MoveAbsolute`, `MoveVelocity`, `PressureHandle`, `Stop` report `COMMANDABORTED = 1` after reset takeover
- their band signals clear immediately

### Step 3: Map recipe owner displaced by direct takeover

Refactor the relevant `MoveProfile` path in `motion_interface.c` so a recipe owner displaced by a direct command reports:

```c
COMMANDABORTED = true
BUSY = false
ACTIVE = false
DONE = false
```

Use dispatcher/preemption facts, not bare state heuristics.

- [ ] **Step 4: Map direct owner displaced by recipe takeover**

For direct commands, use dispatcher/preemption facts so a recipe start can preempt a direct owner consistently.

- [ ] **Step 5: Preserve direct-session and band-signal clearing semantics**

Ensure:

- `INVELOCITY` clears on reset or recipe takeover
- `INPRESSURE` clears on reset or recipe takeover
- latch resets still work on `EXECUTE = 0`

- [ ] **Step 6: Run the three boundary-focused test binaries**

Run:

```bash
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected:

- all new reset/recipe-direct boundary assertions pass

- [ ] **Step 7: Commit the IEC dispatcher mapping refactor**

```bash
git add src/motion_interface.c
git commit -m "refactor: unify reset and recipe direct ownership mapping"
```

## Task 5: Full Regression and Final Verification

**Files:**
- Test: `out/build/unixgcc/*`

- [ ] **Step 1: Run key focused binaries**

Run:

```bash
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_done_signals
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected:

- all four pass

- [ ] **Step 2: Run full suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected:

- all registered tests pass
- zero failures

- [ ] **Step 3: Verify worktree cleanliness**

Run:

```bash
git status --short
```

Expected:

- clean working tree after final commit

- [ ] **Step 4: Commit final verified boundary integration**

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_unit.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_arbitration.c
git commit -m "feat: unify reset and recipe direct ownership boundaries"
```
