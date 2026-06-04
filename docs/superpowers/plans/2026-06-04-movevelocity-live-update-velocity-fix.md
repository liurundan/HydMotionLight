# MoveVelocity CONTINUOUSUPDATE Velocity Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix MC_MoveVelocity CONTINUOUSUPDATE=1 where VELOCITY=0 or negative triggers COMMAND_NOT_ALLOWED — normalize velocity to absolute magnitude and resolve direction from sign in the IEC adapter layer, and relax the TIME_BASED validator to accept maxVelocity=0.

**Architecture:** Two-point fix: (1) `applyMoveVelocityLiveUpdate` in the IEC adapter gains SHORTEST_WAY direction derivation from velocity sign plus `fabs` normalization, matching the execRising path; (2) `recipe_validator.c` changes `maxVelocity <= 0.0` to `maxVelocity < 0.0` for TIME_BASED non-PRESSURE modes, since the planner already safely handles maxVelocity=0.

**Tech Stack:** C99, no dynamic allocation.

---

### Task 1: Add tests for VELOCITY normalization and direction flip in MoveVelocity live update

**Files:**
- Modify: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Write test — MoveVelocity live update negative VELOCITY flips direction**

Add this test function. Place it before the `test_live_update_request_carries_flags_and_direction` function (or before `main` at file end if that test doesn't exist yet in the current file — append before `main`):

```c
static void test_movevelocity_live_update_negative_velocity_flips_direction(void)
{
    HYD_MotionControlFB fb;
    HYD_MOVEVELOCITY mv;

    printf("--- Test: MoveVelocity live update negative VELOCITY flips direction ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb._params.velocityToFlowGain = 0.2f;
    fb._params.maxFlow = 50.0f;
    fb._params.timeoutLimit = 30.0f;
    fb._params.positionTolerance = 0.5f;

    /* Phase 1: Start MoveVelocity with VELOCITY=5, SHORTEST_WAY */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EXECUTE) = true;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.DIRECTION) = 0;  /* SHORTEST_WAY */
    IEC_VAL(mv.VELOCITY) = 5.0;
    IEC_VAL(mv.ACCELERATION) = 50.0;
    IEC_VAL(mv.DECELERATION) = 50.0;
    IEC_VAL(mv.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.BUSY), "MoveVelocity should be BUSY after execute");

    /* Simulate 1 scan cycle to latch ownership */
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    /* Verify initial state: direction POSITIVE, maxVelocity=5 */
    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Initial direction should be POSITIVE (derived from +VELOCITY)");
    ASSERT_TRUE(fb._activeSegment.maxVelocity == 5.0,
        "Initial maxVelocity should be 5.0");

    /* Phase 2: Live update VELOCITY=-5, still SHORTEST_WAY.
     * Expected: direction flips to NEGATIVE, maxVelocity=fabs(-5)=5.0 */
    IEC_VAL(mv.VELOCITY) = -5.0;
    fb.AXIS_REF.timestamp += 0.001;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_FALSE(IEC_VAL(mv.ERROR),
        "Live update with negative VELOCITY should NOT produce ERROR");
    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should flip to NEGATIVE after negative VELOCITY update");
    ASSERT_TRUE(fb._activeSegment.maxVelocity == 5.0,
        "maxVelocity should be 5.0 (fabs of -5) after negative update");

    printf("  PASS: MoveVelocity live update negative VELOCITY flips direction\n");
}
```

- [ ] **Step 2: Write test — MoveVelocity live update VELOCITY=0**

```c
static void test_movevelocity_live_update_zero_velocity_decel_to_stop(void)
{
    HYD_MotionControlFB fb;
    HYD_MOVEVELOCITY mv;

    printf("--- Test: MoveVelocity live update VELOCITY=0 decelerates to stop ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb._params.velocityToFlowGain = 0.2f;
    fb._params.maxFlow = 50.0f;
    fb._params.timeoutLimit = 30.0f;
    fb._params.positionTolerance = 0.5f;

    /* Phase 1: Start MoveVelocity with VELOCITY=5, SHORTEST_WAY */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EXECUTE) = true;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.DIRECTION) = 0;  /* SHORTEST_WAY */
    IEC_VAL(mv.VELOCITY) = 5.0;
    IEC_VAL(mv.ACCELERATION) = 50.0;
    IEC_VAL(mv.DECELERATION) = 50.0;
    IEC_VAL(mv.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.BUSY), "MoveVelocity should be BUSY after execute");

    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp += 0.001;

    /* Phase 2: Live update VELOCITY=0.
     * Expected: no ERROR, maxVelocity=0, direction stays POSITIVE.
     * The planner will return 0.0 velocity for maxVelocity<=0. */
    IEC_VAL(mv.VELOCITY) = 0.0;
    fb.AXIS_REF.timestamp += 0.001;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_FALSE(IEC_VAL(mv.ERROR),
        "Live update with VELOCITY=0 should NOT produce ERROR");
    ASSERT_TRUE(fb._activeSegment.maxVelocity == 0.0,
        "maxVelocity should be 0.0 after zero VELOCITY update");
    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Direction should stay POSITIVE (lastActiveDirection for zero VELOCITY)");

    /* Verify planner produces zero output with maxVelocity=0 */
    ASSERT_TRUE(fb.STATE.plannedVelocity == 0.0 || fb.STATE.plannedFlow == 0.0,
        "Planner should produce zero output when maxVelocity is 0");

    printf("  PASS: MoveVelocity live update VELOCITY=0 decelerates to stop\n");
}
```

- [ ] **Step 3: Write test — validator accepts maxVelocity=0 for SPEED_RAMP**

```c
static void test_validate_segment_accepts_zero_maxvelocity_speed_ramp(void)
{
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("--- Test: ValidateSegment accepts maxVelocity=0 for SPEED_RAMP ---\n");

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_POSITIVE;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.maxVelocity = 0.0;
    seg.maxAcceleration = 50.0;
    seg.maxDeceleration = 50.0;
    seg.velocityToFlowGain = 0.2f;
    seg.maxFlow = 50.0f;
    seg.timeoutLimit = 30.0f;

    HYD_BOOL result = HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL);
    ASSERT_TRUE(result,
        "ValidateSegment should accept SPEED_RAMP with maxVelocity=0");
    ASSERT_TRUE(code == HYD_DIAG_CODE_NONE,
        "Diagnostic code should be NONE for valid maxVelocity=0 segment");

    printf("  PASS: ValidateSegment accepts maxVelocity=0 for SPEED_RAMP\n");
}
```

- [ ] **Step 4: Register tests and build**

At the end of the file, in the test runner function that calls all tests, add:

```c
    test_movevelocity_live_update_negative_velocity_flips_direction();
    test_movevelocity_live_update_zero_velocity_decel_to_stop();
    test_validate_segment_accepts_zero_maxvelocity_speed_ramp();
```

Build and run the new tests to verify they FAIL (since the fix isn't implemented yet):

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -5
```

Then run the specific tests (they should fail):

```bash
./out/build/unixgcc/test_motion_interface_unit 2>&1 | grep -E "negative_velocity|zero_velocity|accepts_zero"
```

Expected: tests FAIL on ERROR flag or validator rejection.

- [ ] **Step 5: Commit**

```bash
git add tests/test_motion_interface_unit.c
git commit -m "test: failing tests for MoveVelocity negative/zero VELOCITY live update"
```

---

### Task 2: Fix `applyMoveVelocityLiveUpdate` — normalize VELOCITY and resolve direction

**Files:**
- Modify: `src/motion_interface.c:422-445`

- [ ] **Step 1: Apply the fix**

In `src/motion_interface.c`, replace the `applyMoveVelocityLiveUpdate` function (lines 422-445):

```c
static HYD_BOOL applyMoveVelocityLiveUpdate(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_MOVEVELOCITY* data__)
{
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    HYD_REAL rawVelocity = __GET_VAR(data__->VELOCITY);
    HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));

    /* SHORTEST_WAY: derive direction from Velocity sign — match execRising path */
    if (dir == HYD_DIRECTION_SHORTEST_WAY) {
        if (rawVelocity > 0.0f) {
            dir = HYD_DIRECTION_POSITIVE;
        } else if (rawVelocity < 0.0f) {
            dir = HYD_DIRECTION_NEGATIVE;
        } else {
            dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
                  ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
        }
    }

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerExecutionId = (uint16_t)execId;
    request.maxVelocity = (IEC_REAL)fabs((double)rawVelocity);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.direction = dir;
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}
```

- [ ] **Step 2: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 3: Run negative VELOCITY test (should PASS now)**

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -3
./out/build/unixgcc/test_motion_interface_unit 2>&1 | grep -A2 "negative_velocity"
```

Expected: `PASS: MoveVelocity live update negative VELOCITY flips direction`

- [ ] **Step 4: Commit**

```bash
git add src/motion_interface.c
git commit -m "fix: normalize VELOCITY and resolve direction in MoveVelocity live update"
```

---

### Task 3: Fix `recipe_validator.c` — accept maxVelocity=0 for TIME_BASED

**Files:**
- Modify: `src/recipe_validator.c:298-302`

- [ ] **Step 1: Apply the fix**

In `src/recipe_validator.c`, at lines 298-302, change `maxVelocity <= 0.0` to `maxVelocity < 0.0`:

```c
    if ((segment->planner == HYD_PLANNER_TIME_BASED) &&
        (segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->maxVelocity < 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }
```

- [ ] **Step 2: Build**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
```

Expected: clean build.

- [ ] **Step 3: Run all new tests (should all PASS)**

```bash
cmake --build --preset unixgcc --target test_motion_interface_unit 2>&1 | tail -3
./out/build/unixgcc/test_motion_interface_unit 2>&1 | grep -E "PASS|FAIL"
```

Expected: all three new tests show PASS, no FAIL.

- [ ] **Step 4: Commit**

```bash
git add src/recipe_validator.c
git commit -m "fix: accept maxVelocity=0 for TIME_BASED SPEED_RAMP in validator"
```

---

### Task 4: Full regression verification

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

- [ ] **Step 3: Verify no compiler warnings**

```bash
cmake --build --preset unixgcc 2>&1 | grep -i "warning" || echo "No warnings found"
```

Expected: "No warnings found".

- [ ] **Step 4: Commit**

```bash
git commit -m "chore: full regression pass after MoveVelocity velocity fix" --allow-empty
```

---

## Summary

| Task | File | Change |
|------|------|--------|
| 1 | `tests/test_motion_interface_unit.c` | 3 new tests (negative VELOCITY flip, zero VELOCITY stop, validator maxVelocity=0) |
| 2 | `src/motion_interface.c` | `applyMoveVelocityLiveUpdate` — SHORTEST_WAY direction derivation + fabs normalization |
| 3 | `src/recipe_validator.c` | TIME_BASED maxVelocity `<=` → `<` |
| 4 | — | Full regression |
