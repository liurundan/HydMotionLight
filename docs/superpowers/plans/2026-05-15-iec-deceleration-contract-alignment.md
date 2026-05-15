# IEC Deceleration Contract Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the PLC-facing `DECELERATION` inputs on `HYD_MoveAbsolute`, `HYD_MoveVelocity`, and `HYD_AxisMotion` carry real independent runtime semantics instead of silently collapsing to `ACCELERATION`.

**Architecture:** Keep the existing middle-layer boundary intact. This plan does not add machine workflow logic or expand the public XML surface. It introduces a distinct `maxDeceleration` segment property inside the runtime model, maps the existing IEC pins into that property, updates planner/runtime use sites to consume it, and then tightens the architecture docs so the public contract no longer over-promises or under-documents current behavior.

**Tech Stack:** C99, HydroMotionLib runtime and planner (`src/motion_control.c`, `src/motion_planner.c`), IEC adapter (`src/motion_interface.c`), unit tests in `tests/`, CMake/CTest, architecture docs in `docs/architecture/`.

---

## File Structure

- Modify: `include/common_types.h`
  Add a dedicated `maxDeceleration` field to `HYD_MotionSegment` so runtime data can represent acceleration and deceleration independently.
- Modify: `src/motion_planner.c`
  Consume `segment->maxDeceleration` in braking/deceleration paths instead of reusing `segment->maxAcceleration`.
- Modify: `src/recipe_validator.c`
  Validate the new `maxDeceleration` field so recipe/direct segment validation remains explicit.
- Modify: `src/motion_interface.c`
  Map PLC-facing `DECELERATION` inputs into `HYD_MotionSegment.maxDeceleration` and round-trip it back through `HYD_AXISMOTION`.
- Modify: `tests/test_motion_planner.c`
  Add focused planner-level tests proving deceleration is independent from acceleration.
- Modify: `tests/test_motion_interface_unit.c`
  Add IEC-surface tests proving direct FBs and `LoadProfile` preserve `DECELERATION` independently.
- Modify: `docs/architecture/motion-runtime-contract.md`
  Remove the stale note that `DECELERATION` lacks independent semantics and document the supported behavior precisely.
- Modify: `docs/architecture/implementation-contract-gap-list.md`
  Remove or downgrade the deceleration-related gap after implementation lands.

### Task 1: Add failing deceleration contract tests

**Files:**
- Modify: `tests/test_motion_planner.c`
- Modify: `tests/test_motion_interface_unit.c`
- Test: `tests/test_motion_planner.c`
- Test: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add a failing planner test for position-mode braking**

In `tests/test_motion_planner.c`, add this test before `test_speed_ramp_deceleration_on_stop()`:

```c
static void test_position_mode_uses_max_deceleration_for_braking(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_REAL remainingDistance;
    HYD_REAL expectedVelocityMagnitude;

    printf("Testing POSITION braking uses maxDeceleration...\n");

    axisRef = create_test_axis_ref(90.0);
    segment = create_test_segment();
    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 40.0;
    segment.maxDeceleration = 5.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.5;
    input.rampedPressure = 0.0;
    input.decelElapsed = 0.0;
    input.decelStartVel = 0.0;

    HYD_MotionPlanner_Execute(&input, &output);

    remainingDistance = segment.targetPosition - axisRef.position;
    expectedVelocityMagnitude = sqrt(2.0 * segment.maxDeceleration * remainingDistance);
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(fabs(output.targetVelocity - expectedVelocityMagnitude) < 0.001);
    printf("✓ POSITION braking uses maxDeceleration\n");
}
```

- [ ] **Step 2: Add a failing planner test for speed-ramp deceleration**

In `tests/test_motion_planner.c`, update `test_speed_ramp_deceleration_on_stop()` so it stops deriving the deceleration ramp from `segment.maxAcceleration` and instead asserts `segment.maxDeceleration` is used:

```c
    segment.maxAcceleration = 4.0;
    segment.maxDeceleration = 2.0;
    segment.maxVelocity = 20.0;
```

Replace the current deceleration assertions inside that test with:

```c
    startVel = 16.0;

    input.decelStartVel = startVel;
    input.decelElapsed = 1.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - (startVel - segment.maxDeceleration * 1.0)) < 0.001);

    input.decelElapsed = 2.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - (startVel - segment.maxDeceleration * 2.0)) < 0.001);
```

- [ ] **Step 3: Add a failing IEC test for `MoveAbsolute.DECELERATION`**

In `tests/test_motion_interface_unit.c`, add this test after `test_moveabsolute_rejects_invalid_axis_index()`:

```c
static void test_moveabsolute_maps_deceleration_independently(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

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
    IEC_VAL(ma.DECELERATION) = 7.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    __mcl_cmd_MoveAbsolute(&ma);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "MoveAbsolute deceleration test should resolve an FB");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "MoveAbsolute should load a direct segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxAcceleration == 50.0f,
               "MoveAbsolute should preserve ACCELERATION");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxDeceleration == 7.0f,
               "MoveAbsolute should map DECELERATION independently");
}
```

- [ ] **Step 4: Add a failing IEC test for `MoveVelocity.DECELERATION`**

In `tests/test_motion_interface_unit.c`, add this test after `test_movevelocity_rejects_invalid_axis_index()`:

```c
static void test_movevelocity_maps_deceleration_independently(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 6.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "MoveVelocity deceleration test should resolve an FB");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "MoveVelocity should load a direct segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxAcceleration == 100.0f,
               "MoveVelocity should preserve ACCELERATION");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxDeceleration == 6.0f,
               "MoveVelocity should map DECELERATION independently");
}
```

- [ ] **Step 5: Add a failing `HYD_AXISMOTION` round-trip test**

In `tests/test_motion_interface_unit.c`, add this test after `test_loadprofile_keeps_segment_tag_and_type_separate()`:

```c
static void test_loadprofile_preserves_independent_accel_and_decel(void) {
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

    motion.SEGMENTTAG = 10;
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 50.0f;
    motion.SETVELOCITY = 12.0f;
    motion.ACCELERATION = 30.0f;
    motion.DECELERATION = 4.0f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile accel/decel test should resolve an FB");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "LoadProfile should preload one recipe segment");
    ASSERT_TRUE(fb->RECIPE[0].maxAcceleration == 30.0f,
               "LoadProfile should preserve ACCELERATION");
    ASSERT_TRUE(fb->RECIPE[0].maxDeceleration == 4.0f,
               "LoadProfile should preserve DECELERATION independently");
}
```

- [ ] **Step 6: Register the new tests in the test runners**

In `tests/test_motion_planner.c`, add:

```c
    test_position_mode_uses_max_deceleration_for_braking();
```

before:

```c
    test_speed_ramp_deceleration_on_stop();
```

In `tests/test_motion_interface_unit.c`, add:

```c
    test_moveabsolute_maps_deceleration_independently();
    test_loadprofile_preserves_independent_accel_and_decel();
    test_movevelocity_maps_deceleration_independently();
```

inside `main()` near the surrounding related tests.

- [ ] **Step 7: Run the focused tests to verify they fail**

Run:

```bash
cmake --build build -j2 --target test_motion_planner test_motion_interface_unit
ctest --test-dir build --output-on-failure -R "test_motion_planner|test_motion_interface_unit"
```

Expected:

- `test_motion_planner` fails because braking still uses `maxAcceleration`
- `test_motion_interface_unit` fails because `maxDeceleration` does not exist or is not mapped independently

- [ ] **Step 8: Commit the failing tests**

```bash
git add tests/test_motion_planner.c tests/test_motion_interface_unit.c
git commit -m "test: capture deceleration contract gaps"
```

### Task 2: Implement independent deceleration semantics

**Files:**
- Modify: `include/common_types.h`
- Modify: `src/motion_planner.c`
- Modify: `src/recipe_validator.c`
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_planner.c`
- Test: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add `maxDeceleration` to `HYD_MotionSegment`**

In `include/common_types.h`, update the segment definition:

```c
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL maxVelocity;     /* mm/s, velocity magnitude limit */
```

Place `maxDeceleration` immediately after `maxAcceleration` so acceleration/deceleration remain grouped in the runtime model.

- [ ] **Step 2: Validate the new field in recipe validation**

In `src/recipe_validator.c`, add a nonnegative validation check after the existing `maxAcceleration` check:

```c
    if (segment->maxDeceleration < 0.0) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_INVALID;
        }
        return false;
    }
```

- [ ] **Step 3: Use `maxDeceleration` in planner braking paths**

In `src/motion_planner.c`, replace the braking/deceleration uses of `segment->maxAcceleration` with `segment->maxDeceleration` when it is positive, otherwise fall back to `segment->maxAcceleration`.

For position mode, replace:

```c
    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       input->segment->maxAcceleration,
                                                                       input->segment->maxVelocity);
```

with:

```c
    HYD_REAL brakingAcceleration =
        (input->segment->maxDeceleration > 0.0)
            ? input->segment->maxDeceleration
            : input->segment->maxAcceleration;

    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       brakingAcceleration,
                                                                       input->segment->maxVelocity);
```

For speed-ramp deceleration, replace:

```c
        decelVelocity = input->decelStartVel -
            input->segment->maxAcceleration * input->decelElapsed;
```

with:

```c
        HYD_REAL brakingAcceleration =
            (input->segment->maxDeceleration > 0.0)
                ? input->segment->maxDeceleration
                : input->segment->maxAcceleration;

        decelVelocity = input->decelStartVel -
            brakingAcceleration * input->decelElapsed;
```

Apply the same positive-fallback rule to the speed-ramp position-end brake branch.

- [ ] **Step 4: Map `DECELERATION` in direct segment builders**

In `src/motion_interface.c`, update `buildPositionSegment()` and `buildVelocitySegment()` signatures and assignments.

Change the signatures to:

```c
static HYD_MotionSegment buildPositionSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    const HYD_MotionControlFB* fb)
```

and:

```c
static HYD_MotionSegment buildVelocitySegment(
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    const HYD_MotionControlFB* fb)
```

Inside both builders, add:

```c
    seg.maxDeceleration = (deceleration > 0.0f) ? deceleration : acceleration;
```

- [ ] **Step 5: Map `HYD_AXISMOTION.DECELERATION` in profile paths**

In `src/motion_interface.c`, update `buildSegmentFromMotion()` and `writeMotionFromSegment()`.

Add this assignment in `buildSegmentFromMotion()`:

```c
    seg.maxDeceleration = (motion->DECELERATION > 0.0f)
        ? motion->DECELERATION
        : motion->ACCELERATION;
```

Replace the current write-back line:

```c
    motion->DECELERATION = (REAL)seg->maxAcceleration;
```

with:

```c
    motion->DECELERATION = (REAL)seg->maxDeceleration;
```

- [ ] **Step 6: Pass `DECELERATION` through the direct FB command handlers**

In `src/motion_interface.c`, update the `MoveAbsolute` builder call:

```c
        HYD_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

Update the `MoveVelocity` builder call:

```c
        HYD_MotionSegment segment = buildVelocitySegment(
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);
```

- [ ] **Step 7: Run the focused tests to verify they pass**

Run:

```bash
cmake --build build -j2 --target test_motion_planner test_motion_interface_unit
ctest --test-dir build --output-on-failure -R "test_motion_planner|test_motion_interface_unit"
```

Expected:

- `test_motion_planner` passes with the new deceleration-specific assertions
- `test_motion_interface_unit` passes with independent acceleration/deceleration mapping

- [ ] **Step 8: Commit the implementation**

```bash
git add include/common_types.h src/recipe_validator.c src/motion_planner.c src/motion_interface.c tests/test_motion_planner.c tests/test_motion_interface_unit.c
git commit -m "feat: implement independent deceleration contract"
```

### Task 3: Tighten documentation and verify the full suite

**Files:**
- Modify: `docs/architecture/motion-runtime-contract.md`
- Modify: `docs/architecture/implementation-contract-gap-list.md`
- Test: `build/`

- [ ] **Step 1: Update the runtime contract note**

In `docs/architecture/motion-runtime-contract.md`, replace the stale unsupported example:

```md
- `DECELERATION`, `JERK`, and `CONTINUOUSUPDATE` on some IEC FBs do not yet imply full independent runtime semantics
```

with:

```md
- `JERK` and `CONTINUOUSUPDATE` on some IEC FBs are still reserved compatibility pins and do not yet imply full runtime semantics
- `DECELERATION` is independently consumed by `MoveAbsolute`, `MoveVelocity`, and `HYD_AXISMOTION`, but broader PLCopen motion-profile semantics are still not claimed beyond the current braking/deceleration behavior
```

- [ ] **Step 2: Update the gap list summary and detailed gap**

In `docs/architecture/implementation-contract-gap-list.md`, narrow Gap 1 so it no longer lists `DECELERATION` as part of the unsupported exposed-pin set.

Update the detailed bullets from:

```md
- `HYD_MoveAbsolute` exposes `DECELERATION`, `JERK`, and `CONTINUOUSUPDATE`.
- `HYD_MoveVelocity` exposes `DECELERATION`, `JERK`, and `CONTINUOUSUPDATE`.
```

to:

```md
- `HYD_MoveAbsolute` still exposes `JERK` and `CONTINUOUSUPDATE`.
- `HYD_MoveVelocity` still exposes `JERK` and `CONTINUOUSUPDATE`.
```

Update the evidence from:

```md
- `buildPositionSegment()` uses `POSITION`, `VELOCITY`, `ACCELERATION`, and `DIRECTION`, but not `DECELERATION`, `JERK`, or `CONTINUOUSUPDATE`
- `buildVelocitySegment()` uses `VELOCITY`, `ACCELERATION`, and `DIRECTION`, but not `DECELERATION`, `JERK`, or `CONTINUOUSUPDATE`
```

to:

```md
- `buildPositionSegment()` now maps `DECELERATION` independently, but still does not consume `JERK` or `CONTINUOUSUPDATE`
- `buildVelocitySegment()` now maps `DECELERATION` independently, but still does not consume `JERK` or `CONTINUOUSUPDATE`
```

- [ ] **Step 3: Run the full project verification**

Run:

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 tests/test_interface_layout_consistency.py
git diff --check
```

Expected:

- all configured tests pass
- interface layout consistency passes unchanged
- `git diff --check` prints no whitespace errors

- [ ] **Step 4: Commit the documentation alignment**

```bash
git add docs/architecture/motion-runtime-contract.md docs/architecture/implementation-contract-gap-list.md
git commit -m "docs: align deceleration contract notes"
```

## Why This Plan First

1. It removes the highest-value remaining public-contract mismatch without widening the PLC-facing ABI.
2. It preserves backward compatibility better than rejecting `DECELERATION`, because existing XML, examples, and tests already wire that pin.
3. It leaves lower-priority follow-ups isolated for later plans:
   - header-comment/runtime legality cleanup
   - configuration source-of-truth cleanup
   - remaining reserved compatibility pins (`JERK`, `CONTINUOUSUPDATE`)

## Self-Review

Spec coverage against current architecture docs:

- `motion-runtime-contract.md` unsupported-pin caveat is addressed by Tasks 1-3.
- `implementation-contract-gap-list.md` High Gap 1 is narrowed by Tasks 2-3.
- No machine workflow, valve logic, or PLC process-layer behavior is added; this stays inside the documented middle-layer boundary.

Placeholder scan:

- No `TODO`, `TBD`, or “similar to above” placeholders remain.

Type consistency:

- The plan consistently uses `HYD_MotionSegment.maxDeceleration` as the runtime field name.
- IEC-facing names remain `DECELERATION` in `HYD_MoveAbsolute`, `HYD_MoveVelocity`, and `HYD_AXISMOTION`.
