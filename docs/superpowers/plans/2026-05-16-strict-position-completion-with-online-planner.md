# Strict Position Completion With Online Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `HYD_END_POSITION` complete only after position is reached and both planned and actual velocities have settled.

**Architecture:** Keep velocity planning in `motion_planner.c`; move lifecycle correctness into `segment_completion.c`. Add a local settled-velocity resolver that prefers `stableVelocityLimit`, then `velocityTolerance`, then a `1.0 mm/s` default, and apply it before the existing stable-window candidate can start.

**Tech Stack:** C99, CMake preset `unixgcc`, existing `HydroMotionLib`, assert-based C tests.

---

## File Structure

- Modify `src/segment_completion.c`
  - Add `HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE`.
  - Add focused helpers for resolving the position-settled velocity tolerance and checking the position window.
  - Change only the `HYD_END_POSITION` branch so non-position end conditions remain unchanged.
- Modify `tests/segment_completion_test.c`
  - Add unit coverage for planned velocity, actual velocity, settled completion, stable velocity override, default fallback, stable-window reset, and non-position compatibility.
- Modify `tests/test_moveabsolute_stop_integration.c`
  - Add a runtime position-control test that starts a direct position segment already inside the position window while planned and actual velocity are still non-zero, and verifies the segment is not marked done until velocity settles.
- Create this plan file: `docs/superpowers/plans/2026-05-16-strict-position-completion-with-online-planner.md`

## Existing Contract To Preserve

`HYD_SegmentCompletion_CheckWithContext()` may be called without runtime references. In that case, the planned velocity fallback must be `0.0`, so existing standalone calls that use `HYD_SegmentCompletion_Check()` still work when actual axis velocity is settled.

`HYD_SegmentCompletion_ApplyStableWindow()` currently gates `stableVelocityLimit` for every end condition. Leave that broad behavior intact. The new strict position gate duplicates the actual-velocity check only when `HYD_END_POSITION` is evaluated, using the same tolerance if `stableVelocityLimit > 0`.

---

### Task 1: Add Strict Position Completion Unit Tests

**Files:**
- Modify: `tests/segment_completion_test.c`

- [ ] **Step 1: Add a helper for context-based position checks**

Insert this helper after `create_segment()`:

```c
static HYD_BOOL check_position_with_velocities(const HYD_MotionSegment* segment,
                                               HYD_REAL position,
                                               HYD_REAL actualVelocity,
                                               HYD_REAL plannedVelocity,
                                               HYD_TIME timestamp,
                                               HYD_TIME* candidateStart,
                                               HYD_BOOL* candidateActive) {
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_SegmentCompletionContext context = {0};

    axisRef.position = position;
    axisRef.velocity = actualVelocity;
    axisRef.timestamp = timestamp;
    references.elapsedTime = timestamp;
    references.velocityReference = plannedVelocity;

    context.segment = segment;
    context.axisRef = &axisRef;
    context.references = &references;
    context.timestamp = timestamp;
    context.candidateStartTime = candidateStart;
    context.candidateActive = candidateActive;
    return HYD_SegmentCompletion_CheckWithContext(&context);
}
```

- [ ] **Step 2: Add failing unit tests for strict position completion**

Insert these tests after `test_runtime_reference_context_overrides_segment_targets()`:

```c
static void test_position_completion_rejects_unsettled_planned_velocity(void) {
    HYD_MotionSegment segment = create_segment();

    printf("Testing position completion rejects unsettled planned velocity...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 1.0;

    assert(!check_position_with_velocities(&segment, 99.95, 0.2, 6.5, 1.0, NULL, NULL));
    printf("✓ Planned velocity settled gate test passed\n");
}

static void test_position_completion_rejects_unsettled_actual_velocity(void) {
    HYD_MotionSegment segment = create_segment();

    printf("Testing position completion rejects unsettled actual velocity...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 1.0;

    assert(!check_position_with_velocities(&segment, 99.95, 6.5, 0.2, 1.0, NULL, NULL));
    printf("✓ Actual velocity settled gate test passed\n");
}

static void test_position_completion_accepts_settled_planned_and_actual_velocity(void) {
    HYD_MotionSegment segment = create_segment();

    printf("Testing position completion accepts settled planned and actual velocity...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 1.0;

    assert(check_position_with_velocities(&segment, 99.95, 0.5, 0.4, 1.0, NULL, NULL));
    printf("✓ Settled position completion test passed\n");
}

static void test_position_completion_stable_velocity_limit_overrides_velocity_tolerance(void) {
    HYD_MotionSegment segment = create_segment();

    printf("Testing stableVelocityLimit overrides velocityTolerance for position completion...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 5.0;
    segment.stableVelocityLimit = 0.5;

    assert(!check_position_with_velocities(&segment, 99.95, 0.8, 0.8, 1.0, NULL, NULL));
    assert(check_position_with_velocities(&segment, 99.95, 0.4, 0.4, 1.0, NULL, NULL));
    printf("✓ Stable velocity override test passed\n");
}

static void test_position_completion_uses_default_velocity_tolerance_when_unconfigured(void) {
    HYD_MotionSegment segment = create_segment();

    printf("Testing default settled velocity tolerance for position completion...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 0.0;
    segment.stableVelocityLimit = 0.0;

    assert(!check_position_with_velocities(&segment, 99.95, 1.2, 0.8, 1.0, NULL, NULL));
    assert(!check_position_with_velocities(&segment, 99.95, 0.8, 1.2, 1.0, NULL, NULL));
    assert(check_position_with_velocities(&segment, 99.95, 0.8, 0.8, 1.0, NULL, NULL));
    printf("✓ Default settled velocity tolerance test passed\n");
}

static void test_position_completion_stable_window_resets_on_unsettled_planned_velocity(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_TIME candidateStart = 0.0;
    HYD_BOOL candidateActive = false;

    printf("Testing stable window resets on unsettled planned velocity...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 1.0;
    segment.stableWindow = 0.2;

    assert(!check_position_with_velocities(&segment, 99.95, 0.2, 2.0, 1.0,
                                           &candidateStart, &candidateActive));
    assert(!candidateActive);

    assert(!check_position_with_velocities(&segment, 99.95, 0.2, 0.2, 1.1,
                                           &candidateStart, &candidateActive));
    assert(candidateActive);
    assert(fabs(candidateStart - 1.1) < 0.000001);

    assert(check_position_with_velocities(&segment, 99.95, 0.2, 0.2, 1.35,
                                          &candidateStart, &candidateActive));
    printf("✓ Stable window planned velocity reset test passed\n");
}

static void test_pressure_completion_ignores_velocity_reference_gate(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references = {0};
    HYD_SegmentCompletionContext context = {0};

    printf("Testing non-position completion ignores velocity reference gate...\n");
    segment.endCondition = HYD_END_PRESSURE;
    segment.targetPressure = 50.0;
    axisRef = create_axis_ref(0.0, 50.0, 0.0);
    axisRef.velocity = 10.0;
    references.velocityReference = 10.0;
    references.pressureReference = 50.0;

    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = &references;
    context.timestamp = 1.0;

    assert(HYD_SegmentCompletion_CheckWithContext(&context));
    printf("✓ Non-position compatibility test passed\n");
}
```

- [ ] **Step 3: Register the new tests in `main()`**

Add these calls after `test_runtime_reference_context_overrides_segment_targets();`:

```c
    test_position_completion_rejects_unsettled_planned_velocity();
    test_position_completion_rejects_unsettled_actual_velocity();
    test_position_completion_accepts_settled_planned_and_actual_velocity();
    test_position_completion_stable_velocity_limit_overrides_velocity_tolerance();
    test_position_completion_uses_default_velocity_tolerance_when_unconfigured();
    test_position_completion_stable_window_resets_on_unsettled_planned_velocity();
    test_pressure_completion_ignores_velocity_reference_gate();
```

- [ ] **Step 4: Run the unit test and verify RED**

Run:

```bash
cmake --build --preset unixgcc --target segment_completion_test && ./out/build/unixgcc/segment_completion_test
```

Expected: build succeeds, executable fails at `test_position_completion_rejects_unsettled_planned_velocity()` because old `HYD_END_POSITION` completion ignores `velocityReference`.

---

### Task 2: Implement Strict Position Completion Gate

**Files:**
- Modify: `src/segment_completion.c`

- [ ] **Step 1: Add default tolerance and helper functions**

Insert after the includes:

```c
#define HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE 1.0

static HYD_REAL HYD_SegmentCompletion_ResolvePositionSettledVelocityTolerance(
    const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE;
    }
    if (segment->stableVelocityLimit > 0.0) {
        return segment->stableVelocityLimit;
    }
    if (segment->velocityTolerance > 0.0) {
        return segment->velocityTolerance;
    }
    return HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE;
}

static HYD_BOOL HYD_SegmentCompletion_IsPositionReached(
    const HYD_MotionSegment* segment,
    const HYD_AxisRef* axisRef,
    HYD_REAL positionTolerance) {
    HYD_MotionDirection direction;

    direction = HYD_Segment_ResolveDirection(segment, axisRef);
    if (direction == HYD_DIRECTION_EXTEND) {
        return axisRef->position >= segment->targetPosition - positionTolerance;
    }
    if (direction == HYD_DIRECTION_RETRACT) {
        return axisRef->position <= segment->targetPosition + positionTolerance;
    }
    return fabs(axisRef->position - segment->targetPosition) <= positionTolerance;
}

static HYD_BOOL HYD_SegmentCompletion_IsPositionVelocitySettled(
    const HYD_MotionSegment* segment,
    const HYD_AxisRef* axisRef,
    const HYD_ExecutionReference* references) {
    HYD_REAL settledVelocityTolerance;
    HYD_REAL velocityReference;

    settledVelocityTolerance =
        HYD_SegmentCompletion_ResolvePositionSettledVelocityTolerance(segment);
    velocityReference = (references != NULL) ? references->velocityReference : 0.0;

    return fabs(velocityReference) <= settledVelocityTolerance &&
           fabs(axisRef->velocity) <= settledVelocityTolerance;
}
```

- [ ] **Step 2: Replace the `HYD_END_POSITION` branch**

Replace the current `HYD_END_POSITION` case body with:

```c
        case HYD_END_POSITION:
            rawComplete =
                HYD_SegmentCompletion_IsPositionReached(segment, axisRef, positionTolerance) &&
                HYD_SegmentCompletion_IsPositionVelocitySettled(segment, axisRef, references);
            break;
```

- [ ] **Step 3: Remove the now-unused local `direction`**

Delete this declaration from `HYD_SegmentCompletion_CheckWithContext()`:

```c
    HYD_MotionDirection direction;
```

- [ ] **Step 4: Run unit test and verify GREEN**

Run:

```bash
cmake --build --preset unixgcc --target segment_completion_test && ./out/build/unixgcc/segment_completion_test
```

Expected: all `SegmentCompletion` tests pass.

- [ ] **Step 5: Commit Task 1-2**

Run:

```bash
git add src/segment_completion.c tests/segment_completion_test.c
git commit -m "fix: require settled velocity for position completion"
```

---

### Task 3: Add Runtime Integration Coverage

**Files:**
- Modify: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Add a test for near-target position motion that is still decelerating**

Insert this test before `main()`:

```c
static void test_position_segment_does_not_complete_until_velocity_settles(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 99.95;
    fb.AXIS_REF.velocity = 6.5;
    fb.AXIS_REF.flow = 6.5;
    fb.AXIS_REF.pressure = 20.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 200.0;
    segment.maxDeceleration = 200.0;
    segment.maxFlow = 500.0;
    segment.positionTolerance = 0.1;
    segment.velocityTolerance = 1.0;
    segment.velocityToFlowGain = 1.0;

    CHECK(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment),
          "Near-target direct position segment should load");
    CHECK(HYD_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp),
          "Near-target direct position segment should start");

    fb._plannerState.initialized = true;
    fb._plannerState.lastTargetVelocity = 6.5;

    fb.AXIS_REF.timestamp = 0.001;
    HYD_MotionControlFB_Scan(&fb);
    CHECK(fb.STATE.fbState != HYD_FB_STATE_DONE,
          "Position segment should not be DONE while planned and actual velocity are unsettled");
    CHECK(fabs(fb.STATE.plannedVelocity) > 1.0,
          "Planned velocity should still be above settled threshold inside position tolerance");

    fb.AXIS_REF.velocity = 0.2;
    fb.AXIS_REF.flow = 0.2;
    fb._plannerState.lastTargetVelocity = 0.2;
    fb.AXIS_REF.timestamp = 0.010;
    HYD_MotionControlFB_Scan(&fb);
    CHECK(fb.STATE.fbState == HYD_FB_STATE_DONE,
          "Position segment should complete after planned and actual velocity settle");
}
```

- [ ] **Step 2: Register the integration test in `main()`**

Add this call before the results print:

```c
    test_position_segment_does_not_complete_until_velocity_settles();
```

- [ ] **Step 3: Run integration target**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_stop_integration && ./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: all checks pass, including the new near-target completion lifecycle test.

- [ ] **Step 4: Commit Task 3**

Run:

```bash
git add tests/test_moveabsolute_stop_integration.c
git commit -m "test: cover position completion velocity settle in runtime"
```

---

### Task 4: Final Verification

**Files:**
- No additional source edits expected.

- [ ] **Step 1: Run focused verification**

Run:

```bash
cmake --build --preset unixgcc --target segment_completion_test test_moveabsolute_stop_integration && \
./out/build/unixgcc/segment_completion_test && \
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: both executables pass.

- [ ] **Step 2: Run broader build**

Run:

```bash
cmake --build --preset unixgcc
```

Expected: build succeeds.

- [ ] **Step 3: Run full CTest and record known failures separately**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all tests unrelated to the known recipe-size configuration issue pass. If `test_recipe_validator`, `test_sprint_b_integration`, or `test_motion_interface_arbitration` fail because `HYD_MAX_SEGMENTS` is currently configured as `1`, record that as pre-existing/out-of-scope and do not modify `include/hyd_config.h` in this plan.

- [ ] **Step 4: Inspect final diff**

Run:

```bash
git status --short --branch
git diff --stat HEAD
git log --oneline -3
```

Expected: only intended files changed, with commits for the plan and implementation.

## Self-Review

- Spec coverage: Tasks 1-2 implement position reached plus planned and actual velocity settled plus stable-window behavior. Task 3 covers runtime position integration. Task 1 includes non-position compatibility.
- Placeholder scan: The plan contains no `TBD`, no `TODO`, and no unspecified test requests.
- Type consistency: The plan uses existing types `HYD_MotionSegment`, `HYD_AxisRef`, `HYD_ExecutionReference`, `HYD_SegmentCompletionContext`, and existing fields from `include/common_types.h`.
