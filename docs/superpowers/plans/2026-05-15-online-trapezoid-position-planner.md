# Online Trapezoid Position Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the runtime `HYD_PLANNER_POSITION_BASED` position-mode braking-law jump with an online trapezoid velocity planner that ramps servo-pump hydraulic cylinder velocity, flow, and pump speed smoothly.

**Architecture:** Keep the planner inside `motion_planner.c` and reuse `HYD_MotionPlannerState.lastTargetVelocity` as the online planner memory. `HYD_PLANNER_POSITION_BASED` becomes online T planning plus a remaining-distance braking safety cap; `HYD_PLANNER_TIME_BASED` keeps the current elapsed-time ramp plus braking cap for compatibility. Offline `HYD_PlanTrapezoid()` and `HYD_EvalTrapezoid()` remain utility functions and are not wired into the runtime path.

**Tech Stack:** C99, CMake presets, existing `HYD_` style, static memory only, unit tests under `tests/`, no IEC surface changes.

---

## Scope Check

This plan implements one focused planner change from the approved spec:

- position-mode online trapezoid velocity generation
- planner unit-test updates
- one runtime integration test proving `plannedVelocity`, `plannedFlow`, and `PUMP_SPEED` ramp instead of step

This plan does not implement S-curve / jerk-limited planning, continuous target update, new buffer modes, pressure-limited position planning, or IEC POU layout changes.

## File Map

- Modify: `src/motion_planner.c`
  - Add small helper functions for max, resolved deceleration, online trapezoid decision, and braking safety cap.
  - Change `HYD_ComputePositionModeVelocityMagnitude()` so `HYD_PLANNER_POSITION_BASED` uses the online trapezoid planner.
  - Keep `HYD_PLANNER_TIME_BASED` behavior compatible.
- Modify: `tests/test_motion_planner.c`
  - Update legacy position-based tests that expected an immediate velocity jump.
  - Add direct unit coverage for acceleration, deceleration, braking cap, short triangular moves, retract direction, tolerance zero-output, and independent deceleration.
- Modify: `tests/test_moveabsolute_stop_integration.c`
  - Add one runtime position-mode test that starts a direct position segment and verifies `STATE.plannedVelocity`, `STATE.plannedFlow`, and `PUMP_SPEED` ramp on the first active cycles.

## Build And Test Baseline

- [ ] **Step 1: Check the worktree**

Run:

```bash
git status --short --branch
```

Expected: The branch may be ahead because the spec was committed. There should be no uncommitted source changes before implementation begins.

- [ ] **Step 2: Run current focused planner tests**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner
./out/build/unixgcc/test_motion_planner
```

Expected: Current `test_motion_planner` passes before changes.

- [ ] **Step 3: Run current full suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: All current tests pass before planner behavior changes.

## Task 1: Add Failing Online Planner Unit Tests

**Files:**
- Modify: `tests/test_motion_planner.c`

- [ ] **Step 1: Replace the old position-based extend test with a smooth-start test**

In `tests/test_motion_planner.c`, replace the full `test_position_based_extend_velocity()` function with:

```c
static void test_position_based_extend_velocity(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    printf("Testing position-based extend online trapezoid start...\n");
    memset(&state, 0, sizeof(state));
    axisRef = create_test_axis_ref(100.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.maxAcceleration = 5.0;
    segment.maxDeceleration = 5.0;
    segment.maxVelocity = 10.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.1;
    input.deltaTime = 0.1;
    input.rampedPressure = 50.0;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.direction == HYD_DIRECTION_EXTEND);
    assert(fabs(output.targetVelocity - 0.5) < 0.001);
    assert(fabs(output.targetFlow - 0.5 * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based extend online trapezoid start test passed\n");
}
```

- [ ] **Step 2: Replace the old position-based retract test with a smooth-start retract test**

In `tests/test_motion_planner.c`, replace the full `test_position_based_retract_velocity()` function with:

```c
static void test_position_based_retract_velocity(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    printf("Testing position-based retract online trapezoid start...\n");
    memset(&state, 0, sizeof(state));
    axisRef = create_test_axis_ref(120.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.targetPosition = 20.0;
    segment.maxAcceleration = 5.0;
    segment.maxDeceleration = 5.0;
    segment.maxVelocity = 10.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.1;
    input.deltaTime = 0.1;
    input.rampedPressure = 50.0;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.direction == HYD_DIRECTION_RETRACT);
    assert(fabs(output.targetVelocity + 0.5) < 0.001);
    assert(fabs(output.targetFlow - 0.5 * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based retract online trapezoid start test passed\n");
}
```

- [ ] **Step 3: Add online trapezoid helper tests before `main()`**

Add these functions immediately before `int main(void)`:

```c
static void test_position_based_online_trapezoid_acceleration_limit(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;

    printf("Testing position-based online trapezoid acceleration limit...\n");

    memset(&state, 0, sizeof(state));
    axisRef = create_test_axis_ref(0.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 20.0;
    segment.maxDeceleration = 20.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 0.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - 2.0) < 0.001);

    axisRef.timestamp = 0.2;
    input.elapsedTime = 0.2;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - 4.0) < 0.001);

    printf("✓ Position-based online trapezoid acceleration limit test passed\n");
}

static void test_position_based_online_trapezoid_deceleration_limit(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;

    printf("Testing position-based online trapezoid deceleration limit...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    axisRef = create_test_axis_ref(99.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 5.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 1.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.targetVelocity < 20.0);
    assert(fabs(output.targetVelocity - 19.5) < 0.001);
    printf("✓ Position-based online trapezoid deceleration limit test passed\n");
}

static void test_position_based_online_trapezoid_braking_cap(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_REAL expectedCap;

    printf("Testing position-based online trapezoid braking cap...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    axisRef = create_test_axis_ref(99.9);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 5.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 1.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    expectedCap = sqrt(2.0 * segment.maxDeceleration *
                       (segment.targetPosition - axisRef.position));
    assert(fabs(output.targetVelocity - expectedCap) < 0.001);
    printf("✓ Position-based online trapezoid braking cap test passed\n");
}

static void test_position_based_online_trapezoid_short_move_is_triangular(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_REAL safetyCap;

    printf("Testing position-based online trapezoid short triangular move...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 1.0;
    axisRef = create_test_axis_ref(9.95);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.positionTolerance = 0.0;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 0.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    safetyCap = sqrt(2.0 * segment.maxDeceleration *
                     (segment.targetPosition - axisRef.position));
    assert(output.targetVelocity >= 0.0);
    assert(output.targetVelocity <= safetyCap + 0.001);
    assert(output.targetVelocity < 1.0);
    printf("✓ Position-based online trapezoid short triangular move test passed\n");
}

static void test_position_based_online_trapezoid_position_tolerance_outputs_zero(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;

    printf("Testing position-based online trapezoid tolerance zero output...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 5.0;
    axisRef = create_test_axis_ref(99.95);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.1;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 0.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(fabs(output.targetVelocity) < 0.001);
    assert(fabs(output.targetFlow) < 0.001);
    printf("✓ Position-based online trapezoid tolerance zero output test passed\n");
}
```

- [ ] **Step 4: Call the new tests from `main()`**

In `tests/test_motion_planner.c`, add these calls after `test_position_planner_decelerates_with_max_deceleration();`:

```c
    test_position_based_online_trapezoid_acceleration_limit();
    test_position_based_online_trapezoid_deceleration_limit();
    test_position_based_online_trapezoid_braking_cap();
    test_position_based_online_trapezoid_short_move_is_triangular();
    test_position_based_online_trapezoid_position_tolerance_outputs_zero();
```

- [ ] **Step 5: Run the planner test and verify it fails**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner
./out/build/unixgcc/test_motion_planner
```

Expected: `test_position_based_extend_velocity` fails because the current `HYD_PLANNER_POSITION_BASED` path still jumps to the braking-law velocity instead of ramping to `0.5`.

## Task 2: Implement Online Trapezoid Velocity Planning

**Files:**
- Modify: `src/motion_planner.c`

- [ ] **Step 1: Add small real-number helpers**

In `src/motion_planner.c`, after `HYD_MinReal()`, add:

```c
static HYD_REAL HYD_MaxReal(HYD_REAL left, HYD_REAL right) {
    return (left > right) ? left : right;
}

static HYD_REAL HYD_ResolveBrakingAcceleration(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->maxDeceleration > 0.0)
        ? segment->maxDeceleration
        : segment->maxAcceleration;
}
```

- [ ] **Step 2: Add the online trapezoid helper**

In `src/motion_planner.c`, after `HYD_ComputeTimeBasedVelocityMagnitude()`, add:

```c
static HYD_REAL HYD_ComputeOnlineTrapezoidVelocityMagnitude(
    const HYD_MotionPlannerInput* input,
    HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;
    HYD_REAL positionTolerance;
    HYD_REAL previousMagnitude;
    HYD_REAL brakingAcceleration;
    HYD_REAL brakeDistance;
    HYD_REAL velocityMagnitude;
    HYD_REAL safetyVelocityMagnitude;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    brakingAcceleration = HYD_ResolveBrakingAcceleration(input->segment);
    if (input->segment->maxVelocity <= 0.0 ||
        input->segment->maxAcceleration <= 0.0 ||
        brakingAcceleration <= 0.0) {
        return 0.0;
    }

    remainingDistance = HYD_ComputeRemainingDistance(input->segment,
                                                     input->axisRef,
                                                     direction);
    positionTolerance = HYD_Segment_GetPositionTolerance(input->segment);
    if (remainingDistance <= positionTolerance) {
        return 0.0;
    }

    previousMagnitude = 0.0;
    if (input->state != NULL && input->state->initialized) {
        previousMagnitude = fabs(input->state->lastTargetVelocity);
    }

    if (input->deltaTime <= 0.0) {
        velocityMagnitude = previousMagnitude;
    } else {
        brakeDistance = (previousMagnitude * previousMagnitude) /
                        (2.0 * brakingAcceleration);
        if (remainingDistance <= brakeDistance) {
            velocityMagnitude = HYD_MaxReal(
                0.0,
                previousMagnitude - brakingAcceleration * input->deltaTime);
        } else {
            velocityMagnitude = HYD_MinReal(
                input->segment->maxVelocity,
                previousMagnitude + input->segment->maxAcceleration * input->deltaTime);
        }
    }

    safetyVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(
        remainingDistance,
        brakingAcceleration,
        input->segment->maxVelocity);
    return HYD_MinReal(velocityMagnitude, safetyVelocityMagnitude);
}
```

- [ ] **Step 3: Simplify repeated braking acceleration code**

In `src/motion_planner.c`, update both existing local braking-acceleration blocks:

```c
brakingAcceleration = (input->segment->maxDeceleration > 0.0)
    ? input->segment->maxDeceleration
    : input->segment->maxAcceleration;
```

Replace each block with:

```c
brakingAcceleration = HYD_ResolveBrakingAcceleration(input->segment);
```

Expected: This replacement appears in `HYD_ComputePositionModeVelocityMagnitude()` and `HYD_ComputeSpeedRampVelocityMagnitude()`.

- [ ] **Step 4: Route position-based planner through the online helper**

In `HYD_ComputePositionModeVelocityMagnitude()`, replace this block:

```c
remainingDistance = HYD_ComputeRemainingDistance(input->segment, input->axisRef, direction);
brakingAcceleration = HYD_ResolveBrakingAcceleration(input->segment);
brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                   brakingAcceleration,
                                                                   input->segment->maxVelocity);

if (input->segment->planner == HYD_PLANNER_POSITION_BASED) {
    return brakeVelocityMagnitude;
}
```

with:

```c
remainingDistance = HYD_ComputeRemainingDistance(input->segment, input->axisRef, direction);
brakingAcceleration = HYD_ResolveBrakingAcceleration(input->segment);
brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                   brakingAcceleration,
                                                                   input->segment->maxVelocity);

if (input->segment->planner == HYD_PLANNER_POSITION_BASED) {
    return HYD_ComputeOnlineTrapezoidVelocityMagnitude(input, direction);
}
```

- [ ] **Step 5: Ensure planner state initializes for all state-backed motion modes**

In `HYD_MotionPlanner_Execute()`, replace this condition:

```c
if (input->state != NULL &&
    (input->segment->planner == HYD_PLANNER_TIME_BASED ||
     input->segment->mode == HYD_MODE_SPEED_RAMP)) {
```

with:

```c
if (input->state != NULL) {
```

Then replace the inner rate-limit call block:

```c
velocityMagnitude = HYD_ApplyVelocityRateLimit(previousMagnitude,
                                              velocityMagnitude,
                                              input->segment->maxAcceleration,
                                              brakingAcceleration,
                                              input->deltaTime);
flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);
```

with:

```c
if (input->segment->planner == HYD_PLANNER_TIME_BASED ||
    input->segment->mode == HYD_MODE_SPEED_RAMP) {
    velocityMagnitude = HYD_ApplyVelocityRateLimit(previousMagnitude,
                                                  velocityMagnitude,
                                                  input->segment->maxAcceleration,
                                                  brakingAcceleration,
                                                  input->deltaTime);
    flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
    flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);
}
```

Expected: `HYD_PLANNER_POSITION_BASED` initializes and updates `state->lastTargetVelocity`, but it does not get double-rate-limited after the online helper already applied the online T step.

- [ ] **Step 6: Run focused planner tests**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner
./out/build/unixgcc/test_motion_planner
```

Expected: All `test_motion_planner` tests pass.

- [ ] **Step 7: Commit planner implementation**

Run:

```bash
git add src/motion_planner.c tests/test_motion_planner.c
git commit -m "feat: add online trapezoid position planner"
```

Expected: Commit succeeds.

## Task 3: Add Runtime Integration Coverage For Smooth Pump Commands

**Files:**
- Modify: `tests/test_moveabsolute_stop_integration.c`

- [ ] **Step 1: Add a helper test for position segment runtime ramping**

In `tests/test_moveabsolute_stop_integration.c`, add this function before `int main(void)`:

```c
static void test_position_segment_ramps_velocity_flow_and_pump_speed(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
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
    segment.maxAcceleration = 10.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 500.0;
    segment.positionTolerance = 0.01;
    segment.velocityToFlowGain = 2.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HYD_MotionControlFB_Scan(&fb);

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Scan(&fb);
    assert(fabs(fb.STATE.plannedVelocity - 1.0) < 0.001);
    assert(fabs(fb.STATE.plannedFlow - 2.0) < 0.001);
    assert(fabs(fb.PUMP_SPEED - 20.0) < 0.001);

    fb.AXIS_REF.position += fb.STATE.plannedVelocity * 0.1;
    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    fb.AXIS_REF.flow = fb.STATE.plannedFlow;
    fb.AXIS_REF.timestamp = 0.2;
    HYD_MotionControlFB_Scan(&fb);
    assert(fabs(fb.STATE.plannedVelocity - 2.0) < 0.001);
    assert(fabs(fb.STATE.plannedFlow - 4.0) < 0.001);
    assert(fabs(fb.PUMP_SPEED - 40.0) < 0.001);
}
```

- [ ] **Step 2: Call the integration test from `main()`**

In `tests/test_moveabsolute_stop_integration.c`, add this call near the other test calls in `main()`:

```c
    test_position_segment_ramps_velocity_flow_and_pump_speed();
```

- [ ] **Step 3: Run the integration test**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_stop_integration
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: The test passes and confirms position-mode `plannedVelocity`, `plannedFlow`, and `PUMP_SPEED` ramp smoothly.

- [ ] **Step 4: Commit integration coverage**

Run:

```bash
git add tests/test_moveabsolute_stop_integration.c
git commit -m "test: cover online position planner runtime ramp"
```

Expected: Commit succeeds.

## Task 4: Full Verification And Documentation Alignment

**Files:**
- Modify: `docs/superpowers/specs/2026-05-15-online-trapezoid-position-planner-design.md`

- [ ] **Step 1: Update the spec implementation status**

Append this section to `docs/superpowers/specs/2026-05-15-online-trapezoid-position-planner-design.md`:

```markdown

## Implementation Status

Implemented after this design:

- `HYD_PLANNER_POSITION_BASED` uses an online trapezoid velocity planner for `HYD_MODE_POSITION`.
- The offline `HYD_PlanTrapezoid()` / `HYD_EvalTrapezoid()` helpers remain available and tested as utility functions.
- Unit tests cover acceleration limiting, deceleration limiting, braking safety cap, short triangular moves, retract direction, position tolerance zero-output, and independent max deceleration.
- Runtime integration coverage verifies smooth `STATE.plannedVelocity`, `STATE.plannedFlow`, and `PUMP_SPEED` ramping.
```

- [ ] **Step 2: Run focused tests**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner test_moveabsolute_stop_integration
./out/build/unixgcc/test_motion_planner
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: Both focused tests pass.

- [ ] **Step 3: Run full build and full test suite**

Run:

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: Build succeeds and all tests pass.

- [ ] **Step 4: Run deployment smoke if previous deployment package support is still expected**

Run:

```bash
./scripts/deploy_embedded_prod.sh
printf '#include "common_types.h"\n#include "motion_control.h"\nint main(void){ HYD_MotionControlFB fb; HYD_MotionControlFB_Init(&fb); return 0; }\n' | gcc -std=c99 -I out/install/embedded_prod/include -x c -c - -o /tmp/hdy_deploy_motion_smoke.o
```

Expected: Deployment script succeeds and the installed headers compile.

- [ ] **Step 5: Commit spec status update**

Run:

```bash
git add docs/superpowers/specs/2026-05-15-online-trapezoid-position-planner-design.md
git commit -m "docs: mark online trapezoid planner implemented"
```

Expected: Commit succeeds.

- [ ] **Step 6: Final status check**

Run:

```bash
git status --short --branch
git log --oneline -5
```

Expected: Worktree is clean. Branch may be ahead of origin by the new implementation commits.

## Self-Review Checklist

- Spec coverage:
  - Online T behavior: Task 2
  - Offline helper preservation: Task 1 keeps existing offline tests; Task 4 documents status
  - No IEC changes: file map and tasks avoid IEC files
  - Smooth runtime pump commands: Task 3
  - Full verification: Task 4
- Placeholder scan:
  - This plan contains no unresolved markers, no open-ended implementation requests, and no unspecified test request.
- Type consistency:
  - Uses existing `HYD_MotionPlannerState`, `HYD_MotionPlannerInput`, `HYD_MotionPlannerOutput`, `HYD_MotionControlFB`, and `HYD_MotionSegment` fields.
  - Introduces only `static` helpers inside `src/motion_planner.c`, so there is no public API expansion.
