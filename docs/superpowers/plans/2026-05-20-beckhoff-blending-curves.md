# Beckhoff Blending Curves Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement distinct Beckhoff `BlendingLow`, `BlendingPrevious`, `BlendingNext`, and `BlendingHigh` curves for finite direct `MoveAbsolute -> MoveAbsolute` transitions.

**Architecture:** Keep IEC validation and direct command submission in `motion_interface.c`; keep pending-slot ownership and blend cutover in `motion_control.c`; keep the velocity curve math in `motion_planner.c`. The active front segment receives a planner blend context that changes its terminal velocity from zero to a selected through velocity; the pending segment then starts without clearing `_plannerState`.

**Tech Stack:** C99 runtime library, matiec-style IEC structs/macros, CMake, CTest, project C unit tests.

---

## File Structure

- Modify `include/motion_planner.h`: add `HYD_MotionBlendContext` and pass it through `HYD_MotionPlannerInput`.
- Modify `src/motion_planner.c`: generalize online position braking to a terminal velocity, preserving ordinary stop-at-target behavior when no blend is active.
- Modify `include/motion_control.h`: add `_directBlendContext` to `HYD_MotionControlFB`.
- Modify `src/motion_control.c`: create/clear direct blend context, detect eligible `MoveAbsolute -> MoveAbsolute`, pass blend input to planner, and cut over before stop-and-Done completion.
- Modify `tests/test_motion_planner.c`: cover nonzero terminal velocity and cap behavior in pure planner tests.
- Modify `tests/test_motion_interface_arbitration.c`: cover blend context selection, cutover continuity, and reversal fallback through the direct IEC surface.

## Task 1: Planner Blend Context Contract

**Files:**
- Modify: `include/motion_planner.h`
- Test: `tests/test_motion_planner.c`

- [ ] **Step 1: Add failing planner tests for blend terminal velocity**

Add these functions above `main()` in `tests/test_motion_planner.c`:

```c
static void test_position_based_blend_terminal_velocity_inside_tolerance(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_MotionBlendContext blend;

    printf("Testing position blend terminal velocity inside tolerance...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 8.0;

    axisRef = create_test_axis_ref(99.98);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.05;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 10.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 100.0;

    memset(&blend, 0, sizeof(blend));
    blend.active = true;
    blend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
    blend.blendVelocity = 5.0;
    blend.switchPosition = 100.0;
    blend.switchTolerance = 0.05;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 1.0;
    input.state = &state;
    input.blend = &blend;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.targetVelocity > 0.0);
    assert(fabs(output.targetVelocity - 7.0) < 0.001);
    assert(fabs(output.targetFlow - 7.0 * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position blend terminal velocity inside tolerance test passed\n");
}

static void test_position_based_blend_terminal_velocity_cap(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_MotionBlendContext blend;
    HYD_REAL expectedCap;

    printf("Testing position blend terminal velocity cap...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 20.0;

    axisRef = create_test_axis_ref(99.5);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.positionTolerance = 0.0;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 100.0;

    memset(&blend, 0, sizeof(blend));
    blend.active = true;
    blend.bufferMode = HYD_BUFFER_MODE_BLENDING_LOW;
    blend.blendVelocity = 5.0;
    blend.switchPosition = 100.0;
    blend.switchTolerance = 0.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.deltaTime = 0.1;
    input.elapsedTime = 1.0;
    input.state = &state;
    input.blend = &blend;

    HYD_MotionPlanner_Execute(&input, &output);

    expectedCap = sqrt((blend.blendVelocity * blend.blendVelocity) +
                       (2.0 * segment.maxDeceleration *
                        (blend.switchPosition - axisRef.position)));
    assert(fabs(output.targetVelocity - expectedCap) < 0.001);
    printf("✓ Position blend terminal velocity cap test passed\n");
}
```

Add these calls near the end of `main()` before the final success print:

```c
    test_position_based_blend_terminal_velocity_inside_tolerance();
    test_position_based_blend_terminal_velocity_cap();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build build -j2
./build/test_motion_planner
```

Expected before implementation: compile fails with an error like `unknown type name 'HYD_MotionBlendContext'` or `HYD_MotionPlannerInput has no member named 'blend'`.

- [ ] **Step 3: Add planner context types**

In `include/motion_planner.h`, insert this typedef after `HYD_MotionPlannerState`:

```c
typedef struct {
    HYD_BOOL active;
    HYD_BufferMode bufferMode;
    HYD_REAL blendVelocity;
    HYD_REAL switchPosition;
    HYD_REAL switchTolerance;
} HYD_MotionBlendContext;
```

Then add this field at the end of `HYD_MotionPlannerInput`:

```c
    const HYD_MotionBlendContext* blend;
```

The complete `HYD_MotionPlannerInput` should be:

```c
typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL deltaTime;
    HYD_REAL rampedPressure;
    HYD_REAL decelElapsed;
    HYD_REAL decelStartVel;
    HYD_MotionPlannerState* state;
    const HYD_MotionBlendContext* blend;
} HYD_MotionPlannerInput;
```

- [ ] **Step 4: Run test to verify the failure moves to behavior**

Run:

```bash
cmake --build build -j2
./build/test_motion_planner
```

Expected before planner implementation: build succeeds, and `test_position_based_blend_terminal_velocity_inside_tolerance` fails because the planner still decelerates toward zero inside position tolerance.

- [ ] **Step 5: Commit**

```bash
git add include/motion_planner.h tests/test_motion_planner.c
git commit -m "test: cover planner blend terminal velocity"
```

## Task 2: Planner Terminal-Velocity Curve

**Files:**
- Modify: `src/motion_planner.c`
- Test: `tests/test_motion_planner.c`

- [ ] **Step 1: Add blend helper functions**

In `src/motion_planner.c`, add these helpers after `HYD_ComputeRemainingDistance`:

```c
static HYD_BOOL HYD_IsBlendMode(HYD_BufferMode bufferMode) {
    return bufferMode >= HYD_BUFFER_MODE_BLENDING_LOW &&
           bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH;
}

static HYD_BOOL HYD_IsActiveBlendContext(const HYD_MotionPlannerInput* input,
                                         HYD_MotionDirection direction) {
    if (input == NULL || input->blend == NULL || input->segment == NULL ||
        input->axisRef == NULL) {
        return false;
    }
    if (!input->blend->active || !HYD_IsBlendMode(input->blend->bufferMode)) {
        return false;
    }
    if (input->segment->mode != HYD_MODE_POSITION ||
        input->segment->endCondition != HYD_END_POSITION) {
        return false;
    }
    return direction == HYD_DIRECTION_EXTEND ||
           direction == HYD_DIRECTION_RETRACT;
}

static HYD_REAL HYD_ComputeRemainingDistanceToPosition(HYD_REAL targetPosition,
                                                       const HYD_AxisRef* axisRef,
                                                       HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;

    if (axisRef == NULL) {
        return 0.0;
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            remainingDistance = targetPosition - axisRef->position;
            break;
        case HYD_DIRECTION_RETRACT:
            remainingDistance = axisRef->position - targetPosition;
            break;
        default:
            remainingDistance = 0.0;
            break;
    }

    return (remainingDistance > 0.0) ? remainingDistance : 0.0;
}

static HYD_REAL HYD_ApplyTerminalVelocityRateLimit(HYD_REAL previousMagnitude,
                                                   HYD_REAL terminalMagnitude,
                                                   HYD_REAL acceleration,
                                                   HYD_REAL deceleration,
                                                   HYD_REAL deltaTime) {
    if (deltaTime <= 0.0) {
        return previousMagnitude;
    }
    return HYD_ApplyVelocityRateLimit(previousMagnitude,
                                      terminalMagnitude,
                                      acceleration,
                                      deceleration,
                                      deltaTime);
}
```

- [ ] **Step 2: Replace online trapezoid velocity computation**

Replace the full `HYD_ComputeOnlineTrapezoidVelocityMagnitude` function in `src/motion_planner.c` with:

```c
static HYD_REAL HYD_ComputeOnlineTrapezoidVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                            HYD_MotionDirection direction) {
    const HYD_MotionSegment* segment;
    HYD_REAL brakingAcceleration;
    HYD_REAL remainingDistance;
    HYD_REAL positionTolerance;
    HYD_REAL previousMagnitude;
    HYD_REAL velocityMagnitude;
    HYD_REAL safetyVelocityMagnitude;
    HYD_REAL brakeDistance;
    HYD_REAL brakeDecisionTolerance;
    HYD_REAL terminalVelocity;
    HYD_BOOL blendActive;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    segment = input->segment;
    brakingAcceleration = HYD_ResolveBrakingAcceleration(segment);
    if (segment->maxVelocity <= 0.0 ||
        segment->maxAcceleration <= 0.0 ||
        brakingAcceleration <= 0.0) {
        return 0.0;
    }

    blendActive = HYD_IsActiveBlendContext(input, direction);
    terminalVelocity = 0.0;
    remainingDistance = HYD_ComputeRemainingDistance(segment, input->axisRef, direction);
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);

    if (blendActive) {
        terminalVelocity = HYD_ClampReal(input->blend->blendVelocity,
                                         0.0,
                                         segment->maxVelocity);
        remainingDistance = HYD_ComputeRemainingDistanceToPosition(input->blend->switchPosition,
                                                                   input->axisRef,
                                                                   direction);
        if (input->blend->switchTolerance > 0.0) {
            positionTolerance = input->blend->switchTolerance;
        }
    }

    previousMagnitude = 0.0;
    if (input->state != NULL && input->state->initialized) {
        previousMagnitude = fabs(input->state->lastTargetVelocity);
    }

    if (remainingDistance <= positionTolerance) {
        return HYD_ApplyTerminalVelocityRateLimit(previousMagnitude,
                                                 terminalVelocity,
                                                 segment->maxAcceleration,
                                                 brakingAcceleration,
                                                 input->deltaTime);
    }

    if (input->deltaTime <= 0.0) {
        velocityMagnitude = previousMagnitude;
    } else {
        if (previousMagnitude <= terminalVelocity) {
            brakeDistance = 0.0;
        } else {
            brakeDistance = ((previousMagnitude * previousMagnitude) -
                             (terminalVelocity * terminalVelocity)) /
                (2.0 * brakingAcceleration);
        }
        brakeDecisionTolerance = HYD_CompareTolerance(HYD_MaxReal(remainingDistance,
                                                                  brakeDistance));
        if (remainingDistance <= brakeDistance + brakeDecisionTolerance) {
            velocityMagnitude = HYD_ApplyTerminalVelocityRateLimit(previousMagnitude,
                                                                  terminalVelocity,
                                                                  segment->maxAcceleration,
                                                                  brakingAcceleration,
                                                                  input->deltaTime);
        } else {
            velocityMagnitude = HYD_MinReal(segment->maxVelocity,
                                            previousMagnitude +
                                            segment->maxAcceleration * input->deltaTime);
        }
    }

    safetyVelocityMagnitude = sqrt((terminalVelocity * terminalVelocity) +
                                   (2.0 * brakingAcceleration * remainingDistance));
    safetyVelocityMagnitude = HYD_ClampReal(safetyVelocityMagnitude,
                                            terminalVelocity,
                                            segment->maxVelocity);
    return HYD_MinReal(velocityMagnitude, safetyVelocityMagnitude);
}
```

- [ ] **Step 3: Run planner tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_planner
```

Expected after implementation: all `MotionPlanner` tests pass, including the two new blend tests. Existing non-blended tests must still pass because `terminalVelocity` defaults to zero.

- [ ] **Step 4: Commit**

```bash
git add src/motion_planner.c tests/test_motion_planner.c include/motion_planner.h
git commit -m "feat: add planner terminal velocity blending"
```

## Task 3: Core Blend Context Selection

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for four blend velocities**

Add this helper above the new tests in `tests/test_motion_interface_arbitration.c`:

```c
static HYD_MotionControlFB* start_blend_pair(HYD_BufferMode mode,
                                             HYD_REAL firstVelocity,
                                             HYD_REAL secondVelocity) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = firstVelocity;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = secondVelocity;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = mode;
    __mcl_cmd_MoveAbsolute(&second);

    return fb;
}
```

Add this test above `main()`:

```c
static void test_blending_modes_select_distinct_through_velocities(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_LOW, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingLow should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 8.0f) < 0.001f,
               "BlendingLow should use the lower velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_PREVIOUS, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingPrevious should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "BlendingPrevious should use the previous velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingNext should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 8.0f) < 0.001f,
               "BlendingNext should use the next velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_HIGH, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingHigh should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "BlendingHigh should use the higher velocity");
}
```

Add this call in `main()`:

```c
    test_blending_modes_select_distinct_through_velocities();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected before implementation: compile fails with an error like `HYD_MotionControlFB has no member named '_directBlendContext'`.

- [ ] **Step 3: Add internal blend context state**

In `include/motion_control.h`, add this field immediately after `_directPendingBufferMode` in `HYD_MotionControlFB`:

```c
    HYD_MotionBlendContext _directBlendContext;
```

- [ ] **Step 4: Add direct blend helpers**

In `src/motion_control.c`, add these helper functions before `HYD_ClearDirectPendingSlot`:

```c
static HYD_REAL HYD_ResolveSegmentBrakingAccelerationForBlend(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->maxDeceleration > 0.0)
        ? segment->maxDeceleration
        : segment->maxAcceleration;
}

static void HYD_ClearDirectBlendContext(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    memset(&fb->_directBlendContext, 0, sizeof(fb->_directBlendContext));
    fb->_directBlendContext.bufferMode = HYD_BUFFER_MODE_ABORT;
}

static HYD_BOOL HYD_IsDirectBlendMode(HYD_BufferMode bufferMode) {
    return bufferMode >= HYD_BUFFER_MODE_BLENDING_LOW &&
           bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH;
}

static HYD_BOOL HYD_IsFinitePositionSegment(const HYD_MotionSegment* segment) {
    return segment != NULL &&
           segment->mode == HYD_MODE_POSITION &&
           segment->endCondition == HYD_END_POSITION &&
           segment->maxVelocity > 0.0 &&
           segment->maxAcceleration > 0.0 &&
           HYD_ResolveSegmentBrakingAccelerationForBlend(segment) > 0.0;
}

static HYD_BOOL HYD_AreBlendDirectionsCompatible(const HYD_MotionControlFB* fb,
                                                 const HYD_MotionSegment* activeSegment,
                                                 const HYD_MotionSegment* pendingSegment) {
    HYD_MotionDirection activeDirection;
    HYD_MotionDirection pendingDirection;

    if (fb == NULL || activeSegment == NULL || pendingSegment == NULL) {
        return false;
    }

    activeDirection = HYD_Segment_ResolveDirection(activeSegment, &fb->AXIS_REF);
    pendingDirection = HYD_Segment_ResolveDirection(pendingSegment, &fb->AXIS_REF);

    return (activeDirection == HYD_DIRECTION_EXTEND ||
            activeDirection == HYD_DIRECTION_RETRACT) &&
           activeDirection == pendingDirection;
}

static HYD_REAL HYD_SelectDirectBlendVelocity(HYD_BufferMode bufferMode,
                                              const HYD_MotionSegment* activeSegment,
                                              const HYD_MotionSegment* pendingSegment) {
    HYD_REAL previousVelocity;
    HYD_REAL nextVelocity;

    if (activeSegment == NULL || pendingSegment == NULL) {
        return 0.0;
    }

    previousVelocity = activeSegment->maxVelocity;
    nextVelocity = pendingSegment->maxVelocity;

    switch (bufferMode) {
        case HYD_BUFFER_MODE_BLENDING_LOW:
            return HYD_MotionUtils_MinReal(previousVelocity, nextVelocity);
        case HYD_BUFFER_MODE_BLENDING_PREVIOUS:
            return previousVelocity;
        case HYD_BUFFER_MODE_BLENDING_NEXT:
            return nextVelocity;
        case HYD_BUFFER_MODE_BLENDING_HIGH:
            return (previousVelocity > nextVelocity) ? previousVelocity : nextVelocity;
        default:
            return 0.0;
    }
}

static HYD_BOOL HYD_TryCreateDirectBlendContext(HYD_MotionControlFB* fb,
                                                HYD_BufferMode bufferMode,
                                                const HYD_MotionSegment* pendingSegment) {
    HYD_REAL selectedVelocity;
    HYD_REAL tolerance;

    if (fb == NULL || pendingSegment == NULL) {
        return false;
    }

    HYD_ClearDirectBlendContext(fb);

    if (!HYD_IsDirectBlendMode(bufferMode) ||
        !fb->_activeSegmentValid ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != HYD_DIRECT_CMD_MOVE_ABSOLUTE ||
        HYD_InferDirectCommandKindFromSegment(pendingSegment) != HYD_DIRECT_CMD_MOVE_ABSOLUTE ||
        !HYD_IsFinitePositionSegment(&fb->_activeSegment) ||
        !HYD_IsFinitePositionSegment(pendingSegment) ||
        !HYD_AreBlendDirectionsCompatible(fb, &fb->_activeSegment, pendingSegment)) {
        return false;
    }

    selectedVelocity = HYD_SelectDirectBlendVelocity(bufferMode,
                                                    &fb->_activeSegment,
                                                    pendingSegment);
    if (selectedVelocity <= 0.0) {
        return false;
    }

    tolerance = HYD_Segment_GetPositionTolerance(&fb->_activeSegment);

    fb->_directBlendContext.active = true;
    fb->_directBlendContext.bufferMode = bufferMode;
    fb->_directBlendContext.blendVelocity = selectedVelocity;
    fb->_directBlendContext.switchPosition = fb->_activeSegment.targetPosition;
    fb->_directBlendContext.switchTolerance = tolerance;
    return true;
}
```

- [ ] **Step 5: Clear blend context wherever pending direct state is cleared**

Update `HYD_ClearDirectPendingSlot` in `src/motion_control.c` to also clear the blend context:

```c
static void HYD_ClearDirectPendingSlot(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_directPendingValid = false;
    memset(&fb->_directPendingSegment, 0, sizeof(fb->_directPendingSegment));
    fb->_directPendingKind = HYD_DIRECT_CMD_NONE;
    fb->_directPendingBufferMode = HYD_BUFFER_MODE_ABORT;
    HYD_ClearDirectBlendContext(fb);
}
```

- [ ] **Step 6: Create blend context when a pending direct command is accepted**

In `HYD_MotionControlFB_StartDirectCommand`, replace the pending assignment block with this block:

```c
        fb->_directPendingSegment = *segment;
        fb->_directPendingKind = HYD_InferDirectCommandKindFromSegment(segment);
        fb->_directPendingBufferMode = bufferMode;
        fb->_directPendingValid = true;
        (void)HYD_TryCreateDirectBlendContext(fb, bufferMode, segment);
        return true;
```

- [ ] **Step 7: Run arbitration tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected after implementation: `test_blending_modes_select_distinct_through_velocities` passes. Existing arbitration tests must still pass.

- [ ] **Step 8: Commit**

```bash
git add include/motion_control.h src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "feat: select direct blend through velocity"
```

## Task 4: Pass Blend Context Into Planner

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing runtime test for blended front-segment planner output**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_blended_front_segment_keeps_nonzero_velocity_near_switch(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for blend output test");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before near-switch cycle");

    fb->AXIS_REF.position = 99.0f;
    fb->AXIS_REF.velocity = 8.0f;
    fb->AXIS_REF.timestamp += 0.1f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 8.0f;

    __HydMotion_framework_Publish();

    ASSERT_TRUE(fb->STATE.references.velocityReference > 0.1f,
               "Blended front segment should not plan zero velocity near switch");
}
```

Add this call in `main()`:

```c
    test_blended_front_segment_keeps_nonzero_velocity_near_switch();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected before implementation: test fails because `HYD_ExecuteActiveSegmentControl` does not set `plannerInput.blend`, so the planner still treats the front segment as a stop-at-target move.

- [ ] **Step 3: Set planner blend input**

In `HYD_ExecuteActiveSegmentControl`, after `plannerInput.state = &fb->_plannerState;`, add:

```c
        plannerInput.blend = fb->_directBlendContext.active
            ? &fb->_directBlendContext
            : NULL;
```

The planner input setup block should end like this:

```c
        plannerInput.state = &fb->_plannerState;
        plannerInput.blend = fb->_directBlendContext.active
            ? &fb->_directBlendContext
            : NULL;
        HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);
```

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
./build/test_motion_planner
```

Expected after implementation: both tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "feat: pass direct blend context to planner"
```

## Task 5: Blended Cutover Without Planner Reset

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Add failing cutover continuity test**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_blended_cutover_preserves_planner_state(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for cutover test");
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending direct command should be present before cutover");

    fb->AXIS_REF.position = 100.0f;
    fb->AXIS_REF.velocity = 8.0f;
    fb->AXIS_REF.timestamp += 0.1f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 8.0f;

    __HydMotion_framework_Publish();

    ASSERT_TRUE(!fb->_directPendingValid,
               "Pending direct command should be consumed by blended cutover");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should be cleared after blended cutover");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 200.0f) < 0.001f,
               "Pending MoveAbsolute should become active segment after cutover");
    ASSERT_TRUE(fb->_plannerState.initialized,
               "Planner state should remain initialized across blended cutover");
    ASSERT_TRUE(fabs(fb->_plannerState.lastTargetVelocity) > 0.1f,
               "Planner velocity should remain nonzero across blended cutover");
}
```

Add this call in `main()`:

```c
    test_blended_cutover_preserves_planner_state();
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected before implementation: pending command is not consumed before normal completion, or `_plannerState` is reset by `HYD_BeginSegment`.

- [ ] **Step 3: Add cutover helper**

In `src/motion_control.c`, add these helpers after `HYD_TryCreateDirectBlendContext`:

```c
static HYD_BOOL HYD_ShouldCutoverDirectBlend(const HYD_MotionControlFB* fb,
                                             const HYD_MotionSegment* segment) {
    HYD_MotionDirection direction;
    HYD_REAL tolerance;

    if (fb == NULL || segment == NULL ||
        !fb->_directBlendContext.active ||
        !fb->_directPendingValid ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
        return false;
    }

    direction = HYD_Segment_ResolveDirection(segment, &fb->AXIS_REF);
    tolerance = fb->_directBlendContext.switchTolerance;
    if (tolerance <= 0.0) {
        tolerance = HYD_Segment_GetPositionTolerance(segment);
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            return fb->AXIS_REF.position >=
                fb->_directBlendContext.switchPosition - tolerance;
        case HYD_DIRECTION_RETRACT:
            return fb->AXIS_REF.position <=
                fb->_directBlendContext.switchPosition + tolerance;
        default:
            return false;
    }
}
```

- [ ] **Step 4: Preserve planner state in pending direct startup**

Replace `HYD_StartPendingDirectSlot` with this version:

```c
static HYD_BOOL HYD_StartPendingDirectSlot(HYD_MotionControlFB* fb,
                                           HYD_TIME timestamp,
                                           HYD_BOOL preservePlannerState) {
    HYD_MotionSegment segment;
    HYD_BOOL savedUseRecipe;
    HYD_MotionPlannerState preservedPlannerState;

    if (fb == NULL || !fb->_directPendingValid) {
        return false;
    }

    segment = fb->_directPendingSegment;
    preservedPlannerState = fb->_plannerState;
    HYD_ClearDirectPendingSlot(fb);

    savedUseRecipe = fb->USE_RECIPE;
    fb->DIRECT_SEGMENT = segment;
    fb->DIRECT_SEGMENT_VALID = true;
    fb->USE_RECIPE = false;
    if (!HYD_BeginSegment(fb, 0U, timestamp)) {
        fb->USE_RECIPE = savedUseRecipe;
        return false;
    }
    if (preservePlannerState) {
        fb->_plannerState = preservedPlannerState;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return true;
}
```

Update the two existing non-blend calls:

```c
            (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
```

- [ ] **Step 5: Cut over before normal completion handling**

In `HYD_MotionControlFB_RunRunningState`, after the protection-action stop block and before the `_isDecelerating` completion block, insert:

```c
    if (HYD_ShouldCutoverDirectBlend(fb, segment)) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, true);
        return;
    }
```

The surrounding order should be:

```c
    if (fb->DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP) {
        fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
        HYD_ProtectionManager_EnterFaultStop(fb);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
        return;
    }

    if (HYD_ShouldCutoverDirectBlend(fb, segment)) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, true);
        return;
    }

    if (fb->_isDecelerating && fabs(plannerOutput.targetVelocity) < 0.001) {
```

- [ ] **Step 6: Run cutover tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected after implementation: the new cutover test passes, and the existing buffered direct lifecycle tests still pass.

- [ ] **Step 7: Commit**

```bash
git add src/motion_control.c tests/test_motion_interface_arbitration.c
git commit -m "feat: cut over blended direct moves"
```

## Task 6: Fallback Semantics And Full Verification

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`
- Verify: all changed files

- [ ] **Step 1: Add reversal fallback test**

Add this test above `main()` in `tests/test_motion_interface_arbitration.c`:

```c
static void test_reverse_moveabsolute_does_not_create_blend_context(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for reverse fallback test");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    fb->AXIS_REF.position = 10.0f;

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 0.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 2;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(fb->_directPendingValid,
               "Reverse MoveAbsolute should still be accepted into pending slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Reverse MoveAbsolute should not create a nonzero blend context");
}
```

Add this call in `main()`:

```c
    test_reverse_moveabsolute_does_not_create_blend_context();
```

- [ ] **Step 2: Run targeted tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_planner
./build/test_motion_interface_arbitration
./build/test_motion_interface_unit
```

Expected: all three targeted binaries pass.

- [ ] **Step 3: Run layout consistency**

Run:

```bash
python3 tests/test_interface_layout_consistency.py
```

Expected: script passes. This plan does not change IEC POU layout, so any failure indicates an unrelated drift that must be investigated before continuing.

- [ ] **Step 4: Run full CTest**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected:

```text
100% tests passed
```

- [ ] **Step 5: Commit verification test**

```bash
git add tests/test_motion_interface_arbitration.c
git commit -m "test: cover direct blend fallback behavior"
```

If `tests/test_motion_interface_arbitration.c` was already committed in Task 5 with the fallback test included, run:

```bash
git status --short
```

Expected:

```text

```

Then skip this commit because there is no uncommitted change.

## Self-Review

- Spec coverage: Tasks 1-2 implement planner terminal velocity; Tasks 3-5 implement blend mode selection, context passing, and no-reset cutover; Task 6 covers fallback and full verification.
- Unsupported scope remains excluded: no multi-command look-ahead, no jerk-limited planning, no IEC surface changes, no real blend for endless `MoveVelocity` or no-duration `PressureHandle`.
- Type consistency: the plan uses the current repository signatures: `HYD_MotionControlFB_StartDirectCommand(fb, segment, bufferMode, timestamp)`, `HYD_StartPendingDirectSlot` is a private helper, and `HYD_MotionPlannerInput` gains only `blend`.
- Verification coverage: targeted planner, arbitration, interface unit, layout consistency, and full CTest are all required before completion.
