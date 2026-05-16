#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "segment_completion.h"

static HYD_AxisRef create_axis_ref(HYD_REAL position, HYD_REAL pressure, HYD_REAL flow) {
    HYD_AxisRef axisRef = {0};
    axisRef.position = position;
    axisRef.velocity = 0.0;
    axisRef.flow = flow;
    axisRef.pressure = pressure;
    axisRef.timestamp = 0.0;
    return axisRef;
}

static HYD_MotionSegment create_segment(void) {
    HYD_MotionSegment segment = {0};
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxFlow = 10.0;
    segment.tolerance = 0.01;
    segment.positionTolerance = 0.05;
    segment.pressureTolerance = 0.2;
    segment.flowTolerance = 0.15;
    segment.velocityTolerance = 0.3;
    segment.timeoutLimit = 5.0;
    segment.duration = 5.0;
    return segment;
}

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

static void test_position_completion_extend(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by extend position...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;

    axisRef = create_axis_ref(99.96, 0.0, 0.0);
    assert(HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.position = 90.0;
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Extend position completion test passed\n");
}

static void test_position_completion_retract(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by retract position...\n");
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.targetPosition = 20.0;

    axisRef = create_axis_ref(20.04, 0.0, 0.0);
    assert(HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.position = 25.0;
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Retract position completion test passed\n");
}

static void test_time_completion(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by time...\n");
    segment.endCondition = HYD_END_TIME;
    segment.duration = 2.0;

    axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 1.9));
    assert(HYD_SegmentCompletion_Check(&segment, &axisRef, 2.0));
    printf("✓ Time completion test passed\n");
}

static void test_pressure_completion(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by pressure...\n");
    segment.endCondition = HYD_END_PRESSURE;
    segment.targetPressure = 50.0;

    axisRef = create_axis_ref(0.0, 50.18, 0.0);
    assert(HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.pressure = 49.7;
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Pressure completion test passed\n");
}

static void test_flow_completion(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by flow...\n");
    segment.endCondition = HYD_END_FLOW;
    segment.targetFlow = 20.0;

    axisRef = create_axis_ref(0.0, 0.0, 20.14);
    assert(HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.flow = 19.7;
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Flow completion test passed\n");
}

static void test_manual_completion(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;

    printf("Testing segment completion by manual end condition...\n");
    segment.endCondition = HYD_END_MANUAL;

    axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 100.0));
    printf("✓ Manual completion test passed\n");
}

static void test_runtime_reference_context_overrides_segment_targets(void) {
    HYD_MotionSegment segment = create_segment();
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references = {0};
    HYD_SegmentCompletionContext context;

    printf("Testing runtime reference context override behavior...\n");
    segment.endCondition = HYD_END_PRESSURE;
    segment.targetPressure = 50.0;
    axisRef = create_axis_ref(0.0, 45.1, 0.0);

    references.elapsedTime = 0.0;
    references.pressureReference = 45.0;
    references.flowReference = 0.0;
    references.velocityReference = 0.0;
    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = &references;

    assert(HYD_SegmentCompletion_CheckWithContext(&context));
    assert(!HYD_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    segment.endCondition = HYD_END_FLOW;
    segment.targetFlow = 20.0;
    axisRef.flow = 18.9;
    references.flowReference = 19.0;
    assert(HYD_SegmentCompletion_CheckWithContext(&context));
    printf("✓ Runtime reference context override test passed\n");
}

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

static void test_position_completion_requires_stable_window(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_SegmentCompletionContext context;
    HYD_TIME candidateStart = 0.0;
    HYD_BOOL candidateActive = false;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.positionTolerance = 0.1;
    segment.stableWindow = 0.2;
    segment.stableVelocityLimit = 0.5;

    memset(&axisRef, 0, sizeof(axisRef));
    axisRef.position = 9.95;
    axisRef.velocity = 0.1;
    axisRef.timestamp = 1.0;

    memset(&references, 0, sizeof(references));
    references.elapsedTime = 1.0;

    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = &references;
    context.timestamp = 1.0;
    context.candidateStartTime = &candidateStart;
    context.candidateActive = &candidateActive;

    assert(!HYD_SegmentCompletion_CheckWithContext(&context));
    assert(candidateActive);

    axisRef.timestamp = 1.1;
    context.timestamp = 1.1;
    assert(!HYD_SegmentCompletion_CheckWithContext(&context));

    axisRef.timestamp = 1.25;
    context.timestamp = 1.25;
    assert(HYD_SegmentCompletion_CheckWithContext(&context));
    printf("✓ Stable window completion test passed\n");
}

static void test_position_completion_resets_when_velocity_not_settled(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_SegmentCompletionContext context;
    HYD_TIME candidateStart = 0.0;
    HYD_BOOL candidateActive = false;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.positionTolerance = 0.1;
    segment.stableWindow = 0.2;
    segment.stableVelocityLimit = 0.5;

    memset(&axisRef, 0, sizeof(axisRef));
    axisRef.position = 9.95;
    axisRef.velocity = 2.0;
    axisRef.timestamp = 1.0;

    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = NULL;
    context.timestamp = 1.0;
    context.candidateStartTime = &candidateStart;
    context.candidateActive = &candidateActive;

    assert(!HYD_SegmentCompletion_CheckWithContext(&context));
    assert(!candidateActive);
    printf("✓ Stable velocity gate completion reset test passed\n");
}

int main(void) {
    printf("Running SegmentCompletion tests...\n\n");
    test_position_completion_extend();
    test_position_completion_retract();
    test_time_completion();
    test_pressure_completion();
    test_flow_completion();
    test_manual_completion();
    test_runtime_reference_context_overrides_segment_targets();
    test_position_completion_rejects_unsettled_planned_velocity();
    test_position_completion_rejects_unsettled_actual_velocity();
    test_position_completion_accepts_settled_planned_and_actual_velocity();
    test_position_completion_stable_velocity_limit_overrides_velocity_tolerance();
    test_position_completion_uses_default_velocity_tolerance_when_unconfigured();
    test_position_completion_stable_window_resets_on_unsettled_planned_velocity();
    test_pressure_completion_ignores_velocity_reference_gate();
    test_position_completion_requires_stable_window();
    test_position_completion_resets_when_velocity_not_settled();
    printf("\n✅ All SegmentCompletion tests passed successfully!\n");
    return 0;
}
