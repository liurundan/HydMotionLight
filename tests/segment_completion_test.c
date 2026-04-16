#include <assert.h>
#include <stdio.h>
#include "segment_completion.h"

static HDY_AxisRef create_axis_ref(HDY_REAL position, HDY_REAL pressure, HDY_REAL flow) {
    HDY_AxisRef axisRef = {0};
    axisRef.position = position;
    axisRef.velocity = 0.0;
    axisRef.flow = flow;
    axisRef.pressure = pressure;
    axisRef.timestamp = 0.0;
    return axisRef;
}

static HDY_MotionSegment create_segment(void) {
    HDY_MotionSegment segment = {0};
    segment.direction = HDY_DIRECTION_EXTEND;
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

static void test_position_completion_extend(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by extend position...\n");
    segment.endCondition = HDY_END_POSITION;
    segment.direction = HDY_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;

    axisRef = create_axis_ref(99.96, 0.0, 0.0);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.position = 90.0;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Extend position completion test passed\n");
}

static void test_position_completion_retract(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by retract position...\n");
    segment.endCondition = HDY_END_POSITION;
    segment.direction = HDY_DIRECTION_RETRACT;
    segment.targetPosition = 20.0;

    axisRef = create_axis_ref(20.04, 0.0, 0.0);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.position = 25.0;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Retract position completion test passed\n");
}

static void test_time_completion(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by time...\n");
    segment.endCondition = HDY_END_TIME;
    segment.duration = 2.0;

    axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 1.9));
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 2.0));
    printf("✓ Time completion test passed\n");
}

static void test_pressure_completion(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by pressure...\n");
    segment.endCondition = HDY_END_PRESSURE;
    segment.targetPressure = 50.0;

    axisRef = create_axis_ref(0.0, 50.18, 0.0);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.pressure = 49.7;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Pressure completion test passed\n");
}

static void test_flow_completion(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by flow...\n");
    segment.endCondition = HDY_END_FLOW;
    segment.targetFlow = 20.0;

    axisRef = create_axis_ref(0.0, 0.0, 20.14);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.flow = 19.7;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Flow completion test passed\n");
}

static void test_manual_completion(void) {
    HDY_MotionSegment segment = create_segment();
    HDY_AxisRef axisRef;

    printf("Testing segment completion by manual end condition...\n");
    segment.endCondition = HDY_END_MANUAL;

    axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 100.0));
    printf("✓ Manual completion test passed\n");
}

int main(void) {
    printf("Running SegmentCompletion tests...\n\n");
    test_position_completion_extend();
    test_position_completion_retract();
    test_time_completion();
    test_pressure_completion();
    test_flow_completion();
    test_manual_completion();
    printf("\n✅ All SegmentCompletion tests passed successfully!\n");
    return 0;
}
