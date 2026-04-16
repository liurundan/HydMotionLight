#include <stdio.h>
#include <assert.h>
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
    segment.tolerance = 0.1;
    segment.duration = 5.0;
    return segment;
}

void test_position_completion(void) {
    printf("Testing segment completion by position...\n");
    HDY_MotionSegment segment = create_segment();
    segment.endCondition = HDY_END_POSITION;
    segment.targetPosition = 100.0;

    HDY_AxisRef axisRef = create_axis_ref(99.9, 0.0, 0.0);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.position = 90.0;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Position completion test passed\n");
}

void test_time_completion(void) {
    printf("Testing segment completion by time...\n");
    HDY_MotionSegment segment = create_segment();
    segment.endCondition = HDY_END_TIME;
    segment.duration = 2.0;

    HDY_AxisRef axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 1.9));
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 2.0));
    printf("✓ Time completion test passed\n");
}

void test_pressure_completion(void) {
    printf("Testing segment completion by pressure...\n");
    HDY_MotionSegment segment = create_segment();
    segment.endCondition = HDY_END_PRESSURE;
    segment.targetPressure = 50.0;

    HDY_AxisRef axisRef = create_axis_ref(0.0, 50.05, 0.0);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.pressure = 49.8;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Pressure completion test passed\n");
}

void test_flow_completion(void) {
    printf("Testing segment completion by flow...\n");
    HDY_MotionSegment segment = create_segment();
    segment.endCondition = HDY_END_FLOW;
    segment.targetFlow = 20.0;

    HDY_AxisRef axisRef = create_axis_ref(0.0, 0.0, 20.09);
    assert(HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));

    axisRef.flow = 19.8;
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 0.0));
    printf("✓ Flow completion test passed\n");
}

void test_manual_completion(void) {
    printf("Testing segment completion by manual end condition...\n");
    HDY_MotionSegment segment = create_segment();
    segment.endCondition = HDY_END_MANUAL;

    HDY_AxisRef axisRef = create_axis_ref(0.0, 0.0, 0.0);
    assert(!HDY_SegmentCompletion_Check(&segment, &axisRef, 100.0));
    printf("✓ Manual completion test passed\n");
}

int main(void) {
    printf("Running SegmentCompletion tests...\n\n");
    test_position_completion();
    test_time_completion();
    test_pressure_completion();
    test_flow_completion();
    test_manual_completion();
    printf("\n✅ All SegmentCompletion tests passed successfully!\n");
    return 0;
}
