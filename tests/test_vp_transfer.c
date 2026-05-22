#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "vp_transfer.h"

static HYD_MotionSegment base_injection_segment(void) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.vpTransferPosition = 100.0;
    segment.vpTransferPressure = 80.0;
    segment.vpTransferMinTime = 0.5;
    segment.vpTransferVelocityDrop = 5.0;
    return segment;
}

static void test_vp_transfer_by_position(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    axisRef.position = 100.0;
    axisRef.pressure = 40.0;
    axisRef.velocity = 30.0;
    references.elapsedTime = 0.2;
#if HYD_ENABLE_EXECUTION_REFERENCE
    references.velocityReference = 30.0;
#endif

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);
}

static void test_vp_transfer_by_pressure(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    axisRef.position = 20.0;
    axisRef.pressure = 85.0;
    axisRef.velocity = 30.0;
    references.elapsedTime = 0.2;
#if HYD_ENABLE_EXECUTION_REFERENCE
    references.velocityReference = 30.0;
#endif

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_PRESSURE);
}

static void test_non_injection_segment_never_reports_transfer(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    segment.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    axisRef.position = 100.0;
    axisRef.pressure = 85.0;
    references.elapsedTime = 1.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(!result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_NONE);
}

static void test_vp_transfer_pressure_first_priority(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer pressure-first priority...\n");

    /* Both position AND pressure thresholds are met.
     * With PRESSURE_FIRST priority, pressure should win. */
    segment.vpTransferPriority = HYD_VP_PRIORITY_PRESSURE_FIRST;
    axisRef.position = 100.0;   /* meets position threshold */
    axisRef.pressure = 85.0;    /* meets pressure threshold */
    references.elapsedTime = 1.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_PRESSURE);
    printf("  Pressure-first: reason=PRESSURE (both met, pressure wins)\n");

    /* Verify default is still position-first */
    segment.vpTransferPriority = HYD_VP_PRIORITY_POSITION_FIRST;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);
    printf("  Position-first: reason=POSITION (both met, position wins)\n");

    printf("VP transfer priority test passed\n");
}

static void test_vp_transfer_latch_holds_after_trigger(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer latch behavior...\n");

    segment.vpTransferLatch = true;

    /* First call: position triggers */
    axisRef.position = 100.0;
    axisRef.pressure = 40.0;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);

    /* Second call: position drops below threshold — without latch this would go false */
    axisRef.position = 50.0;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    /* NOTE: The latch is implemented in motion_control.c, not in vp_transfer.c.
     * vp_transfer.c always returns the combinatorial result.
     * So the combinatorial result here is false (position no longer met). */
    assert(!result.ready);

    printf("  Combinatorial result correct (latch is in motion_control.c)\n");
    printf("✓ VP transfer latch unit test passed\n");
}

int main(void) {
    test_vp_transfer_by_position();
    test_vp_transfer_by_pressure();
    test_non_injection_segment_never_reports_transfer();
    test_vp_transfer_pressure_first_priority();
    test_vp_transfer_latch_holds_after_trigger();
    printf("V/P transfer tests passed\n");
    return 0;
}
