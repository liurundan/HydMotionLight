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

int main(void) {
    test_vp_transfer_by_position();
    test_vp_transfer_by_pressure();
    test_non_injection_segment_never_reports_transfer();
    printf("V/P transfer tests passed\n");
    return 0;
}
