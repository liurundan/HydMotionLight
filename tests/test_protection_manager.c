/* tests/test_protection_manager.c
 *
 * Unit tests for the ProtectionManager — locks in the runtime-safety
 * helpers that motion_control invokes when transitioning between EN=false,
 * idle, fault-hold, and fault-stop states. These tests exercise the
 * helpers directly (no Execute() cycles) so that any regression in their
 * single-responsibility behavior surfaces immediately.
 */
#include "motion_control.h"
#include "protection_manager.h"
#include "common_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Builds a single-segment validated recipe so LoadRecipe() succeeds. */
static HYD_MotionSegment make_minimal_position_segment(HYD_UINT8 tag) {
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = tag;
    seg.mode = HYD_MODE_POSITION;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;
    return seg;
}

static void test_reset_runtime_actuation_clears_pressure_state(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb._lastFeedbackTimestamp = 1.234;
    fb._segmentStartTime = 5.0;
    fb._holdStateTime = 2.0;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;

    HYD_ProtectionManager_ResetRuntimeActuation(&fb);

    /* Negative sentinel signals "feedback not yet valid". */
    assert(fb._lastFeedbackTimestamp < 0.0);
    assert(fb._segmentStartTime == 0.0);
    assert(fb._holdStateTime == 0.0);
    assert(!fb._activeSegmentValid);
    assert(fb._activeSegmentSource == HYD_SEGMENT_SOURCE_NONE);
    printf("test_reset_runtime_actuation_clears_pressure_state PASSED\n");
}

static void test_apply_disabled_state_with_loaded_recipe_reports_ready(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    seg = make_minimal_position_segment(1);
    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    /* Pre-load runtime spoilage to confirm safe outputs are re-applied. */
    fb.PUMP_SPEED = 250.0;
    fb._lastCommandedFlow = 75.0;
    fb.STATE.active = true;

    HYD_ProtectionManager_ApplyDisabledState(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_DISABLED);
    assert(fb.STATE.status == HYD_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
    assert(!fb.STATE.active);
    assert(!fb.SEGMENT_COMPLETED);
    /* Non-fault path: protectionAction is cleared by ApplySafeOutputs. */
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_NONE);
    printf("test_apply_disabled_state_with_loaded_recipe_reports_ready PASSED\n");
}

static void test_apply_disabled_state_without_source_reports_idle(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    /* No recipe loaded and DIRECT_SEGMENT_VALID=false. */
    HYD_ProtectionManager_ApplyDisabledState(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_DISABLED);
    assert(fb.STATE.status == HYD_STATUS_IDLE);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.STATE.active);
    printf("test_apply_disabled_state_without_source_reports_idle PASSED\n");
}

static void test_apply_disabled_state_with_fault_preserves_fault_status(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    /* Simulate an already-latched fault entering the EN=false branch. */
    fb.STATE.faultActive = true;

    HYD_ProtectionManager_ApplyDisabledState(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_DISABLED);
    assert(fb.STATE.status == HYD_STATUS_FAULT);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_apply_disabled_state_with_fault_preserves_fault_status PASSED\n");
}

static void test_apply_fault_hold_re_applies_safe_outputs(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 999.0;
    fb._lastCommandedFlow = 50.0;
    fb._segmentStartTime = 4.0;
    fb._activeSegmentValid = true;

    HYD_ProtectionManager_ApplyFaultHold(&fb);

    assert(fb.PUMP_SPEED == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
    assert(fb._segmentStartTime == 0.0);
    assert(!fb._activeSegmentValid);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(fb.STATE.status == HYD_STATUS_FAULT);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    printf("test_apply_fault_hold_re_applies_safe_outputs PASSED\n");
}

static void test_enter_fault_stop_clears_pending_and_marks_fault(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 500.0;
    fb._directPendingValid = true;
    fb._lastCommandedFlow = 12.0;

    HYD_ProtectionManager_EnterFaultStop(&fb);

    assert(fb.PUMP_SPEED == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
    assert(!fb._directPendingValid);
    assert(fb.STATE.faultActive);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.status == HYD_STATUS_FAULT);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_enter_fault_stop_clears_pending_and_marks_fault PASSED\n");
}

static void test_apply_idle_state_resets_runtime_and_reports_idle(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb._segmentStartTime = 9.0;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_RECIPE;
    fb.PUMP_SPEED = 300.0;

    HYD_ProtectionManager_ApplyIdleState(&fb, /*finished*/ false, /*segmentCompleted*/ false);

    /* Runtime actuation cleared. */
    assert(!fb._activeSegmentValid);
    assert(fb._activeSegmentSource == HYD_SEGMENT_SOURCE_NONE);
    assert(fb._segmentStartTime == 0.0);
    /* StateReporter idle resolution with no source → IDLE. */
    assert(fb.FB_STATE == HYD_FB_STATE_IDLE);
    assert(fb.STATE.status == HYD_STATUS_IDLE);
    assert(fb.PUMP_SPEED == 0.0);
    printf("test_apply_idle_state_resets_runtime_and_reports_idle PASSED\n");
}

int main(void) {
    test_reset_runtime_actuation_clears_pressure_state();
    test_apply_disabled_state_with_loaded_recipe_reports_ready();
    test_apply_disabled_state_without_source_reports_idle();
    test_apply_disabled_state_with_fault_preserves_fault_status();
    test_apply_fault_hold_re_applies_safe_outputs();
    test_enter_fault_stop_clears_pending_and_marks_fault();
    test_apply_idle_state_resets_runtime_and_reports_idle();
    return 0;
}
