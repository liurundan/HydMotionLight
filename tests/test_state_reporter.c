/* tests/test_state_reporter.c
 *
 * Unit tests for state_reporter module — locks in the semantics of the
 * standalone setters and the higher-level state/diagnostic helpers that
 * motion_control orchestrates each cycle. These tests target the public
 * API as declared in include/state_reporter.h and intentionally avoid
 * driving a full Execute() cycle so behavior changes can be diagnosed
 * directly against the reporter contract.
 */
#include "motion_control.h"
#include "state_reporter.h"
#include "common_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_apply_safe_outputs_zeros_pump_speed_and_planned_outputs(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 1234.5;
    fb.STATE.commandedPumpSpeed = 1234.5;
    fb.STATE.plannedVelocity = 50.0;
    fb.STATE.plannedFlow = 30.0;
    fb.STATE.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    fb.STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb.STATE.active = true;
    fb._simFeedback.targetVelocity = 12.0;
    fb._simFeedback.targetFlow = 9.0;
    fb._simFeedback.targetPressure = 4.5;

    HYD_StateReporter_ApplySafeOutputs(&fb);

    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.commandedPumpSpeed == 0.0);
    assert(fb.STATE.plannedVelocity == 0.0);
    assert(fb.STATE.plannedFlow == 0.0);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_NONE);
    assert(fb.STATE.plannedDirection == HYD_DIRECTION_HOLD);
    assert(!fb.STATE.active);
    assert(fb._simFeedback.targetVelocity == 0.0);
    assert(fb._simFeedback.targetFlow == 0.0);
    assert(fb._simFeedback.targetPressure == 0.0);
    printf("test_apply_safe_outputs_zeros_pump_speed_and_planned_outputs PASSED\n");
}

static void test_set_fb_state_propagates_and_refreshes_error_id(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);

    HYD_StateReporter_SetFbState(&fb, HYD_FB_STATE_RUNNING);
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.ERROR_ID == HYD_DIAG_CODE_NONE);

    HYD_StateReporter_SetFbState(&fb, HYD_FB_STATE_FAULT);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    /* FB_STATE alone does not change diagnostic code, but the standard-output
     * refresh runs and computes ERROR_ID from the (still empty) diagnostic. */
    assert(fb.ERROR_ID == HYD_DIAG_CODE_NONE);
    printf("test_set_fb_state_propagates_and_refreshes_error_id PASSED\n");
}

static void test_report_fault_sets_diagnostic_state_and_error_id(void) {
    HYD_MotionControlFB fb;
    HYD_ExecutionReference refs;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    memset(&seg, 0, sizeof(seg));
    memset(&refs, 0, sizeof(refs));
    seg.segmentTag = 7;
    refs.elapsedTime = 0.25;

    /* TIMEOUT spec is severity=FAULT, protectionAction=STOP — confirms that
     * ReportFault routes through ProtectionManager (FAULT state + STOP action)
     * and then publishes the diagnostic. */
    HYD_StateReporter_ReportFault(&fb,
                                  HYD_DIAG_CODE_TIMEOUT,
                                  /*eventTimestamp*/ 1.5,
                                  &seg,
                                  &refs);

    assert(fb.STATE.faultActive);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.status == HYD_STATUS_FAULT);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_TIMEOUT);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(fb.ERROR_ID == HYD_DIAG_CODE_TIMEOUT);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    assert(fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.LAST_FAULT_SNAPSHOT.diagnostic.code == HYD_DIAG_CODE_TIMEOUT);
    /* Safe-output side effects from EnterFaultStop. */
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
    printf("test_report_fault_sets_diagnostic_state_and_error_id PASSED\n");
}

static void test_set_idle_state_with_finished_flag_resolves_done(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 100.0;
    fb.STATE.faultActive = false;

    HYD_StateReporter_SetIdleState(&fb, /*finished*/ true, /*segmentCompleted*/ true);

    assert(!fb.STATE.active);
    assert(fb.STATE.finished);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FB_STATE == HYD_FB_STATE_DONE);
    assert(fb.STATE.status == HYD_STATUS_FINISHED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_NONE);
    printf("test_set_idle_state_with_finished_flag_resolves_done PASSED\n");
}

static void test_set_idle_state_without_source_resolves_idle(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    /* No recipe loaded, no DIRECT_SEGMENT loaded → IDLE. */
    HYD_StateReporter_SetIdleState(&fb, /*finished*/ false, /*segmentCompleted*/ false);

    assert(!fb.STATE.active);
    assert(!fb.STATE.finished);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.FB_STATE == HYD_FB_STATE_IDLE);
    assert(fb.STATE.status == HYD_STATUS_IDLE);
    printf("test_set_idle_state_without_source_resolves_idle PASSED\n");
}

static void test_clear_current_diagnostic_resets_protection_action(void) {
    HYD_MotionControlFB fb;
    HYD_ExecutionReference refs;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    memset(&seg, 0, sizeof(seg));
    memset(&refs, 0, sizeof(refs));

    /* Stage a non-fault diagnostic. OVER_PRESSURE spec is severity=WARNING,
     * protectionAction=DERATE — exercises the non-fault branch where
     * EnterFaultStop is NOT invoked. */
    HYD_StateReporter_ReportDiagnostic(&fb,
                                       HYD_DIAG_CODE_OVER_PRESSURE,
                                       HYD_DIAG_SEVERITY_WARNING,
                                       /*eventTimestamp*/ 0.5,
                                       &seg,
                                       &refs);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_OVER_PRESSURE);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    assert(!fb.STATE.faultActive);

    HYD_StateReporter_ClearCurrentDiagnostic(&fb);

    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_NONE);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_NONE);
    /* History retention is preserved by ClearCurrentDiagnostic (only the live
     * DIAGNOSTIC field is cleared); ResetDiagnosticRetention clears history. */
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    printf("test_clear_current_diagnostic_resets_protection_action PASSED\n");
}

int main(void) {
    test_apply_safe_outputs_zeros_pump_speed_and_planned_outputs();
    test_set_fb_state_propagates_and_refreshes_error_id();
    test_report_fault_sets_diagnostic_state_and_error_id();
    test_set_idle_state_with_finished_flag_resolves_done();
    test_set_idle_state_without_source_resolves_idle();
    test_clear_current_diagnostic_resets_protection_action();
    return 0;
}
