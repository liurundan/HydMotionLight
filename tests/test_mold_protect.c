/* tests/test_mold_protect.c - Sprint 1 low-pressure mold-protect tests
 *
 * Task 3 placeholder coverage:
 *   Prove that the pressureCeiling diagnostic channel fires under
 *   HYD_MODE_POSITION (the mode used by clamp-close segments). Validates
 *   the core Sprint 1 1.3 change: HYD_UpdateExecutionDiagnostics now
 *   evaluates the ceiling regardless of control mode. Full DERATE -> STOP
 *   escalation, position-window gating across multiple entries, and the
 *   pump-speed derate sequencing are covered by Task 6's end-to-end
 *   extensions.
 */
#include "motion_control.h"
#include "action_profile.h"
#include "segment_limits.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void prime_clamp_close_with_ceiling(HYD_MotionControlFB* fb,
                                           HYD_REAL ceiling,
                                           HYD_REAL ceilingTol,
                                           HYD_REAL windowStart,
                                           HYD_REAL windowEnd) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.2;

    assert(HYD_ActionProfile_BuildClampClose(&seg, &params, 1, 100.0));
    seg.pressureCeiling = ceiling;
    seg.pressureCeilingTolerance = ceilingTol;
    seg.pressureCeilingPositionStart = windowStart;
    seg.pressureCeilingPositionEnd = windowEnd;

    assert(HYD_MotionControlFB_LoadRecipe(fb, &seg, 1));
    fb->USE_RECIPE = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
}

/* Task 3 placeholder: prove POSITION-mode ceiling detection raises
 * PRESSURE_CEILING_EXCEEDED with DERATE. Full DERATE -> STOP escalation
 * lives in Task 6 test_mold_protect_stop_escalation. */
static void test_ceiling_exceeded_in_position_mode(void) {
    HYD_MotionControlFB fb;
    HYD_REAL t;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_clamp_close_with_ceiling(&fb, /*ceiling*/ 5.0, /*tol*/ 0.2,
                                   /*windowStart*/ 70.0, /*windowEnd*/ 100.0);

    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Step 1: outside the protect window. Even at high pressure, the ceiling
     * diagnostic must not fire because position < windowStart=70.
     * Run past the segment's switch-phase end (~0.8s) so we have a clean
     * non-suppressed baseline. */
    for (i = 0; i < 100; i++) {
        t = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.timestamp = t;
        fb.AXIS_REF.position = 30.0;          /* well below windowStart=70 */
        fb.AXIS_REF.pressure = 10.0;          /* way above ceiling */
        HYD_MotionControlFB_Execute(&fb);
        assert(!fb.DIAGNOSTIC.pressureCeilingExceeded);
        assert(!fb.DIAGNOSTIC.pressureCeilingViolated);
    }

    /* Step 2: enter window with pressure above ceiling+tol. The criteria
     * has debounceTime=0.05s (Task 2 calibration) and faultEscalationTime=0.30s.
     * We're starting at t=1.0 well past the switch phase, so the only gate
     * left is the 0.05s debounce. Run 25 steps at 0.01s = 0.25s — past
     * debounce but below the 0.30s fault-escalation threshold. Expect
     * WARNING/DERATE but NOT FAULT. */
    for (i = 0; i < 25; i++) {
        t = (HYD_TIME)(1.0 + i * 0.01);
        fb.AXIS_REF.timestamp = t;
        fb.AXIS_REF.position = 80.0;          /* inside [70, 100] */
        fb.AXIS_REF.pressure = 6.0;           /* > ceiling(5) + tol(0.2) */
        HYD_MotionControlFB_Execute(&fb);
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(!fb.DIAGNOSTIC.pressureCeilingViolated);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);

    printf("test_ceiling_exceeded_in_position_mode PASSED\n");
}

int main(void) {
    test_ceiling_exceeded_in_position_mode();
    return 0;
}
