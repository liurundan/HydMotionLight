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
#include <math.h>
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

/* Drive one execution cycle with the given timestamp/position/pressure.
 * Mirrors the velocity-derived flow that a real plant would report
 * (matches velocityToFlowGain = 0.25 used in the tests below). */
static void tick(HYD_MotionControlFB* fb, HYD_REAL t, HYD_REAL position, HYD_REAL pressure) {
    fb->AXIS_REF.timestamp = t;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.pressure = pressure;
    fb->AXIS_REF.flow = (HYD_REAL)(fabs((double)fb->AXIS_REF.velocity) * 0.25);
    HYD_MotionControlFB_Execute(fb);
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

/* Regression for the one-cycle-pulse bug originally in commit bcd19ef.
 *
 * HYD_DiagnosticCriteria_CheckFaultEscalation returns true only on the
 * transition cycle where state->faultEscalated first flips false -> true.
 * On subsequent cycles the function early-exits returning false even
 * though state->faultEscalated remains true and pressure is still above
 * ceiling. If pressureCeilingViolated is gated on the return value alone
 * the BOOL pulses for exactly one cycle and the priority chain downgrades
 * the protection action from STOP back to DERATE the cycle after escalation.
 *
 * This test asserts that once pressureCeilingViolated becomes true during
 * a sustained breach it stays true until pressure drops below ceiling. */
static void test_ceiling_violated_remains_sticky_across_cycles(void) {
    HYD_MotionControlFB fb;
    HYD_BOOL sawViolatedAtLeastOnce = false;
    HYD_BOOL violatedStuckTrue = true;  /* Tracks whether BOOL stays true after first observed FAULT */
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_clamp_close_with_ceiling(&fb, /*ceiling*/ 5.0, /*tol*/ 0.2,
                                   /*windowStart*/ 70.0, /*windowEnd*/ 100.0);

    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Phase 1: pre-trigger ramp (run beyond switch-suppress, well below ceiling) */
    for (i = 0; i < 100; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.position = 80.0;
        fb.AXIS_REF.pressure = 1.0;     /* below ceiling — no trigger */
        HYD_MotionControlFB_Execute(&fb);
    }

    /* Phase 2: sustained breach. Run long enough past faultEscalationTime
     * (0.30s) to comfortably traverse the transition cycle and observe the
     * latched behavior on subsequent cycles. */
    for (i = 0; i < 100; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(1.0 + i * 0.01);
        fb.AXIS_REF.position = 80.0;
        fb.AXIS_REF.pressure = 6.0;     /* above ceiling+tol */
        HYD_MotionControlFB_Execute(&fb);

        if (fb.DIAGNOSTIC.pressureCeilingViolated) {
            sawViolatedAtLeastOnce = true;
        } else if (sawViolatedAtLeastOnce) {
            /* Once we've seen VIOLATED true, it must remain true while
             * pressure is still above ceiling. Falling back to false here
             * is the original bug. */
            violatedStuckTrue = false;
        }
    }

    assert(sawViolatedAtLeastOnce);        /* escalation actually happened */
    assert(violatedStuckTrue);             /* BOOL was latched, not pulsed */
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP);

    /* Phase 3: confirm the latch clears via operator RESET acknowledgment.
     * Note: once the FB transitions to HYD_FB_STATE_FAULT the diagnostic
     * pipeline (HYD_UpdateExecutionDiagnostics) is bypassed by the
     * fault-hold branch in HYD_MotionControlFB_PublishOutputs, so the
     * latch will not clear from a mere pressure drop while still in FAULT.
     * This matches the safety contract: a fault requires explicit
     * acknowledgment. The latch's underlying state (ceilingState->
     * faultEscalated) is cleared by SoftReset, so after a RESET cycle the
     * BOOL is guaranteed false. */
    fb.RESET = true;
    fb.AXIS_REF.timestamp = 2.5;
    HYD_MotionControlFB_Execute(&fb);
    fb.RESET = false;
    assert(!fb.DIAGNOSTIC.pressureCeilingViolated);
    assert(!fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(fb.DIAGNOSTIC.severity != HYD_DIAG_SEVERITY_FAULT);

    printf("test_ceiling_violated_remains_sticky_across_cycles PASSED\n");
}

/* Task 6 end-to-end DERATE test:
 *
 * Drive the clamp axis through the protect window with a sustained pressure
 * breach. Verify that PUMP_SPEED collapses by the configured derateRatio
 * relative to the un-derated baseline measured outside the window. */
static void test_mold_protect_derate_reduces_pump_speed(void) {
    HYD_MotionControlFB fb;
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    HYD_REAL normalPumpSpeed;
    HYD_REAL deratedPumpSpeed;
    int i;

    HYD_MotionControlFB_Init(&fb);

    /* derateRatio = 0.2 to make the post-DERATE drop unambiguous. */
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, /*targetPosition*/ 100.0,
        /*windowStart*/ 70.0, /*ceiling*/ 5.0, /*ceilingTol*/ (HYD_REAL)0.2,
        /*derate*/ (HYD_REAL)0.2));
    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;  /* 1 L/min ~ 100 rpm */
    fb.PUMP_SPEED_LIMIT = 10000.0;       /* generous to avoid cap (Hazard C) */

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Phase 1: well below protect window — normal pump speed. Drive past
     * the ~0.8 s switch-suppress window before sampling. */
    normalPumpSpeed = 0.0;
    for (i = 0; i < 100; i++) {
        tick(&fb, (HYD_REAL)(0.01 * (i + 1)), (HYD_REAL)(30.0 + i * 0.1), 1.0);
    }
    for (i = 100; i < 130; i++) {
        tick(&fb, (HYD_REAL)(0.01 * (i + 1)), (HYD_REAL)(30.0 + i * 0.1), 1.0);
        normalPumpSpeed = fb.PUMP_SPEED;
    }
    assert(normalPumpSpeed > 0.0);
    assert(!fb.DIAGNOSTIC.pressureCeilingExceeded);

    /* Phase 2: enter window with sustained pressure above ceiling+tol.
     * Use t starting at 1.30s to stay past the switch-suppress window.
     * Cap the loop at 25 iterations: debounce clears around i=5 (t~1.36s)
     * and faultEscalationTime is 0.30s, so escalation fires at i~37. We
     * want to observe DERATE before VIOLATED, so we stop short of i=37. */
    deratedPumpSpeed = 0.0;
    for (i = 0; i < 25; i++) {
        tick(&fb, (HYD_REAL)(1.30 + 0.01 * (i + 1)), 80.0, 6.0);
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(!fb.DIAGNOSTIC.pressureCeilingViolated);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    deratedPumpSpeed = fb.PUMP_SPEED;
    assert(deratedPumpSpeed > 0.0);
    /* Derate factor is 0.2; allow margin (0.25) for transient ramp dynamics. */
    assert(deratedPumpSpeed < normalPumpSpeed * (HYD_REAL)0.25);

    printf("test_mold_protect_derate_reduces_pump_speed PASSED\n");
}

/* Task 6 end-to-end STOP escalation + Abort recovery test.
 *
 * Hold pressure above ceiling beyond the 0.30 s fault-escalation window.
 * Verify the FB transitions to FAULT, PUMP_SPEED collapses to zero, and
 * the operator can recover via Abort (Sprint 0 C-3 mask). */
static void test_mold_protect_escalates_to_stop(void) {
    HYD_MotionControlFB fb;
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    int i;

    HYD_MotionControlFB_Init(&fb);
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, (HYD_REAL)0.2, (HYD_REAL)0.2));
    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 10000.0;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Pre-roll: outside the window, past switch-suppress. */
    for (i = 0; i < 100; i++) {
        tick(&fb, (HYD_REAL)(0.01 * (i + 1)), 30.0, 1.0);
    }
    /* Sustained breach inside the window. 300 * 10 ms = 3 s, well past the
     * 0.30 s faultEscalationTime even after the 0.05 s debounce. */
    for (i = 0; i < 300; i++) {
        tick(&fb, (HYD_REAL)(1.0 + 0.01 * (i + 1)), 80.0, (HYD_REAL)6.5);
        if (fb.FB_STATE == HYD_FB_STATE_FAULT) {
            break;
        }
    }
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.faultActive);

    /* Recovery via Abort (Sprint 0 C-3 path: FAULT mask includes ABORT). */
    assert(HYD_MotionControlFB_Abort(&fb));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_ABORTED);
    assert(!fb.STATE.faultActive);

    printf("test_mold_protect_escalates_to_stop PASSED\n");
}

/* Task 6 end-to-end SPEED_RAMP mode test.
 *
 * Builds an injection-fill segment (HYD_MODE_SPEED_RAMP) and attaches the
 * mold-protect ceiling fields manually. Verifies Task 3's contract that
 * HYD_UpdateExecutionDiagnostics now evaluates the ceiling channel under
 * all control modes (not just POSITION). */
static void test_mold_protect_applies_under_speed_ramp_mode(void) {
    HYD_MotionControlFB fb;
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    int i;

    HYD_MotionControlFB_Init(&fb);
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.velocityTolerance = 1.0;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildInjectionFill(&seg, &params, 2,
                                                /*transferPos*/ 100.0));
    /* No dedicated builder for injection-mold-protect; attach manually. */
    seg.pressureCeiling = 8.0;
    seg.pressureCeilingTolerance = (HYD_REAL)0.2;
    seg.pressureCeilingPositionStart = 70.0;
    seg.pressureCeilingPositionEnd = 100.0;
    seg.derateRatio = (HYD_REAL)0.4;

    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 10000.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Pre-roll: outside the window, low pressure, past switch-suppress. */
    for (i = 0; i < 100; i++) {
        tick(&fb, (HYD_REAL)(0.01 * (i + 1)), 30.0, 1.0);
    }
    /* Sustain a breach inside the window. Stop short of the 0.30 s fault
     * escalation: debounce clears at i~6 (t~1.07s); we want to assert the
     * DERATE state, not VIOLATED. */
    for (i = 0; i < 25; i++) {
        tick(&fb, (HYD_REAL)(1.0 + 0.01 * (i + 1)), 80.0, (HYD_REAL)9.5);
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(!fb.DIAGNOSTIC.pressureCeilingViolated);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);

    printf("test_mold_protect_applies_under_speed_ramp_mode PASSED\n");
}

int main(void) {
    test_ceiling_exceeded_in_position_mode();
    test_ceiling_violated_remains_sticky_across_cycles();
    test_mold_protect_derate_reduces_pump_speed();
    test_mold_protect_escalates_to_stop();
    test_mold_protect_applies_under_speed_ramp_mode();
    return 0;
}
