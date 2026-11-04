/* tests/test_stop_diagnostics_retained.c
 * Verifies that fault detection (sensor / time-rollback / timeout) remains
 * active during the STOP deceleration branch. */
#include "motion_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void prime_velocity_run(HYD_MotionControlFB* fb) {
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 80.0;
    seg.maxFlow = 50.0;
    seg.maxAcceleration = 500.0;
    seg.maxDeceleration = 500.0;
    seg.velocityToFlowGain = 0.25;
    seg.duration = 10.0;
    seg.timeoutLimit = 5.0;
    seg.velocityTolerance = 1.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &seg));
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
}

static void test_stop_branch_detects_timestamp_rollback(void) {
    HYD_MotionControlFB fb;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    /* Run velocity segment briefly to enter RUNNING */
    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }
    assert(fb.STATE.active);

    /* Initiate Stop */
    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));
    HYD_MotionControlFB_Execute(&fb);  /* fb now in stopping branch */

    /* Inject timestamp rollback */
    fb.AXIS_REF.timestamp = -1.0;
    HYD_MotionControlFB_Execute(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);
    printf("test_stop_branch_detects_timestamp_rollback PASSED\n");
}

static void test_normal_stop_does_not_false_alarm(void) {
    HYD_MotionControlFB fb;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }

    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));
    /* Run stop to completion */
    for (i = 20; i < 400; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        if (fb.PUMP_SPEED > 0.0) {
            fb.AXIS_REF.velocity *= 0.9;
        } else {
            fb.AXIS_REF.velocity = 0.0;
        }
        HYD_MotionControlFB_Execute(&fb);
        if (fb.FB_STATE == HYD_FB_STATE_DONE) {
            break;
        }
    }
    assert(fb.FB_STATE == HYD_FB_STATE_DONE);
    assert(!fb.STATE.faultActive);
    printf("test_normal_stop_does_not_false_alarm PASSED\n");
}

static void test_natural_stop_settling_does_not_false_alarm(void) {
    HYD_MotionControlFB fb;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    /* Exercise the default hydraulic settling floor instead of an explicit
     * segment timeout. */
    fb._activeSegment.timeoutLimit = 0.0;
    for (i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }

    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 500.0));
    /* Hydraulic axes may need about two seconds to coast to zero after the
     * commanded 100 ms flow ramp has reached zero.  The stop watchdog must
     * not classify that measured deceleration as a stuck actuator. */
    for (i = 0; i < 300; i++) {
        HYD_TIME elapsed = 0.20 + (HYD_TIME)((i + 1) * 0.01);
        HYD_REAL remaining = 50.0 * (1.0 - ((HYD_REAL)(i + 1) / 200.0));
        fb.AXIS_REF.timestamp = elapsed;
        fb.AXIS_REF.velocity = (remaining > 0.0) ? remaining : 0.0;
        HYD_MotionControlFB_Execute(&fb);
        if (fb.FB_STATE == HYD_FB_STATE_DONE) {
            break;
        }
    }

    assert(fb.FB_STATE == HYD_FB_STATE_DONE);
    assert(!fb.STATE.faultActive);
    assert(fb.DIAGNOSTIC.code != HYD_DIAG_CODE_TIMEOUT);
    printf("test_natural_stop_settling_does_not_false_alarm PASSED\n");
}

static void test_stop_branch_detects_sensor_fault(void) {
    HYD_MotionControlFB fb;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }

    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));
    HYD_MotionControlFB_Execute(&fb);

    /* Inject NaN sensor */
    fb.AXIS_REF.timestamp = 0.21;
    fb.AXIS_REF.velocity = NAN;
    HYD_MotionControlFB_Execute(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);
    printf("test_stop_branch_detects_sensor_fault PASSED\n");
}

static void test_stop_branch_detects_stuck_velocity_timeout(void) {
    /* Simulate stuck-velocity sensor: commanded decel ramp drops to zero,
     * but the measured velocity never falls below the completion threshold.
     * Without a stop-timeout, this hangs forever; with C-4 protection the
     * FB must escalate to FAULT. */
    HYD_MotionControlFB fb;
    int i;

    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }

    /* The hydraulic default floor is 3 s; run up to 30 s to keep the
     * assertion independent of the exact calibrated threshold. */
    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));

    for (i = 0; i < 3000; i++) {
        /* Keep sensor velocity stuck at 30 mm/s so completion never triggers */
        fb.AXIS_REF.timestamp = 0.20 + (HYD_TIME)((i + 1) * 0.01);
        fb.AXIS_REF.velocity = 30.0;
        HYD_MotionControlFB_Execute(&fb);
        if (fb.FB_STATE == HYD_FB_STATE_FAULT) {
            break;
        }
    }

    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);
    printf("test_stop_branch_detects_stuck_velocity_timeout PASSED\n");
}

int main(void) {
    test_stop_branch_detects_timestamp_rollback();
    test_normal_stop_does_not_false_alarm();
    test_natural_stop_settling_does_not_false_alarm();
    test_stop_branch_detects_sensor_fault();
    test_stop_branch_detects_stuck_velocity_timeout();
    return 0;
}
