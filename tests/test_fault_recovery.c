/* tests/test_fault_recovery.c - Verifies FAULT -> ABORTED via Abort() command */
#include "motion_control.h"
#include "action_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void prime_fb_with_simple_recipe(HYD_MotionControlFB* fb) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 200.0;
    params.maxAcceleration = 1000.0;
    params.maxDeceleration = 1000.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;

    assert(HYD_ActionProfile_BuildClampClose(&seg, &params, 1, 100.0));
    assert(HYD_MotionControlFB_LoadRecipe(fb, &seg, 1));
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
    fb->USE_RECIPE = true;
}

static void test_abort_recovers_from_fault(void) {
    HYD_MotionControlFB fb;
    HYD_BOOL abortAccepted;

    HYD_MotionControlFB_Init(&fb);
    prime_fb_with_simple_recipe(&fb);

    /* Step 1: enter RUNNING */
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_STARTING || fb.FB_STATE == HYD_FB_STATE_RUNNING);

    /* Step 2: force FAULT by injecting timestamp rollback */
    fb.AXIS_REF.timestamp = -1.0;
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);

    /* Step 3: Abort() in FAULT state must succeed */
    abortAccepted = HYD_MotionControlFB_Abort(&fb);
    assert(abortAccepted);

    /* Restore a valid timestamp so the next Execute can transition state */
    fb.AXIS_REF.timestamp = 0.5;
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_ABORTED);
    assert(!fb.STATE.faultActive);

    printf("test_abort_recovers_from_fault PASSED\n");
}

int main(void) {
    test_abort_recovers_from_fault();
    return 0;
}
