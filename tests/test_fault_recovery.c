/* tests/test_fault_recovery.c - Verifies FAULT -> ABORTED via Abort() command */
#include "motion_control.h"
#include "action_profile.h"
#include <assert.h>
#include <math.h>
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

static void test_soft_reset_preserves_motion_limits(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    fb._params.maxFlow = 72.0;
    fb._params.maxVelocity = 180.0;
    fb._params.velocityToFlowGain = 0.18;
    fb.pumpConfig.displacementMlRev = 25.0;
    fb.pumpConfig.volumetricEfficiency = 0.92;
    fb.pumpConfig.maxSpeedRpm = 1800.0;
    fb.cylinderConfig.areaExtendMm2 = 8000.0;
    fb.cylinderConfig.areaRetractMm2 = 4500.0;
    fb.cylinderConfig.strokeMm = 500.0;

    HYD_MotionControlFB_SoftReset(&fb);

    assert(fabs(fb._params.maxFlow - 72.0) < 1.0e-6);
    assert(fabs(fb._params.maxVelocity - 180.0) < 1.0e-6);
    assert(fabs(fb._params.velocityToFlowGain - 0.18) < 1.0e-6);
    assert(fabs(fb.pumpConfig.displacementMlRev - 25.0) < 1.0e-6);
    assert(fabs(fb.pumpConfig.volumetricEfficiency - 0.92) < 1.0e-6);
    assert(fabs(fb.pumpConfig.maxSpeedRpm - 1800.0) < 1.0e-6);
    assert(fabs(fb.cylinderConfig.areaExtendMm2 - 8000.0) < 1.0e-6);
    assert(fabs(fb.cylinderConfig.areaRetractMm2 - 4500.0) < 1.0e-6);
    assert(fabs(fb.cylinderConfig.strokeMm - 500.0) < 1.0e-6);

    printf("test_soft_reset_preserves_motion_limits PASSED\n");
}

int main(void) {
    test_abort_recovers_from_fault();
    test_soft_reset_preserves_motion_limits();
    return 0;
}
