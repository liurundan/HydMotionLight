#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "action_profile.h"
#include "recipe_validator.h"

static HYD_MotionFBParams default_params(void) {
    HYD_MotionFBParams params;
    memset(&params, 0, sizeof(params));
    params.positionTolerance = 0.1;
    params.velocityTolerance = 0.5;
    params.flowTolerance = 1.0;
    params.pressureTolerance = 0.5;
    params.timeoutLimit = 10.0;
    params.velocityToFlowGain = 1.0;
    params.velocityKp = 2.0;
    params.velocityDeadband = 0.1;
    params.velocityCorrectionLimit = 20.0;
    params.maxVelocity = 100.0;
    params.maxAcceleration = 50.0;
    params.maxDeceleration = 40.0;
    params.maxFlow = 120.0;
    params.pressureRampRate = 20.0;
    params.pressureKp = 1.0;
    params.pressureKi = 0.2;
    params.pressureControllerType = HYD_PRESSURE_CONTROLLER_PI;
    return params;
}

static void test_clamp_close_profile_is_position_extend(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildClampClose(&segment, &params, 1, 250.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_CLAMPING);
    assert(segment.mode == HYD_MODE_POSITION);
    assert(segment.endCondition == HYD_END_POSITION);
    assert(segment.direction == HYD_DIRECTION_EXTEND);
    assert(segment.targetPosition == 250.0);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

static void test_injection_fill_profile_is_speed_ramp_extend(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildInjectionFill(&segment, &params, 2, 100.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_INJECTION);
    assert(segment.mode == HYD_MODE_SPEED_RAMP);
    assert(segment.planner == HYD_PLANNER_TIME_BASED);
    assert(segment.direction == HYD_DIRECTION_EXTEND);
    assert(segment.targetPosition == 100.0);
    assert(segment.velocityKp == params.velocityKp);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

static void test_holding_profile_is_pressure_closed_loop(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildHoldingPressure(&segment, &params, 3, 80.0, 2.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_HOLDING);
    assert(segment.mode == HYD_MODE_PRESSURE_CLOSED_LOOP);
    assert(segment.endCondition == HYD_END_TIME);
    assert(segment.direction == HYD_DIRECTION_HOLD);
    assert(segment.targetPressure == 80.0);
    assert(segment.duration == 2.0);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

int main(void) {
    test_clamp_close_profile_is_position_extend();
    test_injection_fill_profile_is_speed_ramp_extend();
    test_holding_profile_is_pressure_closed_loop();
    printf("Action profile tests passed\n");
    return 0;
}
