#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "action_profile.h"
#include "recipe_validator.h"
#include "segment_limits.h"

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

static void test_build_clamp_close_with_mold_protect_populates_window(void) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;  /* fallback when ceilingTolerance=0 */

    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, /*targetPosition*/ 100.0,
        /*protectWindowStart*/ 70.0,
        /*pressureCeiling*/ 5.0,
        /*ceilingTolerance*/ 0.0,  /* should fall back */
        /*derateRatio*/ 0.2));

    assert(seg.segmentTag == 1);
    assert(seg.segmentType == HYD_SEGMENT_TYPE_CLAMPING);
    assert(seg.mode == HYD_MODE_POSITION);
    assert(seg.endCondition == HYD_END_POSITION);
    assert(seg.direction == HYD_DIRECTION_EXTEND);
    assert(seg.pressureCeiling == 5.0);
    assert(seg.pressureCeilingPositionStart == 70.0);
    assert(seg.pressureCeilingPositionEnd == 100.0);
    assert(seg.derateRatio == (HYD_REAL)0.2);

    /* ceilingTolerance=0 in struct; getter falls back to pressureTolerance */
    assert(HYD_Segment_GetPressureCeilingTolerance(&seg) == (HYD_REAL)0.3);

    printf("test_build_clamp_close_with_mold_protect_populates_window PASSED\n");
}

static void test_build_clamp_close_with_mold_protect_rejects_bad_args(void) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;

    /* window start >= target */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 100.0, 5.0, 0.2, 0.2));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 120.0, 5.0, 0.2, 0.2));

    /* ceiling <= 0 */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 0.0, 0.2, 0.2));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, -1.0, 0.2, 0.2));

    /* derateRatio out of (0, 1) range - except 0 which is "use default" */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, 1.0));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, -0.1));

    /* derateRatio = 0 is accepted (means "use default") */
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, 0.0));
    assert(seg.derateRatio == 0.0);  /* preserved as 0; runtime resolves */

    printf("test_build_clamp_close_with_mold_protect_rejects_bad_args PASSED\n");
}

int main(void) {
    test_clamp_close_profile_is_position_extend();
    test_injection_fill_profile_is_speed_ramp_extend();
    test_holding_profile_is_pressure_closed_loop();
    test_build_clamp_close_with_mold_protect_populates_window();
    test_build_clamp_close_with_mold_protect_rejects_bad_args();
    printf("Action profile tests passed\n");
    return 0;
}
