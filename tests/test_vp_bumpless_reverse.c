/* tests/test_vp_bumpless_reverse.c
 * Sprint 2 - platform-limit guards for multi-segment VP recipe transitions.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "test_recipe_rejection_helpers.h"

static void test_pressure_to_speed_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segP, segV;

    printf("Testing P->V recipe rejection on current platform...\n");

    HYD_MotionControlFB_Init(&fb);

    /* Preserve the original P->V scenario identity even though
     * HYD_MAX_SEGMENTS=1 rejects the recipe before any transition logic runs. */
    memset(&segP, 0, sizeof(segP));
    segP.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segP.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segP.endCondition = HYD_END_TIME;
    segP.duration = 0.5;
    segP.targetPressure = 10.0;
    segP.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segP.pressureKp = 0.5;
    segP.pressureKi = 0.2;
    segP.pressureIntegralLimit = 10.0;
    segP.maxFlow = 30.0;
    segP.pressureFilterAlpha = 1.0;
    segP.pressureDerivativeFilterAlpha = 1.0;
    segP.direction = HYD_DIRECTION_EXTEND;

    memset(&segV, 0, sizeof(segV));
    segV.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segV.mode = HYD_MODE_SPEED_RAMP;
    segV.planner = HYD_PLANNER_TIME_BASED;
    segV.endCondition = HYD_END_TIME;
    segV.duration = 1.0;
    segV.targetFlow = 20.0;
    segV.maxAcceleration = 100.0;
    segV.maxVelocity = 50.0;
    segV.maxFlow = 30.0;
    segV.velocityToFlowGain = 0.2;
    segV.direction = HYD_DIRECTION_EXTEND;
    segV.pressureFilterAlpha = 1.0;
    segV.pressureDerivativeFilterAlpha = 1.0;

    HYD_MotionSegment recipe[2];
    memcpy(&recipe[0], &segP, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segV, sizeof(HYD_MotionSegment));

    assert_oversized_recipe_load_rejected(&fb, recipe, 2U);

    printf("P->V oversized recipe rejection test passed\n");
}

static void test_speed_to_speed_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segA, segB;

    printf("Testing S->S recipe rejection on current platform...\n");

    HYD_MotionControlFB_Init(&fb);

    /* Preserve the original S->S scenario identity even though
     * HYD_MAX_SEGMENTS=1 rejects the recipe before any blend logic runs. */
    memset(&segA, 0, sizeof(segA));
    segA.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segA.mode = HYD_MODE_SPEED_RAMP;
    segA.planner = HYD_PLANNER_TIME_BASED;
    segA.endCondition = HYD_END_TIME;
    segA.duration = 0.5;
    segA.targetFlow = 10.0;
    segA.maxAcceleration = 100.0;
    segA.maxVelocity = 20.0;
    segA.maxFlow = 20.0;
    segA.velocityToFlowGain = 0.2;
    segA.direction = HYD_DIRECTION_EXTEND;
    segA.pressureFilterAlpha = 1.0;
    segA.pressureDerivativeFilterAlpha = 1.0;

    memset(&segB, 0, sizeof(segB));
    segB.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segB.mode = HYD_MODE_SPEED_RAMP;
    segB.planner = HYD_PLANNER_TIME_BASED;
    segB.endCondition = HYD_END_TIME;
    segB.duration = 1.0;
    segB.targetFlow = 25.0;
    segB.maxAcceleration = 100.0;
    segB.maxVelocity = 50.0;
    segB.maxFlow = 30.0;
    segB.velocityToFlowGain = 0.2;
    segB.direction = HYD_DIRECTION_EXTEND;
    segB.pressureFilterAlpha = 1.0;
    segB.pressureDerivativeFilterAlpha = 1.0;

    HYD_MotionSegment recipe[2];
    memcpy(&recipe[0], &segA, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segB, sizeof(HYD_MotionSegment));

    assert_oversized_recipe_load_rejected(&fb, recipe, 2U);

    printf("S->S oversized recipe rejection test passed\n");
}

int main(void) {
    printf("Running VP platform-limit rejection guards...\n\n");
    test_pressure_to_speed_recipe_is_rejected_when_platform_limit_is_one();
    test_speed_to_speed_recipe_is_rejected_when_platform_limit_is_one();
    printf("\nAll VP platform-limit rejection guards passed.\n");
    return 0;
}
