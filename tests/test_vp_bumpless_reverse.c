/* tests/test_vp_bumpless_reverse.c
 * Sprint 2 - VP bumpless reverse + Speed-to-Speed blending.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"

static void test_pressure_to_speed_bumpless_seeding(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segP, segV;

    printf("Testing P->V bumpless velocity seeding...\n");

    HYD_MotionControlFB_Init(&fb);

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

    assert(HYD_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb.AXIS_REF.flow = 5.0;
    HYD_MotionControlFB_Execute(&fb);

    int i;
    for (i = 0; i < 200; i++) {
        fb.AXIS_REF.timestamp += 0.005;
        fb.AXIS_REF.pressure += 0.1;
        if (fb.AXIS_REF.pressure > segP.targetPressure) {
            fb.AXIS_REF.pressure = segP.targetPressure;
        }
        fb.AXIS_REF.flow = 5.0;
        HYD_MotionControlFB_Execute(&fb);
        if (fb.SEGMENT_COMPLETED) break;
    }

    HYD_REAL lastFlow = fb._lastCommandedFlow;
    printf("  Last commanded flow from P segment: %.3f L/min\n", lastFlow);

    assert(HYD_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));

    fb.AXIS_REF.timestamp += 0.005;
    HYD_MotionControlFB_Execute(&fb);

    assert(fb._previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP);

    HYD_REAL seededVelocity = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Seeded planner velocity: %.3f mm/s\n", seededVelocity);

    HYD_REAL expectedVelocity = lastFlow / segV.velocityToFlowGain;
    printf("  Expected velocity (flow/gain): %.3f mm/s\n", expectedVelocity);
    assert(lastFlow > 0.0);
    assert(seededVelocity > 0.0);
    assert(fabs(seededVelocity - expectedVelocity) < 1.0);
    assert(fb._plannerState.initialized);

    printf("P->V bumpless seeding test passed\n");
}

static void test_speed_to_speed_blending_carryover(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segA, segB;

    printf("Testing S->S speed blending carryover...\n");

    HYD_MotionControlFB_Init(&fb);

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

    assert(HYD_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;

    int i;
    for (i = 0; i < 200; i++) {
        fb.AXIS_REF.timestamp += 0.005;
        HYD_MotionControlFB_Execute(&fb);
        fb.AXIS_REF.velocity = fb._plannerState.lastTargetVelocity;
        if (fb.SEGMENT_COMPLETED) break;
    }

    HYD_REAL lastVelocityA = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Segment A final velocity: %.3f mm/s\n", lastVelocityA);

    assert(HYD_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));

    fb.AXIS_REF.timestamp += 0.005;
    HYD_MotionControlFB_Execute(&fb);

    HYD_REAL seededVelocity = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Segment B seeded velocity: %.3f mm/s\n", seededVelocity);

    assert(seededVelocity > 0.0);
    assert(fabs(seededVelocity - lastVelocityA) < 1.0);
    assert(fb._plannerState.initialized);

    printf("S->S blending carryover test passed\n");
}

int main(void) {
    printf("Running VP bumpless reverse / blending tests...\n\n");
    test_pressure_to_speed_bumpless_seeding();
    test_speed_to_speed_blending_carryover();
    printf("\nAll bumpless/blending tests passed.\n");
    return 0;
}
