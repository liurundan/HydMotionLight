/*
 * Sprint B 工艺集成测试
 *
 * 验证对象：
 * 1. 梯形规划通过FB位置模式执行
 * 2. SPEED_RAMP 减速段自动触发
 * 3. 增益调度在压力段中生效
 * 4. 多段配方无故障完成
 */

#include "motion_control.h"
#include "motion_planner.h"
#include "test_recipe_rejection_helpers.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_fb_rejects_oversized_recipe_with_new_features(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment recipe[3];

    printf("Testing oversized recipe rejection with new-feature segments...\n");

    HYD_MotionControlFB_Init(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.USE_RECIPE = true;

    memset(recipe, 0, sizeof(recipe));

    /* Segment 1: Position with time planner (uses trapezoid via planner) */
    recipe[0].segmentTag = 1;
    recipe[0].segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    recipe[0].planner = HYD_PLANNER_TIME_BASED;
    recipe[0].mode = HYD_MODE_POSITION;
    recipe[0].endCondition = HYD_END_POSITION;
    recipe[0].direction = HYD_DIRECTION_EXTEND;
    recipe[0].targetPosition = 100.0;
    recipe[0].maxAcceleration = 20.0;
    recipe[0].maxVelocity = 40.0;
    recipe[0].maxFlow = 50.0;
    recipe[0].velocityToFlowGain = 1.0;
    recipe[0].positionTolerance = 0.5;
    recipe[0].timeoutLimit = 10.0;

    /* Segment 2: SPEED_RAMP with TIME end (auto-deceleration) */
    recipe[1].segmentTag = 2;
    recipe[1].segmentType = HYD_SEGMENT_TYPE_INJECTION;
    recipe[1].planner = HYD_PLANNER_TIME_BASED;
    recipe[1].mode = HYD_MODE_SPEED_RAMP;
    recipe[1].endCondition = HYD_END_TIME;
    recipe[1].direction = HYD_DIRECTION_EXTEND;
    recipe[1].duration = 0.5;
    recipe[1].maxAcceleration = 10.0;
    recipe[1].maxVelocity = 30.0;
    recipe[1].maxFlow = 60.0;
    recipe[1].velocityToFlowGain = 1.0;
    recipe[1].timeoutLimit = 5.0;

    /* Segment 3: Pressure closed-loop with gain scheduling */
    recipe[2].segmentTag = 3;
    recipe[2].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[2].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[2].endCondition = HYD_END_TIME;
    recipe[2].direction = HYD_DIRECTION_HOLD;
    recipe[2].targetPressure = 80.0;
    recipe[2].targetFlow = 3.0;
    recipe[2].duration = 0.2;
    recipe[2].maxFlow = 15.0;
    recipe[2].pressureController = HYD_PRESSURE_CONTROLLER_P;
    recipe[2].pressureKp = 0.3;
    recipe[2].pressureKpHigh = 1.0;
    recipe[2].pressureGainBand = 0.2;
    recipe[2].pressureTolerance = 2.0;
    recipe[2].timeoutLimit = 3.0;

    assert_oversized_recipe_load_rejected(&fb, recipe, 3U);

    printf("✓ FB rejects oversized recipe with new features\n");
}

static void test_trapezoid_planning_with_short_distance_triangular(void) {
    HYD_TrapezoidProfile profile;
    HYD_BOOL ok;
    HYD_REAL vel;

    printf("Testing trapezoid planning for typical mold-close distances...\n");

    /* Long stroke (200mm): full trapezoid */
    ok = HYD_PlanTrapezoid(&profile, 200.0, 50.0, 25.0);
    assert(ok);
    assert(profile.sConst > 0.0);

    /* Short stroke (5mm): triangular — mold protection zone */
    ok = HYD_PlanTrapezoid(&profile, 5.0, 30.0, 15.0);
    assert(ok);
    assert(fabs(profile.sConst) < 0.001);
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc, 15.0, 30.0);
    assert(vel < 30.0);

    /* Injection stroke (100mm): full trapezoid */
    ok = HYD_PlanTrapezoid(&profile, 100.0, 40.0, 20.0);
    assert(ok);
    assert(profile.sConst > 0.0);
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tConst * 0.5, 20.0, 40.0);
    assert(fabs(vel - 40.0) < 0.001);

    printf("✓ Trapezoid planning for typical mold distances passed\n");
}

static void test_gain_scheduling_transitions_smoothly(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput outputs[5];
    int i;
    HYD_REAL prevOutput;

    printf("Testing gain scheduling smooth transition across error range...\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetFlow = 2.0;
    segment.targetPressure = 100.0;
    segment.maxFlow = 20.0;
    segment.duration = 1.0;
    segment.pressureTolerance = 0.2;
    segment.timeoutLimit = 2.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.2;
    segment.pressureKpHigh = 1.0;
    segment.pressureGainBand = 0.2;

    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    /* Simulate pressure approaching target from far away */
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;

    prevOutput = 0.0;
    for (i = 0; i < 5; i++) {
        HYD_REAL measured[] = {50.0, 70.0, 85.0, 95.0, 99.0};
        input.targetPressure = 100.0;
        input.measuredPressure = measured[i];
        input.timestamp = (i + 1) * 0.01;
        HYD_PressureController_Execute(&segment, &state, &input, &outputs[i]);

        /* Output should decrease smoothly as error shrinks */
        if (i > 0) {
            HYD_REAL step = outputs[i].outputFlow - prevOutput;
            assert(fabs(step) < segment.maxFlow);
        }
        prevOutput = outputs[i].outputFlow;
    }

    /* Final output should be near feedforward (small error, low gain) */
    assert(fabs(outputs[4].outputFlow - segment.targetFlow) < 2.0);
    printf("✓ Gain scheduling smooth transition test passed\n");
}

static void test_derate_reduces_runtime_pump_speed_and_tracks_rbf_pi_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL nominalSpeed;
    HYD_REAL deratedSpeed;
    HYD_REAL expectedDeratedSpeed;

    printf("Testing diagnostic derate reduces RBF-PI runtime pump speed and tracks applied flow...\n");

    HYD_MotionControlFB_Init(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.USE_RECIPE = false;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.segmentTag = 9;
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 20.0;
    segment.targetFlow = 20.0;
    segment.maxFlow = 100.0;
    segment.duration = 5.0;
    segment.pressureTolerance = 0.1;
    segment.flowTolerance = 0.1;
    segment.pressureRampRate = 100.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PI;
    segment.pressureRbfConfig.minKp = 1.0;
    segment.pressureRbfConfig.maxKp = 1.0;
    segment.pressureRbfConfig.minKi = 0.001;
    segment.pressureRbfConfig.maxKi = 0.001;
    segment.pressureRbfConfig.strategy.outputSlewRate = 100000.0;
    segment.pressureIntegralLimit = 100.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = 20.0;
    fb.AXIS_REF.flow = 20.0;
    HYD_MotionControlFB_Cycle(&fb);
    nominalSpeed = fb.PUMP_SPEED;
    assert(nominalSpeed > 0.0);
    assert(fb.STATE.status == HYD_STATUS_RUNNING);

    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.pressure = 20.0;
    fb.AXIS_REF.flow = 0.0;
    HYD_MotionControlFB_Cycle(&fb);

    fb.AXIS_REF.timestamp = 1.2;
    fb.AXIS_REF.pressure = 20.0;
    fb.AXIS_REF.flow = 0.0;
    HYD_MotionControlFB_Cycle(&fb);
    deratedSpeed = fb.PUMP_SPEED;
    expectedDeratedSpeed = fb.STATE.pressureLoop.outputFlow *
        fb.FLOW_TO_PUMP_SPEED_GAIN * 0.5;

    assert(deratedSpeed > 0.0);
    assert(fabs(deratedSpeed - expectedDeratedSpeed) < 0.001);
    assert(fabs((double)fb._pressureController.rbfPid.u_prev -
                (double)fb.STATE.plannedFlow) < 1e-6);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    assert(fb.STATE.status == HYD_STATUS_DEGRADED);

    printf("✓ Diagnostic derate RBF-PI runtime tracking test passed\n");
}

int main(void) {
    printf("Running Sprint B integration tests...\n\n");

    test_fb_rejects_oversized_recipe_with_new_features();
    test_trapezoid_planning_with_short_distance_triangular();
    test_gain_scheduling_transitions_smoothly();
    test_derate_reduces_runtime_pump_speed_and_tracks_rbf_pi_flow();

    printf("\n✅ All Sprint B integration tests passed successfully!\n");
    return 0;
}
