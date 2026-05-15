#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.h"
#include "velocity_controller.h"

static void test_velocity_controller_adds_flow_when_too_slow(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 15.0;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 40.0) < 0.001);
    assert(fabs(output.correctionFlow - 10.0) < 0.001);
    assert(output.active);
}

static void test_velocity_controller_reduces_flow_when_too_fast(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 25.0;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 20.0) < 0.001);
    assert(fabs(output.correctionFlow + 10.0) < 0.001);
    assert(output.active);
}

static void test_velocity_controller_deadband_disables_correction(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 20.05;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 30.0) < 0.001);
    assert(fabs(output.correctionFlow) < 0.001);
    assert(!output.active);
}

static void test_speed_ramp_runtime_applies_velocity_correction(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;
    segment.velocityKp = 2.0;
    segment.velocityDeadband = 0.1;
    segment.velocityCorrectionLimit = 20.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.velocity = 0.0;

    fb.AXIS_REF.timestamp = 0.2;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fabs(fb.STATE.plannedVelocity - 10.0) < 0.001);
    assert(fabs(fb.STATE.plannedFlow - 30.0) < 0.001);
    assert(fabs(fb.PUMP_SPEED - 300.0) < 0.001);
}

int main(void) {
    test_velocity_controller_adds_flow_when_too_slow();
    test_velocity_controller_reduces_flow_when_too_fast();
    test_velocity_controller_deadband_disables_correction();
    test_speed_ramp_runtime_applies_velocity_correction();
    printf("Velocity controller tests passed\n");
    return 0;
}
