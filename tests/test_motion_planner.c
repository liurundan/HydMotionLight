#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

static HDY_AxisRef create_test_axis_ref(HDY_REAL position) {
    HDY_AxisRef axisRef = {0};
    axisRef.position = position;
    axisRef.velocity = 0.0;
    axisRef.flow = 0.0;
    axisRef.pressure = 50.0;
    axisRef.timestamp = 0.0;
    return axisRef;
}

static HDY_MotionSegment create_test_segment(void) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, "TestSegment", HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.planner = HDY_PLANNER_POSITION_BASED;
    segment.mode = HDY_MODE_POSITION;
    segment.endCondition = HDY_END_POSITION;
    segment.direction = HDY_DIRECTION_EXTEND;
    segment.targetPosition = 200.0;
    segment.targetFlow = 50.0;
    segment.targetPressure = 100.0;
    segment.maxAcceleration = 5.0;
    segment.maxVelocity = 10.0;
    segment.maxFlow = 40.0;
    segment.duration = 10.0;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = 12.0;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 10.0;
    return segment;
}

static void test_position_based_extend_velocity(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output = {0};
    HDY_REAL remainingDistance;
    HDY_REAL expectedVelocityMagnitude;

    printf("Testing position-based extend planning...\n");
    axisRef = create_test_axis_ref(100.0);
    segment = create_test_segment();

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlanner_Execute(&input, &output);

    remainingDistance = segment.targetPosition - axisRef.position;
    expectedVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HDY_DIRECTION_EXTEND);
    assert(fabs(output.targetVelocity - expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based extend planning test passed\n");
}

static void test_position_based_retract_velocity(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output = {0};
    HDY_REAL remainingDistance;
    HDY_REAL expectedVelocityMagnitude;

    printf("Testing position-based retract planning...\n");
    axisRef = create_test_axis_ref(120.0);
    segment = create_test_segment();
    segment.direction = HDY_DIRECTION_RETRACT;
    segment.targetPosition = 20.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlanner_Execute(&input, &output);

    remainingDistance = axisRef.position - segment.targetPosition;
    expectedVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HDY_DIRECTION_RETRACT);
    assert(fabs(output.targetVelocity + expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based retract planning test passed\n");
}

static void test_speed_ramp_directional_planning(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output = {0};
    HDY_REAL expectedVelocityMagnitude;

    printf("Testing speed-ramp directional planning...\n");
    axisRef = create_test_axis_ref(50.0);
    segment = create_test_segment();
    segment.mode = HDY_MODE_SPEED_RAMP;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.direction = HDY_DIRECTION_RETRACT;
    segment.endCondition = HDY_END_TIME;
    segment.maxAcceleration = 4.0;
    segment.maxVelocity = 8.0;
    segment.maxFlow = 20.0;
    segment.targetFlow = 20.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 1.5;
    input.rampedPressure = 50.0;

    HDY_MotionPlanner_Execute(&input, &output);

    expectedVelocityMagnitude = segment.maxAcceleration * input.elapsedTime;
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HDY_DIRECTION_RETRACT);
    assert(fabs(output.targetVelocity + expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Speed-ramp directional planning test passed\n");
}

static void test_position_mode_time_planner_brakes_near_target(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output = {0};
    HDY_REAL remainingDistance;
    HDY_REAL brakeVelocityMagnitude;

    printf("Testing position-mode time planner braking near target...\n");
    axisRef = create_test_axis_ref(9.7);
    segment = create_test_segment();
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.targetPosition = 10.0;
    segment.maxAcceleration = 20.0;
    segment.maxVelocity = 30.0;
    segment.maxFlow = 50.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 3.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlanner_Execute(&input, &output);

    remainingDistance = segment.targetPosition - axisRef.position;
    brakeVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (brakeVelocityMagnitude > segment.maxVelocity) {
        brakeVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HDY_DIRECTION_EXTEND);
    assert(fabs(output.targetVelocity - brakeVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - brakeVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-mode time planner braking test passed\n");
}

static void test_pressure_mode_is_left_to_pressure_controller_module(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input;
    HDY_MotionPlannerOutput output = {0};

    printf("Testing pressure mode planner bypass behavior...\n");
    axisRef = create_test_axis_ref(100.0);
    segment = create_test_segment();
    segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
    segment.direction = HDY_DIRECTION_HOLD;
    segment.targetFlow = 5.0;
    segment.maxFlow = 12.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 70.0;

    HDY_MotionPlanner_Execute(&input, &output);

    assert(output.direction == HDY_DIRECTION_HOLD);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    printf("✓ Pressure mode planner bypass test passed\n");
}

static void test_edge_cases(void) {
    HDY_AxisRef axisRef;
    HDY_MotionSegment segment;
    HDY_MotionPlannerInput input = {0};
    HDY_MotionPlannerOutput output = {0};

    printf("Testing planner edge cases...\n");

    HDY_MotionPlanner_Execute(NULL, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    assert(output.direction == HDY_DIRECTION_HOLD);

    HDY_MotionPlanner_Execute(&input, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    assert(output.direction == HDY_DIRECTION_HOLD);

    axisRef = create_test_axis_ref(10.0);
    segment = create_test_segment();
    segment.targetPosition = 10.0;
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 1.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlanner_Execute(&input, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    printf("✓ Planner edge cases test passed\n");
}

int main(void) {
    printf("Running MotionPlanner tests...\n\n");

    test_position_based_extend_velocity();
    test_position_based_retract_velocity();
    test_speed_ramp_directional_planning();
    test_position_mode_time_planner_brakes_near_target();
    test_pressure_mode_is_left_to_pressure_controller_module();
    test_edge_cases();

    printf("\n✅ All MotionPlanner tests passed successfully!\n");
    return 0;
}
