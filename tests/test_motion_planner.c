#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

static HYD_AxisRef create_test_axis_ref(HYD_REAL position) {
    HYD_AxisRef axisRef = {0};
    axisRef.position = position;
    axisRef.velocity = 0.0;
    axisRef.flow = 0.0;
    axisRef.pressure = 50.0;
    axisRef.timestamp = 0.0;
    return axisRef;
}

static HYD_MotionSegment create_test_segment(void) {
    HYD_MotionSegment segment = {0};
    segment.segmentTag = 1;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
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
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL remainingDistance;
    HYD_REAL expectedVelocityMagnitude;

    printf("Testing position-based extend planning...\n");
    axisRef = create_test_axis_ref(100.0);
    segment = create_test_segment();

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 50.0;

    HYD_MotionPlanner_Execute(&input, &output);

    remainingDistance = segment.targetPosition - axisRef.position;
    expectedVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HYD_DIRECTION_EXTEND);
    assert(fabs(output.targetVelocity - expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based extend planning test passed\n");
}

static void test_position_based_retract_velocity(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL remainingDistance;
    HYD_REAL expectedVelocityMagnitude;

    printf("Testing position-based retract planning...\n");
    axisRef = create_test_axis_ref(120.0);
    segment = create_test_segment();
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.targetPosition = 20.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 50.0;

    HYD_MotionPlanner_Execute(&input, &output);

    remainingDistance = axisRef.position - segment.targetPosition;
    expectedVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HYD_DIRECTION_RETRACT);
    assert(fabs(output.targetVelocity + expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-based retract planning test passed\n");
}

static void test_speed_ramp_directional_planning(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL expectedVelocityMagnitude;

    printf("Testing speed-ramp directional planning...\n");
    axisRef = create_test_axis_ref(50.0);
    segment = create_test_segment();
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.endCondition = HYD_END_TIME;
    segment.maxAcceleration = 4.0;
    segment.maxVelocity = 8.0;
    segment.maxFlow = 20.0;
    segment.targetFlow = 20.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 1.5;
    input.rampedPressure = 50.0;

    HYD_MotionPlanner_Execute(&input, &output);

    expectedVelocityMagnitude = segment.maxAcceleration * input.elapsedTime;
    if (expectedVelocityMagnitude > segment.maxVelocity) {
        expectedVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HYD_DIRECTION_RETRACT);
    assert(fabs(output.targetVelocity + expectedVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - expectedVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Speed-ramp directional planning test passed\n");
}

static void test_position_mode_time_planner_brakes_near_target(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL remainingDistance;
    HYD_REAL brakeVelocityMagnitude;

    printf("Testing position-mode time planner braking near target...\n");
    axisRef = create_test_axis_ref(9.7);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.targetPosition = 10.0;
    segment.maxAcceleration = 20.0;
    segment.maxVelocity = 30.0;
    segment.maxFlow = 50.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 3.0;
    input.rampedPressure = 50.0;

    HYD_MotionPlanner_Execute(&input, &output);

    remainingDistance = segment.targetPosition - axisRef.position;
    brakeVelocityMagnitude = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    if (brakeVelocityMagnitude > segment.maxVelocity) {
        brakeVelocityMagnitude = segment.maxVelocity;
    }

    assert(output.direction == HYD_DIRECTION_EXTEND);
    assert(fabs(output.targetVelocity - brakeVelocityMagnitude) < 0.001);
    assert(fabs(output.targetFlow - brakeVelocityMagnitude * segment.velocityToFlowGain) < 0.001);
    printf("✓ Position-mode time planner braking test passed\n");
}

static void test_pressure_mode_is_left_to_pressure_controller_module(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    printf("Testing pressure mode planner bypass behavior...\n");
    axisRef = create_test_axis_ref(100.0);
    segment = create_test_segment();
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetFlow = 5.0;
    segment.maxFlow = 12.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.rampedPressure = 70.0;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.direction == HYD_DIRECTION_HOLD);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    printf("✓ Pressure mode planner bypass test passed\n");
}

static void test_edge_cases(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    printf("Testing planner edge cases...\n");

    HYD_MotionPlanner_Execute(NULL, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    assert(output.direction == HYD_DIRECTION_HOLD);

    HYD_MotionPlanner_Execute(&input, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    assert(output.direction == HYD_DIRECTION_HOLD);

    axisRef = create_test_axis_ref(10.0);
    segment = create_test_segment();
    segment.targetPosition = 10.0;
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 1.0;
    input.rampedPressure = 50.0;

    HYD_MotionPlanner_Execute(&input, &output);
    assert(output.targetVelocity == 0.0);
    assert(output.targetFlow == 0.0);
    printf("✓ Planner edge cases test passed\n");
}

/* ---- Trapezoid profile tests ---- */

static void test_trapezoid_full_profile(void) {
    HYD_TrapezoidProfile profile;
    HYD_BOOL ok;
    HYD_REAL vel;
    HYD_REAL vMax = 10.0;
    HYD_REAL acc = 5.0;
    HYD_REAL distance = 100.0;

    printf("Testing full trapezoid profile (enough distance)...\n");

    ok = HYD_PlanTrapezoid(&profile, distance, vMax, acc);
    assert(ok);

    /* s_brake = 10^2 / (2*5) = 10, 2*s_brake = 20 < 100, full trapezoid */
    assert(profile.sAcc > 0.0);
    assert(profile.sConst > 0.0);
    assert(profile.sDec > 0.0);
    assert(fabs(profile.sAcc + profile.sConst + profile.sDec - distance) < 0.001);
    assert(fabs(profile.sAcc - profile.sDec) < 0.001);

    /* Acceleration phase: at t = tAcc/2, velocity should be acc * tAcc/2 */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc * 0.5, acc, vMax);
    assert(vel > 0.0 && vel < vMax);

    /* At end of acceleration: velocity should be vMax */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc, acc, vMax);
    assert(fabs(vel - vMax) < 0.001);

    /* Constant phase: velocity should be vMax */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tConst * 0.5, acc, vMax);
    assert(fabs(vel - vMax) < 0.001);

    /* Deceleration phase: at mid-decel, velocity should be between 0 and vMax */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tConst + profile.tDec * 0.5, acc, vMax);
    assert(vel > 0.0 && vel < vMax);

    /* At end: velocity should be 0 */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tConst + profile.tDec, acc, vMax);
    assert(fabs(vel) < 0.001);
    printf("✓ Full trapezoid profile test passed\n");
}

static void test_trapezoid_triangular_profile(void) {
    HYD_TrapezoidProfile profile;
    HYD_BOOL ok;
    HYD_REAL vel;
    HYD_REAL vMax = 10.0;
    HYD_REAL acc = 5.0;
    HYD_REAL distance = 10.0;

    printf("Testing triangular profile (short distance)...\n");

    ok = HYD_PlanTrapezoid(&profile, distance, vMax, acc);
    assert(ok);

    /* s_brake = 10, 2*s_brake = 20 > 10, triangular: no const phase */
    assert(profile.sAcc > 0.0);
    assert(fabs(profile.sConst) < 0.001);
    assert(profile.sDec > 0.0);
    assert(fabs(profile.sAcc + profile.sConst + profile.sDec - distance) < 0.001);
    assert(fabs(profile.sAcc - profile.sDec) < 0.001);

    /* At peak, velocity should be sqrt(acc * distance) = sqrt(50) ≈ 7.07 */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc, acc, vMax);
    assert(vel > 0.0 && vel < vMax);
    assert(fabs(vel - sqrt(acc * distance)) < 0.001);

    /* At end: velocity should be 0 */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tDec, acc, vMax);
    assert(fabs(vel) < 0.001);
    printf("✓ Triangular profile test passed\n");
}

static void test_trapezoid_edge_cases(void) {
    HYD_TrapezoidProfile profile;
    HYD_BOOL ok;
    HYD_REAL vel;

    printf("Testing trapezoid edge cases...\n");

    /* Zero distance */
    ok = HYD_PlanTrapezoid(&profile, 0.0, 10.0, 5.0);
    assert(!ok);
    assert(fabs(profile.tAcc) < 0.001);
    assert(fabs(profile.sAcc) < 0.001);

    /* Zero vMax */
    ok = HYD_PlanTrapezoid(&profile, 100.0, 0.0, 5.0);
    assert(!ok);

    /* Zero acc */
    ok = HYD_PlanTrapezoid(&profile, 100.0, 10.0, 0.0);
    assert(!ok);

    /* NULL profile */
    ok = HYD_PlanTrapezoid(NULL, 100.0, 10.0, 5.0);
    assert(!ok);

    /* EvalTrapezoid at negative elapsed */
    ok = HYD_PlanTrapezoid(&profile, 100.0, 10.0, 5.0);
    assert(ok);
    vel = HYD_EvalTrapezoid(&profile, -1.0, 5.0, 10.0);
    assert(fabs(vel) < 0.001);

    /* EvalTrapezoid beyond total time */
    vel = HYD_EvalTrapezoid(&profile, profile.tAcc + profile.tConst + profile.tDec + 10.0, 5.0, 10.0);
    assert(fabs(vel) < 0.001);
    printf("✓ Trapezoid edge cases test passed\n");
}

static void test_speed_ramp_deceleration_on_stop(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;
    HYD_REAL acc;
    HYD_REAL startVel;

    printf("Testing SPEED_RAMP deceleration behavior...\n");

    axisRef = create_test_axis_ref(50.0);
    segment = create_test_segment();
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_TIME;
    segment.maxAcceleration = 4.0;
    segment.maxVelocity = 20.0;
    segment.duration = 10.0;

    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 5.0;
    input.rampedPressure = 50.0;

    /* No deceleration: normal ramp up */
    input.decelElapsed = 0.0;
    input.decelStartVel = 0.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(output.targetVelocity > 0.0);
    assert(fabs(output.targetVelocity - 20.0) < 0.001);

    /* Deceleration active: velocity should decrease from startVel */
    acc = segment.maxAcceleration;
    startVel = 16.0;

    input.decelStartVel = startVel;
    input.decelElapsed = 1.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - (startVel - acc * 1.0)) < 0.001);

    input.decelElapsed = 2.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - (startVel - acc * 2.0)) < 0.001);

    /* Deceleration to zero: velocity clamped at 0 */
    input.decelElapsed = 5.0;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity) < 0.001);

    /* Deceleration with position end condition: brake still applies */
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 60.0;
    input.decelElapsed = 1.0;
    input.decelStartVel = 10.0;
    HYD_MotionPlanner_Execute(&input, &output);
    /* Should be min(ramp braking, deceleration) */
    assert(output.targetVelocity >= 0.0);
    assert(output.targetVelocity <= 10.0);
    printf("✓ SPEED_RAMP deceleration test passed\n");
}

int main(void) {
    printf("Running MotionPlanner tests...\n\n");

    test_position_based_extend_velocity();
    test_position_based_retract_velocity();
    test_speed_ramp_directional_planning();
    test_position_mode_time_planner_brakes_near_target();
    test_pressure_mode_is_left_to_pressure_controller_module();
    test_edge_cases();
    test_trapezoid_full_profile();
    test_trapezoid_triangular_profile();
    test_trapezoid_edge_cases();
    test_speed_ramp_deceleration_on_stop();

    printf("\n✅ All MotionPlanner tests passed successfully!\n");
    return 0;
}
