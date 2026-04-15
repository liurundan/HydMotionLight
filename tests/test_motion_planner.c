#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "motion_planner.h"

// Helper function to create test axis reference
HDY_AxisRef create_test_axis_ref() {
    HDY_AxisRef axisRef = {0};
    axisRef.position = 100.0;
    axisRef.velocity = 0.0;
    axisRef.flow = 0.0;
    axisRef.pressure = 50.0;
    return axisRef;
}

// Helper function to create test motion segment
HDY_MotionSegment create_test_segment() {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, "TestSegment", HDY_NAME_MAX - 1);
    segment.planner = HDY_PLANNER_POSITION_BASED;
    segment.mode = HDY_MODE_POSITION;
    segment.targetPosition = 200.0;
    segment.targetFlow = 50.0;
    segment.targetPressure = 100.0;
    segment.maxVelocity = 10.0;
    segment.maxAcceleration = 5.0;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 10.0;
    segment.endCondition = HDY_END_POSITION;
    segment.duration = 10.0;
    segment.tolerance = 0.1;
    return segment;
}

// Test position-based velocity computation
void test_position_based_velocity() {
    printf("Testing position-based velocity computation...\n");

    HDY_AxisRef axisRef = create_test_axis_ref();
    HDY_MotionSegment segment = create_test_segment();

    HDY_MotionPlannerInput input = {0};
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.flowToPumpSpeedGain = 10.0;
    input.pumpSpeedLimit = 1000.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlannerOutput output = {0};
    HDY_MotionPlanner_Execute(&input, &output);

    // Expected velocity based on distance remaining
    HDY_REAL remainingDistance = segment.targetPosition - axisRef.position;
    HDY_REAL expectedVelocity = sqrt(2.0 * segment.maxAcceleration * remainingDistance);
    expectedVelocity = (expectedVelocity > segment.maxVelocity) ? segment.maxVelocity : expectedVelocity;

    assert(fabs(output.targetVelocity - expectedVelocity) < 0.001);
    assert(output.targetFlow == expectedVelocity * segment.velocityToFlowGain);
    assert(output.pumpSpeed == output.targetFlow * input.flowToPumpSpeedGain);

    printf("✓ Position-based velocity test passed\n");
}

// Test time-based velocity computation
void test_time_based_velocity() {
    printf("Testing time-based velocity computation...\n");

    HDY_AxisRef axisRef = create_test_axis_ref();
    HDY_MotionSegment segment = create_test_segment();

    // Change to time-based planner
    segment.planner = HDY_PLANNER_TIME_BASED;

    HDY_MotionPlannerInput input = {0};
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 2.0;
    input.flowToPumpSpeedGain = 10.0;
    input.pumpSpeedLimit = 1000.0;
    input.rampedPressure = 50.0;

    HDY_MotionPlannerOutput output = {0};
    HDY_MotionPlanner_Execute(&input, &output);

    // Expected velocity based on time
    HDY_REAL expectedVelocity = segment.maxAcceleration * input.elapsedTime;
    expectedVelocity = (expectedVelocity > segment.maxVelocity) ? segment.maxVelocity : expectedVelocity;

    assert(fabs(output.targetVelocity - expectedVelocity) < 0.001);
    assert(output.targetFlow == expectedVelocity * segment.velocityToFlowGain);
    assert(output.pumpSpeed == output.targetFlow * input.flowToPumpSpeedGain);

    printf("✓ Time-based velocity test passed\n");
}

// Test pressure closed loop mode
void test_pressure_closed_loop() {
    printf("Testing pressure closed loop mode...\n");

    HDY_AxisRef axisRef = create_test_axis_ref();
    HDY_MotionSegment segment = create_test_segment();

    // Change to pressure closed loop mode
    segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;

    HDY_MotionPlannerInput input = {0};
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.flowToPumpSpeedGain = 10.0;
    input.pumpSpeedLimit = 1000.0;
    input.rampedPressure = 80.0; // Ramp pressure

    HDY_MotionPlannerOutput output = {0};
    HDY_MotionPlanner_Execute(&input, &output);

    // In pressure mode, target flow is based on pressure error
    HDY_REAL pressureError = input.rampedPressure - axisRef.pressure;
    HDY_REAL kP = 1.5;
    HDY_REAL expectedFlow = segment.targetFlow + kP * pressureError;
    expectedFlow = (expectedFlow < 0.0) ? 0.0 : expectedFlow;
    expectedFlow = (expectedFlow > segment.maxVelocity) ? segment.maxVelocity : expectedFlow;

    assert(fabs(output.targetFlow - expectedFlow) < 0.001);
    assert(output.targetVelocity == 0.0); // Should not compute velocity in pressure mode
    assert(output.pumpSpeed == expectedFlow * input.flowToPumpSpeedGain);

    printf("✓ Pressure closed loop test passed\n");
}

// Test edge cases
void test_edge_cases() {
    printf("Testing edge cases...\n");

    // Test NULL input
    HDY_MotionPlannerOutput output1 = {0};
    HDY_MotionPlanner_Execute(NULL, &output1);
    // Should not crash

    HDY_MotionPlannerInput input2 = {0};
    HDY_MotionPlannerOutput output2 = {0};
    HDY_MotionPlanner_Execute(&input2, &output2);
    // Should not crash

    // Test negative distances
    HDY_AxisRef axisRef = create_test_axis_ref();
    HDY_MotionSegment segment = create_test_segment();
    segment.targetPosition = 50.0; // Less than current position

    HDY_MotionPlannerInput input3 = {0};
    input3.axisRef = &axisRef;
    input3.segment = &segment;
    input3.elapsedTime = 0.0;
    input3.flowToPumpSpeedGain = 10.0;
    input3.pumpSpeedLimit = 1000.0;
    input3.rampedPressure = 50.0;

    HDY_MotionPlannerOutput output3 = {0};
    HDY_MotionPlanner_Execute(&input3, &output3);

    // Should compute velocity based on zero remaining distance
    assert(output3.targetVelocity == 0.0);
    assert(output3.targetFlow == 0.0);

    printf("✓ Edge cases test passed\n");
}

// Test velocity to flow conversion with different gains
void test_velocity_to_flow_conversion() {
    printf("Testing velocity to flow conversion...\n");

    HDY_AxisRef axisRef = create_test_axis_ref();
    HDY_MotionSegment segment = create_test_segment();

    // Test different gain values
    HDY_REAL testGains[] = {0.5, 1.0, 2.0, 0.0};
    HDY_REAL expectedFlows[4];

    for (int i = 0; i < 4; i++) {
        segment.velocityToFlowGain = testGains[i];

        HDY_MotionPlannerInput input = {0};
        input.axisRef = &axisRef;
        input.segment = &segment;
        input.elapsedTime = 0.0;
        input.flowToPumpSpeedGain = 10.0;
        input.pumpSpeedLimit = 1000.0;
        input.rampedPressure = 50.0;

        HDY_MotionPlannerOutput output = {0};
        HDY_MotionPlanner_Execute(&input, &output);

        // Handle special case for zero gain
        if (segment.velocityToFlowGain <= 0.0) {
            expectedFlows[i] = 1.0 * input.segment->maxVelocity; // Default gain of 1.0
        } else {
            expectedFlows[i] = 5.0 * segment.velocityToFlowGain; // 5.0 is targetVelocity
        }
        expectedFlows[i] = (expectedFlows[i] < 0.0) ? 0.0 : expectedFlows[i];
        expectedFlows[i] = (expectedFlows[i] > segment.maxVelocity * segment.velocityToFlowGain) ?
                           segment.maxVelocity * segment.velocityToFlowGain : expectedFlows[i];

        assert(fabs(output.targetFlow - expectedFlows[i]) < 0.001);
    }

    printf("✓ Velocity to flow conversion test passed\n");
}

int main() {
    printf("Running MotionPlanner tests...\n\n");

    test_position_based_velocity();
    test_time_based_velocity();
    test_pressure_closed_loop();
    test_edge_cases();
    test_velocity_to_flow_conversion();

    printf("\n✅ All tests passed successfully!\n");

    return 0;
}