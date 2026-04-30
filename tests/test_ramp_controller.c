/**
 * Test program to verify pressure ramp controller behavior
 * Tests both pressure increase and decrease scenarios
 */

#include "../include/ramp_controller.h"
#include "../include/common_types.h"
#include <stdio.h>
#include <math.h>

#define EPSILON 0.0001

/**
 * Test pressure increase (升压测试)
 */
void test_pressure_increase(void) {
    printf("\n=== Test 1: Pressure Increase (升压) ===\n");

    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    // Initialize: current pressure = 2.0 MPa
    HYD_RampController_Init(&controller, 2.0, 0.0);

    // Set target: ramp from 2.0 to 10.0 MPa at 5.0 MPa/s
    input.targetPressure = 10.0;
    input.rampRate = 5.0;
    input.currentTime = 0.0;

    printf("Initial pressure: %.2f MPa\n", controller.rampedPressure);
    printf("Target pressure: %.2f MPa\n", input.targetPressure);
    printf("Ramp rate: %.2f MPa/s\n", input.rampRate);
    printf("\n");

    // Simulate 2 seconds of execution
    HYD_TIME dt = 0.01;  // 10ms cycle
    for (int i = 0; i < 200; i++) {
        input.currentTime = (i + 1) * dt;
        HYD_RampController_Execute(&controller, &input, &output);

        // Print every 50 cycles (0.5s)
        if ((i + 1) % 50 == 0) {
            printf("t=%.2fs: pressure=%.2f MPa\n", input.currentTime, output.rampedPressure);
        }
    }

    printf("Final pressure: %.2f MPa\n", output.rampedPressure);

    // Verification: after 2 seconds at 5 MPa/s, pressure should increase by 10 MPa
    // Expected: 2.0 + 5.0 * 2.0 = 12.0, but clamped to target 10.0
    HYD_REAL expected = input.targetPressure;
    if (fabs(output.rampedPressure - expected) < EPSILON) {
        printf("✓ PASS: Final pressure %.2f MPa equals target %.2f MPa\n", output.rampedPressure, expected);
    } else {
        printf("✗ FAIL: Expected %.2f MPa, got %.2f MPa\n", expected, output.rampedPressure);
    }
}

/**
 * Test pressure decrease (降压测试)
 */
void test_pressure_decrease(void) {
    printf("\n=== Test 2: Pressure Decrease (降压) ===\n");

    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    // Initialize: current pressure = 10.0 MPa
    HYD_RampController_Init(&controller, 10.0, 0.0);

    // Set target: ramp from 10.0 to 2.0 MPa at 5.0 MPa/s
    input.targetPressure = 2.0;
    input.rampRate = 5.0;
    input.currentTime = 0.0;

    printf("Initial pressure: %.2f MPa\n", controller.rampedPressure);
    printf("Target pressure: %.2f MPa\n", input.targetPressure);
    printf("Ramp rate: %.2f MPa/s\n", input.rampRate);
    printf("\n");

    // Simulate 2 seconds of execution
    HYD_TIME dt = 0.01;  // 10ms cycle
    for (int i = 0; i < 200; i++) {
        input.currentTime = (i + 1) * dt;
        HYD_RampController_Execute(&controller, &input, &output);

        // Print every 50 cycles (0.5s)
        if ((i + 1) % 50 == 0) {
            printf("t=%.2fs: pressure=%.2f MPa\n", input.currentTime, output.rampedPressure);
        }
    }

    printf("Final pressure: %.2f MPa\n", output.rampedPressure);

    // Verification: after 2 seconds at 5 MPa/s, pressure should decrease by 10 MPa
    // Expected: 10.0 - 5.0 * 2.0 = 0.0, but clamped to target 2.0
    HYD_REAL expected = input.targetPressure;
    if (fabs(output.rampedPressure - expected) < EPSILON) {
        printf("✓ PASS: Final pressure %.2f MPa equals target %.2f MPa\n", output.rampedPressure, expected);
    } else {
        printf("✗ FAIL: Expected %.2f MPa, got %.2f MPa\n", expected, output.rampedPressure);
    }
}

/**
 * Test ramp rate symmetry (斜坡对称性测试)
 */
void test_ramp_symmetry(void) {
    printf("\n=== Test 3: Ramp Rate Symmetry (斜坡对称性) ===\n");

    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    HYD_REAL startPressure = 5.0;
    HYD_REAL rampRate = 2.0;
    HYD_TIME rampTime = 1.0;

    // Test increase: 5.0 -> 7.0 MPa
    printf("\nPart A: Increase from %.1f to %.1f MPa at %.1f MPa/s\n",
           startPressure, startPressure + rampRate, rampRate);
    HYD_RampController_Init(&controller, startPressure, 0.0);
    input.targetPressure = startPressure + rampRate;
    input.rampRate = rampRate;
    input.currentTime = 0.0;

    int steps = (int)(rampTime / 0.01);
    HYD_REAL pressureAfterIncrease;
    for (int i = 0; i < steps; i++) {
        input.currentTime = (i + 1) * 0.01;
        HYD_RampController_Execute(&controller, &input, &output);
    }
    pressureAfterIncrease = output.rampedPressure;
    printf("Result: %.4f MPa\n", pressureAfterIncrease);

    // Test decrease: 7.0 -> 5.0 MPa
    printf("\nPart B: Decrease from %.1f to %.1f MPa at %.1f MPa/s\n",
           pressureAfterIncrease, startPressure, rampRate);
    HYD_RampController_Init(&controller, pressureAfterIncrease, 0.0);
    input.targetPressure = startPressure;
    input.rampRate = rampRate;
    input.currentTime = 0.0;

    for (int i = 0; i < steps; i++) {
        input.currentTime = (i + 1) * 0.01;
        HYD_RampController_Execute(&controller, &input, &output);
    }
    printf("Result: %.4f MPa\n", output.rampedPressure);

    // Verification: should return to start value
    if (fabs(output.rampedPressure - startPressure) < 0.01) {
        printf("✓ PASS: Ramp is symmetric (returned to %.2f MPa)\n", startPressure);
    } else {
        printf("✗ FAIL: Expected %.2f MPa, got %.2f MPa\n", startPressure, output.rampedPressure);
    }
}

/**
 * Test ramp rate calculation (斜坡变化率验证)
 */
void test_ramp_rate_calculation(void) {
    printf("\n=== Test 4: Ramp Rate Calculation (变化率验证) ===\n");

    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    HYD_REAL startPressure = 0.0;
    HYD_REAL rampRate = 10.0;
    HYD_TIME dt = 0.01;

    printf("Testing pressure increase from %.1f MPa with rate %.1f MPa/s\n",
           startPressure, rampRate);
    printf("Cycle time: %.3f s\n", dt);
    printf("Expected change per cycle: %.4f MPa\n", rampRate * dt);
    printf("\n");

    HYD_RampController_Init(&controller, startPressure, 0.0);
    input.targetPressure = 100.0;  // Far target
    input.rampRate = rampRate;
    input.currentTime = 0.0;

    // Check first 10 cycles
    HYD_REAL previousPressure = startPressure;
    for (int i = 0; i < 10; i++) {
        input.currentTime = (i + 1) * dt;
        HYD_RampController_Execute(&controller, &input, &output);

        HYD_REAL actualChange = output.rampedPressure - previousPressure;
        HYD_REAL expectedChange = rampRate * dt;

        printf("Cycle %d: pressure=%.4f MPa, change=%.4f MPa (expected=%.4f MPa)\n",
               i + 1, output.rampedPressure, actualChange, expectedChange);

        if (fabs(actualChange - expectedChange) > 0.0001) {
            printf("✗ FAIL: Change mismatch!\n");
        }

        previousPressure = output.rampedPressure;
    }

    if (fabs(previousPressure - (startPressure + rampRate * 10 * dt)) < 0.001) {
        printf("✓ PASS: Ramp rate is accurate\n");
    }
}

/**
 * Test zero ramp rate (零斜坡率测试)
 */
void test_zero_ramp_rate(void) {
    printf("\n=== Test 5: Zero Ramp Rate (立即跳转) ===\n");

    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    HYD_RampController_Init(&controller, 5.0, 0.0);

    input.targetPressure = 10.0;
    input.rampRate = 0.0;  // Zero ramp rate should cause immediate jump
    input.currentTime = 0.01;

    printf("Initial pressure: %.2f MPa\n", controller.rampedPressure);
    printf("Target pressure: %.2f MPa\n", input.targetPressure);
    printf("Ramp rate: 0.0 (immediate jump)\n");

    HYD_RampController_Execute(&controller, &input, &output);

    printf("Result: %.2f MPa\n", output.rampedPressure);

    if (fabs(output.rampedPressure - input.targetPressure) < EPSILON) {
        printf("✓ PASS: Zero ramp rate causes immediate jump to target\n");
    } else {
        printf("✗ FAIL: Expected immediate jump\n");
    }
}

int main(void) {
    printf("╔════════════════════════════════════════╗\n");
    printf("║  Pressure Ramp Controller Test Suite  ║\n");
    printf("╚════════════════════════════════════════╝\n");

    test_pressure_increase();
    test_pressure_decrease();
    test_ramp_symmetry();
    test_ramp_rate_calculation();
    test_zero_ramp_rate();

    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  All tests completed                  ║\n");
    printf("╚════════════════════════════════════════╝\n");

    return 0;
}
