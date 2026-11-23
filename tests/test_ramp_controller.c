/**
 * Test program to verify pressure ramp controller behavior
 * Tests both pressure increase and decrease scenarios
 */

#include "../include/ramp_controller.h"
#include "../include/common_types.h"
#include <assert.h>
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

    // Initialize: current pressure = 2.0 bar
    HYD_RampController_Init(&controller, 2.0, 0.0);

    // Set target: ramp from 2.0 to 10.0 bar at 5.0 bar/s
    input.targetPressure = 10.0;
    input.rampRate = 5.0;
    input.currentTime = 0.0;

    printf("Initial pressure: %.2f bar\n", controller.rampedPressure);
    printf("Target pressure: %.2f bar\n", input.targetPressure);
    printf("Ramp rate: %.2f bar/s\n", input.rampRate);
    printf("\n");

    // Simulate 2 seconds of execution
    HYD_TIME dt = 0.01;  // 10ms cycle
    for (int i = 0; i < 200; i++) {
        input.currentTime = (i + 1) * dt;
        HYD_RampController_Execute(&controller, &input, &output);

        // Print every 50 cycles (0.5s)
        if ((i + 1) % 50 == 0) {
            printf("t=%.2fs: pressure=%.2f bar\n", input.currentTime, output.rampedPressure);
        }
    }

    printf("Final pressure: %.2f bar\n", output.rampedPressure);

    // Verification: after 2 seconds at 5 bar/s, pressure should increase by 10 bar
    // Expected: 2.0 + 5.0 * 2.0 = 12.0, but clamped to target 10.0
    HYD_REAL expected = input.targetPressure;
    if (fabs(output.rampedPressure - expected) < EPSILON) {
        printf("✓ PASS: Final pressure %.2f bar equals target %.2f bar\n", output.rampedPressure, expected);
    } else {
        printf("✗ FAIL: Expected %.2f bar, got %.2f bar\n", expected, output.rampedPressure);
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

    // Initialize: current pressure = 10.0 bar
    HYD_RampController_Init(&controller, 10.0, 0.0);

    // Set target: ramp from 10.0 to 2.0 bar at 5.0 bar/s
    input.targetPressure = 2.0;
    input.rampRate = 5.0;
    input.currentTime = 0.0;

    printf("Initial pressure: %.2f bar\n", controller.rampedPressure);
    printf("Target pressure: %.2f bar\n", input.targetPressure);
    printf("Ramp rate: %.2f bar/s\n", input.rampRate);
    printf("\n");

    // Simulate 2 seconds of execution
    HYD_TIME dt = 0.01;  // 10ms cycle
    for (int i = 0; i < 200; i++) {
        input.currentTime = (i + 1) * dt;
        HYD_RampController_Execute(&controller, &input, &output);

        // Print every 50 cycles (0.5s)
        if ((i + 1) % 50 == 0) {
            printf("t=%.2fs: pressure=%.2f bar\n", input.currentTime, output.rampedPressure);
        }
    }

    printf("Final pressure: %.2f bar\n", output.rampedPressure);

    // Verification: after 2 seconds at 5 bar/s, pressure should decrease by 10 bar
    // Expected: 10.0 - 5.0 * 2.0 = 0.0, but clamped to target 2.0
    HYD_REAL expected = input.targetPressure;
    if (fabs(output.rampedPressure - expected) < EPSILON) {
        printf("✓ PASS: Final pressure %.2f bar equals target %.2f bar\n", output.rampedPressure, expected);
    } else {
        printf("✗ FAIL: Expected %.2f bar, got %.2f bar\n", expected, output.rampedPressure);
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

    // Test increase: 5.0 -> 7.0 bar
    printf("\nPart A: Increase from %.1f to %.1f bar at %.1f bar/s\n",
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
    printf("Result: %.4f bar\n", pressureAfterIncrease);

    // Test decrease: 7.0 -> 5.0 bar
    printf("\nPart B: Decrease from %.1f to %.1f bar at %.1f bar/s\n",
           pressureAfterIncrease, startPressure, rampRate);
    HYD_RampController_Init(&controller, pressureAfterIncrease, 0.0);
    input.targetPressure = startPressure;
    input.rampRate = rampRate;
    input.currentTime = 0.0;

    for (int i = 0; i < steps; i++) {
        input.currentTime = (i + 1) * 0.01;
        HYD_RampController_Execute(&controller, &input, &output);
    }
    printf("Result: %.4f bar\n", output.rampedPressure);

    // Verification: should return to start value
    if (fabs(output.rampedPressure - startPressure) < 0.01) {
        printf("✓ PASS: Ramp is symmetric (returned to %.2f bar)\n", startPressure);
    } else {
        printf("✗ FAIL: Expected %.2f bar, got %.2f bar\n", startPressure, output.rampedPressure);
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

    printf("Testing pressure increase from %.1f bar with rate %.1f bar/s\n",
           startPressure, rampRate);
    printf("Cycle time: %.3f s\n", dt);
    printf("Expected change per cycle: %.4f bar\n", rampRate * dt);
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

        printf("Cycle %d: pressure=%.4f bar, change=%.4f bar (expected=%.4f bar)\n",
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

    printf("Initial pressure: %.2f bar\n", controller.rampedPressure);
    printf("Target pressure: %.2f bar\n", input.targetPressure);
    printf("Ramp rate: 0.0 (immediate jump)\n");

    HYD_RampController_Execute(&controller, &input, &output);

    printf("Result: %.2f bar\n", output.rampedPressure);

    if (fabs(output.rampedPressure - input.targetPressure) < EPSILON) {
        printf("✓ PASS: Zero ramp rate causes immediate jump to target\n");
    } else {
        printf("✗ FAIL: Expected immediate jump\n");
    }
}

/* ===================== Profiled reference (spec 2026-11-23 §4.2, T1) ===================== */

typedef struct {
    HYD_REAL maxRef;
    HYD_REAL minRef;
    HYD_REAL maxAbsRate;
    HYD_REAL maxAbsRateStep;
    int arrivalStep; /* first step with ref == target and rate == 0, -1 if never */
} ProfileRun;

static const HYD_RampProfileLimits kProfileLimits = {600.0f, 300.0f, 0.12f};

static void run_profile(HYD_RampController* controller, HYD_REAL target, HYD_REAL rampMs,
                        const HYD_RampProfileLimits* limits, int steps, ProfileRun* run) {
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;
    HYD_REAL previousRate = controller->profileRate;
    int k;

    run->maxRef = controller->rampedPressure;
    run->minRef = controller->rampedPressure;
    run->maxAbsRate = 0.0f;
    run->maxAbsRateStep = 0.0f;
    run->arrivalStep = -1;
    input.targetPressure = target;
    input.rampRate = rampMs;
    for (k = 1; k <= steps; ++k) {
        input.currentTime = controller->lastTimestamp + 0.001f;
        HYD_RampController_ExecuteProfiledWithDt(controller, &input, limits, 0.001f, &output);
        if (output.rampedPressure > run->maxRef) run->maxRef = output.rampedPressure;
        if (output.rampedPressure < run->minRef) run->minRef = output.rampedPressure;
        if (fabs(output.rampedPressureRate) > run->maxAbsRate) {
            run->maxAbsRate = (HYD_REAL)fabs(output.rampedPressureRate);
        }
        if (fabs(output.rampedPressureRate - previousRate) > run->maxAbsRateStep) {
            run->maxAbsRateStep = (HYD_REAL)fabs(output.rampedPressureRate - previousRate);
        }
        previousRate = output.rampedPressureRate;
        if (run->arrivalStep < 0 && output.rampedPressure == target &&
            output.rampedPressureRate == 0.0f) {
            run->arrivalStep = k;
        }
    }
}

static void test_profile_rise_is_bounded_and_lands_exactly(void) {
    HYD_RampController controller;
    ProfileRun run;
    /* a = 600 / 0.12 = 5000 bar/s^2: accelerate 120 ms (36 bar), cruise
     * 28 bar at 600 bar/s (47 ms), brake 120 ms (36 bar) -> about 287 ms. */
    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.maxRef <= 100.0f);
    assert(run.maxAbsRate <= 600.0f + 1.0e-3f);
    assert(run.maxAbsRateStep <= 3.0f * 5000.0f * 0.001f);
    assert(run.arrivalStep >= 270 && run.arrivalStep <= 300);
    assert(controller.rampedPressure == 100.0f);
    printf("✓ profile rise: arrival %d ms, max slope %.1f bar/s\n",
           run.arrivalStep, (double)run.maxAbsRate);
}

static void test_profile_fall_uses_fall_bound(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 140.0f, 0.0f);
    run_profile(&controller, 80.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.minRef >= 80.0f);
    assert(run.maxAbsRate <= 300.0f + 1.0e-3f);
    assert(run.maxAbsRate >= 290.0f);
    assert(run.arrivalStep > 0);
    assert(controller.rampedPressure == 80.0f);
    printf("✓ profile fall: arrival %d ms, max slope %.1f bar/s\n",
           run.arrivalStep, (double)run.maxAbsRate);
}

static void test_profile_slower_user_time_wins(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 1000.0f, &kProfileLimits, 1500, &run);

    assert(run.maxAbsRate <= 100.0f + 1.0e-3f);
    assert(run.arrivalStep >= 1000 && run.arrivalStep <= 1100);
    printf("✓ profile 1000 ms transition: arrival %d ms\n", run.arrivalStep);
}

static void test_profile_reversal_stops_reference(void) {
    HYD_RampController controller;
    ProfileRun run;
    HYD_REAL atSwitch;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 140.0f, 20.0f, &kProfileLimits, 200, &run);
    atSwitch = controller.rampedPressure;
    assert(atSwitch > 70.0f);
    assert(controller.profileRate > 100.0f);

    run_profile(&controller, 60.0f, 20.0f, &kProfileLimits, 1000, &run);
    assert(run.maxRef <= atSwitch);
    assert(run.minRef >= 60.0f);
    assert(controller.rampedPressure == 60.0f);
    printf("✓ profile reversal: stopped at %.2f bar, landed on 60\n", (double)atSwitch);
}

static void test_profile_retarget_same_direction_is_continuous(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 100, &run);
    run_profile(&controller, 120.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.maxAbsRateStep <= 3.0f * 5000.0f * 0.001f);
    assert(run.maxRef <= 120.0f);
    assert(controller.rampedPressure == 120.0f);
    printf("✓ profile retarget 100 -> 120 while rising is continuous\n");
}

static void test_profile_zero_bounds_snap_to_target(void) {
    HYD_RampController controller;
    HYD_RampProfileLimits zero = {0.0f, 0.0f, 0.12f};
    ProfileRun run;

    HYD_RampController_Init(&controller, 10.0f, 0.0f);
    run_profile(&controller, 50.0f, 20.0f, &zero, 1, &run);
    assert(controller.rampedPressure == 50.0f);
    assert(run.maxAbsRate == 0.0f);
    printf("✓ profile with zero bounds snaps to the target\n");
}

static void test_legacy_reports_zero_rate(void) {
    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    input.targetPressure = 100.0f;
    input.rampRate = 100.0f;
    input.currentTime = 0.001f;
    output.rampedPressureRate = 123.0f;
    HYD_RampController_ExecuteWithDt(&controller, &input, 0.001f, &output);
    assert(output.rampedPressure > 0.0f && output.rampedPressure < 100.0f);
    assert(output.rampedPressureRate == 0.0f);
    printf("✓ legacy ramp reports zero slope\n");
}

static void test_profile_then_legacy_continues_to_target(void) {
    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;
    ProfileRun run;
    HYD_REAL atSwitch;
    int k;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 30, &run);
    atSwitch = controller.rampedPressure;
    assert(atSwitch > 0.0f && atSwitch < 100.0f);
    assert(controller.profileLatched);

    /* Parameter 37 set to 0 mid-ramp: the time-based ramp takes over with
     * the same target and transition time. */
    input.targetPressure = 100.0f;
    input.rampRate = 20.0f;
    for (k = 0; k < 30; ++k) {
        input.currentTime = controller.lastTimestamp + 0.001f;
        HYD_RampController_ExecuteWithDt(&controller, &input, 0.001f, &output);
    }
    assert(output.rampedPressure == 100.0f);
    assert(!controller.profileLatched);
    assert(output.rampedPressureRate == 0.0f);
    printf("✓ profile -> legacy at %.2f bar continues to 100\n", (double)atSwitch);
}

static void test_profile_nan_dt_keeps_reference(void) {
    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;
    ProfileRun run;
    HYD_REAL before;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 30, &run);
    before = controller.rampedPressure;
    assert(before > 0.0f && before < 100.0f);

    input.targetPressure = 100.0f;
    input.rampRate = 20.0f;
    input.currentTime = controller.lastTimestamp + 0.001f;
    HYD_RampController_ExecuteProfiledWithDt(&controller, &input, &kProfileLimits,
                                             (HYD_TIME)NAN, &output);
    assert(isfinite(controller.rampedPressure));
    assert(isfinite(controller.profileRate));
    assert(controller.rampedPressure == before);
    assert(output.rampedPressure == before);
    printf("✓ profile ignores NaN dt\n");
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
    test_profile_rise_is_bounded_and_lands_exactly();
    test_profile_fall_uses_fall_bound();
    test_profile_slower_user_time_wins();
    test_profile_reversal_stops_reference();
    test_profile_retarget_same_direction_is_continuous();
    test_profile_zero_bounds_snap_to_target();
    test_legacy_reports_zero_rate();
    test_profile_then_legacy_continues_to_target();
    test_profile_nan_dt_keeps_reference();

    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  All tests completed                  ║\n");
    printf("╚════════════════════════════════════════╝\n");

    return 0;
}
