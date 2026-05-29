#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pressure_controller.h"

static HYD_MotionSegment make_pressure_segment(void) {
    HYD_MotionSegment segment = {0};
    segment.segmentTag = 1;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetFlow = 3.0;
    segment.targetPressure = 12.0;
    segment.maxFlow = 10.0;
    segment.duration = 1.0;
    segment.pressureTolerance = 0.2;
    segment.timeoutLimit = 2.0;
    segment.pressureRampRate = 5.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    return segment;
}

static void assert_rbf_pid_internal_limits(const HYD_PressureControllerState* state) {
    assert(state->rbfPid.KP >= state->rbfPid.min_KP - 1e-6f);
    assert(state->rbfPid.KP <= state->rbfPid.max_KP + 1e-6f);
    assert(state->rbfPid.KI >= state->rbfPid.min_KI - 1e-6f);
    assert(state->rbfPid.KI <= state->rbfPid.max_KI + 1e-6f);
    assert(state->rbfPid.KD >= state->rbfPid.min_KD - 1e-6f);
    assert(state->rbfPid.KD <= state->rbfPid.max_KD + 1e-6f);
}

static void test_legacy_default_strategy_matches_fixed_p_behavior(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing legacy default pressure strategy behavior...\n");
    segment = make_pressure_segment();
    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 70.0;
    input.measuredPressure = 50.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_P);
    assert(fabs(output.proportionalTerm - 30.0) < 0.001);
    assert(fabs(output.outputFlow - segment.maxFlow) < 0.001);
    assert(!output.trackingApplied);
    assert(output.saturated);
    printf("✓ Legacy default pressure strategy test passed\n");
}

static void test_pi_strategy_accumulates_integral_output(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing PI pressure strategy integral accumulation...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 5.0;

    HYD_PressureController_InitState(&state, 8.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.0;
    input.measuredPressure = 8.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    input.timestamp = 1.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output0.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(fabs(output0.integralTerm) < 0.001);
    assert(fabs(output0.outputFlow - 4.0) < 0.001);
    assert(fabs(output1.integralTerm - 2.0) < 0.001);
    assert(fabs(output1.outputFlow - 6.0) < 0.001);
    assert(output1.outputFlow > output0.outputFlow);
    printf("✓ PI pressure strategy integral accumulation test passed\n");
}

static void test_deadband_filter_and_anti_windup(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing pressure deadband, filter, and anti-windup...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 1.0;
    segment.pressureKi = 2.0;
    segment.pressureIntegralLimit = 1.0;
    segment.pressureDeadband = 0.5;
    segment.pressureFilterAlpha = 0.25;
    segment.maxFlow = 4.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.2;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.5;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    assert(fabs(output0.filteredPressure - 10.0) < 0.001);
    assert(fabs(output0.controlError) < 0.001);
    assert(fabs(output0.outputFlow - segment.targetFlow) < 0.001);

    input.targetPressure = 20.0;
    input.measuredPressure = 14.0;
    input.timestamp = 1.5;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.filteredPressure > 10.0);
    assert(output1.filteredPressure < 14.0);
    assert(output1.outputFlow <= segment.maxFlow + 0.001);
    assert(output1.saturated);
    assert(output1.integralTerm <= segment.pressureIntegralLimit + 0.001);
    printf("✓ Pressure deadband/filter/anti-windup test passed\n");
}

static void test_output_tracking_back_calculates_integral_term(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing bumpless tracking / integral back-calculation...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);
    HYD_PressureController_RequestTracking(&state, 6.5);

    input.targetPressure = 10.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    input.timestamp = 1.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output0.trackingApplied);
    assert(fabs(output0.integralTerm - 3.5) < 0.001);
    assert(fabs(output0.outputFlow - 6.5) < 0.001);
    assert(fabs(output1.integralTerm - 3.5) < 0.001);
    assert(fabs(output1.outputFlow - 6.5) < 0.001);
    printf("✓ Bumpless tracking / integral back-calculation test passed\n");
}

static void test_pid_derivative_uses_measurement_rate_and_filter(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing PID derivative measurement filtering and kick suppression...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PID;
    segment.pressureKp = 0.5;
    segment.pressureKi = 0.0;
    segment.pressureKd = 1.0;
    segment.pressureDerivativeFilterAlpha = 0.25;
    segment.maxFlow = 20.0;

    HYD_PressureController_InitState(&state, 10.0, 5.0, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = 5.0;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 1.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    input.measuredPressure = 14.0;
    input.timestamp = 2.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(fabs(output0.derivativeTerm) < 0.001);
    assert(fabs(output0.outputFlow - 10.0) < 0.001);
    assert(fabs(output1.filteredPressureRate - 1.0) < 0.001);
    assert(fabs(output1.derivativeTerm + 1.0) < 0.001);
    assert(fabs(output1.outputFlow - 7.0) < 0.001);
    printf("✓ PID derivative measurement/filter test passed\n");
}

static void test_strategy_switch_uses_descriptor_based_tracking(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing strategy-switch tracking via descriptor resolution...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.5;
    segment.maxFlow = 10.0;
    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 14.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);
    assert(fabs(output0.outputFlow - 5.0) < 0.001);
    assert(output0.appliedStrategy == HYD_PRESSURE_CONTROLLER_P);

    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 1.0;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.trackingApplied);
    assert(output1.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(fabs(output1.integralTerm + 2.0) < 0.001);
    assert(fabs(output1.outputFlow - output0.outputFlow) < 0.001);
    printf("✓ Strategy-switch tracking test passed\n");
}

static void test_rbf_pid_strategy_executes_within_limits_and_adapts(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_REAL feedback;
    int step;

    printf("Testing adaptive RBF-PID pressure strategy integration...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.targetFlow = 0.0;
    segment.maxFlow = 1.0;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    feedback = 0.0;

    for (step = 0; step < 20; ++step) {
        input.targetPressure = 100.0;
        input.measuredPressure = feedback;
        input.feedforwardFlow = 0.0;
        input.outputMin = 0.0;
        input.outputMax = segment.maxFlow;
        input.flowToPumpSpeedGain = 20.0;
        input.pumpSpeedLimit = 1800.0;
        input.timestamp = (step + 1) * 0.01;

        HYD_PressureController_Execute(&segment, &state, &input, &output);

        assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
        assert(output.outputFlow >= -1e-6);
        assert(output.outputFlow <= segment.maxFlow + 1e-6);
        assert(output.unsaturatedOutputFlow >= MIN_OUTPUT - 1e-6);
        assert(output.samplingPeriod > 0.0);
        assert(output.adaptiveActive);
        assert(state.activeStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
        assert(state.rbfInitialized);
        assert(state.rbfPid.Status == 1);
        assert(state.rbfPid.TuneResult == 66);
        assert_rbf_pid_internal_limits(&state);

        feedback += output.outputFlow * 5.0;
        if (feedback > 100.0) {
            feedback = 100.0;
        }
    }

    assert(feedback > 1.0);
    printf("✓ Adaptive RBF-PID pressure strategy integration test passed\n");
}

static void test_rbf_pid_strategy_uses_segment_level_tuning_profile(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing RBF-PID segment-level tuning profile mapping...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.targetFlow = 0.25;
    segment.maxFlow = 1.2;
    segment.pressureRbfConfig.minKp = 0.81;
    segment.pressureRbfConfig.maxKp = 0.82;
    segment.pressureRbfConfig.minKi = 0.019;
    segment.pressureRbfConfig.maxKi = 0.021;
    segment.pressureRbfConfig.minKd = 1.24;
    segment.pressureRbfConfig.maxKd = 1.26;
    segment.pressureRbfConfig.etaW = 0.14;
    segment.pressureRbfConfig.etaC = 0.15;
    segment.pressureRbfConfig.etaB = 0.16;
    segment.pressureRbfConfig.etaP = 0.11;
    segment.pressureRbfConfig.etaI = 0.12;
    segment.pressureRbfConfig.etaD = 0.13;

    HYD_PressureController_InitState(&state, 5.0, segment.targetFlow, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 5.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.flowToPumpSpeedGain = 20.0;
    input.pumpSpeedLimit = 1800.0;
    input.timestamp = 0.02;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
    assert(output.adaptiveActive);
    assert(fabs(output.feedforwardFlow - segment.targetFlow) < 0.001);
    assert(fabs(output.samplingPeriod - 0.02) < 0.001);
    assert(state.rbfInitialized);
    assert(fabsf(state.rbfPid.min_KP - 0.81f) < 1e-6f);
    assert(fabsf(state.rbfPid.max_KP - 0.82f) < 1e-6f);
    assert(fabsf(state.rbfPid.min_KI - 0.019f) < 1e-6f);
    assert(fabsf(state.rbfPid.max_KI - 0.021f) < 1e-6f);
    assert(fabsf(state.rbfPid.min_KD - 1.24f) < 1e-6f);
    assert(fabsf(state.rbfPid.max_KD - 1.26f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_w - 0.14f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_c - 0.15f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_b - 0.16f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_p - 0.11f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_i - 0.12f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_d - 0.13f) < 1e-6f);
    assert(output.adaptiveKp >= 0.81 - 1e-6 && output.adaptiveKp <= 0.82 + 1e-6);
    assert(output.adaptiveKi >= 0.019 - 1e-6 && output.adaptiveKi <= 0.021 + 1e-6);
    assert(output.adaptiveKd >= 1.24 - 1e-6 && output.adaptiveKd <= 1.26 + 1e-6);
    assert(output.outputFlow >= -1e-6);
    assert(output.outputFlow <= segment.maxFlow + 1e-6);
    printf("✓ RBF-PID segment-level tuning profile test passed\n");
}

static void test_rbf_pid_strategy_switch_tracks_previous_output_bumplessly(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing RBF-PID strategy switch bumpless tracking...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;
    segment.maxFlow = 10.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 14.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    assert(output0.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(fabs(output0.outputFlow - 5.0) < 0.001);

    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    input.targetPressure = 10.0;
    input.measuredPressure = 10.0;
    input.flowToPumpSpeedGain = 20.0;
    input.pumpSpeedLimit = 1800.0;
    input.timestamp = 0.1;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.trackingApplied);
    assert(output1.appliedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
    assert(fabs(output1.outputFlow - output0.outputFlow) < 0.05);
    assert(state.activeStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
    assert(state.rbfInitialized);
    assert(state.rbfPid.Status == 1);
    assert(state.rbfPid.TuneResult == 66);
    printf("✓ RBF-PID strategy switch bumpless tracking test passed\n");
}

/* ---- P1-5: Boundary and edge-case tests ---- */

static void test_cross_controller_switch_seeds_rbf_within_clamp_window(void) {
    HYD_MotionSegment segmentA;
    HYD_MotionSegment segmentB;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing cross-controller PI->RBF switch clamp seeding...\n");

    /* Segment A: PI controller, brief working state. */
    segmentA = make_pressure_segment();
    segmentA.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segmentA.pressureKp = 0.5;
    segmentA.pressureKi = 0.1;
    segmentA.pressureIntegralLimit = 5.0;
    segmentA.maxFlow = 30.0;

    HYD_PressureController_InitState(&state, 18.0, 5.0, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 18.0;
    input.feedforwardFlow = 5.0;
    input.outputMin = 0.0;
    input.outputMax = 30.0;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segmentA, &state, &input, &output);
    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);

    /* Segment B: RBF controller with window [1.5, 2.0] outside Init default KP=0.8 */
    segmentB = make_pressure_segment();
    segmentB.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segmentB.targetPressure = 30.0;
    segmentB.maxFlow = 30.0;
    segmentB.pressureCeiling = 50.0;
    segmentB.pressureRbfConfig.minKp = 1.5;
    segmentB.pressureRbfConfig.maxKp = 2.0;
    segmentB.pressureRbfConfig.minKi = 0.005;
    segmentB.pressureRbfConfig.maxKi = 0.050;
    segmentB.pressureRbfConfig.minKd = 0.5;
    segmentB.pressureRbfConfig.maxKd = 2.0;

    input.targetPressure = 30.0;
    input.measuredPressure = 25.0;
    input.flowToPumpSpeedGain = 20.0;
    input.pumpSpeedLimit = 1800.0;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segmentB, &state, &input, &output);

    /* After PI->RBF switch with custom window, all 3 gains must be clamped into the new window. */
    assert(state.rbfPid.KP >= 1.5f - 1e-4f);
    assert(state.rbfPid.KP <= 2.0f + 1e-4f);
    assert(state.rbfPid.KI >= 0.005f - 1e-4f);
    assert(state.rbfPid.KI <= 0.050f + 1e-4f);
    assert(state.rbfPid.KD >= 0.5f - 1e-4f);
    assert(state.rbfPid.KD <= 2.0f + 1e-4f);
    /* Output must remain within [outputMin, outputMax] — no first-step spike. */
    assert(output.outputFlow >= 0.0);
    assert(output.outputFlow <= input.outputMax + 1e-4);
    printf("✓ Cross-controller PI->RBF clamp seeding test passed (KP=%.3f KI=%.4f KD=%.3f)\n",
           state.rbfPid.KP, state.rbfPid.KI, state.rbfPid.KD);
}

/* ---- P1-5: Boundary and edge-case tests ---- */

static void test_pi_integral_saturates_and_back_calculates_on_recovery(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int step;

    printf("Testing PI integral saturation and back-calculation on recovery...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 2.0;
    segment.pressureIntegralLimit = 3.0;
    segment.maxFlow = 5.0;

    HYD_PressureController_InitState(&state, 5.0, segment.targetFlow, 0.0);

    /* Step 1: Apply sustained error to saturate integral */
    input.targetPressure = 20.0;
    input.measuredPressure = 5.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;

    for (step = 0; step < 10; step++) {
        input.timestamp = (step + 1) * 0.01;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
    }

    /* Integral should be clamped at the limit */
    assert(output.saturated);
    assert(output.integralTerm <= segment.pressureIntegralLimit + 0.001);
    assert(output.outputFlow <= segment.maxFlow + 0.001);

    /* Step 2: Recovery — pressure catches up, error reverses */
    input.targetPressure = 10.0;
    input.measuredPressure = 15.0;
    input.timestamp = 0.2;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    /* After recovery the output should decrease significantly;
     * back-calculation prevents integral windup from holding output high */
    assert(output.outputFlow < segment.maxFlow);
    assert(!output.saturated || output.integralTerm < segment.pressureIntegralLimit);
    printf("✓ PI integral saturation and back-calculation test passed\n");
}

static void test_p_to_pi_strategy_switch_preinitializes_integral(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output0;
    HYD_PressureControllerOutput output1;

    printf("Testing P-to-PI strategy switch integral pre-initialization...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 1.0;
    segment.maxFlow = 20.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    /* Run in P mode first */
    input.targetPressure = 14.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output0);

    assert(output0.appliedStrategy == HYD_PRESSURE_CONTROLLER_P);
    assert(fabs(output0.integralTerm) < 0.001);

    /* Switch to PI — tracking should pre-initialize integral for bumpless transfer */
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;
    input.timestamp = 0.1;
    HYD_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(output1.trackingApplied);
    /* Integral should be non-zero after tracking initialization */
    assert(fabs(output1.integralTerm) > 0.001);
    /* Output should be continuous (no bump) */
    assert(fabs(output1.outputFlow - output0.outputFlow) < 1.0);
    printf("✓ P-to-PI strategy switch integral pre-initialization test passed\n");
}

static void test_long_run_integral_stability(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int step;
    HYD_REAL lastOutput = 0.0;
    HYD_REAL maxOscillation = 0.0;

    printf("Testing long-run PI controller stability...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 0.5;
    segment.pressureIntegralLimit = 10.0;
    segment.maxFlow = 20.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 12.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;

    for (step = 0; step < 1000; step++) {
        input.timestamp = (step + 1) * 0.001;

        /* Simulate simple plant: pressure rises with flow */
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        input.measuredPressure += (output.outputFlow - input.measuredPressure) * 0.005;

        if (step > 100) {
            HYD_REAL oscillation = fabs(output.outputFlow - lastOutput);
            if (oscillation > maxOscillation) {
                maxOscillation = oscillation;
            }
        }
        lastOutput = output.outputFlow;
    }

    /* After 1000 steps, controller should have settled — no large oscillations */
    assert(maxOscillation < 1.0);
    /* Integral should be within bounds */
    assert(output.integralTerm >= -0.001);
    assert(output.integralTerm <= segment.pressureIntegralLimit + 0.001);
    printf("✓ Long-run PI controller stability test passed (max oscillation=%.4f)\n", maxOscillation);
}

static void test_small_kp_produces_proportional_output(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_REAL expectedProportional;
    HYD_REAL expectedOutput;

    printf("Testing small-Kp P controller produces proportional output...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.01;  /* Very small but nonzero gain */
    segment.maxFlow = 20.0;

    HYD_PressureController_InitState(&state, 5.0, segment.targetFlow, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 5.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    /* With Kp=0.01 and error=15 MPa, proportional term = 0.01 * 15 = 0.15 */
    expectedProportional = segment.pressureKp * (20.0 - 5.0);
    assert(fabs(output.proportionalTerm - expectedProportional) < 0.001);
    /* Output should be feedforward + proportional term */
    expectedOutput = segment.targetFlow + expectedProportional;
    assert(fabs(output.outputFlow - expectedOutput) < 0.001);
    assert(!output.saturated);
    printf("✓ Small-Kp P controller test passed\n");
}

static void test_default_filter_applies_smoothing(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing default filter alpha applies smoothing...\n");
    segment = make_pressure_segment();
    segment.pressureFilterAlpha = 0.0;
    segment.pressureDerivativeFilterAlpha = 0.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    /* With default alpha < 1.0, filtered pressure should be between
     * previous (10.0) and measured (10.0) - for first call with
     * alpha=0.1, filtered = 10.0 + 0.1*(10.0-10.0) = 10.0 */
    assert(fabs(output.filteredPressure - 10.0) < 0.001);

    /* Step change: measured jumps from 10 to 15.
     * With alpha=0.1: filtered = 10.0 + 0.1*(15.0-10.0) = 10.5 */
    input.measuredPressure = 15.0;
    input.timestamp = 0.02;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.filteredPressure > 10.0);
    assert(output.filteredPressure < 15.0);
    /* filtered should be ~10.5 with alpha=0.1 */
    assert(fabs(output.filteredPressure - 10.5) < 0.01);
    printf("✓ Default filter alpha smoothing test passed\n");
}

static void test_gain_scheduling_with_error_magnitude(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput outputLow;
    HYD_PressureControllerOutput outputHigh;

    printf("Testing gain scheduling with error magnitude...\n");

    /* Without scheduling: Kp constant regardless of error */
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.5;
    segment.pressureKpHigh = 0.0;
    segment.maxFlow = 20.0;

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    /* Small error */
    input.targetPressure = 11.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segment, &state, &input, &outputLow);
    assert(fabs(outputLow.proportionalTerm - 0.5) < 0.001);

    /* Large error - same Kp */
    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.timestamp = 0.02;
    HYD_PressureController_Execute(&segment, &state, &input, &outputHigh);
    assert(fabs(outputHigh.proportionalTerm - 5.0) < 0.001);

    /* With scheduling: Kp varies by error magnitude */
    segment.pressureKp = 0.2;       /* low-error gain */
    segment.pressureKpHigh = 1.0;   /* high-error gain */
    segment.pressureGainBand = 0.2; /* 20% error threshold */

    HYD_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    /* Small error (1 MPa / 11 ≈ 9%): closer to low gain */
    input.targetPressure = 11.0;
    input.measuredPressure = 10.0;
    input.timestamp = 0.01;
    HYD_PressureController_Execute(&segment, &state, &input, &outputLow);
    /* Effective Kp should be closer to 0.2 than 1.0 */
    /* errorRatio = 1/11 ≈ 0.09, fraction = 0.09/0.2 = 0.45 */
    /* kpEff = 0.2 + 0.45*(1.0-0.2) = 0.2 + 0.36 = 0.56 */
    /* proportionalTerm = 0.56 * 1.0 = 0.56 */
    assert(outputLow.proportionalTerm > 0.2);
    assert(outputLow.proportionalTerm < 0.8);

    /* Large error (10 MPa / 20 = 50%): closer to high gain */
    segment.pressureKpHigh = 1.0;
    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.timestamp = 0.02;
    HYD_PressureController_Execute(&segment, &state, &input, &outputHigh);
    /* errorRatio = 10/20 = 0.5, fraction = min(0.5/0.2, 1.0) = 1.0 */
    /* kpEff = 0.2 + 1.0*(1.0-0.2) = 1.0 */
    /* proportionalTerm = 1.0 * 10 = 10.0 */
    assert(outputHigh.proportionalTerm > 5.0);
    assert(fabs(outputHigh.proportionalTerm - 10.0) < 0.01);
    printf("✓ Gain scheduling with error magnitude test passed\n");
}

int main(void) {
    printf("Running PressureController tests...\n\n");

    test_legacy_default_strategy_matches_fixed_p_behavior();
    test_pi_strategy_accumulates_integral_output();
    test_deadband_filter_and_anti_windup();
    test_output_tracking_back_calculates_integral_term();
    test_pid_derivative_uses_measurement_rate_and_filter();
    test_strategy_switch_uses_descriptor_based_tracking();
    test_rbf_pid_strategy_executes_within_limits_and_adapts();
    test_rbf_pid_strategy_uses_segment_level_tuning_profile();
    test_rbf_pid_strategy_switch_tracks_previous_output_bumplessly();
    test_cross_controller_switch_seeds_rbf_within_clamp_window();
    test_pi_integral_saturates_and_back_calculates_on_recovery();
    test_p_to_pi_strategy_switch_preinitializes_integral();
    test_long_run_integral_stability();
    test_small_kp_produces_proportional_output();
    test_default_filter_applies_smoothing();
    test_gain_scheduling_with_error_magnitude();

    printf("\n✅ All PressureController tests passed successfully!\n");
    return 0;
}
