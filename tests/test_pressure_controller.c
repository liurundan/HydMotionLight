#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pressure_controller.h"

static HDY_MotionSegment make_pressure_segment(void) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, "PressureSegment", HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HDY_END_TIME;
    segment.direction = HDY_DIRECTION_HOLD;
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

static void assert_rbf_pid_internal_limits(const HDY_PressureControllerState* state) {
    assert(state->rbfPid.KP >= state->rbfPid.min_KP - 1e-6f);
    assert(state->rbfPid.KP <= state->rbfPid.max_KP + 1e-6f);
    assert(state->rbfPid.KI >= state->rbfPid.min_KI - 1e-6f);
    assert(state->rbfPid.KI <= state->rbfPid.max_KI + 1e-6f);
    assert(state->rbfPid.KD >= state->rbfPid.min_KD - 1e-6f);
    assert(state->rbfPid.KD <= state->rbfPid.max_KD + 1e-6f);
}

static void test_legacy_default_strategy_matches_fixed_p_behavior(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output;

    printf("Testing legacy default pressure strategy behavior...\n");
    segment = make_pressure_segment();
    HDY_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 70.0;
    input.measuredPressure = 50.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HDY_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.appliedStrategy == HDY_PRESSURE_CONTROLLER_P);
    assert(fabs(output.proportionalTerm - 30.0) < 0.001);
    assert(fabs(output.outputFlow - segment.maxFlow) < 0.001);
    assert(!output.trackingApplied);
    assert(output.saturated);
    printf("✓ Legacy default pressure strategy test passed\n");
}

static void test_pi_strategy_accumulates_integral_output(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing PI pressure strategy integral accumulation...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 5.0;

    HDY_PressureController_InitState(&state, 8.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.0;
    input.measuredPressure = 8.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);

    input.timestamp = 1.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output0.appliedStrategy == HDY_PRESSURE_CONTROLLER_PI);
    assert(fabs(output0.integralTerm) < 0.001);
    assert(fabs(output0.outputFlow - 4.0) < 0.001);
    assert(fabs(output1.integralTerm - 2.0) < 0.001);
    assert(fabs(output1.outputFlow - 6.0) < 0.001);
    assert(output1.outputFlow > output0.outputFlow);
    printf("✓ PI pressure strategy integral accumulation test passed\n");
}

static void test_deadband_filter_and_anti_windup(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing pressure deadband, filter, and anti-windup...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 1.0;
    segment.pressureKi = 2.0;
    segment.pressureIntegralLimit = 1.0;
    segment.pressureDeadband = 0.5;
    segment.pressureFilterAlpha = 0.25;
    segment.maxFlow = 4.0;

    HDY_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.2;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.5;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);

    assert(fabs(output0.filteredPressure - 10.0) < 0.001);
    assert(fabs(output0.controlError) < 0.001);
    assert(fabs(output0.outputFlow - segment.targetFlow) < 0.001);

    input.targetPressure = 20.0;
    input.measuredPressure = 14.0;
    input.timestamp = 1.5;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.filteredPressure > 10.0);
    assert(output1.filteredPressure < 14.0);
    assert(output1.outputFlow <= segment.maxFlow + 0.001);
    assert(output1.saturated);
    assert(output1.integralTerm <= segment.pressureIntegralLimit + 0.001);
    printf("✓ Pressure deadband/filter/anti-windup test passed\n");
}

static void test_output_tracking_back_calculates_integral_term(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing bumpless tracking / integral back-calculation...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;

    HDY_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);
    HDY_PressureController_RequestTracking(&state, 6.5);

    input.targetPressure = 10.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);

    input.timestamp = 1.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output0.trackingApplied);
    assert(fabs(output0.integralTerm - 3.5) < 0.001);
    assert(fabs(output0.outputFlow - 6.5) < 0.001);
    assert(fabs(output1.integralTerm - 3.5) < 0.001);
    assert(fabs(output1.outputFlow - 6.5) < 0.001);
    printf("✓ Bumpless tracking / integral back-calculation test passed\n");
}

static void test_pid_derivative_uses_measurement_rate_and_filter(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing PID derivative measurement filtering and kick suppression...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PID;
    segment.pressureKp = 0.5;
    segment.pressureKi = 0.0;
    segment.pressureKd = 1.0;
    segment.pressureDerivativeFilterAlpha = 0.25;
    segment.maxFlow = 20.0;

    HDY_PressureController_InitState(&state, 10.0, 5.0, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = 5.0;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 1.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);

    input.measuredPressure = 14.0;
    input.timestamp = 2.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(fabs(output0.derivativeTerm) < 0.001);
    assert(fabs(output0.outputFlow - 10.0) < 0.001);
    assert(fabs(output1.filteredPressureRate - 1.0) < 0.001);
    assert(fabs(output1.derivativeTerm + 1.0) < 0.001);
    assert(fabs(output1.outputFlow - 7.0) < 0.001);
    printf("✓ PID derivative measurement/filter test passed\n");
}

static void test_strategy_switch_uses_descriptor_based_tracking(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing strategy-switch tracking via descriptor resolution...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.5;
    segment.maxFlow = 10.0;
    HDY_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 14.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);
    assert(fabs(output0.outputFlow - 5.0) < 0.001);
    assert(output0.appliedStrategy == HDY_PRESSURE_CONTROLLER_P);

    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 1.0;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.trackingApplied);
    assert(output1.appliedStrategy == HDY_PRESSURE_CONTROLLER_PI);
    assert(fabs(output1.integralTerm + 2.0) < 0.001);
    assert(fabs(output1.outputFlow - output0.outputFlow) < 0.001);
    printf("✓ Strategy-switch tracking test passed\n");
}

static void test_rbf_pid_strategy_executes_within_limits_and_adapts(void) {
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output;
    HDY_REAL feedback;
    int step;

    printf("Testing adaptive RBF-PID pressure strategy integration...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    segment.targetFlow = 0.0;
    segment.maxFlow = 1.0;

    HDY_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    feedback = 0.0;

    for (step = 0; step < 20; ++step) {
        input.targetPressure = 100.0;
        input.measuredPressure = feedback;
        input.feedforwardFlow = 0.0;
        input.outputMin = 0.0;
        input.outputMax = segment.maxFlow;
        input.timestamp = (step + 1) * 0.01;

        HDY_PressureController_Execute(&segment, &state, &input, &output);

        assert(output.appliedStrategy == HDY_PRESSURE_CONTROLLER_RBF_PID);
        assert(output.outputFlow >= -1e-6);
        assert(output.outputFlow <= segment.maxFlow + 1e-6);
        assert(output.unsaturatedOutputFlow >= MIN_OUTPUT - 1e-6);
        assert(output.samplingPeriod > 0.0);
        assert(output.adaptiveActive);
        assert(state.activeStrategy == HDY_PRESSURE_CONTROLLER_RBF_PID);
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
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output;

    printf("Testing RBF-PID segment-level tuning profile mapping...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
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

    HDY_PressureController_InitState(&state, 5.0, segment.targetFlow, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 5.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.02;
    HDY_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.appliedStrategy == HDY_PRESSURE_CONTROLLER_RBF_PID);
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
    HDY_MotionSegment segment;
    HDY_PressureControllerState state;
    HDY_PressureControllerInput input;
    HDY_PressureControllerOutput output0;
    HDY_PressureControllerOutput output1;

    printf("Testing RBF-PID strategy switch bumpless tracking...\n");
    segment = make_pressure_segment();
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5;
    segment.pressureKi = 1.0;
    segment.pressureIntegralLimit = 10.0;
    segment.maxFlow = 10.0;

    HDY_PressureController_InitState(&state, 10.0, segment.targetFlow, 0.0);

    input.targetPressure = 14.0;
    input.measuredPressure = 10.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;
    HDY_PressureController_Execute(&segment, &state, &input, &output0);

    assert(output0.appliedStrategy == HDY_PRESSURE_CONTROLLER_PI);
    assert(fabs(output0.outputFlow - 5.0) < 0.001);

    segment.pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    input.targetPressure = 10.0;
    input.measuredPressure = 10.0;
    input.timestamp = 0.1;
    HDY_PressureController_Execute(&segment, &state, &input, &output1);

    assert(output1.trackingApplied);
    assert(output1.appliedStrategy == HDY_PRESSURE_CONTROLLER_RBF_PID);
    assert(fabs(output1.outputFlow - output0.outputFlow) < 0.05);
    assert(state.activeStrategy == HDY_PRESSURE_CONTROLLER_RBF_PID);
    assert(state.rbfInitialized);
    assert(state.rbfPid.Status == 1);
    assert(state.rbfPid.TuneResult == 66);
    printf("✓ RBF-PID strategy switch bumpless tracking test passed\n");
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

    printf("\n✅ All PressureController tests passed successfully!\n");
    return 0;
}
