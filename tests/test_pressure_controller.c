#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"

static HYD_MotionSegment make_segment(HYD_PressureControllerType strategy) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0;
    segment.maxFlow = 20.0;
    segment.pressureController = strategy;
    segment.pressureFilterAlpha = 0.1;
    segment.pressureDerivativeFilterAlpha = 0.2;
    segment.pressureDeadband = 0.0;
    segment.systemGain = 30.0;
    return segment;
}

static void fill_input(HYD_PressureControllerInput* input, HYD_TIME timestamp) {
    memset(input, 0, sizeof(*input));
    input->targetPressure = 100.0;
    input->measuredPressure = 0.0;
    input->feedforwardFlow = 0.0;
    input->outputMin = -5.0;
    input->outputMax = 20.0;
    input->flowToPumpSpeedGain = 20.0;
    input->pumpSpeedLimit = 1800.0;
    input->systemGainBarPerRpm = 1.5;
    input->timestamp = timestamp;
}

static void test_rbf_basic_execution(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
    assert(isfinite(output.outputFlow));
    assert(isfinite(output.adaptiveKp));
    assert(isfinite(output.adaptiveKi));
    assert(isfinite(output.adaptiveKd));
    assert(state.rbfInitialized);
}

static void test_pressure_feedback_is_not_filtered_by_controller(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    input.measuredPressure = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    input.timestamp = 0.002;
    input.measuredPressure = 80.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(fabs(output.filteredPressure - 80.0) < 1.0e-6);
}

static void test_ksys_is_converted_to_flow_domain(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    input.systemGainBarPerRpm = 1.5;
    input.flowToPumpSpeedGain = 20.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(state.rbfInitialized);
    assert(state.rbfPid.ksys_valid);
    assert(fabs(state.rbfPid.K - 30.0) < 1.0e-6);
}

static void test_negative_flow_is_bounded_and_finite(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int i;

    HYD_PressureController_InitState(&state, 100.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    input.targetPressure = 20.0;
    input.measuredPressure = 100.0;
    input.outputMin = -5.0;
    input.outputMax = 20.0;

    for (i = 0; i < 10; ++i) {
        input.timestamp = 0.001 * (i + 1);
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        assert(isfinite(output.outputFlow));
        assert(output.outputFlow >= -5.0 - 1.0e-6);
        assert(output.outputFlow <= 20.0 + 1.0e-6);
    }
}

static void test_soft_reset_preserves_network(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    float weight;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    weight = state.rbfPid.w[0];
    RBF_PID_SoftReset(&state.rbfPid);
    assert(fabsf(state.rbfPid.w[0] - weight) < 1.0e-7f);
}

int main(void) {
    test_rbf_basic_execution();
    test_pressure_feedback_is_not_filtered_by_controller();
    test_ksys_is_converted_to_flow_domain();
    test_negative_flow_is_bounded_and_finite();
    test_soft_reset_preserves_network();
    puts("Pressure controller tests passed.");
    return 0;
}
