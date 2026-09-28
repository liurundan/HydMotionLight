#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"
#include "rbf_pid.h"

static HYD_MotionSegment make_segment(HYD_PressureControllerType strategy) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 20.0f;
    segment.pressureController = strategy;
    segment.pressureFilterAlpha = 0.1f;
    segment.pressureDerivativeFilterAlpha = 0.12f;
    segment.systemGain = 200.0f;
    return segment;
}

static void fill_input(HYD_PressureControllerInput* input, HYD_TIME timestamp) {
    memset(input, 0, sizeof(*input));
    input->targetPressure = 100.0f;
    input->measuredPressure = 0.0f;
    input->outputMin = 0.0f;
    input->outputMax = 20.0f;
    input->flowToPumpSpeedGain = 40.0f;
    input->pumpSpeedLimit = 1800.0f;
    input->systemGain = 200.0f;
    input->plantTauS = 1.0f;
    input->timestamp = timestamp;
    input->enforceFixedSampling = true;
}

static void test_fixed_dt_holds_on_invalid_sample(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_REAL previous;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    fill_input(&input, 0.001f);
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    previous = output.outputFlow;

    input.timestamp = 0.003f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(!output.dtValid);
    assert(output.adaptationFreezeReason ==
           HYD_PRESSURE_ADAPTATION_FREEZE_INVALID_DT);
    assert(fabs(output.outputFlow - previous) < 1.0e-6);

    input.timestamp = 0.004f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.dtValid);
}

static void test_uncalibrated_request_is_conservative_pi(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_FF_PI);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    fill_input(&input, 0.001f);
    input.plantTauS = 0.0f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.requestedStrategy == HYD_PRESSURE_CONTROLLER_FF_PI);
    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(output.calibrationStatus == HYD_PRESSURE_CALIBRATION_UNCALIBRATED);
    assert(output.steadyStateFF == 0.0f);
    assert(output.adaptationFreezeReason ==
           HYD_PRESSURE_ADAPTATION_FREEZE_UNCALIBRATED);
}

static void test_rbf_consumes_resolved_cap_and_filters_d(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_REAL first_d;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    fill_input(&input, 0.001f);
    input.boostFlowLimitLmin = 2.0f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(state.rbfPid.effective_upper_cap_valid);
    assert(output.outputFlow <= output.effectiveUpperCap + 1.0e-6f);
    first_d = state.rbfPid.prev_d_term;

    input.measuredPressure = 10.0f;
    input.timestamp = 0.002f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(isfinite(state.rbfPid.prev_d_term));
    assert(fabs(state.rbfPid.prev_d_term - first_d) < 100.0);
}

static void test_shadow_requires_excitation_and_tracks_innovation(void) {
    RBF_PID_Handle pid;
    RBF_PID_ShadowState shadow;

    memset(&shadow, 0, sizeof(shadow));
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    pid.Jacobian = 0.2f;
    RBF_PID_ShadowUpdate(&shadow, &pid, 100.0f, 20.0f, 0.0f,
                         0.001f, true);
    assert(shadow.valid);
    assert(shadow.confidence_sample_count == 0U);
    RBF_PID_ShadowUpdate(&shadow, &pid, 100.0f, 20.4f, 2.0f,
                         0.001f, true);
    assert(shadow.valid);
    assert(shadow.confidence_sample_count == 1U);
    assert(fabsf(shadow.residual) < 1.0f);

    RBF_PID_ShadowUpdate(&shadow, &pid, 100.0f, 20.4f, 2.0f,
                         0.002f, false);
    assert(!shadow.valid);
    assert(shadow.confidence_sample_count == 0U);
}

static void test_adaptation_promotes_only_after_quality_window(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int i;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    fill_input(&input, 0.001f);
    for (i = 0; i < 600; ++i) {
        input.timestamp = (HYD_TIME)(i + 1) * 0.001;
        input.measuredPressure = (i % 2 == 0) ? 0.0f : 0.1f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        assert(state.calibrationStatus !=
               HYD_PRESSURE_CALIBRATION_ADAPTATION_CONFIDENT);
    }
    assert(state.promotionValidSamples == 0U ||
           state.promotionValidSamples < 500U);
}

int main(void) {
    test_fixed_dt_holds_on_invalid_sample();
    test_uncalibrated_request_is_conservative_pi();
    test_rbf_consumes_resolved_cap_and_filters_d();
    test_shadow_requires_excitation_and_tracks_innovation();
    test_adaptation_promotes_only_after_quality_window();
    puts("RBF-PID practicality tests passed.");
    return 0;
}
