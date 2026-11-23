#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
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

static void test_reverse_limit_is_passed_to_rbf(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 100.0, 0.0, 0.0);
    fill_input(&input, 0.001);
    input.targetPressure = 20.0;
    input.measuredPressure = 100.0;
    input.outputMin = -90.0;
    input.outputMax = 20.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(fabs(output.effectiveOutputMin - (-90.0)) < 1.0e-6);
    assert(fabs((double)state.rbfPid.output_min_flow - (-90.0)) < 1.0e-6);
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

/* ===================== Legacy characterization (spec 2026-11-23 §6 T4) =====================
 * Golden values were captured on commit 0a8afa4, before the 2-DOF pressure
 * loop existed. Kq = 0 must keep the non-RBF controller and the FB pressure
 * path bit-identical. If a later, intentional change to the legacy path moves
 * these values, re-capture them in that change. */

static HYD_REAL legacy_trace_pressure(int k) {
    HYD_REAL p = 100.0f * (1.0f - expf(-(HYD_REAL)k / 300.0f)) +
                 3.0f * sinf((HYD_REAL)k * 0.05f);
    if (k >= 1500 && k < 1700) {
        p += 15.0f;
    }
    return p;
}

static void legacy_controller_trace(HYD_PressureControllerType strategy,
                                    double* sumOut, HYD_REAL* lastOut) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 20.0f;
    segment.pressureController = strategy;
    segment.pressureKp = 0.5f;
    segment.pressureKi = 4.0f;
    segment.pressureKd = 0.01f;
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;
    segment.pressureDerivativeFilterAlpha = 0.5f;

    memset(&input, 0, sizeof(input));
    input.outputMin = -7.2f;
    input.outputMax = 20.0f;
    input.flowToPumpSpeedGain = 20.0f;
    input.pumpSpeedLimit = 1800.0f;
    input.feedforwardFlow = 0.5f;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    *sumOut = 0.0;
    *lastOut = 0.0f;
    for (k = 1; k <= 2000; ++k) {
        input.targetPressure = (k < 1000) ? 100.0f : 60.0f;
        input.measuredPressure = legacy_trace_pressure(k);
        input.timestamp = (HYD_TIME)k * 0.001f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        *sumOut += (double)output.outputFlow;
        *lastOut = output.outputFlow;
    }
}

static void legacy_fb_trace(double* sumSpeed, HYD_REAL* lastSpeed) {
    static HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL pressure = 0.0f;
    int k;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 20.0f;
    segment.pressureRampRate = 20.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5f;
    segment.pressureKi = 4.0f;
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;

    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.0f;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0f));
    *sumSpeed = 0.0;
    *lastSpeed = 0.0f;
    for (k = 1; k <= 1500; ++k) {
        fb.AXIS_REF.timestamp = (HYD_TIME)k * 0.001f;
        fb.AXIS_REF.pressure = pressure;
        HYD_MotionControlFB_Cycle(&fb);
        pressure += 64.0f * (fb.PUMP_SPEED / 20.0f - pressure / 104.0f) * 0.001f;
        *sumSpeed += (double)fb.PUMP_SPEED;
        *lastSpeed = fb.PUMP_SPEED;
    }
}

static void test_legacy_path_is_bit_identical(void) {
    double sum;
    HYD_REAL last;

    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_P, &sum, &last);
    assert(sum == 0x1.0168b5c18p+12 && last == -0x1.ccccccp+2f);
    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_PI, &sum, &last);
    assert(sum == 0x1.424019228p+13 && last == -0x1.ccccccp+2f);
    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_PID, &sum, &last);
    assert(sum == 0x1.3f431ec84p+13 && last == -0x1.ccccccp+2f);
    legacy_fb_trace(&sum, &last);
    assert(sum == 0x1.cfb7e910cp+15 && last == 0x1.33b09cp+4f);
}

int main(void) {
    test_rbf_basic_execution();
    test_pressure_feedback_is_not_filtered_by_controller();
    test_ksys_is_converted_to_flow_domain();
    test_negative_flow_is_bounded_and_finite();
    test_reverse_limit_is_passed_to_rbf();
    test_soft_reset_preserves_network();
    test_legacy_path_is_bit_identical();
    puts("Pressure controller tests passed.");
    return 0;
}
