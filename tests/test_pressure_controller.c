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

/* ===================== Model path (spec 2026-11-23 §4.3/§4.4, T2/T3) ===================== */

static HYD_MotionSegment make_model_segment(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_PI);
    segment.pressureKp = 0.5;
    segment.pressureKi = 4.0;
    segment.pressureIntegralLimit = 10.0;
    segment.pressureDeadband = 0.0;
    return segment;
}

static void fill_model_input(HYD_PressureControllerInput* input, HYD_TIME timestamp) {
    fill_input(input, timestamp);
    input->outputMin = -6.8;
    input->flowToPumpSpeedGain = 20.0;
    input->pumpSpeedLimit = 1700.0;
    input->systemGainBarPerRpm = 5.2;   /* R_f = 104 bar per L/min */
    input->kqBarPerSecRpm = 3.2;        /* Kq_f = 64 bar/s per L/min */
}

static void test_rate_limits_formula(void) {
    HYD_REAL rise = -1.0;
    HYD_REAL fall = -1.0;

    /* Kq_f = 64, R_f = 104, P = 104 bar -> hold flow 1 L/min. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 104.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 19.0) < 1.0e-2);
    assert(fabs(fall - 0.6 * 64.0 * 7.8) < 1.0e-2);

    /* ksys = 0 drops the leak term. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 0.0, 20.0, 104.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 20.0) < 1.0e-2);
    assert(fabs(fall - 0.6 * 64.0 * 6.8) < 1.0e-2);

    /* Hold flow above the cap: the 10 % floor keeps the reference moving. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 2080.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 2.0) < 1.0e-2);

    /* The gain converts the rpm-domain parameters: G = 21.05 scales Kq_f. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 0.0, 21.05, 0.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 3.2 * 21.05 * 20.0) < 1.0e-2);

    assert(!HYD_PressureController_ResolveRateLimits(0.0, 5.2, 20.0, 100.0, 20.0, -6.8, &rise, &fall));
    assert(rise == 0.0 && fall == 0.0);
    assert(!HYD_PressureController_ResolveRateLimits(3.2, 5.2, 0.0, 100.0, 20.0, -6.8, &rise, &fall));
    assert(!HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 100.0, 0.0, -6.8, &rise, &fall));
}

static void test_model_feedforward_replaces_legacy_feedforward(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 104.0, 0.0, 0.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 104.0;
    input.measuredPressure = 104.0;
    input.targetPressureRate = 128.0;   /* 128 / 64 = 2 L/min */
    input.feedforwardFlow = 7.0;        /* ignored on the model path */
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(fabs(output.feedforwardFlow - 3.0) < 1.0e-5);
    assert(fabs(output.outputFlow - 3.0) < 1.0e-5);
    assert(fabs(output.referencePressureRate - 128.0) < 1.0e-5);

    /* ksys = 0: rate term only. */
    HYD_PressureController_InitState(&state, 104.0, 0.0, 0.0);
    input.systemGainBarPerRpm = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(fabs(output.feedforwardFlow - 2.0) < 1.0e-5);
}

static void test_model_error_uses_delayed_reference(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    HYD_PressureController_InitState(&state, 50.0, 0.0, 0.0);
    fill_model_input(&input, 0.0);
    input.measuredPressure = 50.0;
    for (k = 0; k < HYD_PRESSURE_REF_DELAY_SAMPLES + 3; ++k) {
        input.targetPressure = 50.0 + (HYD_REAL)k;
        input.timestamp = (HYD_TIME)(k + 1) * 0.001;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        if (k < HYD_PRESSURE_REF_DELAY_SAMPLES) {
            /* The line was filled with the first reference (50 bar). */
            assert(fabs(output.controlError) < 1.0e-5);
        } else {
            assert(fabs(output.controlError -
                        (HYD_REAL)(k - HYD_PRESSURE_REF_DELAY_SAMPLES)) < 1.0e-4);
        }
    }

    /* A new segment refills the line from its first reference. */
    HYD_PressureController_InitState(&state, 80.0, 0.0, 0.0);
    input.targetPressure = 90.0;
    input.measuredPressure = 80.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(fabs(output.controlError - 10.0) < 1.0e-5);
}

static void test_model_tracking_is_bumpless(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 60.0, 4.0, 0.0);
    HYD_PressureController_RequestTracking(&state, 4.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 60.0;
    input.measuredPressure = 60.0;
    input.targetPressureRate = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.trackingApplied);
    assert(fabs(output.outputFlow - 4.0) < 1.0e-4);
    /* Integral absorbs everything the model feedforward does not explain. */
    assert(fabs(output.integralTerm - (4.0 - 60.0 / 104.0)) < 1.0e-4);
}

static void test_model_relief_holds_while_reference_falls(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 100.0, 1.0, 0.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 99.7;
    input.measuredPressure = 100.0;
    input.targetPressureRate = -300.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.reliefActive);
    assert(output.outputFlow < 0.0);
    assert(output.effectiveOutputMin < 0.0);

    /* Pressure already at the reference, but the reference is still falling:
     * relief must stay active. */
    input.timestamp = 0.002;
    input.targetPressure = 99.4;
    input.measuredPressure = 99.4;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.reliefActive);

    /* Reference stopped and pressure within 0.5 bar: relief exits. */
    input.timestamp = 0.003;
    input.targetPressureRate = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(!output.reliefActive);
    assert(output.outputFlow >= 0.0);
}

static void test_model_conditional_integration_at_limit(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_model_input(&input, 0.0);
    input.targetPressure = 100.0;
    input.measuredPressure = 20.0;     /* e = 80 -> P term 40 L/min > cap */
    for (k = 1; k <= 50; ++k) {
        input.timestamp = (HYD_TIME)k * 0.001;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
    }
    assert(output.saturated);
    assert(fabs(output.outputFlow - 20.0) < 1.0e-5);
    assert(fabs(output.integralTerm) < 1.0e-6);
}

static void test_model_path_flags_clipped_output_for_ripple(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PressureRippleCompState ripple;

    HYD_PressureRippleComp_Clear(&ripple);
    HYD_PressureController_InitState(&state, 20.0, 0.0, 0.0);
    fill_model_input(&input, 0.001);
    input.pumpRippleOrder = 12U;
    input.rippleComp = &ripple;
    input.targetPressure = 100.0;
    input.measuredPressure = 20.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.saturated);
    assert(ripple.lastClipped);
    assert(ripple.order == 12U);

    input.timestamp = 0.002;
    input.measuredPressure = 100.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(!output.saturated);
    assert(!ripple.lastClipped);
}

static void test_rbf_ignores_kq(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState stateA;
    HYD_PressureControllerState stateB;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput outputA;
    HYD_PressureControllerOutput outputB;
    int k;

    HYD_PressureController_InitState(&stateA, 0.0, 0.0, 0.0);
    HYD_PressureController_InitState(&stateB, 0.0, 0.0, 0.0);
    fill_input(&input, 0.0);
    for (k = 1; k <= 200; ++k) {
        input.timestamp = (HYD_TIME)k * 0.001;
        input.measuredPressure = (HYD_REAL)k * 0.4;
        input.kqBarPerSecRpm = 0.0;
        HYD_PressureController_Execute(&segment, &stateA, &input, &outputA);
        input.kqBarPerSecRpm = 3.2;
        HYD_PressureController_Execute(&segment, &stateB, &input, &outputB);
        assert(outputA.outputFlow == outputB.outputFlow);
    }
    assert(outputB.referencePressureRate == 0.0);
}

int main(void) {
    test_rbf_basic_execution();
    test_pressure_feedback_is_not_filtered_by_controller();
    test_ksys_is_converted_to_flow_domain();
    test_negative_flow_is_bounded_and_finite();
    test_reverse_limit_is_passed_to_rbf();
    test_soft_reset_preserves_network();
    test_legacy_path_is_bit_identical();
    test_rate_limits_formula();
    test_model_feedforward_replaces_legacy_feedforward();
    test_model_error_uses_delayed_reference();
    test_model_tracking_is_bumpless();
    test_model_relief_holds_while_reference_falls();
    test_model_conditional_integration_at_limit();
    test_model_path_flags_clipped_output_for_ripple();
    test_rbf_ignores_kq();
    puts("Pressure controller tests passed.");
    return 0;
}
