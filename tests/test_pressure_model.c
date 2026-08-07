#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "fixtures/open10203040_measurement_reference.h"
#include "fixtures/pressure_model_open_loop_reference.h"
#include "pressure_model.h"

#define DT_S 0.001f

static PressureModelParams physical_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;
    return params;
}

static void step_n(const PressureModelParams *params,
                   PressureModelState *state,
                   float rpm,
                   float load_flow_m3_s,
                   int count,
                   PressureModelOutput *out) {
    PressureModelInput input;
    int i;

    input.target_rpm = rpm;
    input.load_flow_m3_s = load_flow_m3_s;
    input.dt_s = DT_S;
    for (i = 0; i < count; ++i) {
        PressureModel_StepInput(params, state, &input, out);
    }
}

static void test_profile_alias_and_first_order_regression(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;

    assert(PRESSURE_MODEL_TYPE_PHYSICAL == PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED);
    PressureModel_InitParams(&params);
    assert(params.model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
    params.first_order_k_bar_per_rpm = 0.5f;
    params.first_order_tau_s = 0.0f;
    PressureModel_Reset(&state, 1u);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    assert(fabsf(out.real_pressure_bar - 50.0f) < 1.0e-4f);
    assert(out.measured_pressure_bar == out.real_pressure_bar);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
}

static void test_physical_params_validate_and_invalid_holds_safely(void) {
    PressureModelParams params = physical_params();
    PressureModelParams invalid_eta;
    PressureModelParams invalid_motor_delay;
    PressureModelParams invalid_sensor_delay;
    PressureModelParams invalid_runtime;
    PressureModelState state;
    PressureModelOutput out;
    float held_pressure;

    assert(PressureModel_ValidatePhysicalParams(&params.physical));
    PressureModel_Reset(&state, 2u);
    step_n(&params, &state, 100.0f, 0.0f, 3000, &out);
    held_pressure = out.real_pressure_bar;
    params.physical.outlet_volume_m3 = 0.0f;
    assert(!PressureModel_ValidatePhysicalParams(&params.physical));
    params.physical.outlet_volume_m3 = 1.0e-4f;
    params.physical.chamber_volume_m3 = 1.0e-10f;
    assert(!PressureModel_ValidatePhysicalParams(&params.physical));
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    assert(isfinite(out.real_pressure_bar));
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));

    invalid_eta = physical_params();
    invalid_motor_delay = physical_params();
    invalid_sensor_delay = physical_params();
    invalid_runtime = physical_params();
    invalid_eta.physical.eta_v_min = NAN;
    invalid_motor_delay.physical.motor_delay_s = NAN;
    invalid_sensor_delay.physical.sensor_delay_s = NAN;
    invalid_runtime.pump_displacement_m3_rev = NAN;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    assert(!PressureModel_ValidatePhysicalParams(&invalid_motor_delay.physical));
    assert(!PressureModel_ValidatePhysicalParams(&invalid_sensor_delay.physical));
    invalid_eta = physical_params();
    invalid_eta.physical.motor_torque_limit_permille = -1.0f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta.physical.motor_torque_limit_permille = 1000.0f;
    invalid_eta.physical.torque_ripple13_peak = 1.01f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta = physical_params();
    invalid_eta.physical.motor_natural_freq_hz =
        PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_NATURAL_FREQ_HZ + 1.0f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta = physical_params();
    invalid_eta.physical.motor_damping =
        PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_DAMPING + 0.01f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta = physical_params();
    invalid_eta.physical.motor_accel_limit_rpm_s =
        PRESSURE_MODEL_PHYSICAL_MAX_MOTOR_ACCEL_RPM_S + 1.0f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta = physical_params();
    invalid_eta.physical.eta_m_min = invalid_eta.physical.eta_m_nominal + 0.01f;
    assert(!PressureModel_ValidatePhysicalParams(&invalid_eta.physical));
    invalid_eta.physical.eta_v_min = NAN;
    PressureModel_Step(&invalid_eta, &state, 100.0f, DT_S, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    PressureModel_Step(&invalid_motor_delay, &state, 100.0f, DT_S, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    PressureModel_Step(&invalid_sensor_delay, &state, 100.0f, DT_S, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    PressureModel_Step(&invalid_runtime, &state, 100.0f, DT_S, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
}

static void test_nonfinite_step_input_holds_physical_state(void) {
    PressureModelParams params = physical_params();
    PressureModelState state;
    PressureModelOutput out;
    PressureModelInput input;
    float held_pressure;
    float held_motor_rpm;

    PressureModel_Reset(&state, 3u);
    step_n(&params, &state, 100.0f, 0.0f, 3000, &out);
    held_pressure = out.real_pressure_bar;
    held_motor_rpm = out.actual_motor_rpm;

    input.target_rpm = NAN;
    input.load_flow_m3_s = 0.0f;
    input.dt_s = DT_S;
    PressureModel_StepInput(&params, &state, &input, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(fabsf(out.actual_motor_rpm - held_motor_rpm) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));

    input.target_rpm = 100.0f;
    input.load_flow_m3_s = NAN;
    PressureModel_StepInput(&params, &state, &input, &out);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(fabsf(out.actual_motor_rpm - held_motor_rpm) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
}

static void test_physical_extreme_admission_holds_state(void) {
    PressureModelParams params = physical_params();
    PressureModelParams invalid_params;
    PressureModelState state;
    PressureModelState held_state;
    PressureModelOutput out;
    PressureModelInput input;
    float held_pressure;
    int i;

    PressureModel_Reset(&state, 4u);
    step_n(&params, &state, 100.0f, 0.0f, 3000, &out);
    held_pressure = out.real_pressure_bar;
    held_state = state;
    input.target_rpm = 100.0f;
    input.load_flow_m3_s = FLT_MAX;
    input.dt_s = DT_S;
    PressureModel_StepInput(&params, &state, &input, &out);
    held_state.timestamp_s += DT_S;
    assert(memcmp(&state, &held_state, sizeof(state)) == 0);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);
    assert(!HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));

    for (i = 0; i < 2; ++i) {
        held_state = state;
        input.load_flow_m3_s = i == 0 ? -FLT_MAX : FLT_MAX;
        PressureModel_StepInput(&params, &state, &input, &out);
        held_state.timestamp_s += DT_S;
        assert(memcmp(&state, &held_state, sizeof(state)) == 0);
        assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
        assert(out.pump_flow_m3_s == 0.0f);
        assert(out.net_flow_m3_s == 0.0f);
    }

    invalid_params = params;
    invalid_params.pump_displacement_m3_rev = FLT_MAX;
    held_state = state;
    input.load_flow_m3_s = 0.0f;
    PressureModel_StepInput(&invalid_params, &state, &input, &out);
    held_state.timestamp_s += DT_S;
    assert(memcmp(&state, &held_state, sizeof(state)) == 0);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);

    invalid_params = params;
    invalid_params.max_rpm = PRESSURE_MODEL_PHYSICAL_MAX_ABS_RPM + 1.0f;
    held_state = state;
    PressureModel_StepInput(&invalid_params, &state, &input, &out);
    held_state.timestamp_s += DT_S;
    assert(memcmp(&state, &held_state, sizeof(state)) == 0);
    assert(fabsf(out.real_pressure_bar - held_pressure) < 1.0e-5f);
    assert(out.pump_flow_m3_s == 0.0f);
    assert(out.net_flow_m3_s == 0.0f);
}

static void test_positive_pressure_profile_handover_advances_time(void) {
    PressureModelParams params = physical_params();
    PressureModelState state;
    PressureModelOutput out;
    float timestamp_s;

    PressureModel_Reset(&state, 5u);
    step_n(&params, &state, 100.0f, 0.0f, 3000, &out);
    assert(out.real_pressure_bar > 0.0f);

    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    timestamp_s = state.timestamp_s;
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    assert(fabsf(state.timestamp_s - (timestamp_s + DT_S)) < 1.0e-6f);

    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
    timestamp_s = state.timestamp_s;
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    assert(fabsf(state.timestamp_s - (timestamp_s + DT_S)) < 1.0e-6f);
}

static void test_measured_open_loop_reference_contract(void) {
    int i;

    assert(OPEN10203040_MEASUREMENT_TIMESTAMP_DELTA_MS == 1);
    assert(OPEN10203040_MEASUREMENT_TARGET_PRESSURE_BAR == 0);
    assert(OPEN10203040_MEASUREMENT_REFERENCE_COUNT == 4);
    for (i = 0; i < OPEN10203040_MEASUREMENT_REFERENCE_COUNT; ++i) {
        const Open10203040MeasurementReference *reference =
            &kOpen10203040MeasurementReference[i];

        assert(reference->timestamp_start_ms < reference->timestamp_end_ms);
        assert(reference->command_rpm == 10.0f * (float)(i + 1));
        assert(reference->mean_feedback_rpm > 0.0f);
        assert(reference->mean_feedback_pressure_bar > 0.0f);
        assert(reference->tail_pressure_peak_to_peak_bar > 0.0f);
        assert(reference->angle_synchronous_order13_amplitude_bar > 0.0f);
    }
}

static void test_fixed_substeps_and_invalid_dt_are_safe(void) {
    PressureModelParams params = physical_params();
    PressureModelState a;
    PressureModelState b;
    PressureModelOutput out_a;
    PressureModelOutput out_b;
    PressureModelInput input;

    PressureModel_Reset(&a, 3u);
    PressureModel_Reset(&b, 3u);
    input.target_rpm = 120.0f;
    input.load_flow_m3_s = 0.0f;
    input.dt_s = 0.004f;
    PressureModel_StepInput(&params, &a, &input, &out_a);
    step_n(&params, &b, 120.0f, 0.0f, 4, &out_b);
    assert(fabsf(out_a.real_pressure_bar - out_b.real_pressure_bar) < 1.0e-5f);
    input.dt_s = 0.0015f;
    PressureModel_StepInput(&params, &a, &input, &out_a);
    assert(isfinite(out_a.real_pressure_bar));
    assert(fabsf(a.timestamp_s - 0.005f) < 1.0e-6f);
}

static void test_load_leakage_and_gas_are_causal(void) {
    PressureModelParams baseline = physical_params();
    PressureModelParams leaky = baseline;
    PressureModelParams gassy = baseline;
    PressureModelState zero_load;
    PressureModelState positive_load;
    PressureModelState low_leak;
    PressureModelState high_leak;
    PressureModelState gas_state;
    PressureModelOutput out_zero;
    PressureModelOutput out_load;
    PressureModelOutput out_low_leak;
    PressureModelOutput out_high_leak;
    PressureModelOutput out_baseline_short;
    PressureModelOutput out_gas;

    leaky.physical.pump_leak_c0_m3_pa_s *= 4.0f;
    gassy.physical.gas_fraction = 0.02f;
    PressureModel_Reset(&zero_load, 4u);
    PressureModel_Reset(&positive_load, 4u);
    PressureModel_Reset(&low_leak, 4u);
    PressureModel_Reset(&high_leak, 4u);
    PressureModel_Reset(&gas_state, 4u);
    step_n(&baseline, &zero_load, 80.0f, 0.0f, 5000, &out_zero);
    step_n(&baseline, &positive_load, 80.0f, 1.0e-6f, 5000, &out_load);
    step_n(&baseline, &low_leak, 80.0f, 0.0f, 5000, &out_low_leak);
    step_n(&leaky, &high_leak, 80.0f, 0.0f, 5000, &out_high_leak);
    step_n(&baseline, &gas_state, 80.0f, 0.0f, 5000, &out_baseline_short);
    PressureModel_Reset(&gas_state, 4u);
    step_n(&gassy, &gas_state, 80.0f, 0.0f, 5000, &out_gas);
    assert(out_load.real_pressure_bar < out_zero.real_pressure_bar);
    assert(out_high_leak.real_pressure_bar < out_low_leak.real_pressure_bar);
    assert(out_gas.real_pressure_bar < out_baseline_short.real_pressure_bar);
    assert(PressureModel_EffectiveBulkModulusPa(&gassy.physical,
                                                 gassy.physical.atmospheric_pressure_pa) <
           PressureModel_EffectiveBulkModulusPa(&gassy.physical,
                                                 gassy.physical.atmospheric_pressure_pa +
                                                     20.0e6f));
}

static void test_line_relief_and_reverse_flow(void) {
    PressureModelParams params = physical_params();
    PressureModelParams high_resistance = params;
    PressureModelState charged;
    PressureModelState passive;
    PressureModelState reverse;
    PressureModelState low_line;
    PressureModelState high_line;
    PressureModelState relief_low;
    PressureModelState relief_high;
    PressureModelOutput out;
    PressureModelOutput low_out;
    PressureModelOutput high_out;
    PressureModelOutput relief_low_out;
    PressureModelOutput relief_high_out;
    float relief_before;

    params.physical.relief_set_pa = 1.0e6f;
    PressureModel_Reset(&charged, 5u);
    step_n(&params, &charged, 300.0f, 0.0f, 6000, &out);
    assert(out.relief_active);
    relief_before = out.relief_flow_m3_s;
    assert(relief_before > 0.0f);
    passive = charged;
    reverse = charged;
    step_n(&params, &passive, 0.0f, 0.0f, 1000, &out);
    step_n(&params, &reverse, -100.0f, 0.0f, 1000, &out);
    assert(reverse.pressure_pa < passive.pressure_pa);
    assert(reverse.pressure_pa >= 0.0f);

    high_resistance.physical.line_resistance_pa_s_per_m3 *= 20.0f;
    PressureModel_Reset(&low_line, 8u);
    PressureModel_Reset(&high_line, 8u);
    low_line.outlet_pressure_pa = 5.0e6f;
    high_line.outlet_pressure_pa = 5.0e6f;
    PressureModel_Step(&params, &low_line, 0.0f, DT_S, &low_out);
    PressureModel_Step(&high_resistance, &high_line, 0.0f, DT_S, &high_out);
    assert(low_line.line_flow_m3_s > high_line.line_flow_m3_s);

    PressureModel_Reset(&relief_low, 9u);
    PressureModel_Reset(&relief_high, 9u);
    relief_low.outlet_pressure_pa =
        params.physical.relief_set_pa + params.physical.relief_deadband_pa + 0.1e6f;
    relief_high.outlet_pressure_pa =
        params.physical.relief_set_pa + params.physical.relief_deadband_pa + 1.0e6f;
    PressureModel_Step(&params, &relief_low, 0.0f, DT_S, &relief_low_out);
    PressureModel_Step(&params, &relief_high, 0.0f, DT_S, &relief_high_out);
    assert(relief_low_out.relief_active && relief_high_out.relief_active);
    assert(relief_high_out.relief_flow_m3_s > relief_low_out.relief_flow_m3_s);
}

static void test_model_baseline_speed_sections_are_ordered(void) {
    /*
     * Uncalibrated model baseline only: these deterministic 10/20/30/40 RPM
     * sections are not measured-machine or calibration acceptance values.
     */
    float previous_pressure = -1.0f;
    int i;

    for (i = 0; i < PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT; ++i) {
        PressureModelParams params = physical_params();
        PressureModelState state;
        PressureModelOutput out;

        PressureModel_Reset(&state, (uint32_t)(20 + i));
        step_n(&params, &state, kPressureModelOpenLoopReference[i].command_rpm,
               0.0f, 5000, &out);
        assert(out.actual_motor_rpm >
               kPressureModelOpenLoopReference[i].command_rpm * 0.95f);
        assert(out.real_pressure_bar > previous_pressure);
        assert(out.real_pressure_bar < params.sensor_range_bar);
        previous_pressure = out.real_pressure_bar;
    }
}

static void test_motor_sensor_order_and_torque_packet(void) {
    PressureModelParams params = physical_params();
    PressureModelState state;
    PressureModelOutput out;
    float first_rpm;
    float first_measured;

    params.physical.motor_delay_s = 0.003f;
    params.physical.sensor_delay_s = 0.002f;
    params.physical.sensor_quantization_bar = 0.5f;
    params.physical.motor_accel_limit_rpm_s = 1000.0f;
    params.physical.ripple13_peak = 0.1f;
    params.physical.ripple26_peak = 0.1f;
    params.physical.ripple39_peak = 0.1f;
    PressureModel_Reset(&state, 6u);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    first_rpm = out.actual_motor_rpm;
    first_measured = out.measured_pressure_bar;
    assert(first_rpm == 0.0f);
    step_n(&params, &state, 100.0f, 0.0f, 20, &out);
    assert(out.actual_motor_rpm > 0.0f);
    assert(out.actual_motor_rpm <= 20.0f + 1.0e-4f);
    assert(fabsf(out.measured_pressure_bar * 2.0f -
                 roundf(out.measured_pressure_bar * 2.0f)) < 1.0e-4f);
    assert(first_measured == 0.0f);
    assert(HYD_PumpFeedback_HasValid(out.pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM |
                                         HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                         HYD_PUMP_FEEDBACK_VALID_TIMESTAMP |
                                         HYD_PUMP_FEEDBACK_VALID_TORQUE));
    assert(out.pumpFeedback.angleDeg >= 0.0f && out.pumpFeedback.angleDeg < 360.0f);
    assert(isfinite(out.pumpFeedback.torquePermille));
}

static void test_second_order_motor_and_exact_sixty_four_ms_delays(void) {
    PressureModelParams params = physical_params();
    PressureModelState motor_state;
    PressureModelState sensor_state;
    PressureModelOutput out;
    int i;

    params.physical.motor_delay_s = 0.064f;
    params.physical.sensor_delay_s = 0.064f;
    params.physical.motor_accel_limit_rpm_s = 1000.0f;
    PressureModel_Reset(&motor_state, 12u);
    for (i = 0; i < 64; ++i) {
        PressureModel_Step(&params, &motor_state, 100.0f, DT_S, &out);
        assert(out.actual_motor_rpm == 0.0f);
    }
    PressureModel_Step(&params, &motor_state, 100.0f, DT_S, &out);
    assert(out.actual_motor_rpm > 0.0f);
    assert(fabsf(motor_state.motor_accel_rpm_s) <=
           params.physical.motor_accel_limit_rpm_s + 1.0e-4f);

    params.physical.motor_delay_s = 0.0f;
    params.physical.pump_leak_c0_m3_pa_s = 0.0f;
    params.physical.pump_leak_speed_m3_pa_s_per_rpm = 0.0f;
    params.physical.outlet_leak_m3_pa_s = 0.0f;
    params.physical.cylinder_leak_m3_pa_s = 0.0f;
    PressureModel_Reset(&sensor_state, 13u);
    sensor_state.pressure_pa = 10.0e6f;
    sensor_state.outlet_pressure_pa = 10.0e6f;
    for (i = 0; i < 64; ++i) {
        PressureModel_Step(&params, &sensor_state, 0.0f, DT_S, &out);
        assert(out.measured_pressure_bar == 0.0f);
    }
    PressureModel_Step(&params, &sensor_state, 0.0f, DT_S, &out);
    assert(out.measured_pressure_bar > 0.0f);
}

static void test_thirteenth_phase_and_true_torque_units(void) {
    PressureModelParams params = physical_params();
    PressureModelState state;
    PressureModelOutput out;
    float expected_flow;
    float expected_torque;
    float phase13;
    float eta_m;
    float high_speed_expected_flow;

    params.physical.ripple13_peak = 0.1f;
    params.physical.ripple26_peak = 0.0f;
    params.physical.ripple39_peak = 0.0f;
    params.physical.pump_leak_c0_m3_pa_s = 0.0f;
    params.physical.pump_leak_speed_m3_pa_s_per_rpm = 0.0f;
    params.physical.outlet_leak_m3_pa_s = 0.0f;
    params.physical.cylinder_leak_m3_pa_s = 0.0f;
    PressureModel_Reset(&state, 10u);
    state.motor_rpm = 60.0f;
    state.outlet_pressure_pa = 10.0e6f;
    PressureModel_Step(&params, &state, 60.0f, DT_S, &out);
    phase13 = 2.0f * 3.14159265358979323846f * 13.0f * 0.001f;
    expected_flow = params.pump_displacement_m3_rev *
                    (1.0f + params.physical.ripple13_peak *
                     sinf(phase13));
    assert(fabsf(out.pump_flow_m3_s - expected_flow) < 1.0e-10f);
    eta_m = params.physical.eta_m_nominal -
            params.physical.eta_m_pressure_loss_per_pa * 10.0e6f -
            params.physical.eta_m_speed_loss_per_rpm * 60.0f;
    expected_torque = 1000.0f * 10.0e6f * params.pump_displacement_m3_rev /
                      (2.0f * 3.14159265358979323846f * eta_m) /
                      params.physical.rated_motor_torque_nm;
    expected_torque *= 1.0f + params.physical.torque_ripple13_peak *
                       sinf(phase13 + params.physical.torque_ripple13_phase_rad);
    assert(fabsf(out.pumpFeedback.torquePermille - expected_torque) < 1.0e-3f);

    PressureModel_Reset(&state, 11u);
    state.motor_rpm = 1200.0f;
    params.physical.ripple26_peak = 0.1f;
    params.physical.ripple39_peak = 0.1f;
    PressureModel_Step(&params, &state, 1200.0f, DT_S, &out);
    phase13 = 2.0f * 3.14159265358979323846f * 13.0f * 0.02f;
    high_speed_expected_flow = params.pump_displacement_m3_rev * 20.0f *
        (1.0f + params.physical.ripple13_peak *
         sinf(phase13));
    assert(fabsf(out.pump_flow_m3_s - high_speed_expected_flow) < 1.0e-9f);
    assert(out.active_order_mask == PRESSURE_MODEL_ORDER_13_ACTIVE);
}

static void test_repeatable_one_ms_physical_result(void) {
    PressureModelParams params = physical_params();
    PressureModelState a;
    PressureModelState b;
    PressureModelOutput out_a;
    PressureModelOutput out_b;
    int i;

    PressureModel_Reset(&a, 7u);
    PressureModel_Reset(&b, 7u);
    for (i = 0; i < 2000; ++i) {
        PressureModel_Step(&params, &a, 150.0f, DT_S, &out_a);
        PressureModel_Step(&params, &b, 150.0f, DT_S, &out_b);
    }
    assert(isfinite(out_a.real_pressure_bar));
    assert(out_a.real_pressure_bar == out_b.real_pressure_bar);
    assert(out_a.pumpFeedback.torquePermille == out_b.pumpFeedback.torquePermille);
}

int main(void) {
    test_profile_alias_and_first_order_regression();
    test_physical_params_validate_and_invalid_holds_safely();
    test_nonfinite_step_input_holds_physical_state();
    test_physical_extreme_admission_holds_state();
    test_positive_pressure_profile_handover_advances_time();
    test_measured_open_loop_reference_contract();
    test_fixed_substeps_and_invalid_dt_are_safe();
    test_load_leakage_and_gas_are_causal();
    test_model_baseline_speed_sections_are_ordered();
    test_line_relief_and_reverse_flow();
    test_motor_sensor_order_and_torque_packet();
    test_second_order_motor_and_exact_sixty_four_ms_delays();
    test_thirteenth_phase_and_true_torque_units();
    test_repeatable_one_ms_physical_result();
    puts("pressure model tests passed");
    return 0;
}
