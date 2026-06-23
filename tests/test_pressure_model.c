#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fixtures/pressure_model_open_loop_reference.h"
#include "pressure_model.h"

#define DT_S 0.001f
#define PRESSURE_EPS 1e-4f
#define RPM_EPS 1e-3f

#define ASSERT_TRUE(condition)                                                         \
    do {                                                                               \
        if (!(condition)) {                                                            \
            fprintf(stderr, "Assertion failed at %s:%d: %s\n", __FILE__, __LINE__,    \
                    #condition);                                                       \
            return 0;                                                                  \
        }                                                                              \
    } while (0)

#define ASSERT_NEAR(actual, expected, tolerance)                                       \
    do {                                                                               \
        if (fabs((double)((actual) - (expected))) > (double)(tolerance)) {            \
            fprintf(stderr,                                                            \
                    "Assertion failed at %s:%d: %s=%f expected=%f tolerance=%f\n",    \
                    __FILE__,                                                          \
                    __LINE__,                                                          \
                    #actual,                                                           \
                    (double)(actual),                                                  \
                    (double)(expected),                                                \
                    (double)(tolerance));                                              \
            return 0;                                                                  \
        }                                                                              \
    } while (0)

static PressureModelParams make_deterministic_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.sensor_noise_std_bar = 0.0f;
    params.sensor_bias_bar = 0.0f;
    params.motor_noise_std_rpm = 0.0f;
    params.process_noise_std_m3_s = 0.0f;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;

    return params;
}

static PressureModelParams make_first_order_params(float gain, float tau_s, float delay_s) {
    PressureModelParams params = make_deterministic_params();

    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = gain;
    params.first_order_tau_s = tau_s;
    params.first_order_delay_s = delay_s;
    params.sensor_range_bar = 10000.0f;
    params.motor_tau_s = 0.0f;

    return params;
}

static void run_steps(const PressureModelParams *params,
                      PressureModelState *state,
                      float target_rpm,
                      int cycles,
                      float dt_s,
                      PressureModelOutput *out) {
    int i;

    for (i = 0; i < cycles; ++i) {
        PressureModel_Step(params, state, target_rpm, dt_s, out);
    }
}

typedef struct {
    float head_pressure_bar;
    float tail_pressure_bar;
    float head_motor_rpm;
    float tail_motor_rpm;
    float tail_tooth_span_bar;
    float tail_tooth_min_phase;
    float tail_torque_trend;
} PressureModelSectionSummary;

static void summarize_open_loop_section(const PressureModelParams *params,
                                        const PressureModelOpenLoopReference *reference,
                                        PressureModelSectionSummary *summary) {
    PressureModelState state;
    PressureModelOutput out;
    float head_pressure_sum = 0.0f;
    float tail_pressure_sum = 0.0f;
    float head_motor_sum = 0.0f;
    float tail_motor_sum = 0.0f;
    float tail_torque_sum = 0.0f;
    float bins[26];
    int counts[26];
    int i;

    memset(&out, 0, sizeof(out));
    memset(summary, 0, sizeof(*summary));
    memset(bins, 0, sizeof(bins));
    memset(counts, 0, sizeof(counts));
    PressureModel_Reset(&state, 0x61616161u + (unsigned int)reference->command_rpm);

    for (i = 0; i < reference->sample_count; ++i) {
        float tooth_phase;
        int bin_index;

        PressureModel_Step(params, &state, reference->command_rpm, DT_S, &out);
        if (i < 2000) {
            head_pressure_sum += out.measured_pressure_bar;
            head_motor_sum += out.actual_motor_rpm;
        }
        if (i >= reference->sample_count - 2000) {
            tail_pressure_sum += out.measured_pressure_bar;
            tail_motor_sum += out.actual_motor_rpm;
            tail_torque_sum += out.estimated_torque_trend;
        }
        if (i >= reference->sample_count - 5000) {
            tooth_phase = fmodf(13.0f * state.pump_phase_rev, 1.0f);
            if (tooth_phase < 0.0f) {
                tooth_phase += 1.0f;
            }
            bin_index = (int)(tooth_phase * 26.0f);
            if (bin_index > 25) {
                bin_index = 25;
            }
            bins[bin_index] += out.measured_pressure_bar;
            counts[bin_index] += 1;
        }
    }

    summary->head_pressure_bar = head_pressure_sum / 2000.0f;
    summary->tail_pressure_bar = tail_pressure_sum / 2000.0f;
    summary->head_motor_rpm = head_motor_sum / 2000.0f;
    summary->tail_motor_rpm = tail_motor_sum / 2000.0f;
    summary->tail_torque_trend = tail_torque_sum / 2000.0f;

    {
        float min_value = 1.0e30f;
        float max_value = -1.0e30f;
        int min_index = 0;

        for (i = 0; i < 26; ++i) {
            float mean_value;
            if (counts[i] == 0) {
                continue;
            }
            mean_value = bins[i] / (float)counts[i];
            if (mean_value < min_value) {
                min_value = mean_value;
                min_index = i;
            }
            if (mean_value > max_value) {
                max_value = mean_value;
            }
        }

        summary->tail_tooth_span_bar = max_value - min_value;
        summary->tail_tooth_min_phase = ((float)min_index + 0.5f) / 26.0f;
    }
}

static int test_zero_speed_holds_zero_pressure(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x12345678u);

    run_steps(&params, &state, 0.0f, 2000, DT_S, &out);

    ASSERT_NEAR(out.actual_motor_rpm, 0.0f, RPM_EPS);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.measured_pressure_bar, 0.0f, PRESSURE_EPS);

    return 1;
}

static int test_explicit_first_order_tuning_contract(void) {
    PressureModelParams params = make_first_order_params(5.4f, 1.0f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x51515151u);

    ASSERT_TRUE(params.model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
    ASSERT_NEAR(params.first_order_k_bar_per_rpm, 5.4f, 1e-6f);
    ASSERT_NEAR(params.first_order_tau_s, 1.0f, 1e-6f);
    ASSERT_NEAR(params.first_order_delay_s, 0.0f, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);
    ASSERT_NEAR(state.first_order_prev_pressure_bar, 0.0f, 1e-6f);
    ASSERT_TRUE(state.first_order_buffer_index == 0);

    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);

    {
        float expected_pressure_bar =
            ((params.first_order_k_bar_per_rpm * out.actual_motor_rpm * DT_S) +
             (params.first_order_tau_s * 0.0f)) /
            (params.first_order_tau_s + DT_S);

        ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
        ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);
        ASSERT_NEAR(out.real_pressure_bar, expected_pressure_bar, 1e-6f);
    }

    return 1;
}

static int test_init_params_expose_open_loop_fit_knobs(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;
    PressureModel_Reset(&state, 0x51515151u);
    PressureModel_Step(&params, &state, 0.0f, DT_S, &out);

    ASSERT_NEAR(params.veff_base_m3, 4.4e-4f, 1e-9f);
    ASSERT_NEAR(params.leak_base_m3_pa_s, 1.2245e-12f, 1e-16f);
    ASSERT_NEAR(params.flow_ripple_ratio, 0.08f, 1e-6f);
    ASSERT_NEAR(params.tooth_drop_depth_base, 0.16f, 1e-6f);
    ASSERT_NEAR(params.tooth_drop_width_ratio, 0.20f, 1e-6f);
    ASSERT_NEAR(params.tooth_drop_phase_base, 0.58f, 1e-6f);
    ASSERT_NEAR(params.veff_speed_scale[0], 1.18f, 1e-6f);
    ASSERT_NEAR(params.veff_speed_scale[1], 1.00f, 1e-6f);
    ASSERT_NEAR(params.veff_speed_scale[2], 0.86f, 1e-6f);
    ASSERT_NEAR(params.leak_speed_scale[0], 1.27f, 1e-6f);
    ASSERT_NEAR(params.leak_speed_scale[1], 1.00f, 1e-6f);
    ASSERT_NEAR(params.leak_speed_scale[2], 0.875f, 1e-6f);
    ASSERT_NEAR(params.drop_depth_scale[0], 1.00f, 1e-6f);
    ASSERT_NEAR(params.drop_depth_scale[1], 0.62f, 1e-6f);
    ASSERT_NEAR(params.drop_depth_scale[2], 0.30f, 1e-6f);
    ASSERT_NEAR(params.drop_phase_offset[0], 0.00f, 1e-6f);
    ASSERT_NEAR(params.drop_phase_offset[1], 0.07f, 1e-6f);
    ASSERT_NEAR(params.drop_phase_offset[2], 0.11f, 1e-6f);
    ASSERT_NEAR(params.torque_bias, 400.0f, 1e-6f);
    ASSERT_NEAR(params.torque_from_pressure_gain, 110.0f, 1e-6f);
    ASSERT_NEAR(params.torque_from_speed_gain, 8.0f, 1e-6f);
    ASSERT_NEAR(out.estimated_torque_trend, 400.0f, 1e-4f);

    return 1;
}

static int test_open_loop_sections_match_measured_pressure_envelope(void) {
    PressureModelParams params = make_deterministic_params();
    int i;

    for (i = 0; i < PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT; ++i) {
        PressureModelSectionSummary summary;
        const PressureModelOpenLoopReference *reference = &kPressureModelOpenLoopReference[i];

        summarize_open_loop_section(&params, reference, &summary);

        ASSERT_NEAR(summary.head_pressure_bar, reference->head_pressure_bar, 4.0f);
        ASSERT_NEAR(summary.tail_pressure_bar, reference->tail_pressure_bar, 3.0f);
        ASSERT_NEAR(summary.head_motor_rpm, reference->head_motor_rpm, 0.8f);
        ASSERT_NEAR(summary.tail_motor_rpm, reference->tail_motor_rpm, 0.4f);
    }

    return 1;
}

static int test_open_loop_sections_match_tooth_phase_and_torque_trend(void) {
    PressureModelParams params = make_deterministic_params();
    float previous_tail_torque = -1.0f;
    int i;

    for (i = 0; i < PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT; ++i) {
        PressureModelSectionSummary summary;
        const PressureModelOpenLoopReference *reference = &kPressureModelOpenLoopReference[i];

        summarize_open_loop_section(&params, reference, &summary);
        ASSERT_NEAR(summary.tail_tooth_span_bar, reference->tail_tooth_span_bar, 1.5f);
        ASSERT_NEAR(summary.tail_tooth_min_phase, reference->tail_tooth_min_phase, 0.08f);
        ASSERT_TRUE(summary.tail_torque_trend > previous_tail_torque);
        ASSERT_TRUE(summary.tail_torque_trend > reference->tail_torque_trend * 0.80f);
        ASSERT_TRUE(summary.tail_torque_trend < reference->tail_torque_trend * 1.20f);
        previous_tail_torque = summary.tail_torque_trend;
    }

    return 1;
}

static int test_motor_state_is_continuous_across_steps(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out0;
    PressureModelOutput out1;

    memset(&out0, 0, sizeof(out0));
    memset(&out1, 0, sizeof(out1));
    PressureModel_Reset(&state, 0x12345678u);

    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out0);
    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out1);

    ASSERT_TRUE(out0.actual_motor_rpm > 0.0f);
    ASSERT_TRUE(out0.actual_motor_rpm < 1000.0f);
    ASSERT_TRUE(out1.actual_motor_rpm > out0.actual_motor_rpm);

    return 1;
}

static int test_first_order_zero_input_holds_zero_pressure(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x78787878u);

    run_steps(&params, &state, 0.0f, 200, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.measured_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.actual_motor_rpm, 0.0f, RPM_EPS);

    return 1;
}

static int test_first_order_tau_zero_matches_gain_times_actual_rpm(void) {
    PressureModelParams params = make_first_order_params(0.25f, 0.0f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x79797979u);
    PressureModel_Step(&params, &state, 120.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar,
                params.first_order_k_bar_per_rpm * out.actual_motor_rpm,
                1e-4f);
    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);

    return 1;
}

static int test_first_order_tau_positive_matches_discrete_recurrence_from_reset(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;
    float expected_pressure_bar = 0.0f;
    int i;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7c7c7c7cu);

    for (i = 0; i < 4; ++i) {
        PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
        expected_pressure_bar =
            ((params.first_order_k_bar_per_rpm * 100.0f * DT_S) +
             (params.first_order_tau_s * expected_pressure_bar)) /
            (params.first_order_tau_s + DT_S);

        ASSERT_NEAR(out.actual_motor_rpm, 100.0f, RPM_EPS);
        ASSERT_NEAR(out.real_pressure_bar, expected_pressure_bar, 1e-6f);
        ASSERT_NEAR(out.measured_pressure_bar, expected_pressure_bar, 1e-6f);
    }

    return 1;
}

static int test_first_order_delay_defers_visible_output(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.0f, 0.003f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7a7a7a7au);

    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_TRUE(out.real_pressure_bar > 0.0f);

    return 1;
}

static int test_first_order_outputs_measured_equal_real_and_zero_flow_terms(void) {
    PressureModelParams params = make_first_order_params(0.1f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7b7b7b7bu);
    run_steps(&params, &state, 250.0f, 50, DT_S, &out);

    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);
    ASSERT_NEAR(out.pump_flow_m3_s, 0.0f, 1e-6f);
    ASSERT_NEAR(out.net_flow_m3_s, 0.0f, 1e-6f);

    return 1;
}

static int test_invalid_model_type_matches_physical_branch(void) {
    PressureModelParams physical_params = make_deterministic_params();
    PressureModelParams invalid_params = physical_params;
    PressureModelState physical_state;
    PressureModelState invalid_state;
    PressureModelOutput physical_out;
    PressureModelOutput invalid_out;
    int i;

    invalid_params.model_type = 99u;
    memset(&physical_out, 0, sizeof(physical_out));
    memset(&invalid_out, 0, sizeof(invalid_out));
    PressureModel_Reset(&physical_state, 0x7c7c7c7cu);
    PressureModel_Reset(&invalid_state, 0x7c7c7c7cu);

    for (i = 0; i < 500; ++i) {
        PressureModel_Step(&physical_params, &physical_state, 40.0f, DT_S, &physical_out);
        PressureModel_Step(&invalid_params, &invalid_state, 40.0f, DT_S, &invalid_out);
    }

    ASSERT_NEAR(invalid_out.real_pressure_bar, physical_out.real_pressure_bar, 1e-6f);
    ASSERT_NEAR(invalid_out.measured_pressure_bar, physical_out.measured_pressure_bar, 1e-6f);
    ASSERT_NEAR(invalid_out.actual_motor_rpm, physical_out.actual_motor_rpm, 1e-6f);

    return 1;
}

static int test_switch_from_physical_to_first_order_preserves_pressure(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float charged_pressure;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7d7d7d7du);
    run_steps(&params, &state, 40.0f, 12000, DT_S, &out);
    charged_pressure = out.real_pressure_bar;

    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 1.0f;
    params.first_order_tau_s = 0.2f;
    params.first_order_delay_s = 0.0f;
    PressureModel_Step(&params, &state, 40.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_NEAR(out.measured_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);

    return 1;
}

static int test_switch_from_first_order_to_physical_preserves_pressure(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;
    float charged_pressure;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7e7e7e7eu);
    run_steps(&params, &state, 200.0f, 400, DT_S, &out);
    charged_pressure = out.real_pressure_bar;

    params = make_deterministic_params();
    PressureModel_Step(&params, &state, 200.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_NEAR(out.measured_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);

    return 1;
}

static int count_visible_tooth_valleys(const float *measured,
                                       const float *real,
                                       int samples,
                                       float min_gap_bar) {
    int i;
    int valleys = 0;

    for (i = 1; i + 1 < samples; ++i) {
        if (measured[i] < measured[i - 1] &&
            measured[i] <= measured[i + 1] &&
            (real[i] - measured[i]) >= min_gap_bar) {
            ++valleys;
        }
    }

    return valleys;
}

static int test_negative_speed_depressurizes_faster_than_passive_leak(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState charged_state;
    PressureModelState leak_only_state;
    PressureModelState reverse_state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&charged_state, 0x11111111u);
    run_steps(&params, &charged_state, 10.0f, 15000, DT_S, &out);

    leak_only_state = charged_state;
    reverse_state = charged_state;

    run_steps(&params, &leak_only_state, 0.0f, 2000, DT_S, &out);
    run_steps(&params, &reverse_state, -50.0f, 2000, DT_S, &out);

    ASSERT_TRUE(reverse_state.pressure_pa < leak_only_state.pressure_pa);
    ASSERT_TRUE(reverse_state.pressure_pa >= 0.0f);

    return 1;
}

static int test_tooth_drop_is_visible_once_per_tooth(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float measured[100];
    float real[100];
    int i;
    int valleys;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x22222222u);

    run_steps(&params, &state, 600.0f, 2000, DT_S, &out);
    for (i = 0; i < 100; ++i) {
        PressureModel_Step(&params, &state, 600.0f, DT_S, &out);
        measured[i] = out.measured_pressure_bar;
        real[i] = out.real_pressure_bar;
    }

    valleys = count_visible_tooth_valleys(measured, real, 100, 0.05f);
    ASSERT_TRUE(valleys >= 10 && valleys <= 16);

    return 1;
}

static int test_legacy_tooth_drop_ratio_still_controls_visible_tooth_drop(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float max_gap_bar = 0.0f;
    int i;

    params.tooth_drop_depth_ratio = 0.0f;
    params.sensor_range_bar = 10000.0f;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x26262626u);

    run_steps(&params, &state, 600.0f, 2000, DT_S, &out);
    for (i = 0; i < 100; ++i) {
        PressureModel_Step(&params, &state, 600.0f, DT_S, &out);
        if ((out.real_pressure_bar - out.measured_pressure_bar) > max_gap_bar) {
            max_gap_bar = out.real_pressure_bar - out.measured_pressure_bar;
        }
    }

    ASSERT_NEAR(max_gap_bar, 0.0f, 1e-3f);

    return 1;
}

static int test_legacy_leak_coeff_still_controls_pressure_response(void) {
    PressureModelParams baseline_params = make_deterministic_params();
    PressureModelParams legacy_params = baseline_params;
    PressureModelState baseline_state;
    PressureModelState legacy_state;
    PressureModelOutput baseline_out;
    PressureModelOutput legacy_out;

    legacy_params.leak_coeff_m3_pa_s = baseline_params.leak_coeff_m3_pa_s * 2.0f;
    baseline_params.sensor_range_bar = 10000.0f;
    legacy_params.sensor_range_bar = 10000.0f;

    memset(&baseline_out, 0, sizeof(baseline_out));
    memset(&legacy_out, 0, sizeof(legacy_out));
    PressureModel_Reset(&baseline_state, 0x28282828u);
    PressureModel_Reset(&legacy_state, 0x28282828u);

    run_steps(&baseline_params, &baseline_state, 10.0f, 15000, DT_S, &baseline_out);
    run_steps(&legacy_params, &legacy_state, 10.0f, 15000, DT_S, &legacy_out);

    ASSERT_TRUE(legacy_out.real_pressure_bar < baseline_out.real_pressure_bar - 10.0f);

    return 1;
}

static int test_legacy_chamber_volume_still_controls_pressure_rise(void) {
    PressureModelParams baseline_params = make_deterministic_params();
    PressureModelParams legacy_params = baseline_params;
    PressureModelState baseline_state;
    PressureModelState legacy_state;
    PressureModelOutput baseline_out;
    PressureModelOutput legacy_out;

    legacy_params.chamber_volume_m3 = baseline_params.chamber_volume_m3 * 2.0f;
    baseline_params.sensor_range_bar = 10000.0f;
    legacy_params.sensor_range_bar = 10000.0f;

    memset(&baseline_out, 0, sizeof(baseline_out));
    memset(&legacy_out, 0, sizeof(legacy_out));
    PressureModel_Reset(&baseline_state, 0x29292929u);
    PressureModel_Reset(&legacy_state, 0x29292929u);

    run_steps(&baseline_params, &baseline_state, 10.0f, 500, DT_S, &baseline_out);
    run_steps(&legacy_params, &legacy_state, 10.0f, 500, DT_S, &legacy_out);

    ASSERT_TRUE(legacy_out.real_pressure_bar < baseline_out.real_pressure_bar - 5.0f);

    return 1;
}

static int test_torque_speed_gain_affects_estimated_torque_trend(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float expected_speed_torque;

    params.torque_bias = 0.0f;
    params.torque_from_pressure_gain = 0.0f;
    params.torque_from_speed_gain = 0.01f;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x27272727u);

    run_steps(&params, &state, 100.0f, 200, DT_S, &out);

    expected_speed_torque = params.torque_from_speed_gain * fabsf(out.actual_motor_rpm);
    ASSERT_TRUE(out.actual_motor_rpm > 0.0f);
    ASSERT_NEAR(out.estimated_torque_trend, expected_speed_torque, 1e-5f);

    return 1;
}

static int test_torque_bias_and_pressure_gain_affect_estimated_torque_trend(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float expected_torque;

    params.torque_bias = 1.25f;
    params.torque_from_pressure_gain = 0.05f;
    params.torque_from_speed_gain = 0.0f;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x2b2b2b2bu);

    run_steps(&params, &state, 10.0f, 500, DT_S, &out);

    expected_torque = params.torque_bias +
                      params.torque_from_pressure_gain * out.measured_pressure_bar;
    ASSERT_NEAR(out.estimated_torque_trend, expected_torque, 1e-5f);

    return 1;
}

static int test_relief_caps_measured_output_at_two_hundred_fifty_bar(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x33333333u);

    run_steps(&params, &state, 2000.0f, 30000, DT_S, &out);

    ASSERT_TRUE(out.measured_pressure_bar <= 250.0f + 1e-3f);
    ASSERT_TRUE(out.relief_active);

    return 1;
}

static int test_noise_control_is_repeatable_with_fixed_seed(void) {
    PressureModelParams params;
    PressureModelState state_a;
    PressureModelState state_b;
    PressureModelOutput out_a;
    PressureModelOutput out_b;
    int i;

    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 1u;
    params.enable_motor_noise = 1u;
    params.enable_process_noise = 1u;
    params.process_noise_std_m3_s = 1.0e-7f;

    memset(&out_a, 0, sizeof(out_a));
    memset(&out_b, 0, sizeof(out_b));
    PressureModel_Reset(&state_a, 0x44444444u);
    PressureModel_Reset(&state_b, 0x44444444u);

    for (i = 0; i < 500; ++i) {
        PressureModel_Step(&params, &state_a, 800.0f, DT_S, &out_a);
        PressureModel_Step(&params, &state_b, 800.0f, DT_S, &out_b);
        ASSERT_NEAR(out_a.measured_pressure_bar, out_b.measured_pressure_bar, 1e-6f);
        ASSERT_NEAR(out_a.actual_motor_rpm, out_b.actual_motor_rpm, 1e-6f);
    }

    return 1;
}

static int test_legacy_pressure_update_keeps_motor_state_between_calls(void) {
    float pressure_state = 0.0f;
    float real_pressure0 = 0.0f;
    float real_pressure1 = 0.0f;
    float rpm0 = 0.0f;
    float rpm1 = 0.0f;
    float measured0;
    float measured1;

    measured0 = pressure_update(1000.0f, 0.000f, &pressure_state, &real_pressure0, &rpm0);
    measured1 = pressure_update(1000.0f, 0.001f, &pressure_state, &real_pressure1, &rpm1);

    ASSERT_TRUE(rpm1 > rpm0);
    ASSERT_TRUE(real_pressure1 >= real_pressure0);
    ASSERT_TRUE(measured0 >= 0.0f && measured1 >= 0.0f);

    return 1;
}

int main(void) {
    int passed = 0;
    int failed = 0;

    if (test_zero_speed_holds_zero_pressure()) {
        ++passed;
        printf("PASS test_zero_speed_holds_zero_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_zero_speed_holds_zero_pressure\n");
    }
    if (test_explicit_first_order_tuning_contract()) {
        ++passed;
        printf("PASS test_explicit_first_order_tuning_contract\n");
    } else {
        ++failed;
        printf("FAIL test_explicit_first_order_tuning_contract\n");
    }

    if (test_init_params_expose_open_loop_fit_knobs()) {
        ++passed;
        printf("PASS test_init_params_expose_open_loop_fit_knobs\n");
    } else {
        ++failed;
        printf("FAIL test_init_params_expose_open_loop_fit_knobs\n");
    }

    if (test_open_loop_sections_match_measured_pressure_envelope()) {
        ++passed;
        printf("PASS test_open_loop_sections_match_measured_pressure_envelope\n");
    } else {
        ++failed;
        printf("FAIL test_open_loop_sections_match_measured_pressure_envelope\n");
    }

    if (test_open_loop_sections_match_tooth_phase_and_torque_trend()) {
        ++passed;
        printf("PASS test_open_loop_sections_match_tooth_phase_and_torque_trend\n");
    } else {
        ++failed;
        printf("FAIL test_open_loop_sections_match_tooth_phase_and_torque_trend\n");
    }

    if (test_motor_state_is_continuous_across_steps()) {
        ++passed;
        printf("PASS test_motor_state_is_continuous_across_steps\n");
    } else {
        ++failed;
        printf("FAIL test_motor_state_is_continuous_across_steps\n");
    }

    if (test_first_order_zero_input_holds_zero_pressure()) {
        ++passed;
        printf("PASS test_first_order_zero_input_holds_zero_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_zero_input_holds_zero_pressure\n");
    }

    if (test_first_order_tau_zero_matches_gain_times_actual_rpm()) {
        ++passed;
        printf("PASS test_first_order_tau_zero_matches_gain_times_actual_rpm\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_tau_zero_matches_gain_times_actual_rpm\n");
    }

    if (test_first_order_tau_positive_matches_discrete_recurrence_from_reset()) {
        ++passed;
        printf("PASS test_first_order_tau_positive_matches_discrete_recurrence_from_reset\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_tau_positive_matches_discrete_recurrence_from_reset\n");
    }

    if (test_first_order_delay_defers_visible_output()) {
        ++passed;
        printf("PASS test_first_order_delay_defers_visible_output\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_delay_defers_visible_output\n");
    }

    if (test_first_order_outputs_measured_equal_real_and_zero_flow_terms()) {
        ++passed;
        printf("PASS test_first_order_outputs_measured_equal_real_and_zero_flow_terms\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_outputs_measured_equal_real_and_zero_flow_terms\n");
    }

    if (test_invalid_model_type_matches_physical_branch()) {
        ++passed;
        printf("PASS test_invalid_model_type_matches_physical_branch\n");
    } else {
        ++failed;
        printf("FAIL test_invalid_model_type_matches_physical_branch\n");
    }

    if (test_switch_from_physical_to_first_order_preserves_pressure()) {
        ++passed;
        printf("PASS test_switch_from_physical_to_first_order_preserves_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_switch_from_physical_to_first_order_preserves_pressure\n");
    }

    if (test_switch_from_first_order_to_physical_preserves_pressure()) {
        ++passed;
        printf("PASS test_switch_from_first_order_to_physical_preserves_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_switch_from_first_order_to_physical_preserves_pressure\n");
    }

    if (test_negative_speed_depressurizes_faster_than_passive_leak()) {
        ++passed;
        printf("PASS test_negative_speed_depressurizes_faster_than_passive_leak\n");
    } else {
        ++failed;
        printf("FAIL test_negative_speed_depressurizes_faster_than_passive_leak\n");
    }

    if (test_tooth_drop_is_visible_once_per_tooth()) {
        ++passed;
        printf("PASS test_tooth_drop_is_visible_once_per_tooth\n");
    } else {
        ++failed;
        printf("FAIL test_tooth_drop_is_visible_once_per_tooth\n");
    }

    if (test_legacy_tooth_drop_ratio_still_controls_visible_tooth_drop()) {
        ++passed;
        printf("PASS test_legacy_tooth_drop_ratio_still_controls_visible_tooth_drop\n");
    } else {
        ++failed;
        printf("FAIL test_legacy_tooth_drop_ratio_still_controls_visible_tooth_drop\n");
    }

    if (test_legacy_leak_coeff_still_controls_pressure_response()) {
        ++passed;
        printf("PASS test_legacy_leak_coeff_still_controls_pressure_response\n");
    } else {
        ++failed;
        printf("FAIL test_legacy_leak_coeff_still_controls_pressure_response\n");
    }

    if (test_legacy_chamber_volume_still_controls_pressure_rise()) {
        ++passed;
        printf("PASS test_legacy_chamber_volume_still_controls_pressure_rise\n");
    } else {
        ++failed;
        printf("FAIL test_legacy_chamber_volume_still_controls_pressure_rise\n");
    }

    if (test_torque_speed_gain_affects_estimated_torque_trend()) {
        ++passed;
        printf("PASS test_torque_speed_gain_affects_estimated_torque_trend\n");
    } else {
        ++failed;
        printf("FAIL test_torque_speed_gain_affects_estimated_torque_trend\n");
    }

    if (test_torque_bias_and_pressure_gain_affect_estimated_torque_trend()) {
        ++passed;
        printf("PASS test_torque_bias_and_pressure_gain_affect_estimated_torque_trend\n");
    } else {
        ++failed;
        printf("FAIL test_torque_bias_and_pressure_gain_affect_estimated_torque_trend\n");
    }

    if (test_relief_caps_measured_output_at_two_hundred_fifty_bar()) {
        ++passed;
        printf("PASS test_relief_caps_measured_output_at_two_hundred_fifty_bar\n");
    } else {
        ++failed;
        printf("FAIL test_relief_caps_measured_output_at_two_hundred_fifty_bar\n");
    }

    if (test_noise_control_is_repeatable_with_fixed_seed()) {
        ++passed;
        printf("PASS test_noise_control_is_repeatable_with_fixed_seed\n");
    } else {
        ++failed;
        printf("FAIL test_noise_control_is_repeatable_with_fixed_seed\n");
    }

    if (test_legacy_pressure_update_keeps_motor_state_between_calls()) {
        ++passed;
        printf("PASS test_legacy_pressure_update_keeps_motor_state_between_calls\n");
    } else {
        ++failed;
        printf("FAIL test_legacy_pressure_update_keeps_motor_state_between_calls\n");
    }

    printf("Passed: %d Failed: %d\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
