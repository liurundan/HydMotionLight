#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "pump_converter.h"
#include "pressure_controller.h"
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

static PressureModelParams make_physical_params(void) {
    PressureModelParams params = make_deterministic_params();

    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL;

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
    float peak_real_pressure_bar;
    float tail_filtered_min_bar;
    float tail_filtered_max_bar;
} ClosedLoopPressureMetrics;

typedef struct {
    float target_bar;
    int total_steps;
    int tail_start_step;
    float flow_to_speed_gain;
    float pump_speed_limit_rpm;
    PressureModelParams params;
    HYD_MotionSegment segment;
} ClosedLoopPressureCase;

static HYD_MotionSegment make_closed_loop_pressure_segment(float target_bar,
                                                           float flow_to_speed_gain,
                                                           float pump_speed_limit_rpm,
                                                           float system_gain) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = 10.0;
    segment.targetPressure = target_bar;
    segment.maxFlow = pump_speed_limit_rpm / flow_to_speed_gain;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = target_bar * 3.0f;
    segment.pressureFilterAlpha = 1.0f;
    segment.pressureDerivativeFilterAlpha = 1.0f;
    segment.systemGain = system_gain;
    segment.pressureRbfConfig.minKp = 0.040f;
    segment.pressureRbfConfig.maxKp = 0.060f;
    segment.pressureRbfConfig.minKi = 0.0008f;
    segment.pressureRbfConfig.maxKi = 0.0016f;
    segment.pressureRbfConfig.minKd = 0.015f;
    segment.pressureRbfConfig.maxKd = 0.035f;
    segment.pressureRbfConfig.etaW = 0.0020f;
    segment.pressureRbfConfig.etaC = 0.0020f;
    segment.pressureRbfConfig.etaB = 0.0010f;
    segment.pressureRbfConfig.etaP = 0.00010f;
    segment.pressureRbfConfig.etaI = 0.00005f;
    segment.pressureRbfConfig.etaD = 0.00010f;
    segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0f;

    return segment;
}

static ClosedLoopPressureCase make_first_order_closed_loop_case(void) {
    ClosedLoopPressureCase test_case;

    memset(&test_case, 0, sizeof(test_case));
    test_case.target_bar = 100.0f;
    test_case.total_steps = 8000;
    test_case.tail_start_step = 7000;
    test_case.flow_to_speed_gain = 20.0f;
    test_case.pump_speed_limit_rpm = 1800.0f;
    test_case.params = make_first_order_params(5.4f, 1.0f, 0.0f);
    test_case.segment = make_closed_loop_pressure_segment(test_case.target_bar,
                                                          test_case.flow_to_speed_gain,
                                                          test_case.pump_speed_limit_rpm,
                                                          5.4f * test_case.flow_to_speed_gain);
    return test_case;
}

static ClosedLoopPressureCase make_physical_closed_loop_case(void) {
    ClosedLoopPressureCase test_case;

    memset(&test_case, 0, sizeof(test_case));
    test_case.target_bar = 100.0f;
    test_case.total_steps = 30000;
    test_case.tail_start_step = 29000;
    test_case.flow_to_speed_gain = 20.0f;
    test_case.pump_speed_limit_rpm = 1800.0f;
    test_case.params = make_physical_params();
    test_case.params.sensor_range_bar = 10000.0f;
    test_case.params.flow_ripple_ratio = 0.0f;
    test_case.params.tooth_drop_depth_ratio = 0.0f;
    test_case.params.tooth_drop_depth_base = 0.0f;
    test_case.segment = make_closed_loop_pressure_segment(test_case.target_bar,
                                                          test_case.flow_to_speed_gain,
                                                          test_case.pump_speed_limit_rpm,
                                                          30.0f);
    test_case.segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    test_case.segment.pressureFilterAlpha = 0.20f;
    test_case.segment.pressureKp = 0.05f;
    test_case.segment.pressureKi = 0.005f;
    test_case.segment.pressureIntegralLimit = test_case.segment.maxFlow;
    return test_case;
}

static float closed_loop_tail_filtered_p2p_bar(const ClosedLoopPressureMetrics *metrics) {
    return metrics->tail_filtered_max_bar - metrics->tail_filtered_min_bar;
}

static void run_closed_loop_pressure_case(const ClosedLoopPressureCase *test_case,
                                          ClosedLoopPressureMetrics *metrics) {
    PressureModelState plant_state;
    PressureModelOutput plant_out;
    HYD_PressureControllerState controller_state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PumpConverterInput pump_input;
    HYD_PumpConverterOutput pump_output;
    int step;

    memset(metrics, 0, sizeof(*metrics));
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    memset(&pump_input, 0, sizeof(pump_input));
    memset(&pump_output, 0, sizeof(pump_output));
    metrics->tail_filtered_min_bar = 1.0e30f;
    metrics->tail_filtered_max_bar = -1.0e30f;

    PressureModel_Reset(&plant_state, 0x5a5a5a5au);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);

    for (step = 0; step < test_case->total_steps; ++step) {
        input.targetPressure = test_case->target_bar;
        input.measuredPressure = plant_out.measured_pressure_bar;
        input.feedforwardFlow = 0.0;
        input.outputMin = 0.0;
        input.outputMax = test_case->segment.maxFlow;
        input.flowToPumpSpeedGain = test_case->flow_to_speed_gain;
        input.pumpSpeedLimit = test_case->pump_speed_limit_rpm;
        input.timestamp = (HYD_REAL)((step + 1) * DT_S);

        HYD_PressureController_Execute(&test_case->segment, &controller_state, &input, &output);

        pump_input.requestedFlow = output.outputFlow;
        pump_input.flowToPumpSpeedGain = input.flowToPumpSpeedGain;
        pump_input.pumpSpeedLimit = input.pumpSpeedLimit;
        pump_input.direction = test_case->segment.direction;
        HYD_PumpConverter_Execute(&pump_input, &pump_output);

        PressureModel_Step(&test_case->params,
                           &plant_state,
                           (float)pump_output.pumpSpeed,
                           DT_S,
                           &plant_out);

        if (plant_out.real_pressure_bar > metrics->peak_real_pressure_bar) {
            metrics->peak_real_pressure_bar = plant_out.real_pressure_bar;
        }

        if (step >= test_case->tail_start_step) {
            float filtered_pressure_bar = (float)output.filteredPressure;

            if (filtered_pressure_bar < metrics->tail_filtered_min_bar) {
                metrics->tail_filtered_min_bar = filtered_pressure_bar;
            }
            if (filtered_pressure_bar > metrics->tail_filtered_max_bar) {
                metrics->tail_filtered_max_bar = filtered_pressure_bar;
            }
        }
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

    {
        const float command_rpm = 100.0f;
        const float prev_pressure_bar = 0.0f;
        float expected_pressure_bar;

        PressureModel_Step(&params, &state, command_rpm, DT_S, &out);
        expected_pressure_bar =
            ((params.first_order_k_bar_per_rpm * command_rpm * DT_S) +
             (params.first_order_tau_s * prev_pressure_bar)) /
            (params.first_order_tau_s + DT_S);

        ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
        ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);
        ASSERT_NEAR(out.real_pressure_bar, expected_pressure_bar, 1e-6f);
    }

    return 1;
}

static int test_init_params_expose_runtime_defaults(void) {
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

    ASSERT_TRUE(params.model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
    ASSERT_NEAR(params.first_order_k_bar_per_rpm, 5.4f, 1e-6f);
    ASSERT_NEAR(params.first_order_tau_s, 1.0f, 1e-6f);
    ASSERT_NEAR(params.first_order_delay_s, 0.0f, 1e-6f);
    ASSERT_TRUE(params.sensor_range_bar > 0.0f);
    ASSERT_TRUE(out.measured_pressure_bar >= 0.0f);

    return 1;
}

static int test_motor_state_is_continuous_across_steps(void) {
    PressureModelParams params = make_physical_params();
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
    PressureModelParams physical_params = make_physical_params();
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
    PressureModelParams params = make_physical_params();
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

    params = make_physical_params();
    PressureModel_Step(&params, &state, 200.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_NEAR(out.measured_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);

    return 1;
}

static int test_negative_speed_depressurizes_faster_than_passive_leak(void) {
    PressureModelParams params = make_physical_params();
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

static int test_relief_caps_measured_output_at_two_hundred_fifty_bar(void) {
    PressureModelParams params = make_physical_params();
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

    params = make_physical_params();
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

static int test_physical_closed_loop_steady_state_ripple_below_one_percent_of_pset(void) {
    ClosedLoopPressureCase test_case = make_physical_closed_loop_case();
    ClosedLoopPressureMetrics metrics;

    run_closed_loop_pressure_case(&test_case, &metrics);

    ASSERT_TRUE(closed_loop_tail_filtered_p2p_bar(&metrics) < test_case.target_bar * 0.01f);

    return 1;
}

static int test_first_order_closed_loop_overshoot_within_five_percent_of_pset(void) {
    ClosedLoopPressureCase test_case = make_first_order_closed_loop_case();
    ClosedLoopPressureMetrics metrics;

    run_closed_loop_pressure_case(&test_case, &metrics);

    ASSERT_TRUE(metrics.peak_real_pressure_bar <= test_case.target_bar * 1.05f);

    return 1;
}

static int test_first_order_closed_loop_steady_state_ripple_below_one_percent_of_pset(void) {
    ClosedLoopPressureCase test_case = make_first_order_closed_loop_case();
    ClosedLoopPressureMetrics metrics;

    run_closed_loop_pressure_case(&test_case, &metrics);

    ASSERT_TRUE(closed_loop_tail_filtered_p2p_bar(&metrics) < test_case.target_bar * 0.01f);

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

    if (test_init_params_expose_runtime_defaults()) {
        ++passed;
        printf("PASS test_init_params_expose_runtime_defaults\n");
    } else {
        ++failed;
        printf("FAIL test_init_params_expose_runtime_defaults\n");
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

    if (test_physical_closed_loop_steady_state_ripple_below_one_percent_of_pset()) {
        ++passed;
        printf("PASS test_physical_closed_loop_steady_state_ripple_below_one_percent_of_pset\n");
    } else {
        ++failed;
        printf("FAIL test_physical_closed_loop_steady_state_ripple_below_one_percent_of_pset\n");
    }

    if (test_first_order_closed_loop_overshoot_within_five_percent_of_pset()) {
        ++passed;
        printf("PASS test_first_order_closed_loop_overshoot_within_five_percent_of_pset\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_closed_loop_overshoot_within_five_percent_of_pset\n");
    }

    if (test_first_order_closed_loop_steady_state_ripple_below_one_percent_of_pset()) {
        ++passed;
        printf("PASS test_first_order_closed_loop_steady_state_ripple_below_one_percent_of_pset\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_closed_loop_steady_state_ripple_below_one_percent_of_pset\n");
    }

    printf("Passed: %d Failed: %d\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
