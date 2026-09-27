#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "pump_converter.h"
#include "pressure_controller.h"
#include "pressure_model.h"

#define HOLD_DT_S 0.001f
#define HOLD_TOTAL_STEPS 30000
#define HOLD_SETTLE_START_STEP 10000
#define HOLD_TARGET_BAR 100.0f
#define HOLD_FLOW_TO_SPEED_GAIN 20.0f
#define HOLD_PUMP_SPEED_LIMIT 1800.0f

typedef struct {
    float target_bar;
    int total_steps;
    int settle_start_step;
    float dt_s;
    PressureModelParams params;
    HYD_MotionSegment segment;
} HoldCaseConfig;

typedef struct {
    float real_p2p_bar;
    float measured_p2p_bar;
    float filtered_p2p_bar;
    float filtered_mae_bar;
    float output_p2p_lmin;
} HoldMetrics;

static PressureModelParams make_default_model_params(void);
static HYD_MotionSegment make_default_rbf_segment(float target_bar);
static HoldCaseConfig make_default_hold_case(void);
static void run_hold_case(const HoldCaseConfig *config, HoldMetrics *metrics);
static void test_current_100_bar_hold_preserves_visible_ripple_with_bounded_hold_error(void);
static void test_sensor_noise_changes_measurement_without_changing_real_pressure(void);
static void test_stronger_filter_changes_closed_loop_hold_metrics(void);
static void test_disabling_pressure_accel_feedforward_changes_hold_metrics(void);
static void test_disabling_gain_compensation_increases_hold_error_materially(void);

static PressureModelParams make_default_model_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    /* This suite measures pump-to-pressure ripple, which only exists in the
     * calibrated physical model. Init defaults intentionally use first-order
     * mode for lightweight generic simulation. */
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params.sensor_noise_std_bar = 0.8f;
    params.motor_noise_std_rpm = 1.0f;
    params.tooth_drop_depth_ratio = 0.34f;
    return params;
}

static HYD_MotionSegment make_default_rbf_segment(float target_bar) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = (HYD_REAL)(HOLD_TOTAL_STEPS * HOLD_DT_S);
    segment.targetPressure = target_bar;
    segment.maxFlow = HOLD_PUMP_SPEED_LIMIT / HOLD_FLOW_TO_SPEED_GAIN;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.pressureFilterAlpha = 0.20;
    segment.pressureDerivativeFilterAlpha = 1.0;
    segment.systemGain = 30.0;
    segment.pressureRbfConfig.minKp = 0.5;
    segment.pressureRbfConfig.maxKp = 1.2;
    segment.pressureRbfConfig.minKi = 0.005;
    segment.pressureRbfConfig.maxKi = 0.050;
    segment.pressureRbfConfig.minKd = 0.5;
    segment.pressureRbfConfig.maxKd = 2.0;
    return segment;
}

static HoldCaseConfig make_default_hold_case(void) {
    HoldCaseConfig config;

    memset(&config, 0, sizeof(config));
    config.target_bar = HOLD_TARGET_BAR;
    config.total_steps = HOLD_TOTAL_STEPS;
    config.settle_start_step = HOLD_SETTLE_START_STEP;
    config.dt_s = HOLD_DT_S;
    config.params = make_default_model_params();
    config.segment = make_default_rbf_segment(config.target_bar);
    return config;
}

static void run_hold_case(const HoldCaseConfig *config, HoldMetrics *metrics) {
    PressureModelState plant_state;
    PressureModelOutput plant_out;
    HYD_PressureControllerState controller_state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PumpConverterInput pump_input;
    HYD_PumpConverterOutput pump_output;
    float real_min = 1.0e30f;
    float real_max = -1.0e30f;
    float measured_min = 1.0e30f;
    float measured_max = -1.0e30f;
    float filtered_min = 1.0e30f;
    float filtered_max = -1.0e30f;
    float output_min = 1.0e30f;
    float output_max = -1.0e30f;
    float filtered_abs_error_sum = 0.0f;
    int filtered_samples = 0;
    int step;

    memset(metrics, 0, sizeof(*metrics));
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    memset(&pump_input, 0, sizeof(pump_input));
    memset(&pump_output, 0, sizeof(pump_output));

    PressureModel_Reset(&plant_state, 0x5a5a5a5au);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);

    for (step = 0; step < config->total_steps; ++step) {
        input.targetPressure = config->target_bar;
        input.measuredPressure = plant_out.measured_pressure_bar;
        input.feedforwardFlow = 0.0;
        input.outputMin = -5.0;
        input.outputMax = config->segment.maxFlow;
        input.flowToPumpSpeedGain = HOLD_FLOW_TO_SPEED_GAIN;
        input.pumpSpeedLimit = HOLD_PUMP_SPEED_LIMIT;
        input.timestamp = (HYD_REAL)((step + 1) * config->dt_s);

        HYD_PressureController_Execute(&config->segment, &controller_state, &input, &output);

        pump_input.requestedFlow = output.outputFlow;
        pump_input.flowToPumpSpeedGain = input.flowToPumpSpeedGain;
        pump_input.pumpSpeedLimit = input.pumpSpeedLimit;
        pump_input.direction = config->segment.direction;
        HYD_PumpConverter_Execute(&pump_input, &pump_output);
        PressureModel_Step(&config->params,
                           &plant_state,
                           (float)pump_output.pumpSpeed,
                           config->dt_s,
                           &plant_out);

        if (step >= config->settle_start_step) {
            if (plant_out.real_pressure_bar < real_min) real_min = plant_out.real_pressure_bar;
            if (plant_out.real_pressure_bar > real_max) real_max = plant_out.real_pressure_bar;
            if (plant_out.measured_pressure_bar < measured_min)
                measured_min = plant_out.measured_pressure_bar;
            if (plant_out.measured_pressure_bar > measured_max)
                measured_max = plant_out.measured_pressure_bar;
            if ((float)output.filteredPressure < filtered_min)
                filtered_min = (float)output.filteredPressure;
            if ((float)output.filteredPressure > filtered_max)
                filtered_max = (float)output.filteredPressure;
            if ((float)pump_output.commandFlow < output_min)
                output_min = (float)pump_output.commandFlow;
            if ((float)pump_output.commandFlow > output_max)
                output_max = (float)pump_output.commandFlow;
            filtered_abs_error_sum += fabsf((float)output.filteredPressure - config->target_bar);
            ++filtered_samples;
        }
    }

    metrics->real_p2p_bar = real_max - real_min;
    metrics->measured_p2p_bar = measured_max - measured_min;
    metrics->filtered_p2p_bar = filtered_max - filtered_min;
    metrics->filtered_mae_bar =
        (filtered_samples > 0) ? (filtered_abs_error_sum / (float)filtered_samples) : 0.0f;
    metrics->output_p2p_lmin = output_max - output_min;
}

static void test_hold_harness_produces_finite_metrics(void) {
    HoldCaseConfig config = make_default_hold_case();
    HoldMetrics metrics;

    memset(&metrics, 0, sizeof(metrics));
    run_hold_case(&config, &metrics);

    assert(isfinite(metrics.real_p2p_bar));
    assert(isfinite(metrics.measured_p2p_bar));
    assert(isfinite(metrics.filtered_p2p_bar));
    assert(isfinite(metrics.filtered_mae_bar));
    assert(isfinite(metrics.output_p2p_lmin));
}

static void test_current_100_bar_hold_preserves_visible_ripple_with_bounded_hold_error(void) {
    HoldCaseConfig config = make_default_hold_case();
    HoldMetrics metrics;

    run_hold_case(&config, &metrics);

    printf("baseline hold: real=%.2f measured=%.2f filtered=%.2f mae=%.2f output=%.2f\n",
           metrics.real_p2p_bar,
           metrics.measured_p2p_bar,
           metrics.filtered_p2p_bar,
           metrics.filtered_mae_bar,
           metrics.output_p2p_lmin);

    /* Sensor noise is intentionally enabled in this baseline. The physical
     * plant's pressure ripple and noisy measurement envelope are separate
     * signals, so only require a bounded gap rather than an old-model 1 bar
     * equality assumption. */
    assert(fabsf(metrics.measured_p2p_bar - metrics.real_p2p_bar) < 8.0f);
    assert(metrics.real_p2p_bar > 5.0f);
    assert(metrics.filtered_p2p_bar > 5.0f);
    /* The hold loop should stay bounded and keep the filtered error modest. */
    assert(metrics.filtered_mae_bar < 6.0f);
}

static void test_sensor_noise_changes_measurement_without_changing_real_pressure(void) {
    PressureModelParams noisy = make_default_model_params();
    PressureModelParams quiet = noisy;
    PressureModelState noisy_state;
    PressureModelState quiet_state;
    PressureModelOutput noisy_output;
    PressureModelOutput quiet_output;
    bool saw_measurement_difference = false;

    quiet.enable_sensor_noise = 0u;
    quiet.sensor_noise_std_bar = 0.0f;
    memset(&noisy_output, 0, sizeof(noisy_output));
    memset(&quiet_output, 0, sizeof(quiet_output));
    PressureModel_Reset(&noisy_state, 0x56565656u);
    PressureModel_Reset(&quiet_state, 0x56565656u);

    for (int step = 0; step < 1000; ++step) {
        PressureModel_Step(&noisy, &noisy_state, 40.0f, HOLD_DT_S, &noisy_output);
        PressureModel_Step(&quiet, &quiet_state, 40.0f, HOLD_DT_S, &quiet_output);
        assert(fabsf(noisy_output.real_pressure_bar - quiet_output.real_pressure_bar) < 1e-6f);
        if (fabsf(noisy_output.measured_pressure_bar - quiet_output.measured_pressure_bar) > 1e-3f) {
            saw_measurement_difference = true;
        }
    }

    assert(saw_measurement_difference);
}

static void test_stronger_filter_changes_closed_loop_hold_metrics(void) {
    HoldCaseConfig raw = make_default_hold_case();
    HoldCaseConfig filtered = raw;
    HoldMetrics raw_metrics;
    HoldMetrics filtered_metrics;

    filtered.segment.pressureFilterAlpha = 0.10;

    run_hold_case(&raw, &raw_metrics);
    run_hold_case(&filtered, &filtered_metrics);

    assert(fabsf(filtered_metrics.filtered_p2p_bar - raw_metrics.filtered_p2p_bar) > 0.50f);
    assert(fabsf(filtered_metrics.filtered_mae_bar - raw_metrics.filtered_mae_bar) > 0.10f);
}

static void test_disabling_pressure_accel_feedforward_changes_hold_metrics(void) {
    HoldCaseConfig enabled = make_default_hold_case();
    HoldCaseConfig disabled = enabled;
    HoldMetrics enabled_metrics;
    HoldMetrics disabled_metrics;

    disabled.segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;

    run_hold_case(&enabled, &enabled_metrics);
    run_hold_case(&disabled, &disabled_metrics);

    /* The feed-forward term is an active control contribution. Its effect is
     * plant- and tuning-dependent, so verify that disabling it changes the
     * closed-loop response without asserting an unsupported direction. */
    assert(fabsf(disabled_metrics.filtered_mae_bar - enabled_metrics.filtered_mae_bar) > 0.25f);
    assert(fabsf(disabled_metrics.filtered_p2p_bar - enabled_metrics.filtered_p2p_bar) > 0.5f);
}

static void test_disabling_gain_compensation_increases_hold_error_materially(void) {
    HoldCaseConfig compensated = make_default_hold_case();
    HoldCaseConfig uncompensated = compensated;
    HoldMetrics compensated_metrics;
    HoldMetrics uncompensated_metrics;

    uncompensated.segment.systemGain = 0.0;

    run_hold_case(&compensated, &compensated_metrics);
    run_hold_case(&uncompensated, &uncompensated_metrics);

    assert(uncompensated_metrics.filtered_mae_bar > compensated_metrics.filtered_mae_bar + 0.5f);
}

int main(void) {
    printf("Running pressure hold diagnosis tests...\n\n");
    test_hold_harness_produces_finite_metrics();
    test_current_100_bar_hold_preserves_visible_ripple_with_bounded_hold_error();
    test_sensor_noise_changes_measurement_without_changing_real_pressure();
    test_stronger_filter_changes_closed_loop_hold_metrics();
    test_disabling_pressure_accel_feedforward_changes_hold_metrics();
    test_disabling_gain_compensation_increases_hold_error_materially();
    printf("\nPASS pressure hold diagnosis harness\n");
    return 0;
}
