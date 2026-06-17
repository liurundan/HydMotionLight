#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
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
static void test_current_100_bar_hold_reproduces_large_visible_ripple(void);
static void test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple(void);
static void test_motor_noise_ablation_reduces_real_and_filtered_ripple(void);
static void test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure(void);
static void test_stronger_filter_changes_closed_loop_hold_metrics(void);
static void test_disabling_pressure_accel_feedforward_increases_filtered_ripple(void);
static void test_disabling_gain_compensation_increases_hold_error_materially(void);

static PressureModelParams make_default_model_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
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

    PressureModel_Reset(&plant_state, 0x5a5a5a5au);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);

    for (step = 0; step < config->total_steps; ++step) {
        float target_rpm;

        input.targetPressure = config->target_bar;
        input.measuredPressure = plant_out.measured_pressure_bar;
        input.feedforwardFlow = 0.0;
        input.outputMin = -5.0;
        input.outputMax = config->segment.maxFlow;
        input.flowToPumpSpeedGain = HOLD_FLOW_TO_SPEED_GAIN;
        input.pumpSpeedLimit = HOLD_PUMP_SPEED_LIMIT;
        input.timestamp = (HYD_REAL)((step + 1) * config->dt_s);

        HYD_PressureController_Execute(&config->segment, &controller_state, &input, &output);

        target_rpm = (float)(output.outputFlow * input.flowToPumpSpeedGain);
        PressureModel_Step(&config->params, &plant_state, target_rpm, config->dt_s, &plant_out);

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
            if ((float)output.outputFlow < output_min) output_min = (float)output.outputFlow;
            if ((float)output.outputFlow > output_max) output_max = (float)output.outputFlow;
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

static void test_current_100_bar_hold_reproduces_large_visible_ripple(void) {
    HoldCaseConfig config = make_default_hold_case();
    HoldMetrics metrics;

    run_hold_case(&config, &metrics);

    printf("baseline hold: real=%.2f measured=%.2f filtered=%.2f mae=%.2f output=%.2f\n",
           metrics.real_p2p_bar,
           metrics.measured_p2p_bar,
           metrics.filtered_p2p_bar,
           metrics.filtered_mae_bar,
           metrics.output_p2p_lmin);

    assert(metrics.measured_p2p_bar > metrics.real_p2p_bar + 5.0f);
    assert(metrics.filtered_p2p_bar > 10.0f);
    assert(metrics.filtered_mae_bar > 5.0f);
}

static void test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple(void) {
    HoldCaseConfig baseline = make_default_hold_case();
    HoldCaseConfig flat = baseline;
    HoldMetrics base_metrics;
    HoldMetrics flat_metrics;

    flat.params.tooth_drop_depth_ratio = 0.0f;
    flat.params.tooth_drop_depth_base = 0.0f;

    run_hold_case(&baseline, &base_metrics);
    run_hold_case(&flat, &flat_metrics);

    assert(flat_metrics.measured_p2p_bar < base_metrics.measured_p2p_bar * 0.70f);
    assert(fabsf(flat_metrics.real_p2p_bar - base_metrics.real_p2p_bar) <
           (base_metrics.real_p2p_bar * 0.30f + 0.5f));
}

static void test_motor_noise_ablation_reduces_real_and_filtered_ripple(void) {
    HoldCaseConfig noisy = make_default_hold_case();
    HoldCaseConfig quiet = noisy;
    HoldMetrics noisy_metrics;
    HoldMetrics quiet_metrics;

    quiet.params.enable_motor_noise = 0u;
    quiet.params.motor_noise_std_rpm = 0.0f;

    run_hold_case(&noisy, &noisy_metrics);
    run_hold_case(&quiet, &quiet_metrics);

    assert(quiet_metrics.real_p2p_bar < noisy_metrics.real_p2p_bar * 0.85f);
    assert(quiet_metrics.filtered_p2p_bar < noisy_metrics.filtered_p2p_bar * 0.85f);
}

static void test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure(void) {
    HoldCaseConfig noisy = make_default_hold_case();
    HoldCaseConfig quiet = noisy;
    HoldMetrics noisy_metrics;
    HoldMetrics quiet_metrics;

    quiet.params.enable_sensor_noise = 0u;
    quiet.params.sensor_noise_std_bar = 0.0f;

    run_hold_case(&noisy, &noisy_metrics);
    run_hold_case(&quiet, &quiet_metrics);

    assert(fabsf(quiet_metrics.real_p2p_bar - noisy_metrics.real_p2p_bar) <
           (noisy_metrics.real_p2p_bar * 0.15f + 0.25f));
    assert(quiet_metrics.measured_p2p_bar <= noisy_metrics.measured_p2p_bar);
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

static void test_disabling_pressure_accel_feedforward_increases_filtered_ripple(void) {
    HoldCaseConfig enabled = make_default_hold_case();
    HoldCaseConfig disabled = enabled;
    HoldMetrics enabled_metrics;
    HoldMetrics disabled_metrics;

    disabled.segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;

    run_hold_case(&enabled, &enabled_metrics);
    run_hold_case(&disabled, &disabled_metrics);

    assert(disabled_metrics.filtered_p2p_bar > enabled_metrics.filtered_p2p_bar + 0.5f);
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
    test_current_100_bar_hold_reproduces_large_visible_ripple();
    test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple();
    test_motor_noise_ablation_reduces_real_and_filtered_ripple();
    test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure();
    test_stronger_filter_changes_closed_loop_hold_metrics();
    test_disabling_pressure_accel_feedforward_increases_filtered_ripple();
    test_disabling_gain_compensation_increases_hold_error_materially();
    printf("\nPASS pressure hold diagnosis harness\n");
    return 0;
}
