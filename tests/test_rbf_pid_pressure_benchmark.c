#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"
#include "pressure_model.h"

#define BENCHMARK_DT_S 0.001f
#define BENCHMARK_HOLD_S 5.0f
#define BENCHMARK_STEPS_PER_TARGET 5000
#define BENCHMARK_FLOW_TO_RPM 20.0f
#define BENCHMARK_PUMP_LIMIT_RPM 1800.0f
#define BENCHMARK_MAX_FLOW_LPM 90.0f
#define BENCHMARK_MODEL_GAIN_BAR_PER_RPM 0.66f
#define BENCHMARK_MODEL_TAU_S 1.0f
#define BENCHMARK_CONTROLLER_GAIN_BAR_PER_LPM \
    (BENCHMARK_MODEL_GAIN_BAR_PER_RPM * BENCHMARK_FLOW_TO_RPM)

typedef enum {
    BENCHMARK_PI,
    BENCHMARK_RBF_DEFAULT,
    BENCHMARK_RBF_CONSERVATIVE
} BenchmarkControllerKind;

typedef struct {
    float final_pressure_bar;
    float peak_pressure_bar;
    float tail_mae_bar;
    float tail_max_error_bar;
    float tail_ripple_bar;
    float max_flow_lpm;
    float max_flow_step_lpm;
    int finite;
    int limit_ok;
} BenchmarkMetrics;

static HYD_MotionSegment make_segment(float target_bar,
                                      BenchmarkControllerKind kind) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = target_bar;
    segment.maxFlow = BENCHMARK_MAX_FLOW_LPM;
    segment.pressureCeiling = target_bar * 3.0f;
    segment.pressureFilterAlpha = 1.0f;
    segment.pressureDerivativeFilterAlpha = 1.0f;
    segment.systemGain = BENCHMARK_CONTROLLER_GAIN_BAR_PER_LPM;

    if (kind == BENCHMARK_PI) {
        segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
        segment.pressureKp = 1.0f /
            (BENCHMARK_CONTROLLER_GAIN_BAR_PER_LPM * 0.5f);
        segment.pressureKi = segment.pressureKp / BENCHMARK_MODEL_TAU_S;
        segment.pressureIntegralLimit = BENCHMARK_MAX_FLOW_LPM;
    } else {
        segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
        if (kind == BENCHMARK_RBF_CONSERVATIVE) {
            segment.pressureRbfConfig.minKp = 0.04f;
            segment.pressureRbfConfig.maxKp = 0.06f;
            segment.pressureRbfConfig.minKi = 0.0008f;
            segment.pressureRbfConfig.maxKi = 0.0016f;
            segment.pressureRbfConfig.minKd = 0.015f;
            segment.pressureRbfConfig.maxKd = 0.035f;
            segment.pressureRbfConfig.etaW = 0.002f;
            segment.pressureRbfConfig.etaC = 0.002f;
            segment.pressureRbfConfig.etaB = 0.001f;
            segment.pressureRbfConfig.etaP = 0.0001f;
            segment.pressureRbfConfig.etaI = 0.00005f;
            segment.pressureRbfConfig.etaD = 0.0001f;
            segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;
        }
    }

    return segment;
}

static void init_model(PressureModelParams *params, PressureModelState *state) {
    PressureModel_InitParams(params);
    params->model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params->first_order_k_bar_per_rpm = BENCHMARK_MODEL_GAIN_BAR_PER_RPM;
    params->first_order_tau_s = BENCHMARK_MODEL_TAU_S;
    params->first_order_delay_s = 0.0f;
    params->enable_sensor_noise = 0u;
    params->enable_motor_noise = 0u;
    PressureModel_Reset(state, 0x9e3779b9u);
    assert(PressureModel_ValidateParams(params));
}

static void init_metrics(BenchmarkMetrics *metrics) {
    memset(metrics, 0, sizeof(*metrics));
    metrics->finite = 1;
    metrics->limit_ok = 1;
}

static void run_controller(BenchmarkControllerKind kind,
                            BenchmarkMetrics *metrics) {
    static const float targets[] = {50.0f, 80.0f, 100.0f, 50.0f, 80.0f};
    HYD_PressureControllerState controller;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_output;
    float pressure_bar = 0.0f;
    float previous_flow = 0.0f;
    float tail_abs_error_sum = 0.0f;
    float tail_max_error = 0.0f;
    float tail_min_pressure = 1.0e9f;
    float tail_max_pressure = -1.0e9f;
    float stage_peak_pressure;
    int tail_samples = 0;
    int global_step;
    int target_index;

    init_metrics(metrics);
    init_model(&params, &plant);
    HYD_PressureController_InitState(&controller, 0.0, 0.0, 0.0);
    memset(&input, 0, sizeof(input));

    for (target_index = 0; target_index < 5; ++target_index) {
        HYD_MotionSegment segment = make_segment(targets[target_index], kind);

        tail_abs_error_sum = 0.0f;
        tail_max_error = 0.0f;
        tail_min_pressure = 1.0e9f;
        tail_max_pressure = -1.0e9f;
        stage_peak_pressure = 0.0f;
        tail_samples = 0;

        for (global_step = 0;
             global_step < BENCHMARK_STEPS_PER_TARGET;
             ++global_step) {
            float error;
            float pump_rpm;

            input.targetPressure = targets[target_index];
            input.measuredPressure = pressure_bar;
            input.feedforwardFlow = 0.0f;
            input.outputMin = 0.0f;
            input.outputMax = BENCHMARK_MAX_FLOW_LPM;
            input.flowToPumpSpeedGain = BENCHMARK_FLOW_TO_RPM;
            input.pumpSpeedLimit = BENCHMARK_PUMP_LIMIT_RPM;
            input.timestamp = (float)(target_index * BENCHMARK_STEPS_PER_TARGET +
                                      global_step + 1) * BENCHMARK_DT_S;

            HYD_PressureController_Execute(&segment, &controller,
                                           &input, &output);
            pump_rpm = output.outputFlow * BENCHMARK_FLOW_TO_RPM;

            if (!isfinite(output.outputFlow) || !isfinite(pump_rpm)) {
                metrics->finite = 0;
            }
            if (output.outputFlow < -1.0e-5f ||
                output.outputFlow > BENCHMARK_MAX_FLOW_LPM + 1.0e-5f ||
                pump_rpm < -1.0e-5f ||
                pump_rpm > BENCHMARK_PUMP_LIMIT_RPM + 1.0e-5f) {
                metrics->limit_ok = 0;
            }
            if (output.outputFlow > metrics->max_flow_lpm) {
                metrics->max_flow_lpm = output.outputFlow;
            }
            if (fabsf(output.outputFlow - previous_flow) > metrics->max_flow_step_lpm) {
                metrics->max_flow_step_lpm = fabsf(output.outputFlow - previous_flow);
            }
            previous_flow = output.outputFlow;

            PressureModel_Step(&params, &plant, pump_rpm, BENCHMARK_DT_S,
                               &plant_output);
            pressure_bar = plant_output.measured_pressure_bar;
            if (pressure_bar > metrics->peak_pressure_bar) {
                metrics->peak_pressure_bar = pressure_bar;
            }
            if (pressure_bar > stage_peak_pressure) {
                stage_peak_pressure = pressure_bar;
            }

            if (global_step >= BENCHMARK_STEPS_PER_TARGET - 1000) {
                error = fabsf(pressure_bar - targets[target_index]);
                tail_abs_error_sum += error;
                if (error > tail_max_error) {
                    tail_max_error = error;
                }
                if (pressure_bar < tail_min_pressure) {
                    tail_min_pressure = pressure_bar;
                }
                if (pressure_bar > tail_max_pressure) {
                    tail_max_pressure = pressure_bar;
                }
                ++tail_samples;
            }
        }

        metrics->final_pressure_bar = pressure_bar;
        metrics->tail_mae_bar = tail_samples > 0
            ? tail_abs_error_sum / (float)tail_samples : 0.0f;
        metrics->tail_max_error_bar = tail_max_error;
        metrics->tail_ripple_bar = tail_max_pressure - tail_min_pressure;
        printf("  target=%.0f final=%.3f stage_peak=%.3f tail_mae=%.3f "
               "tail_max=%.3f ripple=%.3f max_flow=%.3f max_du=%.3f\n",
               targets[target_index], metrics->final_pressure_bar,
               stage_peak_pressure, metrics->tail_mae_bar,
               metrics->tail_max_error_bar, metrics->tail_ripple_bar,
               metrics->max_flow_lpm, metrics->max_flow_step_lpm);
        assert(metrics->tail_mae_bar <= targets[target_index] * 0.01f);
        assert(metrics->tail_max_error_bar <= targets[target_index] * 0.01f);
        assert(metrics->tail_ripple_bar <= targets[target_index] * 0.01f);
    }
}

static const char *controller_name(BenchmarkControllerKind kind) {
    switch (kind) {
    case BENCHMARK_PI:
        return "PI";
    case BENCHMARK_RBF_DEFAULT:
        return "RBF-PID default";
    case BENCHMARK_RBF_CONSERVATIVE:
        return "RBF-PID conservative";
    default:
        return "unknown";
    }
}

int main(void) {
    BenchmarkControllerKind kind;

    printf("RBF-PID pressure benchmark: K=%.2f bar/rpm, T=%.2f s, "
           "Ts=%.3f s, sequence=50/80/100/50/80, hold=%.1f s\n",
           BENCHMARK_MODEL_GAIN_BAR_PER_RPM, BENCHMARK_MODEL_TAU_S,
           BENCHMARK_DT_S, BENCHMARK_HOLD_S);
    for (kind = BENCHMARK_PI; kind <= BENCHMARK_RBF_CONSERVATIVE; ++kind) {
        BenchmarkMetrics metrics;

        printf("%s:\n", controller_name(kind));
        run_controller(kind, &metrics);
        assert(metrics.finite);
        assert(metrics.limit_ok);
    }
    return 0;
}
