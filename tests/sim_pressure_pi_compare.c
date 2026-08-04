/*
 * PI+FF vs PI-RBF pressure step comparison using the physical PressureModel.
 *
 * Output:
 *   sim_output/pressure_pi_ff_vs_pi_rbf_step.csv
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include "common_types.h"
#include "pressure_controller.h"
#include "pressure_model.h"
#include "pump_converter.h"

#define SIM_DT_S 0.001f
#define SIM_DURATION_S 6.0f
#define SIM_STEPS ((int)(SIM_DURATION_S / SIM_DT_S))
#define SIM_STEP_TIME_S 0.20f
#define SIM_TAIL_START_S 5.0f
#define SIM_FLOW_TO_SPEED 20.0f
#define SIM_SPEED_LIMIT_RPM 1800.0f
#define SIM_SYSTEM_GAIN 60.0f

typedef struct {
    HYD_PressureControllerType strategy;
    const char* name;
} ControllerDef;

typedef struct {
    PressureModelState plant;
    PressureModelOutput plantOut;
    HYD_PressureControllerState controller;
    HYD_MotionSegment segment;
    float lastFiltered;
    float lastFlow;
    float riseTime;
    float lastOutsideBand;
    float peakReal;
    float tailRealMin;
    float tailRealMax;
    float tailMeasuredMin;
    float tailMeasuredMax;
    float tailFilteredMin;
    float tailFilteredMax;
    float tailSumReal;
    int tailCount;
} SimChannel;

typedef struct {
    float riseMs;
    float settleMs;
    float overshootPct;
    float steadyErrorBar;
    float realP2pBar;
    float measuredP2pBar;
    float filteredP2pBar;
} Metrics;

static float g_target_bar = 100.0f;

static void make_output_dir(void) {
#ifdef _WIN32
    if (_mkdir("sim_output") != 0 && errno != EEXIST) {
        fprintf(stderr, "warning: cannot create sim_output\n");
    }
#else
    if (mkdir("sim_output", 0777) != 0 && errno != EEXIST) {
        fprintf(stderr, "warning: cannot create sim_output\n");
    }
#endif
}

static PressureModelParams make_physical_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params.sensor_range_bar = 300.0f;
    params.motor_tau_s = 0.06f;
    params.flow_ripple_ratio = 0.08f;
    params.tooth_drop_depth_ratio = 0.16f;
    params.tooth_drop_depth_base = 0.16f;
    params.tooth_drop_width_ratio = 0.20f;
    params.relief_set_pa = 245.0e5f;
    params.sensor_noise_std_bar = 0.0f;
    params.sensor_bias_bar = 0.0f;
    params.motor_noise_std_rpm = 0.0f;
    params.process_noise_std_m3_s = 0.0f;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;
    return params;
}

static HYD_MotionSegment make_segment(const ControllerDef* controller) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = SIM_DURATION_S;
    segment.targetPressure = g_target_bar;
    segment.maxFlow = SIM_SPEED_LIMIT_RPM / SIM_FLOW_TO_SPEED;
    segment.pressureController = controller->strategy;
    segment.pressureCeiling = 245.0f;
    segment.pressureKp = 0.05f;
    segment.pressureKi = 0.005f;
    segment.pressureKd = 0.0f;
    segment.pressureFilterAlpha = 0.20f;
    segment.pressureDerivativeFilterAlpha = 0.20f;
    segment.pressureIntegralLimit = segment.maxFlow;
    segment.systemGain = SIM_SYSTEM_GAIN;

    if (controller->strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) {
        segment.pressureRbfConfig.minKp = 0.040f;
        segment.pressureRbfConfig.maxKp = 0.060f;
        segment.pressureRbfConfig.minKi = 0.0040f;
        segment.pressureRbfConfig.maxKi = 0.0060f;
        segment.pressureRbfConfig.minKd = 0.0f;
        segment.pressureRbfConfig.maxKd = 0.0f;
        segment.pressureRbfConfig.etaW = 0.0020f;
        segment.pressureRbfConfig.etaC = 0.0020f;
        segment.pressureRbfConfig.etaB = 0.0010f;
        segment.pressureRbfConfig.etaP = 0.00010f;
        segment.pressureRbfConfig.etaI = 0.00005f;
        segment.pressureRbfConfig.etaD = 0.0f;
        segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0f;
    }

    return segment;
}

static void init_channel(SimChannel* channel,
                         const PressureModelParams* params,
                         const ControllerDef* controller,
                         unsigned int seed) {
    (void)params;
    memset(channel, 0, sizeof(*channel));
    channel->segment = make_segment(controller);
    channel->riseTime = -1.0f;
    channel->lastOutsideBand = -1.0f;
    channel->tailRealMin = 1.0e30f;
    channel->tailRealMax = -1.0e30f;
    channel->tailMeasuredMin = 1.0e30f;
    channel->tailMeasuredMax = -1.0e30f;
    channel->tailFilteredMin = 1.0e30f;
    channel->tailFilteredMax = -1.0e30f;
    PressureModel_Reset(&channel->plant, seed);
    HYD_PressureController_InitState(&channel->controller, 0.0, 0.0, 0.0);
}

static void update_metrics_state(SimChannel* channel, float t, float target) {
    float real = channel->plantOut.real_pressure_bar;
    float measured = channel->plantOut.measured_pressure_bar;
    float filtered = channel->lastFiltered;
    float tolerance = 0.02f * target;

    if (target > 0.0f) {
        if (channel->riseTime < 0.0f && real >= 0.90f * target) {
            channel->riseTime = t - SIM_STEP_TIME_S;
        }
        if (fabsf(real - target) > tolerance) {
            channel->lastOutsideBand = t - SIM_STEP_TIME_S;
        }
    }
    if (real > channel->peakReal) {
        channel->peakReal = real;
    }

    if (t >= SIM_TAIL_START_S) {
        if (real < channel->tailRealMin) channel->tailRealMin = real;
        if (real > channel->tailRealMax) channel->tailRealMax = real;
        if (measured < channel->tailMeasuredMin) channel->tailMeasuredMin = measured;
        if (measured > channel->tailMeasuredMax) channel->tailMeasuredMax = measured;
        if (filtered < channel->tailFilteredMin) channel->tailFilteredMin = filtered;
        if (filtered > channel->tailFilteredMax) channel->tailFilteredMax = filtered;
        channel->tailSumReal += real;
        ++channel->tailCount;
    }
}

static void step_channel(SimChannel* channel,
                         const PressureModelParams* params,
                         float t,
                         float target) {
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PumpConverterInput pumpInput;
    HYD_PumpConverterOutput pumpOutput;

    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    memset(&pumpInput, 0, sizeof(pumpInput));
    memset(&pumpOutput, 0, sizeof(pumpOutput));

    input.targetPressure = target;
    input.measuredPressure = channel->plantOut.measured_pressure_bar;
    input.feedforwardFlow = (target > 0.0f) ? target / SIM_SYSTEM_GAIN : 0.0f;
    input.outputMin = 0.0f;
    input.outputMax = channel->segment.maxFlow;
    input.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
    input.pumpSpeedLimit = SIM_SPEED_LIMIT_RPM;
    input.timestamp = t;
    input.pumpAngleRev = channel->plant.pump_phase_rev;
    channel->segment.targetPressure = target;

    HYD_PressureController_Execute(&channel->segment,
                                   &channel->controller,
                                   &input,
                                   &output);
    channel->lastFiltered = (float)output.filteredPressure;
    channel->lastFlow = (float)output.outputFlow;

    pumpInput.requestedFlow = output.outputFlow;
    pumpInput.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
    pumpInput.pumpSpeedLimit = SIM_SPEED_LIMIT_RPM;
    pumpInput.direction = HYD_DIRECTION_HOLD;
    HYD_PumpConverter_Execute(&pumpInput, &pumpOutput);

    PressureModel_Step(params,
                       &channel->plant,
                       (float)pumpOutput.pumpSpeed,
                       SIM_DT_S,
                       &channel->plantOut);
    update_metrics_state(channel, t, target);
}

static Metrics finalize_metrics(const SimChannel* channel) {
    Metrics metrics;

    memset(&metrics, 0, sizeof(metrics));
    metrics.riseMs = (channel->riseTime < 0.0f) ? -1.0f : channel->riseTime * 1000.0f;
    metrics.settleMs = (channel->lastOutsideBand < 0.0f) ? 0.0f : channel->lastOutsideBand * 1000.0f;
    metrics.overshootPct = channel->peakReal > g_target_bar
        ? (channel->peakReal - g_target_bar) * 100.0f / g_target_bar : 0.0f;
    metrics.steadyErrorBar = channel->tailCount > 0
        ? (channel->tailSumReal / (float)channel->tailCount) - g_target_bar : 0.0f;
    metrics.realP2pBar = channel->tailRealMax - channel->tailRealMin;
    metrics.measuredP2pBar = channel->tailMeasuredMax - channel->tailMeasuredMin;
    metrics.filteredP2pBar = channel->tailFilteredMax - channel->tailFilteredMin;
    return metrics;
}

static void print_metrics(const char* name, const Metrics* metrics) {
    printf("%-8s rise_ms=%7.1f settle_ms=%7.1f overshoot_pct=%6.2f "
           "steady_error_bar=%7.3f real_p2p_bar=%7.3f measured_p2p_bar=%7.3f filtered_p2p_bar=%7.3f\n",
           name,
           metrics->riseMs,
           metrics->settleMs,
           metrics->overshootPct,
           metrics->steadyErrorBar,
           metrics->realP2pBar,
           metrics->measuredP2pBar,
           metrics->filteredP2pBar);
}

int main(int argc, char** argv) {
    const ControllerDef piFf = {HYD_PRESSURE_CONTROLLER_PI, "PI+FF"};
    const ControllerDef piRbf = {HYD_PRESSURE_CONTROLLER_PI_RBF, "PI-RBF"};
    PressureModelParams params = make_physical_params();
    SimChannel chPiFf;
    SimChannel chPiRbf;
    FILE* csv;
    char csvName[128];
    int step;

    make_output_dir();
    if (argc > 1) {
        float requestedTarget = (float)atof(argv[1]);
        if (isfinite(requestedTarget) && requestedTarget > 0.0f) {
            g_target_bar = requestedTarget;
        }
    }
    init_channel(&chPiFf, &params, &piFf, 0x20260803u);
    init_channel(&chPiRbf, &params, &piRbf, 0x20260803u);

    snprintf(csvName, sizeof(csvName),
             "sim_output/pressure_pi_ff_vs_pi_rbf_step_%.0fbar.csv",
             g_target_bar);
    csv = fopen(csvName, "w");
    if (csv == NULL) {
        fprintf(stderr, "cannot open %s\n", csvName);
        return 1;
    }

    fprintf(csv,
            "time_s,target_bar,pi_ff_real_bar,pi_ff_measured_bar,pi_ff_filtered_bar,pi_ff_flow_lmin,"
            "pi_rbf_real_bar,pi_rbf_measured_bar,pi_rbf_filtered_bar,pi_rbf_flow_lmin\n");

    for (step = 0; step < SIM_STEPS; ++step) {
        float t = (float)(step + 1) * SIM_DT_S;
        float target = (t >= SIM_STEP_TIME_S) ? g_target_bar : 0.0f;

        step_channel(&chPiFf, &params, t, target);
        step_channel(&chPiRbf, &params, t, target);

        fprintf(csv,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                target,
                chPiFf.plantOut.real_pressure_bar,
                chPiFf.plantOut.measured_pressure_bar,
                chPiFf.lastFiltered,
                chPiFf.lastFlow,
                chPiRbf.plantOut.real_pressure_bar,
                chPiRbf.plantOut.measured_pressure_bar,
                chPiRbf.lastFiltered,
                chPiRbf.lastFlow);
    }
    fclose(csv);

    {
        Metrics mPiFf = finalize_metrics(&chPiFf);
        Metrics mPiRbf = finalize_metrics(&chPiRbf);
        printf("Physical model step comparison: target=%.1f bar, step_time=%.2f s, dt=%.1f ms\n",
               g_target_bar, SIM_STEP_TIME_S, SIM_DT_S * 1000.0f);
        printf("Plant: motor_tau=%.0f ms, flow_ripple=%.1f%%, tooth_drop=%.1f%%, deterministic noise off\n",
               params.motor_tau_s * 1000.0f,
               params.flow_ripple_ratio * 100.0f,
               params.tooth_drop_depth_base * 100.0f);
        print_metrics(piFf.name, &mPiFf);
        print_metrics(piRbf.name, &mPiRbf);
    }

    printf("CSV: %s\n", csvName);
    return 0;
}
