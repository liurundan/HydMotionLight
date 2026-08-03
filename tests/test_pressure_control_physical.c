/*
 * Deterministic nonlinear pressure-loop simulation.
 *
 * This test deliberately uses the repository's physical PressureModel rather
 * than the first-order shortcut used by sim_pressure_control.  It exercises
 * motor lag, leakage, relief, speed-dependent effective volume, pump-flow and
 * tooth ripple, sensor noise/bias, gain mismatch, load demand, and pressure
 * segment transitions.  The metrics separate real process pressure from the
 * noisy sensor signal and the controller's filtered signal.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "pressure_controller.h"
#include "pressure_model.h"
#include "pressure_ripple_comp.h"
#include "pump_converter.h"

#define SIM_DT_S 0.001f
#define SIM_DURATION_S 8.0f
#define SIM_STEPS ((int)(SIM_DURATION_S / SIM_DT_S))
#define SIM_TAIL_S 1.0f
#define SIM_TAIL_START ((int)((SIM_DURATION_S - SIM_TAIL_S) / SIM_DT_S))
#define SIM_FLOW_TO_SPEED 20.0f
#define SIM_SPEED_LIMIT 1800.0f
#define SIM_NOMINAL_GAIN 60.0f
#define SIM_TWO_PI 6.2831853071795864769f

typedef struct {
    HYD_PressureControllerType strategy;
    const char* name;
    HYD_BOOL feedforward;
    HYD_BOOL ripple_comp;
} ControllerCase;

typedef struct {
    float target_bar;
    float gain_scale;
    HYD_BOOL load_step;
    HYD_BOOL encoder_loss;
    unsigned int seed;
} Scenario;

typedef struct {
    float rise_ms;
    float settle_ms;
    float overshoot_pct;
    float steady_error_bar;
    float real_p2p_bar;
    float filtered_p2p_bar;
    float raw_p2p_bar;
    float recovery_ms;
    int finite;
} Metrics;

static HYD_MotionSegment make_segment(const ControllerCase* controller,
                                      float target_bar) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = SIM_DURATION_S;
    segment.targetPressure = target_bar;
    segment.maxFlow = SIM_SPEED_LIMIT / SIM_FLOW_TO_SPEED;
    segment.pressureController = controller->strategy;
    segment.pressureCeiling = 245.0f;
    segment.pressureFilterAlpha =
        (controller->strategy == HYD_PRESSURE_CONTROLLER_PI) ? 0.20f : 0.30f;
    segment.pressureDerivativeFilterAlpha = 0.20f;
    segment.pressureIntegralLimit = segment.maxFlow;
    segment.systemGain = SIM_NOMINAL_GAIN;
    segment.pressureKp = 0.05f;
    segment.pressureKi = 0.005f;
    segment.pressureKd = 0.0f;

    if (controller->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
        controller->strategy == HYD_PRESSURE_CONTROLLER_RBF_PID ||
        controller->strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) {
        segment.pressureRbfConfig.minKp = 0.040f;
        segment.pressureRbfConfig.maxKp = 0.060f;
        segment.pressureRbfConfig.minKi = 0.0008f;
        segment.pressureRbfConfig.maxKi = 0.0016f;
        segment.pressureRbfConfig.minKd =
            (controller->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
             controller->strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) ? 0.0f : 0.015f;
        segment.pressureRbfConfig.maxKd =
            (controller->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
             controller->strategy == HYD_PRESSURE_CONTROLLER_PI_RBF) ? 0.0f : 0.035f;
        segment.pressureRbfConfig.etaW = 0.0020f;
        segment.pressureRbfConfig.etaC = 0.0020f;
        segment.pressureRbfConfig.etaB = 0.0010f;
        segment.pressureRbfConfig.etaP = 0.00010f;
        segment.pressureRbfConfig.etaI = 0.00005f;
        segment.pressureRbfConfig.etaD = 0.00010f;
        segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0f;
    }

    return segment;
}

static void init_physical_params(PressureModelParams* params,
                                 float gain_scale,
                                 unsigned int seed) {
    (void)seed;
    PressureModel_InitParams(params);
    params->model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params->sensor_range_bar = 300.0f;
    params->sensor_noise_std_bar = 0.08f;
    params->sensor_bias_bar = 0.12f;
    params->motor_noise_std_rpm = 1.0f;
    params->process_noise_std_m3_s = 2.0e-9f;
    params->enable_sensor_noise = 1u;
    params->enable_motor_noise = 1u;
    params->enable_process_noise = 1u;
    params->pump_displacement_m3_rev *= gain_scale;
    params->flow_ripple_ratio = 0.08f;
    params->tooth_drop_depth_base = 0.16f;
    params->tooth_drop_depth_ratio = 0.16f;
    params->relief_set_pa = 245.0e5f;
}

static float target_at(float t, float target) {
    return target;
}

static void update_metrics(Metrics* metrics,
                           int step,
                           float t,
                           float target,
                           float real_pressure,
                           float measured_pressure,
                           float filtered_pressure,
                           float* peak_real,
                           float* tail_real_min,
                           float* tail_real_max,
                           float* tail_filtered_min,
                           float* tail_filtered_max,
                           float* tail_raw_min,
                           float* tail_raw_max,
                           float* tail_sum,
                           int* tail_count,
                           float* last_outside,
                           float* rise_time) {
    const float tolerance = 0.02f * target;

    if (real_pressure > *peak_real) {
        *peak_real = real_pressure;
    }
    if (*rise_time < 0.0f && real_pressure >= 0.90f * target) {
        *rise_time = t;
    }
    if (fabsf(real_pressure - target) > tolerance) {
        *last_outside = t;
    }
    if (step >= SIM_TAIL_START) {
        if (real_pressure < *tail_real_min) *tail_real_min = real_pressure;
        if (real_pressure > *tail_real_max) *tail_real_max = real_pressure;
        if (filtered_pressure < *tail_filtered_min) *tail_filtered_min = filtered_pressure;
        if (filtered_pressure > *tail_filtered_max) *tail_filtered_max = filtered_pressure;
        if (measured_pressure < *tail_raw_min) *tail_raw_min = measured_pressure;
        if (measured_pressure > *tail_raw_max) *tail_raw_max = measured_pressure;
        *tail_sum += real_pressure;
        ++*tail_count;
    }
}

static Metrics run_scenario(const ControllerCase* controller,
                            const Scenario* scenario) {
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_out;
    HYD_PressureControllerState controller_state;
    HYD_PressureControllerInput controller_input;
    HYD_PressureControllerOutput controller_output;
    HYD_PumpConverterInput pump_input;
    HYD_PumpConverterOutput pump_output;
    HYD_PressureRippleCompState ripple_state;
    HYD_PressureSteadyGateState ripple_gate;
    HYD_MotionSegment segment;
    Metrics metrics;
    float peak_real = 0.0f;
    float tail_real_min = 1.0e30f;
    float tail_real_max = -1.0e30f;
    float tail_filtered_min = 1.0e30f;
    float tail_filtered_max = -1.0e30f;
    float tail_raw_min = 1.0e30f;
    float tail_raw_max = -1.0e30f;
    float tail_sum = 0.0f;
    float last_outside = -1.0f;
    float rise_time = -1.0f;
    float disturbance_end = 4.5f;
    float recovery_time = -1.0f;
    int tail_count = 0;
    int step;

    memset(&metrics, 0, sizeof(metrics));
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&controller_input, 0, sizeof(controller_input));
    memset(&controller_output, 0, sizeof(controller_output));
    memset(&pump_input, 0, sizeof(pump_input));
    memset(&pump_output, 0, sizeof(pump_output));

    segment = make_segment(controller, scenario->target_bar);
    init_physical_params(&params, scenario->gain_scale, scenario->seed);
    PressureModel_Reset(&plant, scenario->seed);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);
    HYD_PressureRippleComp_Reset(&ripple_state);
    HYD_PressureRippleComp_SetEnabled(&ripple_state, controller->ripple_comp ? 1u : 0u);
    HYD_PressureRippleComp_SetGain(&ripple_state, 1.0f / SIM_NOMINAL_GAIN);
    HYD_PressureSteadyGate_Reset(&ripple_gate, 128u, 8.0f);

    for (step = 0; step < SIM_STEPS; ++step) {
        float t = (float)(step + 1) * SIM_DT_S;
        float target = target_at(t, scenario->target_bar);
        float measured = plant_out.measured_pressure_bar;
        float ripple_ff = 0.0f;
        HYD_UINT8 use_encoder = scenario->encoder_loss && t >= 5.0f ? 0u : 1u;

        if (controller->ripple_comp) {
            HYD_REAL error = (HYD_REAL)measured - (HYD_REAL)target;
            HYD_UINT8 steady = HYD_PressureSteadyGate_Update(&ripple_gate, error);
            HYD_PressureRippleComp_Update(&ripple_state,
                                          error,
                                          (HYD_REAL)plant.pump_phase_rev,
                                          (HYD_REAL)plant.motor_rpm,
                                          SIM_DT_S,
                                          use_encoder,
                                          steady);
            ripple_ff = (float)HYD_PressureRippleComp_GetFF(&ripple_state,
                                                             plant.pump_phase_rev,
                                                             plant.motor_rpm,
                                                             SIM_DT_S,
                                                             use_encoder);
        }

        controller_input.targetPressure = target;
        controller_input.measuredPressure = measured;
        controller_input.feedforwardFlow = controller->feedforward
            ? (target / SIM_NOMINAL_GAIN) + ripple_ff : ripple_ff;
        controller_input.outputMin = 0.0;
        controller_input.outputMax = segment.maxFlow;
        controller_input.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
        controller_input.pumpSpeedLimit = SIM_SPEED_LIMIT;
        controller_input.timestamp = t;
        controller_input.pumpAngleRev = plant.pump_phase_rev;

        HYD_PressureController_Execute(&segment,
                                       &controller_state,
                                       &controller_input,
                                       &controller_output);

        pump_input.requestedFlow = controller_output.outputFlow;
        pump_input.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
        pump_input.pumpSpeedLimit = SIM_SPEED_LIMIT;
        pump_input.direction = HYD_DIRECTION_HOLD;
        HYD_PumpConverter_Execute(&pump_input, &pump_output);
        PressureModel_Step(&params,
                           &plant,
                           (float)pump_output.pumpSpeed,
                           SIM_DT_S,
                           &plant_out);

        if (scenario->load_step && t >= 4.0f && t <= disturbance_end) {
            plant.pressure_pa -= 6.0e5f * SIM_DT_S;
            if (plant.pressure_pa < 0.0f) plant.pressure_pa = 0.0f;
        }

        update_metrics(&metrics,
                       step,
                       t,
                       target,
                       plant_out.real_pressure_bar,
                       plant_out.measured_pressure_bar,
                       (float)controller_output.filteredPressure,
                       &peak_real,
                       &tail_real_min,
                       &tail_real_max,
                       &tail_filtered_min,
                       &tail_filtered_max,
                       &tail_raw_min,
                       &tail_raw_max,
                       &tail_sum,
                       &tail_count,
                       &last_outside,
                       &rise_time);

        if (scenario->load_step && t > disturbance_end && recovery_time < 0.0f &&
            fabsf(plant_out.real_pressure_bar - target) <= 0.02f * target) {
            recovery_time = t - disturbance_end;
        }
    }

    metrics.rise_ms = rise_time < 0.0f ? SIM_DURATION_S * 1000.0f : rise_time * 1000.0f;
    metrics.settle_ms = last_outside < 0.0f ? 0.0f : last_outside * 1000.0f;
    metrics.overshoot_pct = peak_real > scenario->target_bar
        ? (peak_real - scenario->target_bar) * 100.0f / scenario->target_bar : 0.0f;
    metrics.steady_error_bar = tail_count > 0
        ? tail_sum / (float)tail_count - scenario->target_bar : 0.0f;
    metrics.real_p2p_bar = tail_real_max - tail_real_min;
    metrics.filtered_p2p_bar = tail_filtered_max - tail_filtered_min;
    metrics.raw_p2p_bar = tail_raw_max - tail_raw_min;
    metrics.recovery_ms = recovery_time < 0.0f ? -1.0f : recovery_time * 1000.0f;
    metrics.finite = isfinite(metrics.rise_ms) && isfinite(metrics.settle_ms) &&
                     isfinite(metrics.overshoot_pct) && isfinite(metrics.steady_error_bar) &&
                     isfinite(metrics.real_p2p_bar) && isfinite(metrics.filtered_p2p_bar) &&
                     isfinite(metrics.raw_p2p_bar);
    return metrics;
}

static int run_transition_case(const ControllerCase* controller) {
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_out;
    HYD_PressureControllerState controller_state;
    HYD_PressureControllerInput controller_input;
    HYD_PressureControllerOutput controller_output;
    HYD_PumpConverterInput pump_input;
    HYD_PumpConverterOutput pump_output;
    HYD_MotionSegment segment;
    float previous_flow = 0.0f;
    int discontinuities = 0;
    int step;

    memset(&plant_out, 0, sizeof(plant_out));
    memset(&controller_input, 0, sizeof(controller_input));
    memset(&controller_output, 0, sizeof(controller_output));
    memset(&pump_input, 0, sizeof(pump_input));
    memset(&pump_output, 0, sizeof(pump_output));
    segment = make_segment(controller, 20.0f);
    init_physical_params(&params, 1.0f, 0x91a2b3c4u);
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;
    PressureModel_Reset(&plant, 0x91a2b3c4u);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);

    for (step = 0; step < 6000; ++step) {
        float t = (float)(step + 1) * SIM_DT_S;
        float target = t < 1.0f ? 20.0f : (t < 2.5f ? 180.0f : 100.0f);
        segment.targetPressure = target;
        controller_input.targetPressure = target;
        controller_input.measuredPressure = plant_out.measured_pressure_bar;
        controller_input.feedforwardFlow = controller->feedforward ? target / SIM_NOMINAL_GAIN : 0.0f;
        controller_input.outputMin = 0.0f;
        controller_input.outputMax = segment.maxFlow;
        controller_input.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
        controller_input.pumpSpeedLimit = SIM_SPEED_LIMIT;
        controller_input.timestamp = t;
        HYD_PressureController_Execute(&segment, &controller_state,
                                       &controller_input, &controller_output);
        if (step > 0 && fabsf((float)controller_output.outputFlow - previous_flow) > 30.0f) {
            ++discontinuities;
        }
        previous_flow = (float)controller_output.outputFlow;
        pump_input.requestedFlow = controller_output.outputFlow;
        pump_input.flowToPumpSpeedGain = SIM_FLOW_TO_SPEED;
        pump_input.pumpSpeedLimit = SIM_SPEED_LIMIT;
        pump_input.direction = HYD_DIRECTION_HOLD;
        HYD_PumpConverter_Execute(&pump_input, &pump_output);
        PressureModel_Step(&params, &plant, (float)pump_output.pumpSpeed,
                           SIM_DT_S, &plant_out);
    }

    return discontinuities == 0 && isfinite(plant_out.real_pressure_bar);
}

int main(void) {
    static const ControllerCase controllers[] = {
        {HYD_PRESSURE_CONTROLLER_PI, "PI", false, false},
        {HYD_PRESSURE_CONTROLLER_PI, "PI+FF", true, false},
        {HYD_PRESSURE_CONTROLLER_PI, "PI+FF+RC", true, true},
        {HYD_PRESSURE_CONTROLLER_PI_RBF, "PI-RBF", true, false},
        {HYD_PRESSURE_CONTROLLER_RBF_PI, "RBF-PI", false, false},
        {HYD_PRESSURE_CONTROLLER_RBF_PID, "RBF-PID", false, false}
    };
    const int controller_count = (int)(sizeof(controllers) / sizeof(controllers[0]));
    const float targets[] = {20.0f, 100.0f, 180.0f};
    int failures = 0;
    int i;

    printf("physical pressure-loop simulation (dt=1 ms, motor tau=60 ms)\n");
    printf("controller,target,gain_scale,rise_ms,settle_ms,overshoot_pct,error_bar,real_p2p,filtered_p2p,raw_p2p,hard_acceptance\n");
    for (i = 0; i < controller_count; ++i) {
        int j;
        for (j = 0; j < (int)(sizeof(targets) / sizeof(targets[0])); ++j) {
            Scenario scenario;
            Metrics metrics;
            scenario.target_bar = targets[j];
            scenario.gain_scale = 1.0f;
            scenario.load_step = false;
            scenario.encoder_loss = false;
            scenario.seed = 0x1000u + (unsigned int)(i * 31 + j);
            metrics = run_scenario(&controllers[i], &scenario);
            {
                int hard_acceptance = metrics.rise_ms <= 100.0f &&
                    metrics.settle_ms <= 300.0f &&
                    metrics.overshoot_pct <= 5.0f &&
                    metrics.real_p2p_bar <= 0.01f * targets[j];
                printf("%s,%.0f,%.2f,%.1f,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%s\n",
                   controllers[i].name, targets[j], scenario.gain_scale,
                   metrics.rise_ms, metrics.settle_ms, metrics.overshoot_pct,
                   metrics.steady_error_bar, metrics.real_p2p_bar,
                   metrics.filtered_p2p_bar, metrics.raw_p2p_bar,
                   hard_acceptance ? "PASS" : "FAIL");
            }
            if (!metrics.finite) ++failures;
        }
    }

    printf("gain mismatch and load disturbance (100 bar target)\n");
    for (i = 0; i < controller_count; ++i) {
        Scenario scenario;
        Metrics metrics;
        scenario.target_bar = 100.0f;
        scenario.gain_scale = 1.30f;
        scenario.load_step = true;
        scenario.encoder_loss = controllers[i].ripple_comp;
        scenario.seed = 0x2000u + (unsigned int)i;
        metrics = run_scenario(&controllers[i], &scenario);
        printf("%s,gain=%.2f,recovery_ms=%.1f,error_bar=%.3f,real_p2p=%.3f,raw_p2p=%.3f\n",
               controllers[i].name, scenario.gain_scale, metrics.recovery_ms,
               metrics.steady_error_bar, metrics.real_p2p_bar, metrics.raw_p2p_bar);
        if (!metrics.finite) ++failures;
    }

    for (i = 0; i < controller_count; ++i) {
        int ok = run_transition_case(&controllers[i]);
        printf("transition %s: %s\n", controllers[i].name, ok ? "PASS" : "FAIL");
        if (!ok) ++failures;
    }

    printf("physical simulation failures: %d\n", failures);
    return failures == 0 ? 0 : 1;
}
