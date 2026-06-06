#include "pressure_controller.h"
#include <math.h>
#include <string.h>

#define HYD_LEGACY_PRESSURE_FLOW_KP 1.5
#define HYD_DEFAULT_PRESSURE_FILTER_ALPHA 0.1
#define HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA 0.05
#define HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD 0.01
#define HYD_DEFAULT_RBF_LEARNING_RATE 0.25

typedef struct {
    HYD_PressureControllerType strategy;
    HYD_BOOL supportsIntegral;
    HYD_BOOL supportsDerivative;
    HYD_BOOL adaptive;
} HYD_PressureStrategySpec;

typedef struct {
    HYD_REAL minKp;
    HYD_REAL maxKp;
    HYD_REAL minKi;
    HYD_REAL maxKi;
    HYD_REAL minKd;
    HYD_REAL maxKd;
    HYD_REAL etaW;
    HYD_REAL etaC;
    HYD_REAL etaB;
    HYD_REAL etaP;
    HYD_REAL etaI;
    HYD_REAL etaD;
} HYD_RbfPidResolvedConfig;

typedef struct {
    const HYD_PressureStrategySpec* strategySpec;
    HYD_PressureControllerType strategy;
    HYD_REAL kp;
    HYD_REAL kpHigh;
    HYD_REAL gainBand;
    HYD_REAL ki;
    HYD_REAL kd;
    HYD_REAL integralLimit;
    HYD_REAL deadband;
    HYD_REAL filterAlpha;
    HYD_REAL derivativeFilterAlpha;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
    HYD_REAL dt;
    HYD_REAL samplingPeriod;
    HYD_RbfPidResolvedConfig rbf;
} HYD_PressureResolvedConfig;

static const HYD_PressureStrategySpec HYD_PRESSURE_STRATEGY_SPECS[] = {
    {HYD_PRESSURE_CONTROLLER_P, false, false, false},
    {HYD_PRESSURE_CONTROLLER_PI, true, false, false},
    {HYD_PRESSURE_CONTROLLER_PID, true, true, false},
    {HYD_PRESSURE_CONTROLLER_RBF_PID, true, true, true}
};

static const HYD_PressureStrategySpec* HYD_FindPressureStrategySpec(HYD_PressureControllerType strategy) {
    size_t index;

    for (index = 0U; index < sizeof(HYD_PRESSURE_STRATEGY_SPECS) / sizeof(HYD_PRESSURE_STRATEGY_SPECS[0]); ++index) {
        if (HYD_PRESSURE_STRATEGY_SPECS[index].strategy == strategy) {
            return &HYD_PRESSURE_STRATEGY_SPECS[index];
        }
    }

    return NULL;
}

static const HYD_PressureStrategySpec* HYD_ResolvePressureStrategySpec(const HYD_MotionSegment* segment) {
    const HYD_PressureStrategySpec* spec;

    if (segment != NULL) {
        spec = HYD_FindPressureStrategySpec(segment->pressureController);
        if (spec != NULL) {
            return spec;
        }
    }

    return HYD_FindPressureStrategySpec(HYD_PRESSURE_CONTROLLER_P);
}

static HYD_REAL HYD_ResolvePositiveOrDefault(HYD_REAL configuredValue, HYD_REAL defaultValue) {
    if (configuredValue > 0.0) {
        return configuredValue;
    }
    return defaultValue;
}

static HYD_REAL HYD_ResolveGain(HYD_REAL configuredGain, HYD_REAL fallbackGain) {
    return HYD_ResolvePositiveOrDefault(configuredGain, fallbackGain);
}

static HYD_REAL HYD_ResolveIntegralLimit(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }

    if (segment->pressureIntegralLimit > 0.0) {
        return segment->pressureIntegralLimit;
    }

    if (segment->maxFlow > 0.0) {
        return segment->maxFlow;
    }

    return 0.0;
}

static HYD_REAL HYD_ResolveFilterAlpha(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_PRESSURE_FILTER_ALPHA;
    }

    if (segment->pressureFilterAlpha > 0.0) {
        return HYD_ClampReal(segment->pressureFilterAlpha, 0.0, 1.0);
    }

    return HYD_DEFAULT_PRESSURE_FILTER_ALPHA;
}

static HYD_REAL HYD_ResolveDerivativeFilterAlpha(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
    }

    if (segment->pressureDerivativeFilterAlpha > 0.0) {
        return HYD_ClampReal(segment->pressureDerivativeFilterAlpha, 0.0, 1.0);
    }

    return HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
}

static HYD_REAL HYD_ResolveDeadband(const HYD_MotionSegment* segment) {
    if (segment == NULL || segment->pressureDeadband <= 0.0) {
        return 0.0;
    }
    return segment->pressureDeadband;
}

static HYD_REAL HYD_ResolveTrackedIntegralOutput(const HYD_MotionSegment* segment,
                                                 const HYD_PressureControllerInput* input,
                                                 HYD_REAL proportionalTerm,
                                                 HYD_REAL derivativeTerm,
                                                 HYD_REAL trackedOutputFlow) {
    HYD_REAL trackedIntegral;
    HYD_REAL integralLimit;

    if (input == NULL) {
        return 0.0;
    }

    trackedIntegral = trackedOutputFlow - input->feedforwardFlow - proportionalTerm - derivativeTerm;
    integralLimit = HYD_ResolveIntegralLimit(segment);
    if (integralLimit > 0.0) {
        trackedIntegral = HYD_ClampReal(trackedIntegral, -integralLimit, integralLimit);
    }
    return trackedIntegral;
}

static HYD_REAL HYD_ResolveAdaptiveSamplingPeriod(const HYD_PressureControllerState* state,
                                                  HYD_REAL dt) {
    if (dt > 0.0) {
        return dt;
    }

    if (state != NULL && state->rbfInitialized && state->rbfPid.sampling_period > 0.0f) {
        return (HYD_REAL)state->rbfPid.sampling_period;
    }

    return HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD;
}

static void HYD_ResolveRbfPidConfig(const HYD_MotionSegment* segment,
                                    HYD_RbfPidResolvedConfig* config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->minKp = (HYD_REAL)PID_MIN_KP;
    config->maxKp = (HYD_REAL)PID_MAX_KP;
    config->minKi = (HYD_REAL)PID_MIN_KI;
    config->maxKi = (HYD_REAL)PID_MAX_KI;
    config->minKd = (HYD_REAL)PID_MIN_KD;
    config->maxKd = (HYD_REAL)PID_MAX_KD;
    config->etaW = HYD_DEFAULT_RBF_LEARNING_RATE;
    config->etaC = HYD_DEFAULT_RBF_LEARNING_RATE;
    config->etaB = HYD_DEFAULT_RBF_LEARNING_RATE;
    config->etaP = HYD_DEFAULT_RBF_LEARNING_RATE;
    config->etaI = HYD_DEFAULT_RBF_LEARNING_RATE;
    config->etaD = HYD_DEFAULT_RBF_LEARNING_RATE;

    if (segment == NULL) {
        return;
    }

    config->minKp = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKp, config->minKp);
    config->maxKp = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKp, config->maxKp);
    config->minKi = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKi, config->minKi);
    config->maxKi = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKi, config->maxKi);
    config->minKd = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKd, config->minKd);
    config->maxKd = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKd, config->maxKd);
    config->etaW = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaW, config->etaW);
    config->etaC = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaC, config->etaC);
    config->etaB = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaB, config->etaB);
    config->etaP = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaP, config->etaP);
    config->etaI = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaI, config->etaI);
    config->etaD = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaD, config->etaD);
}

static void HYD_ResolvePressureControllerConfig(const HYD_MotionSegment* segment,
                                                const HYD_PressureControllerState* state,
                                                const HYD_PressureControllerInput* input,
                                                HYD_PressureResolvedConfig* config) {
    const HYD_PressureStrategySpec* strategySpec;

    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    strategySpec = HYD_ResolvePressureStrategySpec(segment);
    config->strategySpec = strategySpec;
    config->strategy = strategySpec->strategy;
    config->kp = HYD_ResolveGain((segment != NULL) ? segment->pressureKp : 0.0,
                                 HYD_LEGACY_PRESSURE_FLOW_KP);
    config->kpHigh = (segment != NULL && segment->pressureKpHigh > 0.0)
        ? segment->pressureKpHigh : 0.0;
    config->gainBand = (segment != NULL && segment->pressureGainBand > 0.0)
        ? segment->pressureGainBand : 0.2;
    config->ki = strategySpec->supportsIntegral
        ? HYD_ResolveGain((segment != NULL) ? segment->pressureKi : 0.0, 0.0)
        : 0.0;
    config->kd = strategySpec->supportsDerivative
        ? HYD_ResolveGain((segment != NULL) ? segment->pressureKd : 0.0, 0.0)
        : 0.0;
    config->integralLimit = HYD_ResolveIntegralLimit(segment);
    config->deadband = HYD_ResolveDeadband(segment);
    config->filterAlpha = HYD_ResolveFilterAlpha(segment);
    config->derivativeFilterAlpha = HYD_ResolveDerivativeFilterAlpha(segment);
    config->outputMin = (input != NULL) ? input->outputMin : 0.0;
    /* M4: 泄压目标下限钳位：防止过度泄压导致空穴 */
    if (input != NULL && input->targetPressure < 2.0) {
        config->outputMin = 0.0; /* 目标压力 < 2 bar 时禁止负流量 */
    }
    config->outputMax = (input != NULL) ? input->outputMax : 0.0;
    if (config->outputMax < config->outputMin) {
        config->outputMax = config->outputMin;
    }

    if (input != NULL && state != NULL) {
        config->dt = input->timestamp - state->previousTimestamp;
        if (config->dt < 0.0) {
            config->dt = 0.0;
        }
    }

    config->samplingPeriod = HYD_ResolveAdaptiveSamplingPeriod(state, config->dt);
    HYD_ResolveRbfPidConfig(segment, &config->rbf);
}

static void HYD_EnsureRbfPidInitialized(HYD_PressureControllerState* state,
                                        HYD_REAL samplingPeriod,
                                        HYD_REAL outputMax,
                                        HYD_REAL flowToPumpSpeedGain,
                                        HYD_REAL pumpSpeedLimit) {
    HYD_REAL resolvedOutputMax;
    float fMaxFlow, fFlowRateLimit;

    if (state == NULL) {
        return;
    }

    resolvedOutputMax = (outputMax > 0.0) ? outputMax : 0.0;

    /* Compute max pump flow [L/min] = pumpSpeedLimit [rpm] / flowToPumpSpeedGain [rpm/(L/min)] */
    if (flowToPumpSpeedGain > 0.0 && pumpSpeedLimit >= 0.0) {
        fMaxFlow = (float)(pumpSpeedLimit / flowToPumpSpeedGain);
    } else {
        fMaxFlow = 0.0f;
    }
    if (fMaxFlow <= 0.0f) fMaxFlow = 90.0f;  /* safe fallback: 1800 rpm / 20 rpm per L/min */

    /* Compute flow rate limit as fraction of max flow, capped at 1.0 */
    fFlowRateLimit = (resolvedOutputMax > 0.0 && fMaxFlow > 0.0f)
        ? (float)(resolvedOutputMax / (HYD_REAL)fMaxFlow) : 1.0f;
    if (fFlowRateLimit > 1.0f) fFlowRateLimit = 1.0f;

    if (!state->rbfInitialized) {
        RBF_PID_Init(&state->rbfPid,
                     (float)samplingPeriod,
                     fMaxFlow,
                     fFlowRateLimit);
        state->rbfInitialized = true;
    }

    state->rbfPid.sampling_period = (float)samplingPeriod;
    state->rbfPid.fMaxFlow = fMaxFlow;
    state->rbfPid.fFlowRateLimit = fFlowRateLimit;
    state->rbfPid.enable = true;
}

static void HYD_ApplyRbfPidConfig(HYD_PressureControllerState* state,
                                  const HYD_PressureResolvedConfig* config,
                                  const HYD_MotionSegment* segment,
                                  HYD_REAL flowToPumpSpeedGain,
                                  HYD_REAL pumpSpeedLimit) {
    if (state == NULL || config == NULL) {
        return;
    }

    HYD_EnsureRbfPidInitialized(state, config->samplingPeriod, config->outputMax,
                                flowToPumpSpeedGain, pumpSpeedLimit);
    RBF_PID_SetParamLimits(&state->rbfPid,
                           (float)config->rbf.minKp,
                           (float)config->rbf.maxKp,
                           (float)config->rbf.minKi,
                           (float)config->rbf.maxKi,
                           (float)config->rbf.minKd,
                           (float)config->rbf.maxKd);
    RBF_PID_SetLearningRates(&state->rbfPid,
                             (float)config->rbf.etaW,
                             (float)config->rbf.etaC,
                             (float)config->rbf.etaB,
                             (float)config->rbf.etaP,
                             (float)config->rbf.etaI,
                             (float)config->rbf.etaD);
    state->rbfPid.KP = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KP,
                                            config->rbf.minKp,
                                            config->rbf.maxKp);
    state->rbfPid.KI = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KI,
                                            config->rbf.minKi,
                                            config->rbf.maxKi);
    state->rbfPid.KD = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KD,
                                            config->rbf.minKd,
                                            config->rbf.maxKd);

    /* Per-segment RBF pressure normalization scale.
     *
     * 策略 (2026-06-03 修复):
     *   优先使用 pressureCeiling（显式上限）。
     *   否则使用 MAX(250, targetPressure * 5.0) 作为归一化标量。
     *   这确保低压目标（如2-3 bar）时归一化标量不会过小，
     *   避免归一化后的输出被过度放大（与系统物理增益失配）。
     *
     *   原因: 当 targetPressure=2 bar 时，旧逻辑使用 4 bar 作为标量，
     *   归一化后的 PID 输出 du≈0.4，乘以 fMaxFlow=90 → n_out≈36 L/min，
     *   在系统增益 K=5.4 时产生 36*5.4=194 bar 的稳态压力，远超目标。
     *
     *   新逻辑使用 250 bar（满量程）作为标量，归一化后 du≈0.01，
     *   n_out≈0.9 L/min，稳态压力≈4.9 bar — 仍在 PID 可调节范围内。
     *   配合增益补偿因子进一步精确匹配。 */
    {
        HYD_REAL pressureScale = 0.0;
        if (segment != NULL) {
            if (segment->pressureCeiling > 0.0) {
                pressureScale = segment->pressureCeiling;
            } else if (segment->targetPressure > 0.0) {
                /* 使用较大的归一化标量，避免低压时归一化空间压缩 segment->targetPressure * 5.0f*/
                HYD_REAL candidate = MAX_PRESSURE;
                pressureScale = (candidate > (HYD_REAL)MAX_PRESSURE) ? candidate : (HYD_REAL)MAX_PRESSURE;
            }
        }
        RBF_PID_SetPressureNormalization(&state->rbfPid, (float)pressureScale);
    }

    /* 设置系统增益补偿: 将段配置的系统增益 K 传递给 RBF PID，
     * 使其能正确缩放输出以匹配物理系统的压力/流量关系。
     * K = deltaPressure / deltaFlow [bar/(L/min)]
     * 当 segment->systemGain > 0 时启用补偿。 */
    if (segment != NULL && segment->systemGain > 0.0) {
        RBF_PID_SetGainCompensation(&state->rbfPid, (float)segment->systemGain);
    }
}

static void HYD_SynchronizeRbfPidState(HYD_PressureControllerState* state,
                                       HYD_REAL trackedOutputFlow,
                                       HYD_REAL targetPressure,
                                       HYD_REAL measuredPressure,
                                       const HYD_PressureResolvedConfig* config,
                                       const HYD_MotionSegment* segment,
                                       HYD_REAL flowToPumpSpeedGain,
                                       HYD_REAL pumpSpeedLimit) {
    HYD_REAL seededOutput;
    HYD_REAL normScale;

    if (state == NULL || config == NULL) {
        return;
    }

    RBF_PID_Reset(&state->rbfPid);
    HYD_ApplyRbfPidConfig(state, config, segment,
                          flowToPumpSpeedGain, pumpSpeedLimit);

    seededOutput = HYD_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);
    if (seededOutput < (HYD_REAL)MIN_OUTPUT) {
        seededOutput = (HYD_REAL)MIN_OUTPUT;
    }

    /* Use the same normalization scale that RBF_PID_Update will apply, so the
     * seeded Setpoint/Feedback/last_ref values remain consistent across the
     * sync→update boundary and bumpless tracking is preserved. Fallback to
     * MAX_PRESSURE when the per-segment scale is unconfigured (0). */
    normScale = (state->rbfPid.pressure_normalization_scale > 0.0f)
                ? (HYD_REAL)state->rbfPid.pressure_normalization_scale
                : (HYD_REAL)MAX_PRESSURE;

    state->rbfPid.Output = (float)(seededOutput / (HYD_REAL)state->rbfPid.fMaxFlow);
    state->rbfPid.u_prev = (float)(seededOutput / (HYD_REAL)state->rbfPid.fMaxFlow);
    state->rbfPid.n_out = (float)(seededOutput / (HYD_REAL)state->rbfPid.fMaxFlow);
    state->rbfPid.P_set = (float)targetPressure;
    state->rbfPid.P_actual = (float)measuredPressure;
    state->rbfPid.Setpoint = (float)(targetPressure / normScale);
    state->rbfPid.Feedback = (float)(measuredPressure / normScale);
    state->rbfPid.Error = state->rbfPid.Setpoint - state->rbfPid.Feedback;
    state->rbfPid.y_prev1 = state->rbfPid.Feedback;
    state->rbfPid.du = 0.0f;
    state->rbfPid.du_prev = 0.0f;
    /* Initialize error history to current error to prevent derivative kick on
     * first Update() call. Without this, raw_de = Error - 0 = Error on first
     * scan, causing a massive KD contribution that can produce >70% overshoot
     * even with gain compensation enabled. */
    state->rbfPid.e_prev1 = state->rbfPid.Error;
    state->rbfPid.e_prev2 = state->rbfPid.Error;
    state->rbfPid.delta_temp_prev = 0.0f;
    state->rbfPid.fLastActPress = state->rbfPid.Feedback;
    state->rbfPid.fLastActPress2 = state->rbfPid.Feedback;
    state->rbfPid.fUffAcc = 0.0f;
    state->rbfPid.last_ref = state->rbfPid.Setpoint;
    state->rbfPid.Status = 0;
    state->rbfPid.TuneResult = 0;
    state->rbfPid.Reset = false;
    state->rbfPid.FirstScan = false;
}

void HYD_PressureController_ClearState(HYD_PressureControllerState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->activeStrategy = HYD_PRESSURE_CONTROLLER_NONE;
}

void HYD_PressureController_InitState(HYD_PressureControllerState* state,
                                      HYD_REAL initialPressure,
                                      HYD_REAL initialOutputFlow,
                                      HYD_TIME timestamp) {
    if (state == NULL) {
        return;
    }

    HYD_PressureController_ClearState(state);
    state->initialized = true;
    state->trackingRequested = false;
    state->previousFilteredPressure = initialPressure;
    state->previousFilteredPressureRate = 0.0;
    state->previousOutput = initialOutputFlow;
    state->previousTimestamp = timestamp;
}

void HYD_PressureController_RequestTracking(HYD_PressureControllerState* state,
                                           HYD_REAL trackedOutputFlow) {
    if (state == NULL) {
        return;
    }

    state->previousOutput = trackedOutputFlow;
    state->trackingRequested = true;
}

void HYD_PressureController_Execute(const HYD_MotionSegment* segment,
                                    HYD_PressureControllerState* state,
                                    const HYD_PressureControllerInput* input,
                                    HYD_PressureControllerOutput* output) {
    HYD_PressureResolvedConfig config;
    HYD_REAL filteredPressure;
    HYD_REAL rawPressureRate;
    HYD_REAL filteredPressureRate;
    HYD_REAL error;
    HYD_REAL derivativeTerm;
    HYD_REAL proportionalTerm;
    HYD_REAL integralCandidate;
    HYD_REAL integralTerm;
    HYD_REAL trackingTerm;
    HYD_REAL unsaturatedOutput;
    HYD_REAL outputFlow;
    HYD_REAL trackedOutputFlow;
    HYD_BOOL trackingRequested;

    if (output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->appliedStrategy = HYD_PRESSURE_CONTROLLER_NONE;

    if (segment == NULL || state == NULL || input == NULL) {
        return;
    }

    if (!state->initialized) {
        HYD_PressureController_InitState(state,
                                         input->measuredPressure,
                                         HYD_ClampReal(input->feedforwardFlow,
                                                       input->outputMin,
                                                       input->outputMax),
                                         input->timestamp);
    }

    HYD_ResolvePressureControllerConfig(segment, state, input, &config);
    filteredPressure = state->previousFilteredPressure +
        config.filterAlpha * (input->measuredPressure - state->previousFilteredPressure);

    filteredPressureRate = state->previousFilteredPressureRate;
    if (config.dt > 0.0) {
        rawPressureRate = (filteredPressure - state->previousFilteredPressure) / config.dt;
        filteredPressureRate = state->previousFilteredPressureRate +
            config.derivativeFilterAlpha * (rawPressureRate - state->previousFilteredPressureRate);
    }

    error = input->targetPressure - filteredPressure;
    if (fabs(error) <= config.deadband) {
        error = 0.0;
    }

    trackingRequested = state->trackingRequested ||
        ((state->activeStrategy != HYD_PRESSURE_CONTROLLER_NONE) &&
         (state->activeStrategy != config.strategy));
    trackedOutputFlow = HYD_ClampReal(state->previousOutput, config.outputMin, config.outputMax);

    output->appliedStrategy = config.strategy;
    output->targetPressure = input->targetPressure;
    output->filteredPressure = filteredPressure;
    output->filteredPressureRate = filteredPressureRate;
    output->controlError = error;
    output->feedforwardFlow = input->feedforwardFlow;
    output->samplingPeriod = config.dt;
    output->adaptiveActive = config.strategySpec->adaptive;

    if (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PID) {
        HYD_REAL effectiveTargetPressure;
        HYD_REAL rawOutputFlow;
        HYD_BOOL needsAdaptiveReset;

        needsAdaptiveReset = trackingRequested || !state->rbfInitialized;
        HYD_ApplyRbfPidConfig(state, &config, segment,
                              input->flowToPumpSpeedGain, input->pumpSpeedLimit);

        if (needsAdaptiveReset) {
            output->trackingApplied = true;
            HYD_SynchronizeRbfPidState(state,
                                       trackedOutputFlow,
                                       input->targetPressure,
                                       filteredPressure,
                                       &config,
                                       segment,
                                       input->flowToPumpSpeedGain,
                                       input->pumpSpeedLimit);
        }

        effectiveTargetPressure = (error == 0.0) ? filteredPressure : input->targetPressure;
        state->rbfPid.P_set = (float)effectiveTargetPressure;
        state->rbfPid.P_actual = (float)filteredPressure;
        rawOutputFlow = (HYD_REAL)RBF_PID_Update(&state->rbfPid,
                                                 (float)effectiveTargetPressure,
                                                 (float)filteredPressure);
        outputFlow = HYD_ClampReal(rawOutputFlow, config.outputMin, config.outputMax);

        /* 负流量死区：仅当压力偏差 <= -2.0 bar（超压 >= 2 bar）时才允许负流量 */
        if (config.outputMin < 0.0 && outputFlow < 0.0 && error > -2.0) {
            outputFlow = 0.0; /* 小偏差时不使用负流量，防止0附近震荡 */
        }

        output->targetPressure = effectiveTargetPressure;
        output->feedbackFlow = rawOutputFlow - input->feedforwardFlow;
        output->unsaturatedOutputFlow = rawOutputFlow;
        output->outputFlow = outputFlow;
        output->samplingPeriod = config.samplingPeriod;
        output->adaptiveKp = (HYD_REAL)state->rbfPid.KP;
        output->adaptiveKi = (HYD_REAL)state->rbfPid.KI;
        output->adaptiveKd = (HYD_REAL)state->rbfPid.KD;
        output->adaptiveJacobian = (HYD_REAL)state->rbfPid.Jacobian;
        output->saturated = (outputFlow != rawOutputFlow);

        state->initialized = true;
        state->trackingRequested = false;
        state->integralOutput = 0.0;
        state->previousError = error;
        state->previousFilteredPressure = filteredPressure;
        state->previousFilteredPressureRate = filteredPressureRate;
        state->previousOutput = outputFlow;
        state->previousTimestamp = input->timestamp;
        state->activeStrategy = config.strategy;
        return;
    }

    proportionalTerm = config.kp * error;

    if (config.kpHigh > 0.0 && fabs(input->targetPressure) > 0.0) {
        HYD_REAL errorRatio = fabs(error) / input->targetPressure;
        HYD_REAL fraction = HYD_ClampReal(errorRatio / config.gainBand, 0.0, 1.0);
        HYD_REAL kpEff = config.kp + fraction * (config.kpHigh - config.kp);
        proportionalTerm = kpEff * error;
    }

    derivativeTerm = 0.0;
    if (config.strategySpec->supportsDerivative && config.dt > 0.0) {
        derivativeTerm = -config.kd * filteredPressureRate;
    }

    trackingTerm = 0.0;
    integralTerm = state->integralOutput;

    if (trackingRequested) {
        output->trackingApplied = true;
        if (config.strategySpec->supportsIntegral) {
            integralTerm = HYD_ResolveTrackedIntegralOutput(segment,
                                                            input,
                                                            proportionalTerm,
                                                            derivativeTerm,
                                                            trackedOutputFlow);
        } else {
            trackingTerm = trackedOutputFlow - input->feedforwardFlow - proportionalTerm - derivativeTerm;
        }
    }

    if (config.strategySpec->supportsIntegral && config.dt > 0.0 && config.ki > 0.0) {
        integralCandidate = integralTerm + config.ki * error * config.dt;
        if (config.integralLimit > 0.0) {
            integralCandidate = HYD_ClampReal(integralCandidate,
                                              -config.integralLimit,
                                              config.integralLimit);
        }

        unsaturatedOutput = input->feedforwardFlow + proportionalTerm + integralCandidate + derivativeTerm + trackingTerm;
        if ((unsaturatedOutput >= config.outputMin && unsaturatedOutput <= config.outputMax) ||
            (unsaturatedOutput > config.outputMax && error < 0.0) ||
            (unsaturatedOutput < config.outputMin && error > 0.0)) {
            integralTerm = integralCandidate;
        }
    }

    unsaturatedOutput = input->feedforwardFlow + proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    outputFlow = HYD_ClampReal(unsaturatedOutput, config.outputMin, config.outputMax);

    /* 负流量死区：仅当压力偏差 <= -2.0 bar（超压 >= 2 bar）时才允许负流量 */
    if (config.outputMin < 0.0 && outputFlow < 0.0 && error > -2.0) {
        outputFlow = HYD_ClampReal(unsaturatedOutput, 0.0, config.outputMax);
    }

    output->proportionalTerm = proportionalTerm;
    output->integralTerm = integralTerm;
    output->derivativeTerm = derivativeTerm;
    output->trackingTerm = trackingTerm;
    output->feedbackFlow = proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    output->unsaturatedOutputFlow = unsaturatedOutput;
    output->outputFlow = outputFlow;
    output->saturated = (outputFlow != unsaturatedOutput);

    state->initialized = true;
    state->trackingRequested = false;
    state->integralOutput = integralTerm;
    state->previousError = error;
    state->previousFilteredPressure = filteredPressure;
    state->previousFilteredPressureRate = filteredPressureRate;
    state->previousOutput = outputFlow;
    state->previousTimestamp = input->timestamp;
    state->activeStrategy = config.strategy;
}
