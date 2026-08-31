#include "pressure_controller.h"
#include <math.h>
#include <string.h>

#define HYD_LEGACY_PRESSURE_FLOW_KP 1.5
#define HYD_DEFAULT_PRESSURE_FILTER_ALPHA 0.1
#define HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA 0.05

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
    HYD_BOOL disablePressureAccelFeedforward;
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
    {HYD_PRESSURE_CONTROLLER_RBF_PID, true, true, true},
    {HYD_PRESSURE_CONTROLLER_RBF_PI, true, false, true}
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
    config->etaW = (HYD_REAL)HYD_DEFAULT_RBF_W_LEARNING_RATE;
    config->etaC = (HYD_REAL)HYD_DEFAULT_RBF_C_LEARNING_RATE;
    config->etaB = (HYD_REAL)HYD_DEFAULT_RBF_B_LEARNING_RATE;
    config->etaP = (HYD_REAL)HYD_DEFAULT_PID_P_LEARNING_RATE;
    config->etaI = (HYD_REAL)HYD_DEFAULT_PID_I_LEARNING_RATE;
    config->etaD = (HYD_REAL)HYD_DEFAULT_PID_D_LEARNING_RATE;
    config->disablePressureAccelFeedforward = false;

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
    config->disablePressureAccelFeedforward =
        segment->pressureRbfConfig.disablePressureAccelFeedforward > 0.0 ? true : false;
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
    /* M4: 泄压目标下限钳位：仅在目标压和反馈压都进入低压区时禁止负流量 */
    if (input != NULL &&
        input->targetPressure < 5.0 &&
        input->measuredPressure < 5.0) {
        config->outputMin = 0.0; /* 目标压力 < 5 bar 时禁止负流量 */
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
    state->rbfPid.output_min_flow = (float)MIN_OUTPUT;
    state->rbfPid.output_max_flow = (float)resolvedOutputMax;
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
    state->rbfPid.output_min_flow = (float)config->outputMin;
    state->rbfPid.output_max_flow = (float)config->outputMax;
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
    RBF_PID_SetControlMode(
        &state->rbfPid,
        config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI
            ? RBF_PID_CONTROL_MODE_PI
            : RBF_PID_CONTROL_MODE_PID);
    RBF_PID_SetPressureAccelFeedforwardEnabled(
        &state->rbfPid,
        config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
            config->rbf.disablePressureAccelFeedforward ? false : true);
    RBF_PID_SetFlowNormalization(
        &state->rbfPid,
        (float)HYD_ResolvePositiveOrDefault(config->outputMax,
                                            (HYD_REAL)state->rbfPid.fMaxFlow));
    state->rbfPid.KP = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KP,
                                            config->rbf.minKp,
                                            config->rbf.maxKp);
    state->rbfPid.KI = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KI,
                                            config->rbf.minKi,
                                            config->rbf.maxKi);
    state->rbfPid.KD = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KD,
                                            config->rbf.minKd,
                                            config->rbf.maxKd);
    state->rbfPid.pid_mode_kd = state->rbfPid.KD;
    if (config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI) {
        state->rbfPid.KD = 0.0f;
    }

    {
        HYD_REAL pressureScale = 0.0;
        if (segment != NULL && segment->pressureCeiling > 0.0) {
            pressureScale = segment->pressureCeiling;
        } else if (segment != NULL && segment->targetPressure > 0.0) {
            HYD_REAL candidate = segment->targetPressure * 3.0;
            pressureScale = (candidate > (HYD_REAL)MAX_PRESSURE) ?
                candidate : (HYD_REAL)MAX_PRESSURE;
        }
        RBF_PID_SetPressureNormalization(&state->rbfPid, (float)pressureScale);
    }

    if (segment != NULL && segment->systemGain > 0.0) {
        RBF_PID_SetGainCompensation(&state->rbfPid, (float)segment->systemGain);
    } else {
        RBF_PID_SetGainCompensation(&state->rbfPid, 0.0f);
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
    HYD_REAL seededFlow;
    HYD_REAL error;

    if (state == NULL || config == NULL) {
        return;
    }

    RBF_PID_Reset(&state->rbfPid);
    HYD_ApplyRbfPidConfig(state, config, segment,
                          flowToPumpSpeedGain, pumpSpeedLimit);
    seededFlow = HYD_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);

    error = targetPressure - measuredPressure;

    state->rbfPid.Output = (float)seededFlow;
    state->rbfPid.u_prev = (float)seededFlow;

    state->rbfPid.P_set = (float)targetPressure;
    state->rbfPid.P_actual = (float)measuredPressure;
    state->rbfPid.Error = (float)error;
    state->rbfPid.du = 0.0f;
    state->rbfPid.du_prev = 0.0f;
    state->rbfPid.e_prev1 = (float)error;
    state->rbfPid.e_prev2 = (float)error;
    state->rbfPid.y_prev1 = (float)measuredPressure;
    state->rbfPid.y_prev2 = (float)measuredPressure;
    state->rbfPid.fLastActPress = (float)measuredPressure;
    state->rbfPid.fLastActPress2 = (float)measuredPressure;
    state->rbfPid.last_ref = (float)targetPressure;
    state->rbfPid.output_saturated = false;
    state->rbfPid.Status = 1;
    state->rbfPid.TuneResult = 0;
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

    filteredPressure = input->measuredPressure ;
    filteredPressureRate = state->previousFilteredPressureRate;
    if (config.dt > 0.0) {
        rawPressureRate = (filteredPressure - state->previousFilteredPressure) / config.dt;
        filteredPressureRate = state->previousFilteredPressureRate +
            config.derivativeFilterAlpha * (rawPressureRate - state->previousFilteredPressureRate);
    }

    error = input->targetPressure - filteredPressure;

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

    if (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PID ||
        config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI) {
        HYD_REAL effectiveTargetPressure;
        HYD_REAL rawOutputFlow;
        HYD_BOOL needsAdaptiveReset;
        HYD_BOOL internalSaturated;

        needsAdaptiveReset =  !state->rbfInitialized|| trackingRequested;
        		//|| (input->targetPressure + 1e-6 < (HYD_REAL)state->rbfPid.P_set) ;
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

        effectiveTargetPressure = input->targetPressure; //(error == 0.0) ? filteredPressure :
        rawOutputFlow = (HYD_REAL)RBF_PID_Update(&state->rbfPid,
                                                 (float)effectiveTargetPressure,
                                                 (float)filteredPressure);
        internalSaturated = state->rbfPid.output_saturated;

        outputFlow = HYD_ClampReal(rawOutputFlow, config.outputMin, config.outputMax);

        /* 负流量死区：仅当压力偏差 <= -2.0 bar（超压 >= 2 bar）时才允许负流量 */
        if (config.outputMin < 0.0 && outputFlow < 0.0 && fabs(error) < 5.0) {
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
        output->saturated =
            (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI && internalSaturated) ||
            (outputFlow != rawOutputFlow);

        state->rbfPid.output_saturated = output->saturated ? true : false;
        state->rbfPid.Output = (float)outputFlow;
        state->rbfPid.u_prev = (float)outputFlow;

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
