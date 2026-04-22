#include "pressure_controller.h"
#include <math.h>
#include <string.h>

#define HDY_LEGACY_PRESSURE_FLOW_KP 1.5
#define HDY_DEFAULT_PRESSURE_FILTER_ALPHA 1.0
#define HDY_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA 1.0
#define HDY_DEFAULT_RBF_PID_SAMPLING_PERIOD 0.01
#define HDY_DEFAULT_RBF_LEARNING_RATE 0.25

typedef struct {
    HDY_PressureControllerType strategy;
    HDY_BOOL supportsIntegral;
    HDY_BOOL supportsDerivative;
    HDY_BOOL adaptive;
} HDY_PressureStrategySpec;

typedef struct {
    HDY_REAL minKp;
    HDY_REAL maxKp;
    HDY_REAL minKi;
    HDY_REAL maxKi;
    HDY_REAL minKd;
    HDY_REAL maxKd;
    HDY_REAL etaW;
    HDY_REAL etaC;
    HDY_REAL etaB;
    HDY_REAL etaP;
    HDY_REAL etaI;
    HDY_REAL etaD;
} HDY_RbfPidResolvedConfig;

typedef struct {
    const HDY_PressureStrategySpec* strategySpec;
    HDY_PressureControllerType strategy;
    HDY_REAL kp;
    HDY_REAL ki;
    HDY_REAL kd;
    HDY_REAL integralLimit;
    HDY_REAL deadband;
    HDY_REAL filterAlpha;
    HDY_REAL derivativeFilterAlpha;
    HDY_REAL outputMin;
    HDY_REAL outputMax;
    HDY_REAL dt;
    HDY_REAL samplingPeriod;
    HDY_RbfPidResolvedConfig rbf;
} HDY_PressureResolvedConfig;

static const HDY_PressureStrategySpec HDY_PRESSURE_STRATEGY_SPECS[] = {
    {HDY_PRESSURE_CONTROLLER_P, false, false, false},
    {HDY_PRESSURE_CONTROLLER_PI, true, false, false},
    {HDY_PRESSURE_CONTROLLER_PID, true, true, false},
    {HDY_PRESSURE_CONTROLLER_RBF_PID, true, true, true}
};

static const HDY_PressureStrategySpec* HDY_FindPressureStrategySpec(HDY_PressureControllerType strategy) {
    size_t index;

    for (index = 0U; index < sizeof(HDY_PRESSURE_STRATEGY_SPECS) / sizeof(HDY_PRESSURE_STRATEGY_SPECS[0]); ++index) {
        if (HDY_PRESSURE_STRATEGY_SPECS[index].strategy == strategy) {
            return &HDY_PRESSURE_STRATEGY_SPECS[index];
        }
    }

    return NULL;
}

static const HDY_PressureStrategySpec* HDY_ResolvePressureStrategySpec(const HDY_MotionSegment* segment) {
    const HDY_PressureStrategySpec* spec;

    if (segment != NULL) {
        spec = HDY_FindPressureStrategySpec(segment->pressureController);
        if (spec != NULL) {
            return spec;
        }
    }

    return HDY_FindPressureStrategySpec(HDY_PRESSURE_CONTROLLER_P);
}

static HDY_REAL HDY_ResolvePositiveOrDefault(HDY_REAL configuredValue, HDY_REAL defaultValue) {
    if (configuredValue > 0.0) {
        return configuredValue;
    }
    return defaultValue;
}

static HDY_REAL HDY_ResolveGain(HDY_REAL configuredGain, HDY_REAL fallbackGain) {
    return HDY_ResolvePositiveOrDefault(configuredGain, fallbackGain);
}

static HDY_REAL HDY_ResolveIntegralLimit(const HDY_MotionSegment* segment) {
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

static HDY_REAL HDY_ResolveFilterAlpha(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return HDY_DEFAULT_PRESSURE_FILTER_ALPHA;
    }

    if (segment->pressureFilterAlpha > 0.0) {
        return HDY_ClampReal(segment->pressureFilterAlpha, 0.0, 1.0);
    }

    return HDY_DEFAULT_PRESSURE_FILTER_ALPHA;
}

static HDY_REAL HDY_ResolveDerivativeFilterAlpha(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return HDY_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
    }

    if (segment->pressureDerivativeFilterAlpha > 0.0) {
        return HDY_ClampReal(segment->pressureDerivativeFilterAlpha, 0.0, 1.0);
    }

    return HDY_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
}

static HDY_REAL HDY_ResolveDeadband(const HDY_MotionSegment* segment) {
    if (segment == NULL || segment->pressureDeadband <= 0.0) {
        return 0.0;
    }
    return segment->pressureDeadband;
}

static HDY_REAL HDY_ResolveTrackedIntegralOutput(const HDY_MotionSegment* segment,
                                                 const HDY_PressureControllerInput* input,
                                                 HDY_REAL proportionalTerm,
                                                 HDY_REAL derivativeTerm,
                                                 HDY_REAL trackedOutputFlow) {
    HDY_REAL trackedIntegral;
    HDY_REAL integralLimit;

    if (input == NULL) {
        return 0.0;
    }

    trackedIntegral = trackedOutputFlow - input->feedforwardFlow - proportionalTerm - derivativeTerm;
    integralLimit = HDY_ResolveIntegralLimit(segment);
    if (integralLimit > 0.0) {
        trackedIntegral = HDY_ClampReal(trackedIntegral, -integralLimit, integralLimit);
    }
    return trackedIntegral;
}

static HDY_REAL HDY_ResolveAdaptiveSamplingPeriod(const HDY_PressureControllerState* state,
                                                  HDY_REAL dt) {
    if (dt > 0.0) {
        return dt;
    }

    if (state != NULL && state->rbfInitialized && state->rbfPid.sampling_period > 0.0f) {
        return (HDY_REAL)state->rbfPid.sampling_period;
    }

    return HDY_DEFAULT_RBF_PID_SAMPLING_PERIOD;
}

static void HDY_ResolveRbfPidConfig(const HDY_MotionSegment* segment,
                                    HDY_RbfPidResolvedConfig* config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->minKp = (HDY_REAL)PID_MIN_KP;
    config->maxKp = (HDY_REAL)PID_MAX_KP;
    config->minKi = (HDY_REAL)PID_MIN_KI;
    config->maxKi = (HDY_REAL)PID_MAX_KI;
    config->minKd = (HDY_REAL)PID_MIN_KD;
    config->maxKd = (HDY_REAL)PID_MAX_KD;
    config->etaW = HDY_DEFAULT_RBF_LEARNING_RATE;
    config->etaC = HDY_DEFAULT_RBF_LEARNING_RATE;
    config->etaB = HDY_DEFAULT_RBF_LEARNING_RATE;
    config->etaP = HDY_DEFAULT_RBF_LEARNING_RATE;
    config->etaI = HDY_DEFAULT_RBF_LEARNING_RATE;
    config->etaD = HDY_DEFAULT_RBF_LEARNING_RATE;

    if (segment == NULL) {
        return;
    }

    config->minKp = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKp, config->minKp);
    config->maxKp = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKp, config->maxKp);
    config->minKi = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKi, config->minKi);
    config->maxKi = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKi, config->maxKi);
    config->minKd = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKd, config->minKd);
    config->maxKd = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKd, config->maxKd);
    config->etaW = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaW, config->etaW);
    config->etaC = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaC, config->etaC);
    config->etaB = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaB, config->etaB);
    config->etaP = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaP, config->etaP);
    config->etaI = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaI, config->etaI);
    config->etaD = HDY_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaD, config->etaD);
}

static void HDY_ResolvePressureControllerConfig(const HDY_MotionSegment* segment,
                                                const HDY_PressureControllerState* state,
                                                const HDY_PressureControllerInput* input,
                                                HDY_PressureResolvedConfig* config) {
    const HDY_PressureStrategySpec* strategySpec;

    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    strategySpec = HDY_ResolvePressureStrategySpec(segment);
    config->strategySpec = strategySpec;
    config->strategy = strategySpec->strategy;
    config->kp = HDY_ResolveGain((segment != NULL) ? segment->pressureKp : 0.0,
                                 HDY_LEGACY_PRESSURE_FLOW_KP);
    config->ki = strategySpec->supportsIntegral
        ? HDY_ResolveGain((segment != NULL) ? segment->pressureKi : 0.0, 0.0)
        : 0.0;
    config->kd = strategySpec->supportsDerivative
        ? HDY_ResolveGain((segment != NULL) ? segment->pressureKd : 0.0, 0.0)
        : 0.0;
    config->integralLimit = HDY_ResolveIntegralLimit(segment);
    config->deadband = HDY_ResolveDeadband(segment);
    config->filterAlpha = HDY_ResolveFilterAlpha(segment);
    config->derivativeFilterAlpha = HDY_ResolveDerivativeFilterAlpha(segment);
    config->outputMin = (input != NULL) ? input->outputMin : 0.0;
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

    config->samplingPeriod = HDY_ResolveAdaptiveSamplingPeriod(state, config->dt);
    HDY_ResolveRbfPidConfig(segment, &config->rbf);
}

static void HDY_EnsureRbfPidInitialized(HDY_PressureControllerState* state,
                                        HDY_REAL samplingPeriod,
                                        HDY_REAL outputMax) {
    HDY_REAL resolvedOutputMax;

    if (state == NULL) {
        return;
    }

    resolvedOutputMax = (outputMax > 0.0) ? outputMax : 0.0;
    if (!state->rbfInitialized) {
        RBF_PID_Init(&state->rbfPid,
                     (float)samplingPeriod,
                     1.0f,
                     (float)resolvedOutputMax);
        state->rbfInitialized = true;
    }

    state->rbfPid.sampling_period = (float)samplingPeriod;
    state->rbfPid.fMaxMotorSpeed = 1.0f;
    state->rbfPid.fMaxFlowRate = (float)resolvedOutputMax;
    state->rbfPid.enable = true;
}

static void HDY_ApplyRbfPidConfig(HDY_PressureControllerState* state,
                                  const HDY_PressureResolvedConfig* config) {
    if (state == NULL || config == NULL) {
        return;
    }

    HDY_EnsureRbfPidInitialized(state, config->samplingPeriod, config->outputMax);
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
    state->rbfPid.KP = (float)HDY_ClampReal((HDY_REAL)state->rbfPid.KP,
                                            config->rbf.minKp,
                                            config->rbf.maxKp);
    state->rbfPid.KI = (float)HDY_ClampReal((HDY_REAL)state->rbfPid.KI,
                                            config->rbf.minKi,
                                            config->rbf.maxKi);
    state->rbfPid.KD = (float)HDY_ClampReal((HDY_REAL)state->rbfPid.KD,
                                            config->rbf.minKd,
                                            config->rbf.maxKd);
}

static void HDY_SynchronizeRbfPidState(HDY_PressureControllerState* state,
                                       HDY_REAL trackedOutputFlow,
                                       HDY_REAL targetPressure,
                                       HDY_REAL measuredPressure,
                                       const HDY_PressureResolvedConfig* config) {
    HDY_REAL seededOutput;

    if (state == NULL || config == NULL) {
        return;
    }

    RBF_PID_Reset(&state->rbfPid);
    HDY_ApplyRbfPidConfig(state, config);

    seededOutput = HDY_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);
    if (seededOutput < (HDY_REAL)MIN_OUTPUT) {
        seededOutput = (HDY_REAL)MIN_OUTPUT;
    }

    state->rbfPid.Output = (float)seededOutput;
    state->rbfPid.u_prev = (float)seededOutput;
    state->rbfPid.n_out = (float)seededOutput;
    state->rbfPid.P_set = (float)targetPressure;
    state->rbfPid.P_actual = (float)measuredPressure;
    state->rbfPid.Setpoint = (float)(targetPressure / MAX_PRESSURE);
    state->rbfPid.Feedback = (float)(measuredPressure / MAX_PRESSURE);
    state->rbfPid.Error = state->rbfPid.Setpoint - state->rbfPid.Feedback;
    state->rbfPid.y_prev1 = state->rbfPid.Feedback;
    state->rbfPid.du = 0.0f;
    state->rbfPid.du_prev = 0.0f;
    state->rbfPid.e_prev1 = 0.0f;
    state->rbfPid.e_prev2 = 0.0f;
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

void HDY_PressureController_ClearState(HDY_PressureControllerState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->activeStrategy = HDY_PRESSURE_CONTROLLER_NONE;
}

void HDY_PressureController_InitState(HDY_PressureControllerState* state,
                                      HDY_REAL initialPressure,
                                      HDY_REAL initialOutputFlow,
                                      HDY_TIME timestamp) {
    if (state == NULL) {
        return;
    }

    HDY_PressureController_ClearState(state);
    state->initialized = true;
    state->trackingRequested = false;
    state->previousFilteredPressure = initialPressure;
    state->previousFilteredPressureRate = 0.0;
    state->previousOutput = initialOutputFlow;
    state->previousTimestamp = timestamp;
}

void HDY_PressureController_RequestTracking(HDY_PressureControllerState* state,
                                           HDY_REAL trackedOutputFlow) {
    if (state == NULL) {
        return;
    }

    state->previousOutput = trackedOutputFlow;
    state->trackingRequested = true;
}

void HDY_PressureController_Execute(const HDY_MotionSegment* segment,
                                    HDY_PressureControllerState* state,
                                    const HDY_PressureControllerInput* input,
                                    HDY_PressureControllerOutput* output) {
    HDY_PressureResolvedConfig config;
    HDY_REAL filteredPressure;
    HDY_REAL rawPressureRate;
    HDY_REAL filteredPressureRate;
    HDY_REAL error;
    HDY_REAL derivativeTerm;
    HDY_REAL proportionalTerm;
    HDY_REAL integralCandidate;
    HDY_REAL integralTerm;
    HDY_REAL trackingTerm;
    HDY_REAL unsaturatedOutput;
    HDY_REAL outputFlow;
    HDY_REAL trackedOutputFlow;
    HDY_BOOL trackingRequested;

    if (output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->appliedStrategy = HDY_PRESSURE_CONTROLLER_NONE;

    if (segment == NULL || state == NULL || input == NULL) {
        return;
    }

    if (!state->initialized) {
        HDY_PressureController_InitState(state,
                                         input->measuredPressure,
                                         HDY_ClampReal(input->feedforwardFlow,
                                                       input->outputMin,
                                                       input->outputMax),
                                         input->timestamp);
    }

    HDY_ResolvePressureControllerConfig(segment, state, input, &config);
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
        ((state->activeStrategy != HDY_PRESSURE_CONTROLLER_NONE) &&
         (state->activeStrategy != config.strategy));
    trackedOutputFlow = HDY_ClampReal(state->previousOutput, config.outputMin, config.outputMax);

    output->appliedStrategy = config.strategy;
    output->targetPressure = input->targetPressure;
    output->filteredPressure = filteredPressure;
    output->filteredPressureRate = filteredPressureRate;
    output->controlError = error;
    output->feedforwardFlow = input->feedforwardFlow;
    output->samplingPeriod = config.dt;
    output->adaptiveActive = config.strategySpec->adaptive;

    if (config.strategy == HDY_PRESSURE_CONTROLLER_RBF_PID) {
        HDY_REAL effectiveTargetPressure;
        HDY_REAL rawOutputFlow;
        HDY_BOOL needsAdaptiveReset;

        needsAdaptiveReset = trackingRequested || !state->rbfInitialized;
        HDY_ApplyRbfPidConfig(state, &config);

        if (needsAdaptiveReset) {
            output->trackingApplied = true;
            HDY_SynchronizeRbfPidState(state,
                                       trackedOutputFlow,
                                       input->targetPressure,
                                       filteredPressure,
                                       &config);
        }

        effectiveTargetPressure = (error == 0.0) ? filteredPressure : input->targetPressure;
        state->rbfPid.P_set = (float)effectiveTargetPressure;
        state->rbfPid.P_actual = (float)filteredPressure;
        rawOutputFlow = (HDY_REAL)RBF_PID_Update(&state->rbfPid,
                                                 (float)effectiveTargetPressure,
                                                 (float)filteredPressure);
        outputFlow = HDY_ClampReal(rawOutputFlow, config.outputMin, config.outputMax);

        output->targetPressure = effectiveTargetPressure;
        output->feedbackFlow = rawOutputFlow - input->feedforwardFlow;
        output->unsaturatedOutputFlow = rawOutputFlow;
        output->outputFlow = outputFlow;
        output->samplingPeriod = config.samplingPeriod;
        output->adaptiveKp = (HDY_REAL)state->rbfPid.KP;
        output->adaptiveKi = (HDY_REAL)state->rbfPid.KI;
        output->adaptiveKd = (HDY_REAL)state->rbfPid.KD;
        output->adaptiveJacobian = (HDY_REAL)state->rbfPid.Jacobian;
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
    derivativeTerm = 0.0;
    if (config.strategySpec->supportsDerivative && config.dt > 0.0) {
        derivativeTerm = -config.kd * filteredPressureRate;
    }

    trackingTerm = 0.0;
    integralTerm = state->integralOutput;

    if (trackingRequested) {
        output->trackingApplied = true;
        if (config.strategySpec->supportsIntegral) {
            integralTerm = HDY_ResolveTrackedIntegralOutput(segment,
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
            integralCandidate = HDY_ClampReal(integralCandidate,
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
    outputFlow = HDY_ClampReal(unsaturatedOutput, config.outputMin, config.outputMax);

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
