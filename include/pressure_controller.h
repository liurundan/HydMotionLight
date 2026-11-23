#ifndef HYD_PRESSURE_CONTROLLER_H
#define HYD_PRESSURE_CONTROLLER_H

#include "common_types.h"
#include "rbf_pid.h"
#include "pressure_ripple_comp.h"

#define HYD_RBF_GAIN_OVERRIDE_KP 0x01u
#define HYD_RBF_GAIN_OVERRIDE_KI 0x02u
#define HYD_RBF_GAIN_OVERRIDE_KD 0x04u
#define HYD_RBF_DEBUG_GAIN_WINDOW_RATIO 0.20f

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL measuredPressure;
    HYD_REAL feedforwardFlow;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
    HYD_REAL flowToPumpSpeedGain;  /* rpm per L/min, > 0 — used by RBF PID for fMaxFlow derivation */
    HYD_REAL pumpSpeedLimit;       /* rpm, >= 0 — pump speed upper bound */
    HYD_TIME timestamp;
    HYD_REAL systemGainBarPerRpm;  /* offline calibrated Ksys [bar/rpm], 0 disables */
    HYD_PumpFeedback pumpFeedback;  /* shared system-level pump feedback snapshot */
    HYD_REAL targetPressureRate;    /* bar/s, profiled reference slope; 0 with the legacy ramp */
    HYD_REAL kqBarPerSecRpm;        /* bar/s per rpm; > 0 with a non-RBF strategy selects the model path */
    HYD_UINT8 pumpRippleOrder;      /* ripples per pump revolution; 0 disables ripple compensation */
    HYD_PressureRippleCompState* rippleComp; /* learned ripple, owned by the caller; NULL disables it */
} HYD_PressureControllerInput;

typedef struct {
    HYD_BOOL initialized;
    HYD_BOOL trackingRequested;
    HYD_BOOL rbfInitialized;
    HYD_UINT8 rbfDebugGainOverrideMask; /* Active segment PLC debug gains. */
    HYD_REAL integralOutput;
    HYD_REAL previousError;
    HYD_REAL previousFilteredPressure;
    HYD_REAL previousFilteredPressureRate;
    HYD_REAL previousOutput;
    HYD_TIME previousTimestamp;
    HYD_REAL previousTargetPressure;
    HYD_BOOL targetPressureInitialized;
    HYD_BOOL reliefActive;
    HYD_PressureControllerType activeStrategy;
    RBF_PID_Handle rbfPid;
    HYD_REAL referenceDelay[HYD_PRESSURE_REF_DELAY_SAMPLES]; /* model path: reference history */
    HYD_UINT8 referenceDelayIndex;
    HYD_BOOL referenceDelayPrimed;
} HYD_PressureControllerState;

typedef struct {
    HYD_PressureControllerType appliedStrategy;
    HYD_REAL targetPressure;
    HYD_REAL filteredPressure;
    HYD_REAL filteredPressureRate;
    HYD_REAL controlError;
    HYD_REAL proportionalTerm;
    HYD_REAL integralTerm;
    HYD_REAL derivativeTerm;
    HYD_REAL trackingTerm;
    HYD_REAL feedforwardFlow;
    HYD_REAL feedbackFlow;
    HYD_REAL unsaturatedOutputFlow;
    HYD_REAL outputFlow;
    HYD_REAL requestedOutputMin;
    HYD_REAL reliefPolicyMin;
    HYD_REAL pumpCapabilityMin;
    HYD_REAL effectiveOutputMin;
    HYD_REAL samplingPeriod;
    HYD_REAL adaptiveKp;
    HYD_REAL adaptiveKi;
    HYD_REAL adaptiveKd;
    HYD_REAL adaptiveJacobian;
    HYD_BOOL trackingApplied;
    HYD_BOOL saturated;
    HYD_BOOL adaptiveActive;
    HYD_BOOL reliefActive;
    HYD_REAL referencePressureRate; /* bar/s, model path only */
    HYD_REAL rippleCompFlow;        /* L/min, model path only */
} HYD_PressureControllerOutput;

void HYD_PressureController_ClearState(HYD_PressureControllerState* state);
void HYD_PressureController_InitState(HYD_PressureControllerState* state,
                                      HYD_REAL initialPressure,
                                      HYD_REAL initialOutputFlow,
                                      HYD_TIME timestamp);
void HYD_PressureController_RequestTracking(HYD_PressureControllerState* state,
                                           HYD_REAL trackedOutputFlow);
/* Reference slope bounds for the profiled pressure ramp (spec 2026-11-23 §4.3):
 * rise = η·Kq_f·max(outputMax − P/R_f, r·outputMax),
 * fall = η·Kq_f·max(|reliefFlow| + P/R_f, r·outputMax),
 * Kq_f = kq·gain, R_f = ksys·gain (ksys = 0 drops the leak term).
 * Returns false (and zero bounds) when kq, gain or outputMax is not positive. */
HYD_BOOL HYD_PressureController_ResolveRateLimits(HYD_REAL kqBarPerSecRpm,
                                                  HYD_REAL ksysBarPerRpm,
                                                  HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL targetPressure,
                                                  HYD_REAL outputMax,
                                                  HYD_REAL reliefFlow,
                                                  HYD_REAL* riseRate,
                                                  HYD_REAL* fallRate);
void HYD_PressureController_Execute(const HYD_MotionSegment* segment,
                                    HYD_PressureControllerState* state,
                                    const HYD_PressureControllerInput* input,
                                    HYD_PressureControllerOutput* output);

#endif /* HYD_PRESSURE_CONTROLLER_H */
