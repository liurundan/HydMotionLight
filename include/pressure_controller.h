#ifndef HYD_PRESSURE_CONTROLLER_H
#define HYD_PRESSURE_CONTROLLER_H

#include "common_types.h"
#include "rbf_pid.h"
#include "pressure_ripple_comp.h"   /* HYD_PressureSteadyGateState */

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL measuredPressure;
    HYD_REAL feedforwardFlow;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
    HYD_REAL flowToPumpSpeedGain;  /* rpm per L/min, > 0 — used by RBF PID for fMaxFlow derivation */
    HYD_REAL pumpSpeedLimit;       /* rpm, >= 0 — pump speed upper bound */
    HYD_TIME timestamp;
    HYD_REAL pumpAngleRev;   /* 泵轴整圈数相位源（编码器喂入），默认 0 */
} HYD_PressureControllerInput;

typedef struct {
    HYD_BOOL initialized;
    HYD_BOOL trackingRequested;
    HYD_BOOL rbfInitialized;
    HYD_REAL integralOutput;
    HYD_REAL previousError;
    HYD_REAL previousFilteredPressure;
    HYD_REAL previousFilteredPressureRate;
    HYD_REAL previousOutput;
    HYD_TIME previousTimestamp;
    HYD_PressureControllerType activeStrategy;
    HYD_REAL ffTrim;                          /* FF 在线微调偏置 [L/min]，学 systemGain 外残差 */
    HYD_PressureSteadyGateState ffSteadyGate;/* FF-trim 稳态闸门 */
    RBF_PID_Handle rbfPid;
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
    HYD_REAL samplingPeriod;
    HYD_REAL adaptiveKp;
    HYD_REAL adaptiveKi;
    HYD_REAL adaptiveKd;
    HYD_REAL adaptiveJacobian;
    HYD_BOOL trackingApplied;
    HYD_BOOL saturated;
    HYD_BOOL adaptiveActive;
} HYD_PressureControllerOutput;

void HYD_PressureController_ClearState(HYD_PressureControllerState* state);
void HYD_PressureController_InitState(HYD_PressureControllerState* state,
                                      HYD_REAL initialPressure,
                                      HYD_REAL initialOutputFlow,
                                      HYD_TIME timestamp);
void HYD_PressureController_RequestTracking(HYD_PressureControllerState* state,
                                           HYD_REAL trackedOutputFlow);
void HYD_PressureController_Execute(const HYD_MotionSegment* segment,
                                    HYD_PressureControllerState* state,
                                    const HYD_PressureControllerInput* input,
                                    HYD_PressureControllerOutput* output);

#endif /* HYD_PRESSURE_CONTROLLER_H */
