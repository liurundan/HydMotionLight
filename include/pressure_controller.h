#ifndef HDY_PRESSURE_CONTROLLER_H
#define HDY_PRESSURE_CONTROLLER_H

#include "common_types.h"
#include "rbf_pid.h"

typedef struct {
    HDY_REAL targetPressure;
    HDY_REAL measuredPressure;
    HDY_REAL feedforwardFlow;
    HDY_REAL outputMin;
    HDY_REAL outputMax;
    HDY_TIME timestamp;
} HDY_PressureControllerInput;

typedef struct {
    HDY_BOOL initialized;
    HDY_BOOL trackingRequested;
    HDY_BOOL rbfInitialized;
    HDY_REAL integralOutput;
    HDY_REAL previousError;
    HDY_REAL previousFilteredPressure;
    HDY_REAL previousFilteredPressureRate;
    HDY_REAL previousOutput;
    HDY_TIME previousTimestamp;
    HDY_PressureControllerType activeStrategy;
    RBF_PID_Handle rbfPid;
} HDY_PressureControllerState;

typedef struct {
    HDY_PressureControllerType appliedStrategy;
    HDY_REAL targetPressure;
    HDY_REAL filteredPressure;
    HDY_REAL filteredPressureRate;
    HDY_REAL controlError;
    HDY_REAL proportionalTerm;
    HDY_REAL integralTerm;
    HDY_REAL derivativeTerm;
    HDY_REAL trackingTerm;
    HDY_REAL feedforwardFlow;
    HDY_REAL feedbackFlow;
    HDY_REAL unsaturatedOutputFlow;
    HDY_REAL outputFlow;
    HDY_REAL samplingPeriod;
    HDY_REAL adaptiveKp;
    HDY_REAL adaptiveKi;
    HDY_REAL adaptiveKd;
    HDY_REAL adaptiveJacobian;
    HDY_BOOL trackingApplied;
    HDY_BOOL saturated;
    HDY_BOOL adaptiveActive;
} HDY_PressureControllerOutput;

void HDY_PressureController_ClearState(HDY_PressureControllerState* state);
void HDY_PressureController_InitState(HDY_PressureControllerState* state,
                                      HDY_REAL initialPressure,
                                      HDY_REAL initialOutputFlow,
                                      HDY_TIME timestamp);
void HDY_PressureController_RequestTracking(HDY_PressureControllerState* state,
                                           HDY_REAL trackedOutputFlow);
void HDY_PressureController_Execute(const HDY_MotionSegment* segment,
                                    HDY_PressureControllerState* state,
                                    const HDY_PressureControllerInput* input,
                                    HDY_PressureControllerOutput* output);

#endif /* HDY_PRESSURE_CONTROLLER_H */
