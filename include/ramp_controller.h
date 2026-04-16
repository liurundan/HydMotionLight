#ifndef HDY_RAMP_CONTROLLER_H
#define HDY_RAMP_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HDY_REAL rampedPressure;
    HDY_TIME lastTimestamp;
} HDY_RampController;

typedef struct {
    HDY_REAL targetPressure;
    HDY_REAL rampRate;
    HDY_TIME currentTime;
} HDY_RampControllerInput;

typedef struct {
    HDY_REAL rampedPressure;
} HDY_RampControllerOutput;

void HDY_RampController_Init(HDY_RampController* controller, HDY_REAL initialPressure, HDY_TIME initialTime);
void HDY_RampController_Execute(HDY_RampController* controller, const HDY_RampControllerInput* input, HDY_RampControllerOutput* output);

#endif /* HDY_RAMP_CONTROLLER_H */
