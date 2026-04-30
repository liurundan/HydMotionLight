#ifndef HYD_RAMP_CONTROLLER_H
#define HYD_RAMP_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HYD_REAL rampedPressure;
    HYD_TIME lastTimestamp;
} HYD_RampController;

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL rampRate;
    HYD_TIME currentTime;
} HYD_RampControllerInput;

typedef struct {
    HYD_REAL rampedPressure;
} HYD_RampControllerOutput;

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime);
void HYD_RampController_Execute(HYD_RampController* controller, const HYD_RampControllerInput* input, HYD_RampControllerOutput* output);

#endif /* HYD_RAMP_CONTROLLER_H */
