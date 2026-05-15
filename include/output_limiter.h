#ifndef HYD_OUTPUT_LIMITER_H
#define HYD_OUTPUT_LIMITER_H

#include "common_types.h"

typedef struct {
    HYD_REAL requestedFlow;
    HYD_REAL requestedPumpSpeed;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_ProtectionAction protectionAction;
    HYD_REAL derateRatio;
} HYD_OutputLimiterInput;

typedef struct {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_BOOL derated;
} HYD_OutputLimiterOutput;

void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output);

#endif /* HYD_OUTPUT_LIMITER_H */
