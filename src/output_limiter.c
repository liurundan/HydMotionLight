#include "output_limiter.h"
#include <math.h>

#define HYD_DEFAULT_DERATE_RATIO 0.5

static HYD_BOOL HYD_OutputLimiter_IsFinite(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

static HYD_REAL HYD_OutputLimiter_AbsReal(HYD_REAL value) {
    return (value < 0.0) ? -value : value;
}

static HYD_REAL HYD_OutputLimiter_ResolveDerateRatio(HYD_REAL configuredRatio) {
    if (HYD_OutputLimiter_IsFinite(configuredRatio) &&
        configuredRatio > 0.0 &&
        configuredRatio < 1.0) {
        return configuredRatio;
    }

    return HYD_DEFAULT_DERATE_RATIO;
}

void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output) {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_REAL ratio;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->derated = false;

    if (input == NULL) {
        return;
    }

    if (input->protectionAction == HYD_PROTECTION_ACTION_STOP) {
        return;
    }

    if (!HYD_OutputLimiter_IsFinite(input->requestedFlow) ||
        !HYD_OutputLimiter_IsFinite(input->requestedPumpSpeed) ||
        !HYD_OutputLimiter_IsFinite(input->flowToPumpSpeedGain) ||
        !HYD_OutputLimiter_IsFinite(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    commandFlow = HYD_OutputLimiter_AbsReal(input->requestedFlow);
    pumpSpeed = HYD_OutputLimiter_AbsReal(input->requestedPumpSpeed);

    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    if (pumpSpeed > input->pumpSpeedLimit) {
        pumpSpeed = input->pumpSpeedLimit;
        commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}
