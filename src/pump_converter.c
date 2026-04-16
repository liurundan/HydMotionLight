#include "pump_converter.h"

static HDY_REAL HDY_ClampReal(HDY_REAL value, HDY_REAL minimum, HDY_REAL maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void HDY_PumpConverter_Execute(const HDY_PumpConverterInput* input,
                               HDY_PumpConverterOutput* output) {
    HDY_REAL requestedFlow;
    HDY_REAL maxFlowFromPumpLimit;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;

    if (input == NULL) {
        return;
    }

    (void)input->direction;

    if (input->flowToPumpSpeedGain <= 0.0 || input->pumpSpeedLimit < 0.0) {
        return;
    }

    requestedFlow = input->requestedFlow;
    if (requestedFlow < 0.0) {
        requestedFlow = -requestedFlow;
    }

    maxFlowFromPumpLimit = input->pumpSpeedLimit / input->flowToPumpSpeedGain;
    output->commandFlow = HDY_ClampReal(requestedFlow, 0.0, maxFlowFromPumpLimit);
    output->pumpSpeed = output->commandFlow * input->flowToPumpSpeedGain;
}
