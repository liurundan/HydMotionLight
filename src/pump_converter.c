#include "pump_converter.h"
#include <math.h>

static HDY_BOOL HDY_PumpConverter_IsFiniteReal(HDY_REAL value) {
    return isfinite(value) ? true : false;
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

    if (!HDY_PumpConverter_IsFiniteReal(input->requestedFlow) ||
        !HDY_PumpConverter_IsFiniteReal(input->flowToPumpSpeedGain) ||
        !HDY_PumpConverter_IsFiniteReal(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
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

HDY_BOOL HDY_PumpConverter_ValidateConfig(HDY_REAL flowToPumpSpeedGain,
                                          HDY_REAL pumpSpeedLimit,
                                          HDY_DiagnosticCode* code) {
    if (!HDY_PumpConverter_IsFiniteReal(flowToPumpSpeedGain) || flowToPumpSpeedGain <= 0.0) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID;
        }
        return false;
    }

    if (!HDY_PumpConverter_IsFiniteReal(pumpSpeedLimit) || pumpSpeedLimit < 0.0) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID;
        }
        return false;
    }

    if (code != NULL) {
        *code = HDY_DIAG_CODE_NONE;
    }
    return true;
}
