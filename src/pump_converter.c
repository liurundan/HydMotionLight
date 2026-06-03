#include "pump_converter.h"
#include <math.h>

static HYD_BOOL HYD_PumpConverter_IsFiniteReal(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

void HYD_PumpConverter_Execute(const HYD_PumpConverterInput* input,
                               HYD_PumpConverterOutput* output) {
    HYD_REAL requestedFlow;
    HYD_REAL maxFlowFromPumpLimit;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;

    if (input == NULL) {
        return;
    }

    (void)input->direction;

    if (!HYD_PumpConverter_IsFiniteReal(input->requestedFlow) ||
        !HYD_PumpConverter_IsFiniteReal(input->flowToPumpSpeedGain) ||
        !HYD_PumpConverter_IsFiniteReal(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    requestedFlow = input->requestedFlow;
    if (requestedFlow < 0.0) {
        requestedFlow = -requestedFlow;
    }

    maxFlowFromPumpLimit = input->pumpSpeedLimit / input->flowToPumpSpeedGain;
    output->commandFlow = HYD_ClampReal(requestedFlow, -5.0, maxFlowFromPumpLimit);
    output->pumpSpeed = output->commandFlow * input->flowToPumpSpeedGain;
}

HYD_BOOL HYD_PumpConverter_ValidateConfig(HYD_REAL flowToPumpSpeedGain,
                                          HYD_REAL pumpSpeedLimit,
                                          HYD_DiagnosticCode* code) {
    if (!HYD_PumpConverter_IsFiniteReal(flowToPumpSpeedGain) || flowToPumpSpeedGain <= 0.0) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID;
        }
        return false;
    }

    if (!HYD_PumpConverter_IsFiniteReal(pumpSpeedLimit) || pumpSpeedLimit < 0.0) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID;
        }
        return false;
    }

    if (code != NULL) {
        *code = HYD_DIAG_CODE_NONE;
    }
    return true;
}
