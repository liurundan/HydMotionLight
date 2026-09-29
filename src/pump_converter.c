#include "pump_converter.h"
#include <math.h>

static HYD_BOOL HYD_PumpConverter_IsFiniteReal(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

void HYD_PumpConverter_Execute(const HYD_PumpConverterInput* input,
                               HYD_PumpConverterOutput* output) {
    HYD_REAL requestedFlow;
    HYD_REAL maxFlowFromPumpLimit;
    HYD_REAL hardwareMinFlow;
    HYD_REAL effectiveMinFlow;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->maxFlow = 0.0;
    output->speedLimitActive = false;

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

    /* The converter is the final actuator boundary.  A strategy lower bound
     * may request less reverse flow, but it can never exceed the pump's safe
     * reverse-speed capability. */
    maxFlowFromPumpLimit = input->pumpSpeedLimit / input->flowToPumpSpeedGain;
    hardwareMinFlow = -maxFlowFromPumpLimit * HYD_PUMP_NEGATIVE_SPEED_RATIO;
    effectiveMinFlow = input->minimumFlow;
    if (!HYD_PumpConverter_IsFiniteReal(effectiveMinFlow)) {
        effectiveMinFlow = 0.0;
    }
    if (effectiveMinFlow < hardwareMinFlow) {
        effectiveMinFlow = hardwareMinFlow;
    }
    if (effectiveMinFlow > maxFlowFromPumpLimit) {
        effectiveMinFlow = maxFlowFromPumpLimit;
    }
    output->maxFlow = maxFlowFromPumpLimit;
    output->commandFlow = HYD_ClampReal(requestedFlow,
        effectiveMinFlow,
        maxFlowFromPumpLimit);

    output->pumpSpeed = output->commandFlow * input->flowToPumpSpeedGain;
    output->speedLimitActive = output->commandFlow != requestedFlow;

}

void HYD_PumpConverter_ApplySlewLimit(
    const HYD_PumpConverterInput* input,
    HYD_REAL previousPumpSpeed,
    HYD_TIME deltaTime,
    HYD_REAL accelerationRpmPerSecond,
    HYD_REAL decelerationRpmPerSecond,
    HYD_PumpConverterOutput* output) {
    HYD_PumpConverterOutput requested;
    HYD_REAL targetSpeed;
    HYD_REAL limitedSpeed;
    HYD_REAL rate;
    HYD_REAL maxDelta;

    if (output == NULL) {
        return;
    }

    HYD_PumpConverter_Execute(input, &requested);
    if (input == NULL ||
        !HYD_PumpConverter_IsFiniteReal(previousPumpSpeed) ||
        !HYD_PumpConverter_IsFiniteReal(deltaTime) ||
        !HYD_PumpConverter_IsFiniteReal(accelerationRpmPerSecond) ||
        !HYD_PumpConverter_IsFiniteReal(decelerationRpmPerSecond) ||
        deltaTime <= 0.0 ||
        accelerationRpmPerSecond <= 0.0 ||
        decelerationRpmPerSecond <= 0.0 ||
        !HYD_PumpConverter_IsFiniteReal(requested.pumpSpeed)) {
        output->commandFlow = 0.0;
        output->pumpSpeed = 0.0;
        output->maxFlow = requested.maxFlow;
        output->speedLimitActive = true;
        return;
    }

    targetSpeed = requested.pumpSpeed;
    limitedSpeed = previousPumpSpeed;
    if (limitedSpeed > input->pumpSpeedLimit) {
        limitedSpeed = input->pumpSpeedLimit;
    } else {
        HYD_REAL minimumFlow = input->minimumFlow;
        HYD_REAL minimumSpeed;
        HYD_REAL hardwareMinimumSpeed = -input->pumpSpeedLimit *
                                         HYD_PUMP_NEGATIVE_SPEED_RATIO;
        if (!HYD_PumpConverter_IsFiniteReal(minimumFlow)) {
            minimumFlow = 0.0;
        }
        minimumSpeed = minimumFlow * input->flowToPumpSpeedGain;
        if (minimumSpeed < hardwareMinimumSpeed) {
            minimumSpeed = hardwareMinimumSpeed;
        }
        if (minimumSpeed > input->pumpSpeedLimit) {
            minimumSpeed = input->pumpSpeedLimit;
        }
        if (limitedSpeed < minimumSpeed) {
            limitedSpeed = minimumSpeed;
        }
    }

    /* A reversal must first decelerate to zero. This prevents a one-scan
     * sign change from commanding an instantaneous pump direction reversal. */
    if ((limitedSpeed > 0.0 && targetSpeed < 0.0) ||
        (limitedSpeed < 0.0 && targetSpeed > 0.0)) {
        rate = decelerationRpmPerSecond;
        maxDelta = rate * deltaTime;
        if (fabs(limitedSpeed) <= maxDelta) {
            limitedSpeed = 0.0;
        } else {
            limitedSpeed += (limitedSpeed > 0.0) ? -maxDelta : maxDelta;
        }
    } else {
        rate = (fabs(targetSpeed) > fabs(limitedSpeed))
            ? accelerationRpmPerSecond
            : decelerationRpmPerSecond;
        maxDelta = rate * deltaTime;
        if (targetSpeed > limitedSpeed + maxDelta) {
            limitedSpeed += maxDelta;
        } else if (targetSpeed < limitedSpeed - maxDelta) {
            limitedSpeed -= maxDelta;
        } else {
            limitedSpeed = targetSpeed;
        }
    }

    {
        HYD_REAL minimumFlow = input->minimumFlow;
        HYD_REAL minimumSpeed;
        HYD_REAL hardwareMinimumSpeed = -input->pumpSpeedLimit *
                                         HYD_PUMP_NEGATIVE_SPEED_RATIO;
        if (!HYD_PumpConverter_IsFiniteReal(minimumFlow)) {
            minimumFlow = 0.0;
        }
        minimumSpeed = minimumFlow * input->flowToPumpSpeedGain;
        if (minimumSpeed < hardwareMinimumSpeed) {
            minimumSpeed = hardwareMinimumSpeed;
        }
        if (minimumSpeed > input->pumpSpeedLimit) {
            minimumSpeed = input->pumpSpeedLimit;
        }
        limitedSpeed = HYD_ClampReal(limitedSpeed, minimumSpeed,
                                     input->pumpSpeedLimit);
    }
    output->maxFlow = requested.maxFlow;
    output->pumpSpeed = limitedSpeed;
    output->commandFlow = limitedSpeed / input->flowToPumpSpeedGain;
    output->speedLimitActive = requested.speedLimitActive ||
                                limitedSpeed != targetSpeed;
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
