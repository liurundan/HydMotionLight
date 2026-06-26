#include "ramp_controller.h"

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime) {
    if (controller == NULL) {
        return;
    }
    controller->rampedPressure = initialPressure;
    controller->lastTimestamp = initialTime;
}

void HYD_RampController_ExecuteWithDt(HYD_RampController* controller,
                                      const HYD_RampControllerInput* input,
                                      HYD_TIME deltaTime,
                                      HYD_RampControllerOutput* output) {
    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    controller->lastTimestamp = input->currentTime;

    if (input->rampRate > 0.0) {
        HYD_REAL maxChange = input->rampRate * deltaTime;
        if (controller->rampedPressure < input->targetPressure) {
            controller->rampedPressure = HYD_ClampReal(controller->rampedPressure + maxChange,
                                                       controller->rampedPressure,
                                                       input->targetPressure);
        } else if (controller->rampedPressure > input->targetPressure) {
            controller->rampedPressure = HYD_ClampReal(controller->rampedPressure - maxChange,
                                                       input->targetPressure,
                                                       controller->rampedPressure);
        }
    } else {
        controller->rampedPressure = input->targetPressure;
    }

    output->rampedPressure = controller->rampedPressure;
}

void HYD_RampController_Execute(HYD_RampController* controller, const HYD_RampControllerInput* input, HYD_RampControllerOutput* output) {
    HYD_TIME deltaTime;

    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    deltaTime = input->currentTime - controller->lastTimestamp;
    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    HYD_RampController_ExecuteWithDt(controller, input, deltaTime, output);
}
