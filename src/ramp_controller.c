#include "ramp_controller.h"

void HDY_RampController_Init(HDY_RampController* controller, HDY_REAL initialPressure, HDY_TIME initialTime) {
    if (controller == NULL) {
        return;
    }
    controller->rampedPressure = initialPressure;
    controller->lastTimestamp = initialTime;
}

void HDY_RampController_Execute(HDY_RampController* controller, const HDY_RampControllerInput* input, HDY_RampControllerOutput* output) {
    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    HDY_TIME deltaTime = input->currentTime - controller->lastTimestamp;
    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    controller->lastTimestamp = input->currentTime;

    if (input->rampRate > 0.0) {
        HDY_REAL maxChange = input->rampRate * deltaTime;
        if (controller->rampedPressure < input->targetPressure) {
            controller->rampedPressure = HDY_ClampReal(controller->rampedPressure + maxChange,
                                                       controller->rampedPressure,
                                                       input->targetPressure);
        } else if (controller->rampedPressure > input->targetPressure) {
            controller->rampedPressure = HDY_ClampReal(controller->rampedPressure - maxChange,
                                                       input->targetPressure,
                                                       controller->rampedPressure);
        }
    } else {
        controller->rampedPressure = input->targetPressure;
    }

    output->rampedPressure = controller->rampedPressure;
}
