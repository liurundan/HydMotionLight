#include "ramp_controller.h"
#include "motion_time.h"

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime) {
    if (controller == NULL) {
        return;
    }
    controller->rampedPressure = initialPressure;
    controller->lastTimestamp = initialTime;
    controller->transitionTargetPressure = initialPressure;
    controller->transitionRate = 0.0;
    controller->transitionDuration = 0.0;
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

    {
        HYD_TIME transitionTime = HYD_PressureTimeMsToSeconds(input->rampRate);
        if (input->targetPressure != controller->transitionTargetPressure ||
            transitionTime != controller->transitionDuration) {
            controller->transitionTargetPressure = input->targetPressure;
            controller->transitionDuration = transitionTime;
            controller->transitionRate = (transitionTime > 0.0)
                ? (input->targetPressure - controller->rampedPressure) / transitionTime
                : 0.0;
        }

        if (fabs(input->targetPressure - controller->rampedPressure) <=
                fabs(controller->transitionRate * deltaTime) + 1e-6f ||
            transitionTime <= 0.0) {
            controller->rampedPressure = input->targetPressure;
        } else {
            controller->rampedPressure += controller->transitionRate * deltaTime;
        }
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
