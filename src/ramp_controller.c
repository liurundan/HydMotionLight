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
    controller->profileRate = 0.0;
    controller->profileRateLimit = 0.0;
    controller->profileDecel = 0.0;
    controller->profileLatched = false;
}

void HYD_RampController_ExecuteWithDt(HYD_RampController* controller,
                                      const HYD_RampControllerInput* input,
                                      HYD_TIME deltaTime,
                                      HYD_RampControllerOutput* output) {
    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    /* The profiled reference was driving this controller, so restart the time-based ramp from here. */
    if (controller->profileLatched) {
        controller->profileLatched = false;
        controller->profileRate = 0.0;
        controller->transitionDuration = -1.0;
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
    output->rampedPressureRate = 0.0;
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

void HYD_RampController_ExecuteProfiledWithDt(HYD_RampController* controller,
                                              const HYD_RampControllerInput* input,
                                              const HYD_RampProfileLimits* limits,
                                              HYD_TIME deltaTime,
                                              HYD_RampControllerOutput* output) {
    HYD_TIME transitionTime;
    HYD_REAL remaining;
    HYD_REAL direction;
    HYD_REAL slopeTarget;
    HYD_REAL brakingSlope;
    HYD_REAL maxStep;

    if (controller == NULL || input == NULL || limits == NULL || output == NULL) {
        return;
    }

    if (!(deltaTime > 0.0)) { /* also catches NaN */
        deltaTime = 0.0;
    }
    controller->lastTimestamp = input->currentTime;

    /* Latch the slope bound and the deceleration when the target or the
     * transition time changes (including CONTINUOUSUPDATE live updates).
     * The deceleration depends only on the automatic bound, so braking does
     * not get weaker when the user asks for a slow transition. */
    transitionTime = HYD_PressureTimeMsToSeconds(input->rampRate);
    if (!controller->profileLatched ||
        input->targetPressure != controller->transitionTargetPressure ||
        transitionTime != controller->transitionDuration) {
        HYD_REAL distance = input->targetPressure - controller->rampedPressure;
        HYD_REAL slopeBound = (distance >= 0.0) ? limits->riseRate : limits->fallRate;

        controller->transitionTargetPressure = input->targetPressure;
        controller->transitionDuration = transitionTime;
        controller->profileRateLimit = slopeBound;
        if (transitionTime > 0.0 && fabs(distance) / transitionTime < slopeBound) {
            controller->profileRateLimit = fabs(distance) / transitionTime;
        }
        controller->profileDecel = (limits->brakeTime > 0.0)
            ? slopeBound / limits->brakeTime : 0.0;
        controller->profileLatched = true;
    }

    if (controller->profileRateLimit <= 0.0 || controller->profileDecel <= 0.0) {
        controller->rampedPressure = input->targetPressure;
        controller->profileRate = 0.0;
    } else {
        remaining = input->targetPressure - controller->rampedPressure;
        direction = (remaining >= 0.0) ? 1.0 : -1.0;
        brakingSlope = sqrt(2.0 * controller->profileDecel * fabs(remaining));
        slopeTarget = direction * ((brakingSlope < controller->profileRateLimit)
            ? brakingSlope : controller->profileRateLimit);

        /* Reversal: stop the reference first, never coast past the old
         * target while turning around. */
        if (slopeTarget * controller->profileRate < 0.0) {
            controller->profileRate = 0.0;
        }
        if (fabs(slopeTarget) > fabs(controller->profileRate)) {
            maxStep = controller->profileDecel * deltaTime;
            controller->profileRate += HYD_ClampReal(slopeTarget - controller->profileRate,
                                                     -maxStep, maxStep);
        } else {
            controller->profileRate = slopeTarget;
        }

        controller->rampedPressure += controller->profileRate * deltaTime;
        if ((input->targetPressure - controller->rampedPressure) * direction <= 0.0) {
            controller->rampedPressure = input->targetPressure;
            controller->profileRate = 0.0;
        }
    }

    output->rampedPressure = controller->rampedPressure;
    output->rampedPressureRate = controller->profileRate;
}
