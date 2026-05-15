#include "velocity_controller.h"
#include <math.h>

void HYD_VelocityController_Execute(const HYD_VelocityControllerInput* input,
                                    HYD_VelocityControllerOutput* output) {
    HYD_REAL error;
    HYD_REAL correctionLimit;
    HYD_REAL correction;
    HYD_REAL correctedFlow;

    if (output == NULL) {
        return;
    }

    output->velocityError = 0.0;
    output->correctionFlow = 0.0;
    output->correctedFlow = 0.0;
    output->active = false;
    output->saturated = false;

    if (input == NULL) {
        return;
    }

    output->correctedFlow = HYD_ClampReal(input->feedforwardFlow,
                                          input->outputMin,
                                          input->outputMax);

    if (input->kp <= 0.0) {
        return;
    }

    error = fabs(input->targetVelocity) - fabs(input->actualVelocity);
    output->velocityError = error;

    if (input->deadband > 0.0 && fabs(error) <= input->deadband) {
        return;
    }

    correctionLimit = (input->correctionLimit > 0.0) ? input->correctionLimit : input->outputMax;
    correction = HYD_ClampReal(input->kp * error, -correctionLimit, correctionLimit);
    correctedFlow = HYD_ClampReal(input->feedforwardFlow + correction,
                                  input->outputMin,
                                  input->outputMax);

    output->correctionFlow = correctedFlow - input->feedforwardFlow;
    output->correctedFlow = correctedFlow;
    output->active = fabs(output->correctionFlow) > 0.0;
    output->saturated = (correction != input->kp * error) ||
                        (correctedFlow != input->feedforwardFlow + correction);
}
