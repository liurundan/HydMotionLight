#ifndef HYD_VELOCITY_CONTROLLER_H
#define HYD_VELOCITY_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HYD_REAL targetVelocity;
    HYD_REAL actualVelocity;
    HYD_REAL feedforwardFlow;
    HYD_REAL kp;
    HYD_REAL deadband;
    HYD_REAL correctionLimit;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
} HYD_VelocityControllerInput;

typedef struct {
    HYD_REAL velocityError;
    HYD_REAL correctionFlow;
    HYD_REAL correctedFlow;
    HYD_BOOL active;
    HYD_BOOL saturated;
} HYD_VelocityControllerOutput;

void HYD_VelocityController_Execute(const HYD_VelocityControllerInput* input,
                                    HYD_VelocityControllerOutput* output);

#endif /* HYD_VELOCITY_CONTROLLER_H */
