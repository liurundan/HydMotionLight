#ifndef HYD_ACTUATION_MAPPER_H
#define HYD_ACTUATION_MAPPER_H

#include "toggle_kinematics.h"

typedef struct {
    HYD_MechanismType mechanismType;
    const HYD_TogglePreparedConfig *toggleConfig;
    HYD_REAL templatePosition;
    HYD_REAL templateVelocity;
    const HYD_CylinderConfig *cylinderConfig;
    HYD_REAL fallbackCylinderVelocityToFlowGain;
    HYD_REAL maxFlow;
} HYD_ActuationMapperInput;

typedef struct {
    HYD_REAL actuatorPosition;
    HYD_REAL velocityRatio;
    HYD_REAL actuatorVelocity;
    HYD_REAL effectiveCylinderGain;
    HYD_REAL unlimitedRequestedFlow;
    HYD_REAL requestedFlow;
    HYD_REAL maxTemplateVelocity;
    HYD_MotionDirection actuatorDirection;
    HYD_BOOL flowLimitActive;
} HYD_ActuationMapperOutput;

HYD_BOOL HYD_ActuationMapper_MapVelocity(
    const HYD_ActuationMapperInput *input,
    HYD_ActuationMapperOutput *output,
    HYD_ToggleError *error);
HYD_BOOL HYD_ActuationMapper_FlowToTemplateVelocity(
    const HYD_ActuationMapperInput *input,
    /* Magnitude is actuator flow; sign is the caller's template direction. */
    HYD_REAL actuatorFlow,
    HYD_REAL *templateVelocity,
    HYD_ToggleError *error);
/* Highest template velocity that keeps actuator flow <= input->maxFlow over
 * the whole template range [xFrom, xTo] (order free, clipped to the online
 * envelope). direction is the template direction (EXTEND/RETRACT); any other
 * value returns the slower of both directions. Uses cylinderConfig /
 * fallback gain like MapVelocity; templatePosition/templateVelocity unused. */
HYD_BOOL HYD_ActuationMapper_MaxTemplateVelocityOverRange(
    const HYD_ActuationMapperInput *input,
    HYD_REAL xFrom,
    HYD_REAL xTo,
    HYD_MotionDirection direction,
    HYD_REAL *maxTemplateVelocity,
    HYD_REAL *velocityRatioMax,
    HYD_ToggleError *error);

#endif
