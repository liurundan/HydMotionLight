#ifndef HDY_MOTION_PLANNER_H
#define HDY_MOTION_PLANNER_H

#include "common_types.h"

typedef struct {
    const HDY_AxisRef* axisRef;
    const HDY_MotionSegment* segment;
    HDY_REAL elapsedTime;
    HDY_REAL flowToPumpSpeedGain;
    HDY_REAL pumpSpeedLimit;
    HDY_REAL rampedPressure;
} HDY_MotionPlannerInput;

typedef struct {
    HDY_REAL targetVelocity;
    HDY_REAL targetFlow;
    HDY_REAL pumpSpeed;
} HDY_MotionPlannerOutput;

void HDY_MotionPlanner_Execute(const HDY_MotionPlannerInput* input, HDY_MotionPlannerOutput* output);

#endif /* HDY_MOTION_PLANNER_H */