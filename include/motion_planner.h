#ifndef HDY_MOTION_PLANNER_H
#define HDY_MOTION_PLANNER_H

#include "common_types.h"

typedef struct {
    const HDY_AxisRef* axisRef;
    const HDY_MotionSegment* segment;
    HDY_REAL elapsedTime;
    HDY_REAL rampedPressure;
} HDY_MotionPlannerInput;

typedef struct {
    HDY_MotionDirection direction;
    HDY_REAL targetVelocity; /* signed by direction */
    HDY_REAL targetFlow;     /* nonnegative magnitude for pump conversion */
} HDY_MotionPlannerOutput;

void HDY_MotionPlanner_Execute(const HDY_MotionPlannerInput* input, HDY_MotionPlannerOutput* output);

#endif /* HDY_MOTION_PLANNER_H */
