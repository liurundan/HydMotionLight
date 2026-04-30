#ifndef HYD_MOTION_PLANNER_H
#define HYD_MOTION_PLANNER_H

#include "common_types.h"

typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL rampedPressure;
} HYD_MotionPlannerInput;

typedef struct {
    HYD_MotionDirection direction;
    HYD_REAL targetVelocity; /* signed by direction */
    HYD_REAL targetFlow;     /* nonnegative magnitude for pump conversion */
} HYD_MotionPlannerOutput;

void HYD_MotionPlanner_Execute(const HYD_MotionPlannerInput* input, HYD_MotionPlannerOutput* output);

#endif /* HYD_MOTION_PLANNER_H */
