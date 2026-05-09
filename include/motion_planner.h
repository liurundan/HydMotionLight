#ifndef HYD_MOTION_PLANNER_H
#define HYD_MOTION_PLANNER_H

#include "common_types.h"

typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL rampedPressure;
    HYD_REAL decelElapsed;
    HYD_REAL decelStartVel;
} HYD_MotionPlannerInput;

typedef struct {
    HYD_MotionDirection direction;
    HYD_REAL targetVelocity; /* signed by direction */
    HYD_REAL targetFlow;     /* nonnegative magnitude for pump conversion */
} HYD_MotionPlannerOutput;

typedef struct {
    HYD_REAL tAcc;
    HYD_REAL tConst;
    HYD_REAL tDec;
    HYD_REAL sAcc;
    HYD_REAL sConst;
    HYD_REAL sDec;
    HYD_REAL vPeak;
} HYD_TrapezoidProfile;

HYD_BOOL HYD_PlanTrapezoid(HYD_TrapezoidProfile* profile,
                           HYD_REAL distance,
                           HYD_REAL vMax,
                           HYD_REAL acc);

HYD_REAL HYD_EvalTrapezoid(const HYD_TrapezoidProfile* profile,
                          HYD_REAL elapsed,
                          HYD_REAL acc,
                          HYD_REAL vMax);

void HYD_MotionPlanner_Execute(const HYD_MotionPlannerInput* input, HYD_MotionPlannerOutput* output);

#endif /* HYD_MOTION_PLANNER_H */
