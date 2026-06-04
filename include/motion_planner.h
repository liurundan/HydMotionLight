#ifndef HYD_MOTION_PLANNER_H
#define HYD_MOTION_PLANNER_H

#include "common_types.h"

typedef struct {
    HYD_BOOL initialized;
    HYD_REAL lastTargetVelocity;
    HYD_REAL lastTargetFlow;
    HYD_TIME lastTimestamp;
} HYD_MotionPlannerState;

typedef struct {
    HYD_BOOL active;
    HYD_BufferMode bufferMode;
    HYD_REAL blendVelocity;
    HYD_REAL switchPosition;
    HYD_REAL switchTolerance;
} HYD_MotionBlendContext;

typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL deltaTime;
    HYD_REAL rampedPressure;
    HYD_REAL decelElapsed;
    HYD_REAL decelStartVel;
    HYD_MotionPlannerState* state;
    const HYD_MotionBlendContext* blend;
    HYD_MotionDirection lastActiveDirection;  /* for CURRENT direction resolution */
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
