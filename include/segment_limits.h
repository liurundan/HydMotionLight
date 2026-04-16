#ifndef HDY_SEGMENT_LIMITS_H
#define HDY_SEGMENT_LIMITS_H

#include "common_types.h"

HDY_REAL HDY_Segment_GetPositionTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetPressureTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetFlowTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetVelocityTolerance(const HDY_MotionSegment* segment);
HDY_TIME HDY_Segment_GetTimeoutLimit(const HDY_MotionSegment* segment);

#endif /* HDY_SEGMENT_LIMITS_H */
