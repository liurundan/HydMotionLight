#ifndef HYD_SEGMENT_LIMITS_H
#define HYD_SEGMENT_LIMITS_H

#include "common_types.h"

HYD_REAL HYD_Segment_GetPositionTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetFlowTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetVelocityTolerance(const HYD_MotionSegment* segment);
HYD_TIME HYD_Segment_GetTimeoutLimit(const HYD_MotionSegment* segment);

/**
 * @brief Resolve motion direction from segment configuration and axis position.
 *
 * If the segment declares an explicit direction (EXTEND / RETRACT / HOLD),
 * that direction is returned directly. Otherwise the direction is inferred
 * from the signed delta between targetPosition and the current axis position,
 * gated by the segment's position tolerance.
 *
 * This function consolidates the direction-resolution logic previously
 * duplicated in segment_completion and motion_planner.
 *
 * @param segment  Active segment (must not be NULL)
 * @param axisRef  Current axis feedback (must not be NULL)
 * @return Resolved direction: EXTEND, RETRACT, or HOLD
 */
HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef);

#endif /* HYD_SEGMENT_LIMITS_H */
