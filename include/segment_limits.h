#ifndef HDY_SEGMENT_LIMITS_H
#define HDY_SEGMENT_LIMITS_H

#include "common_types.h"

HDY_REAL HDY_Segment_GetPositionTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetPressureTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetFlowTolerance(const HDY_MotionSegment* segment);
HDY_REAL HDY_Segment_GetVelocityTolerance(const HDY_MotionSegment* segment);
HDY_TIME HDY_Segment_GetTimeoutLimit(const HDY_MotionSegment* segment);

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
HDY_MotionDirection HDY_Segment_ResolveDirection(const HDY_MotionSegment* segment,
                                                   const HDY_AxisRef* axisRef);

#endif /* HDY_SEGMENT_LIMITS_H */
