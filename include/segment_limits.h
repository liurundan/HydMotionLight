#ifndef HYD_SEGMENT_LIMITS_H
#define HYD_SEGMENT_LIMITS_H

#include "common_types.h"

HYD_REAL HYD_Segment_GetPositionTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetFlowTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetVelocityTolerance(const HYD_MotionSegment* segment);
HYD_TIME HYD_Segment_GetTimeoutLimit(const HYD_MotionSegment* segment);

/* Pressure-ceiling accessors (Sprint 1 low-pressure mold-protect primitive).
 *
 * The "active at position" helper consolidates the gating logic used by
 * every control-mode path so callers don't have to re-implement the
 * window + ceiling > 0 check. Zero pressureCeiling always returns false.
 * A degenerate window (end <= start) means always-on. */
HYD_REAL HYD_Segment_GetPressureCeiling(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingPositionStart(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingPositionEnd(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetDerateRatio(const HYD_MotionSegment* segment);
HYD_BOOL HYD_Segment_PressureCeilingActiveAt(const HYD_MotionSegment* segment,
                                              HYD_REAL actualPosition);

/**
 * @brief Resolve motion direction from segment configuration, axis position,
 *        and last-active-direction memory.
 *
 * If the segment declares an explicit direction (POSITIVE / NEGATIVE / HOLD),
 * that direction is returned directly. CURRENT inherits the last active motion
 * direction (defaulting to POSITIVE for stationary axes). SHORTEST_WAY is
 * inferred from the signed delta between targetPosition and the current axis
 * position, gated by the segment's position tolerance.
 *
 * This function consolidates the direction-resolution logic previously
 * duplicated in segment_completion and motion_planner.
 *
 * @param segment              Active segment (must not be NULL)
 * @param axisRef              Current axis feedback (must not be NULL)
 * @param lastActiveDirection  Previous non-HOLD motion direction (for CURRENT)
 * @return Resolved direction: POSITIVE, NEGATIVE, or HOLD
 */
HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef,
                                                   HYD_MotionDirection lastActiveDirection);

#endif /* HYD_SEGMENT_LIMITS_H */
