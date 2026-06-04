#include "segment_limits.h"
#include <math.h>

static HYD_REAL HYD_GetTypedTolerance(HYD_REAL typedTolerance, HYD_REAL legacyTolerance) {
    if (typedTolerance > 0.0) {
        return typedTolerance;
    }
    if (legacyTolerance > 0.0) {
        return legacyTolerance;
    }
    return 0.0;
}

HYD_REAL HYD_Segment_GetPositionTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HYD_GetTypedTolerance(segment->positionTolerance, segment->tolerance);
}

HYD_REAL HYD_Segment_GetPressureTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HYD_GetTypedTolerance(segment->pressureTolerance, segment->tolerance);
}

HYD_REAL HYD_Segment_GetFlowTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HYD_GetTypedTolerance(segment->flowTolerance, segment->tolerance);
}

HYD_REAL HYD_Segment_GetVelocityTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HYD_GetTypedTolerance(segment->velocityTolerance, segment->tolerance);
}

HYD_TIME HYD_Segment_GetTimeoutLimit(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }

    if (segment->timeoutLimit > 0.0) {
        return segment->timeoutLimit;
    }

    if (segment->endCondition == HYD_END_TIME && segment->duration > 0.0) {
        return segment->duration * 1.5;
    }

    return 0.0;
}

HYD_MotionDirection HYD_Segment_ResolveDirection(const HYD_MotionSegment* segment,
                                                   const HYD_AxisRef* axisRef,
                                                   HYD_MotionDirection lastActiveDirection) {
    HYD_REAL delta;
    HYD_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HYD_DIRECTION_HOLD;
    }

    /* Explicit direction declarations take precedence. */
    if (segment->direction == HYD_DIRECTION_POSITIVE ||
        segment->direction == HYD_DIRECTION_NEGATIVE ||
        segment->direction == HYD_DIRECTION_HOLD) {
        return segment->direction;
    }

    /* CURRENT: inherit last active motion direction, default POSITIVE for stationary */
    if (segment->direction == HYD_DIRECTION_CURRENT) {
        if (lastActiveDirection == HYD_DIRECTION_POSITIVE ||
            lastActiveDirection == HYD_DIRECTION_NEGATIVE) {
            return lastActiveDirection;
        }
        return HYD_DIRECTION_POSITIVE;  /* stationary axis defaults to positive */
    }

    /* SHORTEST_WAY: infer direction from position delta */
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    delta = segment->targetPosition - axisRef->position;
    if (delta > positionTolerance) {
        return HYD_DIRECTION_POSITIVE;
    }
    if (delta < -positionTolerance) {
        return HYD_DIRECTION_NEGATIVE;
    }
    return HYD_DIRECTION_HOLD;
}

HYD_REAL HYD_Segment_GetPressureCeiling(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->pressureCeiling > 0.0) ? segment->pressureCeiling : 0.0;
}

HYD_REAL HYD_Segment_GetPressureCeilingTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    if (segment->pressureCeilingTolerance > 0.0) {
        return segment->pressureCeilingTolerance;
    }
    /* Fall back to generic pressureTolerance if dedicated value not configured. */
    return HYD_Segment_GetPressureTolerance(segment);
}

HYD_REAL HYD_Segment_GetPressureCeilingPositionStart(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return segment->pressureCeilingPositionStart;
}

HYD_REAL HYD_Segment_GetPressureCeilingPositionEnd(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return segment->pressureCeilingPositionEnd;
}

HYD_REAL HYD_Segment_GetDerateRatio(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    if (segment->derateRatio > 0.0 && segment->derateRatio < 1.0) {
        return segment->derateRatio;
    }
    return 0.0;  /* 0 signals "use library default" to HYD_OutputLimiter */
}

HYD_BOOL HYD_Segment_PressureCeilingActiveAt(const HYD_MotionSegment* segment,
                                              HYD_REAL actualPosition) {
    HYD_REAL start;
    HYD_REAL end;

    if (segment == NULL || HYD_Segment_GetPressureCeiling(segment) <= 0.0) {
        return false;
    }

    start = segment->pressureCeilingPositionStart;
    end = segment->pressureCeilingPositionEnd;

    /* Always-on when window is degenerate (end <= start). */
    if (end <= start) {
        return true;
    }

    return (actualPosition >= start) && (actualPosition <= end);
}
