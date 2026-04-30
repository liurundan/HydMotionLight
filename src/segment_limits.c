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
                                                   const HYD_AxisRef* axisRef) {
    HYD_REAL delta;
    HYD_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HYD_DIRECTION_HOLD;
    }

    /* Explicit direction declarations take precedence. */
    if (segment->direction == HYD_DIRECTION_EXTEND ||
        segment->direction == HYD_DIRECTION_RETRACT ||
        segment->direction == HYD_DIRECTION_HOLD) {
        return segment->direction;
    }

    /* Infer direction from position delta when no explicit direction is set. */
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    delta = segment->targetPosition - axisRef->position;
    if (delta > positionTolerance) {
        return HYD_DIRECTION_EXTEND;
    }
    if (delta < -positionTolerance) {
        return HYD_DIRECTION_RETRACT;
    }
    return HYD_DIRECTION_HOLD;
}
