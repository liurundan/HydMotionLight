#include "segment_limits.h"
#include <math.h>

static HDY_REAL HDY_GetTypedTolerance(HDY_REAL typedTolerance, HDY_REAL legacyTolerance) {
    if (typedTolerance > 0.0) {
        return typedTolerance;
    }
    if (legacyTolerance > 0.0) {
        return legacyTolerance;
    }
    return 0.0;
}

HDY_REAL HDY_Segment_GetPositionTolerance(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HDY_GetTypedTolerance(segment->positionTolerance, segment->tolerance);
}

HDY_REAL HDY_Segment_GetPressureTolerance(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HDY_GetTypedTolerance(segment->pressureTolerance, segment->tolerance);
}

HDY_REAL HDY_Segment_GetFlowTolerance(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HDY_GetTypedTolerance(segment->flowTolerance, segment->tolerance);
}

HDY_REAL HDY_Segment_GetVelocityTolerance(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return HDY_GetTypedTolerance(segment->velocityTolerance, segment->tolerance);
}

HDY_TIME HDY_Segment_GetTimeoutLimit(const HDY_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }

    if (segment->timeoutLimit > 0.0) {
        return segment->timeoutLimit;
    }

    if (segment->endCondition == HDY_END_TIME && segment->duration > 0.0) {
        return segment->duration * 1.5;
    }

    return 0.0;
}

HDY_MotionDirection HDY_Segment_ResolveDirection(const HDY_MotionSegment* segment,
                                                   const HDY_AxisRef* axisRef) {
    HDY_REAL delta;
    HDY_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HDY_DIRECTION_HOLD;
    }

    /* Explicit direction declarations take precedence. */
    if (segment->direction == HDY_DIRECTION_EXTEND ||
        segment->direction == HDY_DIRECTION_RETRACT ||
        segment->direction == HDY_DIRECTION_HOLD) {
        return segment->direction;
    }

    /* Infer direction from position delta when no explicit direction is set. */
    positionTolerance = HDY_Segment_GetPositionTolerance(segment);
    delta = segment->targetPosition - axisRef->position;
    if (delta > positionTolerance) {
        return HDY_DIRECTION_EXTEND;
    }
    if (delta < -positionTolerance) {
        return HDY_DIRECTION_RETRACT;
    }
    return HDY_DIRECTION_HOLD;
}
