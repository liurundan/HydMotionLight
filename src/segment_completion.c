#include "segment_completion.h"
#include <math.h>

HDY_BOOL HDY_SegmentCompletion_Check(const HDY_MotionSegment* segment,
                                     const HDY_AxisRef* axisRef,
                                     HDY_REAL elapsed) {
    if (segment == NULL || axisRef == NULL) {
        return false;
    }

    switch (segment->endCondition) {
        case HDY_END_POSITION:
            return axisRef->position >= segment->targetPosition - segment->tolerance;
        case HDY_END_TIME:
            return elapsed >= segment->duration;
        case HDY_END_PRESSURE:
            return fabs(axisRef->pressure - segment->targetPressure) <= segment->tolerance;
        case HDY_END_FLOW:
            return fabs(axisRef->flow - segment->targetFlow) <= segment->tolerance;
        case HDY_END_MANUAL:
            return false;
        default:
            return false;
    }
}
