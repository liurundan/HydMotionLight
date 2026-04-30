#include "segment_completion.h"
#include "segment_limits.h"
#include <math.h>

HYD_BOOL HYD_SegmentCompletion_CheckWithContext(const HYD_SegmentCompletionContext* context) {
    const HYD_MotionSegment* segment;
    const HYD_AxisRef* axisRef;
    const HYD_ExecutionReference* references;
    HYD_MotionDirection direction;
    HYD_REAL positionTolerance;
    HYD_REAL pressureTolerance;
    HYD_REAL flowTolerance;
    HYD_REAL pressureReference;
    HYD_REAL flowReference;
    HYD_REAL elapsedTime;

    if (context == NULL || context->segment == NULL || context->axisRef == NULL) {
        return false;
    }

    segment = context->segment;
    axisRef = context->axisRef;
    references = context->references;
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    pressureTolerance = HYD_Segment_GetPressureTolerance(segment);
    flowTolerance = HYD_Segment_GetFlowTolerance(segment);
    pressureReference = (references != NULL && references->pressureReference > 0.0)
        ? references->pressureReference
        : segment->targetPressure;
    flowReference = (references != NULL && references->flowReference > 0.0)
        ? references->flowReference
        : segment->targetFlow;
    elapsedTime = (references != NULL) ? references->elapsedTime : 0.0;

    switch (segment->endCondition) {
        case HYD_END_POSITION:
            direction = HYD_Segment_ResolveDirection(segment, axisRef);
            if (direction == HYD_DIRECTION_EXTEND) {
                return axisRef->position >= segment->targetPosition - positionTolerance;
            }
            if (direction == HYD_DIRECTION_RETRACT) {
                return axisRef->position <= segment->targetPosition + positionTolerance;
            }
            return fabs(axisRef->position - segment->targetPosition) <= positionTolerance;
        case HYD_END_TIME:
            return elapsedTime >= segment->duration;
        case HYD_END_PRESSURE:
            return fabs(axisRef->pressure - pressureReference) <= pressureTolerance;
        case HYD_END_FLOW:
            return fabs(fabs(axisRef->flow) - fabs(flowReference)) <= flowTolerance;
        case HYD_END_MANUAL:
            return false;
        default:
            return false;
    }
}

HYD_BOOL HYD_SegmentCompletion_Check(const HYD_MotionSegment* segment,
                                     const HYD_AxisRef* axisRef,
                                     HYD_REAL elapsed) {
    HYD_ExecutionReference references;
    HYD_SegmentCompletionContext context;

    references.elapsedTime = elapsed;
    references.pressureReference = 0.0;
    references.flowReference = 0.0;
    references.velocityReference = 0.0;
    context.segment = segment;
    context.axisRef = axisRef;
    context.references = &references;
    return HYD_SegmentCompletion_CheckWithContext(&context);
}
