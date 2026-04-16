#include "segment_completion.h"
#include "segment_limits.h"
#include <math.h>

static HDY_MotionDirection HDY_ResolvePositionDirection(const HDY_MotionSegment* segment,
                                                        const HDY_AxisRef* axisRef) {
    HDY_REAL delta;
    HDY_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HDY_DIRECTION_HOLD;
    }

    if (segment->direction == HDY_DIRECTION_EXTEND ||
        segment->direction == HDY_DIRECTION_RETRACT ||
        segment->direction == HDY_DIRECTION_HOLD) {
        return segment->direction;
    }

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

HDY_BOOL HDY_SegmentCompletion_CheckWithContext(const HDY_SegmentCompletionContext* context) {
    const HDY_MotionSegment* segment;
    const HDY_AxisRef* axisRef;
    const HDY_ExecutionReference* references;
    HDY_MotionDirection direction;
    HDY_REAL positionTolerance;
    HDY_REAL pressureTolerance;
    HDY_REAL flowTolerance;
    HDY_REAL pressureReference;
    HDY_REAL flowReference;
    HDY_REAL elapsedTime;

    if (context == NULL || context->segment == NULL || context->axisRef == NULL) {
        return false;
    }

    segment = context->segment;
    axisRef = context->axisRef;
    references = context->references;
    positionTolerance = HDY_Segment_GetPositionTolerance(segment);
    pressureTolerance = HDY_Segment_GetPressureTolerance(segment);
    flowTolerance = HDY_Segment_GetFlowTolerance(segment);
    pressureReference = (references != NULL && references->pressureReference > 0.0)
        ? references->pressureReference
        : segment->targetPressure;
    flowReference = (references != NULL && references->flowReference > 0.0)
        ? references->flowReference
        : segment->targetFlow;
    elapsedTime = (references != NULL) ? references->elapsedTime : 0.0;

    switch (segment->endCondition) {
        case HDY_END_POSITION:
            direction = HDY_ResolvePositionDirection(segment, axisRef);
            if (direction == HDY_DIRECTION_EXTEND) {
                return axisRef->position >= segment->targetPosition - positionTolerance;
            }
            if (direction == HDY_DIRECTION_RETRACT) {
                return axisRef->position <= segment->targetPosition + positionTolerance;
            }
            return fabs(axisRef->position - segment->targetPosition) <= positionTolerance;
        case HDY_END_TIME:
            return elapsedTime >= segment->duration;
        case HDY_END_PRESSURE:
            return fabs(axisRef->pressure - pressureReference) <= pressureTolerance;
        case HDY_END_FLOW:
            return fabs(fabs(axisRef->flow) - fabs(flowReference)) <= flowTolerance;
        case HDY_END_MANUAL:
            return false;
        default:
            return false;
    }
}

HDY_BOOL HDY_SegmentCompletion_Check(const HDY_MotionSegment* segment,
                                     const HDY_AxisRef* axisRef,
                                     HDY_REAL elapsed) {
    HDY_ExecutionReference references;
    HDY_SegmentCompletionContext context;

    references.elapsedTime = elapsed;
    references.pressureReference = 0.0;
    references.flowReference = 0.0;
    references.velocityReference = 0.0;
    context.segment = segment;
    context.axisRef = axisRef;
    context.references = &references;
    return HDY_SegmentCompletion_CheckWithContext(&context);
}
