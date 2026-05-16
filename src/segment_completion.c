#include "segment_completion.h"
#include "segment_limits.h"
#include <math.h>

#define HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE 1.0

static HYD_REAL HYD_SegmentCompletion_ResolvePositionSettledVelocityTolerance(
    const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE;
    }
    if (segment->stableVelocityLimit > 0.0) {
        return segment->stableVelocityLimit;
    }
    if (segment->velocityTolerance > 0.0) {
        return segment->velocityTolerance;
    }
    return HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE;
}

static HYD_BOOL HYD_SegmentCompletion_IsPositionReached(
    const HYD_MotionSegment* segment,
    const HYD_AxisRef* axisRef,
    HYD_REAL positionTolerance) {
    HYD_MotionDirection direction;

    direction = HYD_Segment_ResolveDirection(segment, axisRef);
    if (direction == HYD_DIRECTION_EXTEND) {
        return axisRef->position >= segment->targetPosition - positionTolerance;
    }
    if (direction == HYD_DIRECTION_RETRACT) {
        return axisRef->position <= segment->targetPosition + positionTolerance;
    }
    return fabs(axisRef->position - segment->targetPosition) <= positionTolerance;
}

static HYD_BOOL HYD_SegmentCompletion_IsPositionVelocitySettled(
    const HYD_MotionSegment* segment,
    const HYD_AxisRef* axisRef,
    const HYD_ExecutionReference* references) {
    HYD_REAL settledVelocityTolerance;
    HYD_REAL velocityReference;

    settledVelocityTolerance =
        HYD_SegmentCompletion_ResolvePositionSettledVelocityTolerance(segment);
    velocityReference = (references != NULL) ? references->velocityReference : 0.0;

    return fabs(velocityReference) <= settledVelocityTolerance &&
           fabs(axisRef->velocity) <= settledVelocityTolerance;
}

static HYD_BOOL HYD_SegmentCompletion_ApplyStableWindow(
    const HYD_SegmentCompletionContext* context,
    HYD_BOOL rawComplete) {
    HYD_TIME elapsedStable;

    if (context == NULL || context->segment == NULL) {
        return false;
    }

    if (!rawComplete) {
        if (context->candidateActive != NULL) {
            *context->candidateActive = false;
        }
        return false;
    }

    if (context->segment->stableVelocityLimit > 0.0 &&
        context->axisRef != NULL &&
        fabs(context->axisRef->velocity) > context->segment->stableVelocityLimit) {
        if (context->candidateActive != NULL) {
            *context->candidateActive = false;
        }
        return false;
    }

    if (context->segment->stableWindow <= 0.0 ||
        context->candidateStartTime == NULL ||
        context->candidateActive == NULL) {
        return true;
    }

    if (!*context->candidateActive) {
        *context->candidateActive = true;
        *context->candidateStartTime = context->timestamp;
        return false;
    }

    elapsedStable = context->timestamp - *context->candidateStartTime;
    return elapsedStable >= context->segment->stableWindow;
}

HYD_BOOL HYD_SegmentCompletion_CheckWithContext(const HYD_SegmentCompletionContext* context) {
    const HYD_MotionSegment* segment;
    const HYD_AxisRef* axisRef;
    const HYD_ExecutionReference* references;
    HYD_REAL positionTolerance;
    HYD_REAL pressureTolerance;
    HYD_REAL flowTolerance;
    HYD_REAL pressureReference;
    HYD_REAL flowReference;
    HYD_REAL elapsedTime;
    HYD_BOOL rawComplete;

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
            rawComplete =
                HYD_SegmentCompletion_IsPositionReached(segment, axisRef, positionTolerance) &&
                HYD_SegmentCompletion_IsPositionVelocitySettled(segment, axisRef, references);
            break;
        case HYD_END_TIME:
            rawComplete = elapsedTime >= segment->duration;
            break;
        case HYD_END_PRESSURE:
            rawComplete = fabs(axisRef->pressure - pressureReference) <= pressureTolerance;
            break;
        case HYD_END_FLOW:
            rawComplete = fabs(fabs(axisRef->flow) - fabs(flowReference)) <= flowTolerance;
            break;
        case HYD_END_MANUAL:
            rawComplete = false;
            break;
        default:
            rawComplete = false;
            break;
    }

    return HYD_SegmentCompletion_ApplyStableWindow(context, rawComplete);
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
    context.timestamp = (axisRef != NULL) ? axisRef->timestamp : 0.0;
    context.candidateStartTime = NULL;
    context.candidateActive = NULL;
    return HYD_SegmentCompletion_CheckWithContext(&context);
}
