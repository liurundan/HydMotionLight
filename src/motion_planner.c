#include "motion_planner.h"
#include "segment_limits.h"
#include <math.h>

static HDY_REAL HDY_MinReal(HDY_REAL left, HDY_REAL right) {
    return (left < right) ? left : right;
}

static HDY_REAL HDY_GetDirectionSign(HDY_MotionDirection direction) {
    switch (direction) {
        case HDY_DIRECTION_EXTEND:
            return 1.0;
        case HDY_DIRECTION_RETRACT:
            return -1.0;
        default:
            return 0.0;
    }
}

static HDY_BOOL HDY_IsMotionDirectionExplicit(HDY_MotionDirection direction) {
    return (direction == HDY_DIRECTION_EXTEND) ||
           (direction == HDY_DIRECTION_RETRACT) ||
           (direction == HDY_DIRECTION_HOLD);
}

static HDY_MotionDirection HDY_ResolveMotionDirection(const HDY_MotionSegment* segment,
                                                      const HDY_AxisRef* axisRef) {
    HDY_REAL delta;
    HDY_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HDY_DIRECTION_HOLD;
    }

    if (HDY_IsMotionDirectionExplicit(segment->direction)) {
        return segment->direction;
    }

    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
        return HDY_DIRECTION_HOLD;
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

static HDY_REAL HDY_ComputeRemainingDistance(const HDY_MotionSegment* segment,
                                             const HDY_AxisRef* axisRef,
                                             HDY_MotionDirection direction) {
    HDY_REAL remainingDistance;

    if (segment == NULL || axisRef == NULL) {
        return 0.0;
    }

    switch (direction) {
        case HDY_DIRECTION_EXTEND:
            remainingDistance = segment->targetPosition - axisRef->position;
            break;
        case HDY_DIRECTION_RETRACT:
            remainingDistance = axisRef->position - segment->targetPosition;
            break;
        default:
            remainingDistance = 0.0;
            break;
    }

    if (remainingDistance <= 0.0) {
        return 0.0;
    }
    return remainingDistance;
}

static HDY_REAL HDY_ComputePositionBasedVelocityMagnitude(HDY_REAL remainingDistance,
                                                          HDY_REAL acceleration,
                                                          HDY_REAL maxVelocity) {
    HDY_REAL velocityMagnitude;

    if (remainingDistance <= 0.0 || acceleration <= 0.0 || maxVelocity <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = sqrt(2.0 * acceleration * remainingDistance);
    return HDY_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

static HDY_REAL HDY_ComputeTimeBasedVelocityMagnitude(HDY_REAL elapsedTime,
                                                      HDY_REAL acceleration,
                                                      HDY_REAL maxVelocity) {
    HDY_REAL velocityMagnitude;

    if (elapsedTime <= 0.0 || acceleration <= 0.0 || maxVelocity <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = acceleration * elapsedTime;
    return HDY_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

static HDY_REAL HDY_ConvertVelocityToFlowMagnitude(HDY_REAL velocityMagnitude,
                                                   const HDY_MotionSegment* segment) {
    HDY_REAL gain;
    HDY_REAL flowMagnitude;

    if (segment == NULL || velocityMagnitude <= 0.0) {
        return 0.0;
    }

    gain = segment->velocityToFlowGain;
    if (gain <= 0.0) {
        gain = 1.0;
    }

    flowMagnitude = velocityMagnitude * gain;
    return HDY_ClampReal(flowMagnitude, 0.0, segment->maxFlow);
}

static HDY_REAL HDY_ApplyModeFlowCap(const HDY_MotionSegment* segment,
                                     HDY_REAL flowMagnitude) {
    HDY_REAL flowLimit;

    if (segment == NULL) {
        return 0.0;
    }

    flowLimit = segment->maxFlow;
    if ((segment->mode == HDY_MODE_POSITION || segment->mode == HDY_MODE_SPEED_RAMP) &&
        (segment->targetFlow > 0.0) &&
        (segment->targetFlow < flowLimit)) {
        flowLimit = segment->targetFlow;
    }

    return HDY_ClampReal(flowMagnitude, 0.0, flowLimit);
}

static HDY_REAL HDY_ComputePositionModeVelocityMagnitude(const HDY_MotionPlannerInput* input,
                                                         HDY_MotionDirection direction) {
    HDY_REAL remainingDistance;
    HDY_REAL brakeVelocityMagnitude;
    HDY_REAL rampVelocityMagnitude;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    remainingDistance = HDY_ComputeRemainingDistance(input->segment, input->axisRef, direction);
    brakeVelocityMagnitude = HDY_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       input->segment->maxAcceleration,
                                                                       input->segment->maxVelocity);

    if (input->segment->planner == HDY_PLANNER_POSITION_BASED) {
        return brakeVelocityMagnitude;
    }

    rampVelocityMagnitude = HDY_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                                  input->segment->maxAcceleration,
                                                                  input->segment->maxVelocity);
    return HDY_MinReal(rampVelocityMagnitude, brakeVelocityMagnitude);
}

static HDY_REAL HDY_ComputeSpeedRampVelocityMagnitude(const HDY_MotionPlannerInput* input,
                                                      HDY_MotionDirection direction) {
    HDY_REAL velocityMagnitude;
    HDY_REAL remainingDistance;
    HDY_REAL brakeVelocityMagnitude;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    velocityMagnitude = HDY_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                              input->segment->maxAcceleration,
                                                              input->segment->maxVelocity);

    if (input->segment->endCondition != HDY_END_POSITION) {
        return velocityMagnitude;
    }

    remainingDistance = HDY_ComputeRemainingDistance(input->segment,
                                                     input->axisRef,
                                                     direction);
    brakeVelocityMagnitude = HDY_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       input->segment->maxAcceleration,
                                                                       input->segment->maxVelocity);
    return HDY_MinReal(velocityMagnitude, brakeVelocityMagnitude);
}

void HDY_MotionPlanner_Execute(const HDY_MotionPlannerInput* input, HDY_MotionPlannerOutput* output) {
    HDY_MotionDirection direction;
    HDY_REAL directionSign;
    HDY_REAL velocityMagnitude;
    HDY_REAL flowMagnitude;

    if (output == NULL) {
        return;
    }

    output->direction = HDY_DIRECTION_HOLD;
    output->targetVelocity = 0.0;
    output->targetFlow = 0.0;

    if (input == NULL || input->axisRef == NULL || input->segment == NULL) {
        return;
    }

    direction = HDY_ResolveMotionDirection(input->segment, input->axisRef);
    output->direction = direction;

    if (input->segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP ||
        direction == HDY_DIRECTION_HOLD) {
        return;
    }

    if (input->segment->mode == HDY_MODE_POSITION) {
        velocityMagnitude = HDY_ComputePositionModeVelocityMagnitude(input, direction);
    } else {
        velocityMagnitude = HDY_ComputeSpeedRampVelocityMagnitude(input, direction);
    }

    flowMagnitude = HDY_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
    flowMagnitude = HDY_ApplyModeFlowCap(input->segment, flowMagnitude);
    directionSign = HDY_GetDirectionSign(direction);

    output->targetVelocity = velocityMagnitude * directionSign;
    output->targetFlow = flowMagnitude;
}
