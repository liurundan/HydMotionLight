#include "motion_planner.h"
#include "segment_limits.h"
#include <math.h>
#include <string.h>

static HYD_REAL HYD_MinReal(HYD_REAL left, HYD_REAL right) {
    return (left < right) ? left : right;
}

static HYD_REAL HYD_ApplyVelocityRateLimit(HYD_REAL previousVelocity,
                                           HYD_REAL desiredVelocity,
                                           HYD_REAL acceleration,
                                           HYD_REAL deceleration,
                                           HYD_REAL deltaTime) {
    HYD_REAL delta;
    HYD_REAL limit;

    if (deltaTime <= 0.0) {
        return previousVelocity;
    }

    delta = desiredVelocity - previousVelocity;
    if (delta >= 0.0) {
        limit = acceleration * deltaTime;
    } else {
        limit = deceleration * deltaTime;
    }

    if (limit <= 0.0) {
        return desiredVelocity;
    }

    if (delta > limit) {
        return previousVelocity + limit;
    }
    if (delta < -limit) {
        return previousVelocity - limit;
    }
    return desiredVelocity;
}

static HYD_REAL HYD_GetDirectionSign(HYD_MotionDirection direction) {
    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            return 1.0;
        case HYD_DIRECTION_RETRACT:
            return -1.0;
        default:
            return 0.0;
    }
}

static HYD_REAL HYD_ComputeRemainingDistance(const HYD_MotionSegment* segment,
                                             const HYD_AxisRef* axisRef,
                                             HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;

    if (segment == NULL || axisRef == NULL) {
        return 0.0;
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            remainingDistance = segment->targetPosition - axisRef->position;
            break;
        case HYD_DIRECTION_RETRACT:
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

static HYD_REAL HYD_ComputePositionBasedVelocityMagnitude(HYD_REAL remainingDistance,
                                                          HYD_REAL acceleration,
                                                          HYD_REAL maxVelocity) {
    HYD_REAL velocityMagnitude;

    if (remainingDistance <= 0.0 || acceleration <= 0.0 || maxVelocity <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = sqrt(2.0 * acceleration * remainingDistance);
    return HYD_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

static HYD_REAL HYD_ComputeTimeBasedVelocityMagnitude(HYD_REAL elapsedTime,
                                                      HYD_REAL acceleration,
                                                      HYD_REAL maxVelocity) {
    HYD_REAL velocityMagnitude;

    if (elapsedTime <= 0.0 || acceleration <= 0.0 || maxVelocity <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = acceleration * elapsedTime;
    return HYD_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

HYD_BOOL HYD_PlanTrapezoid(HYD_TrapezoidProfile* profile,
                           HYD_REAL distance,
                           HYD_REAL vMax,
                           HYD_REAL acc) {
    HYD_REAL sBrake;

    if (profile == NULL || distance <= 0.0 || vMax <= 0.0 || acc <= 0.0) {
        if (profile != NULL) {
            memset(profile, 0, sizeof(*profile));
        }
        return false;
    }

    sBrake = (vMax * vMax) / (2.0 * acc);

    if (2.0 * sBrake >= distance) {
        HYD_REAL vPeak = sqrt(acc * distance);
        profile->tAcc   = vPeak / acc;
        profile->tConst = 0.0;
        profile->tDec   = profile->tAcc;
        profile->sAcc   = distance * 0.5;
        profile->sConst = 0.0;
        profile->sDec   = distance * 0.5;
        profile->vPeak  = vPeak;
    } else {
        profile->tAcc   = vMax / acc;
        profile->sAcc   = sBrake;
        profile->sDec   = sBrake;
        profile->sConst = distance - 2.0 * sBrake;
        profile->tConst = profile->sConst / vMax;
        profile->tDec   = profile->tAcc;
        profile->vPeak  = vMax;
    }
    return true;
}

HYD_REAL HYD_EvalTrapezoid(const HYD_TrapezoidProfile* profile,
                          HYD_REAL elapsed,
                          HYD_REAL acc,
                          HYD_REAL vMax) {
    HYD_REAL tDec;
    (void)vMax;

    if (profile == NULL || elapsed <= 0.0) {
        return 0.0;
    }

    if (elapsed < profile->tAcc) {
        return acc * elapsed;
    }

    if (elapsed < profile->tAcc + profile->tConst) {
        return profile->vPeak;
    }

    tDec = elapsed - profile->tAcc - profile->tConst;
    if (tDec >= profile->tDec) {
        return 0.0;
    }

    return HYD_ClampReal(profile->vPeak - acc * tDec, 0.0, profile->vPeak);
}

static HYD_REAL HYD_ConvertVelocityToFlowMagnitude(HYD_REAL velocityMagnitude,
                                                   const HYD_MotionSegment* segment) {
    HYD_REAL gain;
    HYD_REAL flowMagnitude;

    if (segment == NULL || velocityMagnitude <= 0.0) {
        return 0.0;
    }

    gain = segment->velocityToFlowGain;
    if (gain <= 0.0) {
        gain = 1.0;
    }

    flowMagnitude = velocityMagnitude * gain;
    return HYD_ClampReal(flowMagnitude, 0.0, segment->maxFlow);
}

static HYD_REAL HYD_ApplyModeFlowCap(const HYD_MotionSegment* segment,
                                     HYD_REAL flowMagnitude) {
    HYD_REAL flowLimit;

    if (segment == NULL) {
        return 0.0;
    }

    flowLimit = segment->maxFlow;
    if ((segment->mode == HYD_MODE_POSITION || segment->mode == HYD_MODE_SPEED_RAMP) &&
        (segment->targetFlow > 0.0) &&
        (segment->targetFlow < flowLimit)) {
        flowLimit = segment->targetFlow;
    }

    return HYD_ClampReal(flowMagnitude, 0.0, flowLimit);
}

static HYD_REAL HYD_ComputePositionModeVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                         HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;
    HYD_REAL brakeVelocityMagnitude;
    HYD_REAL rampVelocityMagnitude;
    HYD_REAL brakingAcceleration;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    remainingDistance = HYD_ComputeRemainingDistance(input->segment, input->axisRef, direction);
    brakingAcceleration = (input->segment->maxDeceleration > 0.0)
        ? input->segment->maxDeceleration
        : input->segment->maxAcceleration;
    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       brakingAcceleration,
                                                                       input->segment->maxVelocity);

    if (input->segment->planner == HYD_PLANNER_POSITION_BASED) {
        return brakeVelocityMagnitude;
    }

    rampVelocityMagnitude = HYD_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                                  input->segment->maxAcceleration,
                                                                  input->segment->maxVelocity);
    return HYD_MinReal(rampVelocityMagnitude, brakeVelocityMagnitude);
}

static HYD_REAL HYD_ComputeSpeedRampVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                      HYD_MotionDirection direction) {
    HYD_REAL velocityMagnitude;
    HYD_REAL remainingDistance;
    HYD_REAL brakeVelocityMagnitude;
    HYD_REAL decelVelocity;
    HYD_REAL brakingAcceleration;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    velocityMagnitude = HYD_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                              input->segment->maxAcceleration,
                                                              input->segment->maxVelocity);

    brakingAcceleration = (input->segment->maxDeceleration > 0.0)
        ? input->segment->maxDeceleration
        : input->segment->maxAcceleration;
    if (input->decelElapsed > 0.0 && input->decelStartVel > 0.0) {
        decelVelocity = input->decelStartVel -
            brakingAcceleration * input->decelElapsed;
        if (decelVelocity < 0.0) {
            decelVelocity = 0.0;
        }
        velocityMagnitude = HYD_MinReal(velocityMagnitude, decelVelocity);
    }

    if (input->segment->endCondition != HYD_END_POSITION) {
        return velocityMagnitude;
    }

    remainingDistance = HYD_ComputeRemainingDistance(input->segment,
                                                     input->axisRef,
                                                     direction);
    brakingAcceleration = (input->segment->maxDeceleration > 0.0)
        ? input->segment->maxDeceleration
        : input->segment->maxAcceleration;
    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       brakingAcceleration,
                                                                       input->segment->maxVelocity);
    return HYD_MinReal(velocityMagnitude, brakeVelocityMagnitude);
}

void HYD_MotionPlanner_Execute(const HYD_MotionPlannerInput* input, HYD_MotionPlannerOutput* output) {
    HYD_MotionDirection direction;
    HYD_REAL directionSign;
    HYD_REAL velocityMagnitude;
    HYD_REAL flowMagnitude;

    if (output == NULL) {
        return;
    }

    output->direction = HYD_DIRECTION_HOLD;
    output->targetVelocity = 0.0;
    output->targetFlow = 0.0;

    if (input == NULL || input->axisRef == NULL || input->segment == NULL) {
        return;
    }

    direction = HYD_Segment_ResolveDirection(input->segment, input->axisRef);
    output->direction = direction;

    if (input->segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP ||
        direction == HYD_DIRECTION_HOLD) {
        return;
    }

    if (input->segment->mode == HYD_MODE_POSITION) {
        velocityMagnitude = HYD_ComputePositionModeVelocityMagnitude(input, direction);
    } else {
        velocityMagnitude = HYD_ComputeSpeedRampVelocityMagnitude(input, direction);
    }

    flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
    flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);

    if (input->state != NULL &&
        (input->segment->planner == HYD_PLANNER_TIME_BASED ||
         input->segment->mode == HYD_MODE_SPEED_RAMP)) {
        HYD_REAL previousMagnitude = fabs(input->state->lastTargetVelocity);
        HYD_REAL brakingAcceleration = (input->segment->maxDeceleration > 0.0)
            ? input->segment->maxDeceleration
            : input->segment->maxAcceleration;

        if (!input->state->initialized) {
            previousMagnitude = 0.0;
            input->state->initialized = true;
        }

        velocityMagnitude = HYD_ApplyVelocityRateLimit(previousMagnitude,
                                                      velocityMagnitude,
                                                      input->segment->maxAcceleration,
                                                      brakingAcceleration,
                                                      input->deltaTime);
        flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
        flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);
    }

    directionSign = HYD_GetDirectionSign(direction);

    output->targetVelocity = velocityMagnitude * directionSign;
    output->targetFlow = flowMagnitude;

    if (input->state != NULL) {
        input->state->lastTargetVelocity = output->targetVelocity;
        input->state->lastTargetFlow = output->targetFlow;
        input->state->lastTimestamp = input->axisRef->timestamp;
    }
}
