#include "motion_planner.h"
#include <math.h>

static HDY_REAL HDY_ClampReal(HDY_REAL value, HDY_REAL minimum, HDY_REAL maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static HDY_REAL HDY_ComputePositionBasedVelocity(HDY_REAL remainingDistance, HDY_REAL acceleration, HDY_REAL maxVelocity) {
    if (remainingDistance <= 0.0 || acceleration <= 0.0) {
        return 0.0;
    }
    HDY_REAL velocity = sqrt(2.0 * acceleration * remainingDistance);
    return HDY_ClampReal(velocity, 0.0, maxVelocity);
}

static HDY_REAL HDY_ComputeTimeBasedVelocity(HDY_REAL elapsedTime, HDY_REAL acceleration, HDY_REAL maxVelocity) {
    if (elapsedTime <= 0.0 || acceleration <= 0.0) {
        return 0.0;
    }
    HDY_REAL velocity = acceleration * elapsedTime;
    return HDY_ClampReal(velocity, 0.0, maxVelocity);
}

static HDY_REAL HDY_ComputePressureClosedLoopFlow(const HDY_MotionSegment* segment, const HDY_AxisRef* axisRef, HDY_REAL rampedPressure) {
    HDY_REAL error = rampedPressure - axisRef->pressure;
    HDY_REAL kP = 1.5;
    HDY_REAL commandedFlow = segment->targetFlow + kP * error;
    return HDY_ClampReal(commandedFlow, 0.0, segment->maxVelocity);
}

static HDY_REAL HDY_ConvertFlowToPumpSpeed(HDY_REAL flow, HDY_REAL gain, HDY_REAL limit) {
    HDY_REAL speed = flow * gain;
    return HDY_ClampReal(speed, 0.0, limit);
}

static HDY_REAL HDY_ConvertVelocityToFlow(HDY_REAL velocity, const HDY_MotionSegment* segment) {
    HDY_REAL gain = segment->velocityToFlowGain;
    if (gain <= 0.0) {
        gain = 1.0;
    }
    HDY_REAL flow = velocity * gain;
    return HDY_ClampReal(flow, 0.0, segment->maxVelocity * gain);
}

void HDY_MotionPlanner_Execute(const HDY_MotionPlannerInput* input, HDY_MotionPlannerOutput* output) {
    if (input == NULL || output == NULL || input->axisRef == NULL || input->segment == NULL) {
        return;
    }

    HDY_REAL targetVelocity;
    if (input->segment->planner == HDY_PLANNER_POSITION_BASED) {
        HDY_REAL remaining = input->segment->targetPosition - input->axisRef->position;
        if (remaining < 0.0) {
            remaining = 0.0;
        }
        targetVelocity = HDY_ComputePositionBasedVelocity(remaining, input->segment->maxAcceleration, input->segment->maxVelocity);
    } else {
        targetVelocity = HDY_ComputeTimeBasedVelocity(input->elapsedTime, input->segment->maxAcceleration, input->segment->maxVelocity);
    }

    HDY_REAL targetFlow;
    if (input->segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
        targetFlow = HDY_ComputePressureClosedLoopFlow(input->segment, input->axisRef, input->rampedPressure);
    } else {
        targetFlow = HDY_ConvertVelocityToFlow(targetVelocity, input->segment);
    }

    output->targetVelocity = targetVelocity;
    output->targetFlow = targetFlow;
    output->pumpSpeed = HDY_ConvertFlowToPumpSpeed(targetFlow, input->flowToPumpSpeedGain, input->pumpSpeedLimit);
}