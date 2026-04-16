#include "motion_control.h"
#include "motion_planner.h"
#include "ramp_controller.h"
#include <math.h>
#include <string.h>

static HDY_REAL HDY_ClampReal(HDY_REAL value, HDY_REAL minimum, HDY_REAL maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static HDY_BOOL HDY_CheckSegmentComplete(const HDY_MotionSegment* segment, const HDY_AxisRef* axisRef, HDY_REAL elapsed) {
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

void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    memset(fb, 0, sizeof(*fb));
    fb->ENO = true;
    fb->_segmentChangedFlag = false;
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
}

void HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb, const HDY_MotionSegment* recipe, size_t recipeSize) {
    if (fb == NULL || recipe == NULL) {
        return;
    }
    if (recipeSize > HDY_MAX_SEGMENTS) {
        recipeSize = HDY_MAX_SEGMENTS;
    }
    memcpy(fb->RECIPE, recipe, recipeSize * sizeof(HDY_MotionSegment));
    fb->RECIPE_SIZE = recipeSize;
    fb->STATE.currentSegmentIndex = 0;
    fb->STATE.finished = (recipeSize == 0);
    fb->ACTIVE = (recipeSize > 0);
    if (fb->ACTIVE) {
        strncpy(fb->CURRENT_SEGMENT_NAME, fb->RECIPE[0].name, HDY_NAME_MAX - 1);
        fb->CURRENT_SEGMENT_NAME[HDY_NAME_MAX - 1] = '\0';
    }
}

void HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb, size_t segmentIndex, HDY_TIME timestamp) {
    if (fb == NULL || segmentIndex >= fb->RECIPE_SIZE) {
        return;
    }
    fb->STATE.currentSegmentIndex = segmentIndex;
    fb->STATE.active = true;
    fb->STATE.finished = false;
    fb->ACTIVE = true;
    fb->FINISHED = false;
    fb->STATE.currentSegmentIndex = segmentIndex;
    fb->STATE.commandedPumpSpeed = 0.0;
    fb->_segmentStartTime = timestamp;
    HDY_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp); /* Start ramp from current pressure */
    strncpy(fb->CURRENT_SEGMENT_NAME, fb->RECIPE[segmentIndex].name, HDY_NAME_MAX - 1);
    fb->CURRENT_SEGMENT_NAME[HDY_NAME_MAX - 1] = '\0';
    memset(&fb->DIAGNOSTIC, 0, sizeof(fb->DIAGNOSTIC));
    fb->_segmentChangedFlag = true;
}

void HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }
    if (fb->STATE.currentSegmentIndex + 1 < fb->RECIPE_SIZE) {
        HDY_MotionControlFB_StartSegment(fb, fb->STATE.currentSegmentIndex + 1, timestamp);
    } else {
        fb->ACTIVE = false;
        fb->STATE.active = false;
        fb->STATE.finished = true;
        fb->FINISHED = true;
        fb->CURRENT_SEGMENT_NAME[0] = '\0';
    }
}

void HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    fb->ACTIVE = false;
    fb->STATE.active = false;
    fb->STATE.finished = true;
    fb->FINISHED = true;
    fb->STATE.commandedPumpSpeed = 0.0;
    strncpy(fb->DIAGNOSTIC.message, "Aborted by caller", HDY_MESSAGE_MAX - 1);
    fb->DIAGNOSTIC.message[HDY_MESSAGE_MAX - 1] = '\0';
}

void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    if (!fb->EN) {
        fb->ENO = false;
        return;
    }

    fb->ENO = true;
    if (fb->RESET) {
        HDY_MotionControlFB_Init(fb);
        return;
    }

    if (fb->START_SEGMENT) {
        if (fb->START_SEGMENT_INDEX < fb->RECIPE_SIZE) {
            HDY_MotionControlFB_StartSegment(fb, fb->START_SEGMENT_INDEX, fb->AXIS_REF.timestamp);
        }
        fb->START_SEGMENT = false;
    }

    if (!fb->ACTIVE || fb->STATE.finished || fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) {
        fb->PUMP_SPEED = 0.0;
        fb->SEGMENT_COMPLETED = fb->STATE.finished;
        fb->ACTIVE = false;
        return;
    }

    const HDY_MotionSegment* segment = &fb->RECIPE[fb->STATE.currentSegmentIndex];
    HDY_REAL elapsed = fb->AXIS_REF.timestamp - fb->_segmentStartTime;
    if (elapsed < 0.0) {
        elapsed = 0.0;
    }

    /* Ramp pressure to prevent sudden changes */
    HDY_RampControllerInput rampInput;
    HDY_RampControllerOutput rampOutput;

    rampInput.targetPressure = segment->targetPressure;
    rampInput.rampRate = segment->pressureRampRate;
    rampInput.currentTime = fb->AXIS_REF.timestamp;

    HDY_RampController_Execute(&fb->_rampController, &rampInput, &rampOutput);

    HDY_MotionPlannerInput plannerInput;
    HDY_MotionPlannerOutput plannerOutput;

    plannerInput.axisRef = &fb->AXIS_REF;
    plannerInput.segment = segment;
    plannerInput.elapsedTime = elapsed;
    plannerInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    plannerInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    plannerInput.rampedPressure = rampOutput.rampedPressure;

    HDY_MotionPlanner_Execute(&plannerInput, &plannerOutput);

    fb->PUMP_SPEED = plannerOutput.pumpSpeed;
    fb->STATE.plannedVelocity = plannerOutput.targetVelocity;
    fb->STATE.plannedFlow = plannerOutput.targetFlow;
    fb->STATE.commandedPumpSpeed = fb->PUMP_SPEED;
    fb->STATE.active = true;
    fb->ACTIVE = true;

    fb->STATE.finished = HDY_CheckSegmentComplete(segment, &fb->AXIS_REF, elapsed);
    fb->SEGMENT_COMPLETED = fb->STATE.finished;
    fb->FINISHED = fb->STATE.finished;

    fb->DIAGNOSTIC.pressureError = segment->targetPressure - fb->AXIS_REF.pressure;
    fb->DIAGNOSTIC.flowError = segment->targetFlow - fb->AXIS_REF.flow;
    fb->DIAGNOSTIC.velocityError = plannerOutput.targetVelocity - fb->AXIS_REF.velocity;
    fb->DIAGNOSTIC.overPressure = fb->AXIS_REF.pressure > segment->targetPressure + segment->tolerance;
    fb->DIAGNOSTIC.underPressure = fb->AXIS_REF.pressure < segment->targetPressure - segment->tolerance;
    fb->DIAGNOSTIC.flowDeviation = fabs(fb->DIAGNOSTIC.flowError) > HDY_ClampReal(0.1, 0.0, segment->tolerance);
    fb->DIAGNOSTIC.positionDeviation = (segment->endCondition == HDY_END_POSITION) &&
        (fabs(segment->targetPosition - fb->AXIS_REF.position) > segment->tolerance);

    if ((segment->endCondition == HDY_END_TIME) && (elapsed > segment->duration * 1.5)) {
        fb->DIAGNOSTIC.timeout = true;
        strncpy(fb->DIAGNOSTIC.message, "Segment exceeded expected duration", HDY_MESSAGE_MAX - 1);
        fb->DIAGNOSTIC.message[HDY_MESSAGE_MAX - 1] = '\0';
    }
    fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
    fb->_segmentChangedFlag = false;
}