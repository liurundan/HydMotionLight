#include "state_reporter.h"
#include <string.h>

static HDY_ControllerStatus HDY_StateReporter_ResolveIdleStatus(const HDY_MotionControlFB* fb,
                                                                HDY_BOOL finished,
                                                                HDY_BOOL segmentCompleted) {
    if (finished) {
        return HDY_STATUS_FINISHED;
    }

    if (segmentCompleted) {
        return HDY_STATUS_SEGMENT_COMPLETE;
    }

    if (fb != NULL && fb->RECIPE_SIZE > 0U) {
        return HDY_STATUS_READY;
    }

    return HDY_STATUS_IDLE;
}

void HDY_StateReporter_SetActive(HDY_MotionControlFB* fb, HDY_BOOL active) {
    if (fb == NULL) {
        return;
    }

    fb->ACTIVE = active;
    fb->STATE.active = active;
}

void HDY_StateReporter_SetFinished(HDY_MotionControlFB* fb, HDY_BOOL finished) {
    if (fb == NULL) {
        return;
    }

    fb->FINISHED = finished;
    fb->STATE.finished = finished;
}

void HDY_StateReporter_SetFault(HDY_MotionControlFB* fb, HDY_BOOL fault) {
    if (fb == NULL) {
        return;
    }

    fb->FAULT = fault;
    fb->STATE.faultActive = fault;
}

void HDY_StateReporter_SetStatus(HDY_MotionControlFB* fb, HDY_ControllerStatus status) {
    if (fb == NULL) {
        return;
    }

    fb->STATUS = status;
    fb->STATE.status = status;
}

void HDY_StateReporter_SetPlannedDirection(HDY_MotionControlFB* fb, HDY_MotionDirection direction) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.plannedDirection = direction;
}

void HDY_StateReporter_ResetTransitionFlags(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_segmentChangedFlag = false;
    fb->SEGMENT_CHANGED = false;
}

void HDY_StateReporter_ApplySafeOutputs(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->PUMP_SPEED = 0.0;
    fb->STATE.plannedVelocity = 0.0;
    fb->STATE.plannedFlow = 0.0;
    fb->STATE.commandedPumpSpeed = 0.0;
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetActive(fb, false);
}

void HDY_StateReporter_ClearSegmentName(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->CURRENT_SEGMENT_NAME[0] = '\0';
    fb->STATE.currentSegmentName[0] = '\0';
}

void HDY_StateReporter_SetSegmentName(HDY_MotionControlFB* fb, const char* name) {
    if (fb == NULL) {
        return;
    }

    if (name == NULL) {
        HDY_StateReporter_ClearSegmentName(fb);
        return;
    }

    strncpy(fb->CURRENT_SEGMENT_NAME, name, HDY_NAME_MAX - 1);
    fb->CURRENT_SEGMENT_NAME[HDY_NAME_MAX - 1] = '\0';
    strncpy(fb->STATE.currentSegmentName, name, HDY_NAME_MAX - 1);
    fb->STATE.currentSegmentName[HDY_NAME_MAX - 1] = '\0';
}

void HDY_StateReporter_SetIdleState(HDY_MotionControlFB* fb,
                                    HDY_BOOL finished,
                                    HDY_BOOL segmentCompleted) {
    HDY_ControllerStatus status;

    if (fb == NULL) {
        return;
    }

    status = HDY_StateReporter_ResolveIdleStatus(fb, finished, segmentCompleted);
    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_SetFinished(fb, finished);
    HDY_StateReporter_SetFault(fb, false);
    fb->SEGMENT_COMPLETED = segmentCompleted;
    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_SetStatus(fb, status);
}

void HDY_StateReporter_EnterFaultState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_SetFinished(fb, false);
    fb->SEGMENT_COMPLETED = false;
    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_SetFault(fb, true);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
}

void HDY_StateReporter_ReportExecution(HDY_MotionControlFB* fb,
                                       const HDY_MotionPlannerOutput* plannerOutput,
                                       const HDY_PumpConverterOutput* pumpOutput) {
    if (fb == NULL || plannerOutput == NULL || pumpOutput == NULL) {
        return;
    }

    fb->PUMP_SPEED = pumpOutput->pumpSpeed;
    fb->STATE.plannedVelocity = plannerOutput->targetVelocity;
    fb->STATE.plannedFlow = pumpOutput->commandFlow;
    fb->STATE.commandedPumpSpeed = pumpOutput->pumpSpeed;
    HDY_StateReporter_SetPlannedDirection(fb, plannerOutput->direction);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
}
