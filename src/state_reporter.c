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

static HDY_FbState HDY_StateReporter_ResolveIdleFbState(const HDY_MotionControlFB* fb,
                                                        HDY_BOOL finished,
                                                        HDY_BOOL segmentCompleted) {
    if (finished) {
        return HDY_FB_STATE_DONE;
    }

    if (segmentCompleted) {
        return HDY_FB_STATE_SEGMENT_COMPLETE;
    }

    if (fb != NULL && fb->RECIPE_SIZE > 0U) {
        return HDY_FB_STATE_READY;
    }

    return HDY_FB_STATE_IDLE;
}

static HDY_ControllerStatus HDY_StateReporter_ResolveExecutionStatus(const HDY_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return HDY_STATUS_RUNNING;
    }

    if (diagnostic->protectionAction == HDY_PROTECTION_ACTION_DERATE) {
        return HDY_STATUS_DEGRADED;
    }

    return HDY_STATUS_RUNNING;
}

static void HDY_StateReporter_ClearPressureLoopState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->STATE.pressureLoop, 0, sizeof(fb->STATE.pressureLoop));
}

static void HDY_StateReporter_ClearExecutionReferences(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->STATE.references, 0, sizeof(fb->STATE.references));
    fb->STATE.pressureControllerApplied = HDY_PRESSURE_CONTROLLER_NONE;
    HDY_StateReporter_ClearPressureLoopState(fb);
}

static void HDY_StateReporter_SetExecutionReferences(HDY_MotionControlFB* fb,
                                                     const HDY_ExecutionReference* references,
                                                     HDY_PressureControllerType pressureControllerApplied) {
    if (fb == NULL) {
        return;
    }

    if (references == NULL) {
        HDY_StateReporter_ClearExecutionReferences(fb);
        return;
    }

    fb->STATE.references = *references;
    fb->STATE.pressureControllerApplied = pressureControllerApplied;
}

static void HDY_StateReporter_SetPressureLoopState(HDY_MotionControlFB* fb,
                                                   const HDY_PressureControllerOutput* pressureOutput) {
    if (fb == NULL) {
        return;
    }

    if (pressureOutput == NULL || pressureOutput->appliedStrategy == HDY_PRESSURE_CONTROLLER_NONE) {
        HDY_StateReporter_ClearPressureLoopState(fb);
        return;
    }

    fb->STATE.pressureLoop.targetPressure = pressureOutput->targetPressure;
    fb->STATE.pressureLoop.filteredPressure = pressureOutput->filteredPressure;
    fb->STATE.pressureLoop.filteredPressureRate = pressureOutput->filteredPressureRate;
    fb->STATE.pressureLoop.controlError = pressureOutput->controlError;
    fb->STATE.pressureLoop.feedforwardFlow = pressureOutput->feedforwardFlow;
    fb->STATE.pressureLoop.feedbackFlow = pressureOutput->feedbackFlow;
    fb->STATE.pressureLoop.outputFlow = pressureOutput->outputFlow;
    fb->STATE.pressureLoop.unsaturatedOutputFlow = pressureOutput->unsaturatedOutputFlow;
    fb->STATE.pressureLoop.samplingPeriod = pressureOutput->samplingPeriod;
    fb->STATE.pressureLoop.adaptiveKp = pressureOutput->adaptiveKp;
    fb->STATE.pressureLoop.adaptiveKi = pressureOutput->adaptiveKi;
    fb->STATE.pressureLoop.adaptiveKd = pressureOutput->adaptiveKd;
    fb->STATE.pressureLoop.adaptiveJacobian = pressureOutput->adaptiveJacobian;
    fb->STATE.pressureLoop.trackingApplied = pressureOutput->trackingApplied;
    fb->STATE.pressureLoop.saturated = pressureOutput->saturated;
    fb->STATE.pressureLoop.adaptiveActive = pressureOutput->adaptiveActive;
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

void HDY_StateReporter_SetFbState(HDY_MotionControlFB* fb, HDY_FbState state) {
    if (fb == NULL) {
        return;
    }

    fb->FB_STATE = state;
}

void HDY_StateReporter_SetProtectionAction(HDY_MotionControlFB* fb, HDY_ProtectionAction action) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.protectionAction = action;
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
    HDY_StateReporter_ClearExecutionReferences(fb);
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_NONE);
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetActive(fb, false);
}

void HDY_StateReporter_ClearSegmentName(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

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

    strncpy(fb->STATE.currentSegmentName, name, HDY_NAME_MAX - 1);
    fb->STATE.currentSegmentName[HDY_NAME_MAX - 1] = '\0';
}

void HDY_StateReporter_SetIdleState(HDY_MotionControlFB* fb,
                                    HDY_BOOL finished,
                                    HDY_BOOL segmentCompleted) {
    HDY_ControllerStatus status;
    HDY_FbState fbState;

    if (fb == NULL) {
        return;
    }

    status = HDY_StateReporter_ResolveIdleStatus(fb, finished, segmentCompleted);
    fbState = HDY_StateReporter_ResolveIdleFbState(fb, finished, segmentCompleted);
    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_SetFinished(fb, finished);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_NONE);
    fb->SEGMENT_COMPLETED = segmentCompleted;
    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_SetStatus(fb, status);
    HDY_StateReporter_SetFbState(fb, fbState);
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
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_STOP);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_FAULT);
}

void HDY_StateReporter_ReportExecution(HDY_MotionControlFB* fb,
                                       const HDY_MotionPlannerOutput* plannerOutput,
                                       const HDY_PumpConverterOutput* pumpOutput,
                                       const HDY_ExecutionReference* references,
                                       HDY_PressureControllerType pressureControllerApplied,
                                       const HDY_PressureControllerOutput* pressureOutput,
                                       const HDY_DiagnosticInfo* diagnostic) {
    if (fb == NULL || plannerOutput == NULL || pumpOutput == NULL) {
        return;
    }

    fb->PUMP_SPEED = pumpOutput->pumpSpeed;
    fb->STATE.plannedVelocity = plannerOutput->targetVelocity;
    fb->STATE.plannedFlow = pumpOutput->commandFlow;
    fb->STATE.commandedPumpSpeed = pumpOutput->pumpSpeed;
    HDY_StateReporter_SetExecutionReferences(fb, references, pressureControllerApplied);
    HDY_StateReporter_SetPressureLoopState(fb, pressureOutput);
    HDY_StateReporter_SetProtectionAction(fb,
                                          (diagnostic != NULL) ? diagnostic->protectionAction
                                                               : HDY_PROTECTION_ACTION_NONE);
    HDY_StateReporter_SetPlannedDirection(fb, plannerOutput->direction);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_StateReporter_ResolveExecutionStatus(diagnostic));
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_RUNNING);
}
