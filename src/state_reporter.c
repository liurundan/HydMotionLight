#include "state_reporter.h"
#include "diagnostics.h"
#include "protection_manager.h"
#include <string.h>

static HDY_BOOL HDY_StateReporter_HasSelectedStartSource(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (fb->USE_RECIPE) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

static HDY_ControllerStatus HDY_StateReporter_ResolveIdleStatus(const HDY_MotionControlFB* fb,
                                                                HDY_BOOL finished,
                                                                HDY_BOOL segmentCompleted) {
    if (finished) {
        return HDY_STATUS_FINISHED;
    }

    if (segmentCompleted) {
        return HDY_STATUS_SEGMENT_COMPLETE;
    }

    if (HDY_StateReporter_HasSelectedStartSource(fb)) {
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

    if (HDY_StateReporter_HasSelectedStartSource(fb)) {
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

static HDY_BOOL HDY_StateReporter_ResolveBusy(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    switch (fb->FB_STATE) {
        case HDY_FB_STATE_STARTING:
        case HDY_FB_STATE_RUNNING:
        case HDY_FB_STATE_SEGMENT_COMPLETE:
        case HDY_FB_STATE_HOLD:
            return true;
        case HDY_FB_STATE_DISABLED:
        case HDY_FB_STATE_IDLE:
        case HDY_FB_STATE_READY:
        case HDY_FB_STATE_DONE:
        case HDY_FB_STATE_ABORTED:
        case HDY_FB_STATE_FAULT:
        default:
            return false;
    }
}

static HDY_BOOL HDY_StateReporter_ResolveDone(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    return fb->FB_STATE == HDY_FB_STATE_DONE;
}

static HDY_BOOL HDY_StateReporter_ResolveError(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    return fb->FAULT || fb->FB_STATE == HDY_FB_STATE_FAULT ||
        fb->DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT;
}

static HDY_DiagnosticCode HDY_StateReporter_ResolveErrorId(const HDY_MotionControlFB* fb) {
    if (!HDY_StateReporter_ResolveError(fb)) {
        return HDY_DIAG_CODE_NONE;
    }

    if (fb->DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
        return fb->DIAGNOSTIC.code;
    }

    if (fb->LAST_FAULT_SNAPSHOT.valid &&
        fb->LAST_FAULT_SNAPSHOT.diagnostic.code != HDY_DIAG_CODE_NONE) {
        return fb->LAST_FAULT_SNAPSHOT.diagnostic.code;
    }

    if (fb->DIAGNOSTIC_LATCH.severity == HDY_DIAG_SEVERITY_FAULT &&
        fb->DIAGNOSTIC_LATCH.code != HDY_DIAG_CODE_NONE) {
        return fb->DIAGNOSTIC_LATCH.code;
    }

    return HDY_DIAG_CODE_NONE;
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
    HDY_StateReporter_RefreshStandardOutputs(fb);
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
    HDY_StateReporter_RefreshStandardOutputs(fb);
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

void HDY_StateReporter_SetSegmentSource(HDY_MotionControlFB* fb, HDY_SegmentSource source) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.segmentSource = source;
}

void HDY_StateReporter_RefreshStandardOutputs(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->BUSY = HDY_StateReporter_ResolveBusy(fb);
    fb->DONE = HDY_StateReporter_ResolveDone(fb);
    fb->ERROR = HDY_StateReporter_ResolveError(fb);
    fb->ERROR_ID = HDY_StateReporter_ResolveErrorId(fb);
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

void HDY_StateReporter_SetHoldState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_NONE);
    fb->SEGMENT_COMPLETED = false;
    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_HOLD);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_HOLD);
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

void HDY_StateReporter_ReportDiagnostic(HDY_MotionControlFB* fb,
                                        HDY_DiagnosticCode code,
                                        HDY_DiagnosticSeverity severity,
                                        const char* message,
                                        HDY_TIME eventTimestamp,
                                        const HDY_MotionSegment* segment,
                                        const HDY_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_SetEvent(&fb->DIAGNOSTIC, code, severity, message);
    HDY_StateReporter_SetProtectionAction(fb, fb->DIAGNOSTIC.protectionAction);
    HDY_StateReporter_RefreshStandardOutputs(fb);
    HDY_StateReporter_RecordDiagnosticEvent(fb, eventTimestamp, segment, references);
}

void HDY_StateReporter_ReportFault(HDY_MotionControlFB* fb,
                                   HDY_DiagnosticCode code,
                                   const char* message,
                                   HDY_TIME eventTimestamp,
                                   const HDY_MotionSegment* segment,
                                   const HDY_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_EnterFaultStop(fb);
    HDY_StateReporter_ReportDiagnostic(fb, code, HDY_DIAG_SEVERITY_FAULT, message, eventTimestamp, segment, references);
}

void HDY_StateReporter_ClearCurrentDiagnostic(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    fb->_lastRecordedDiagnosticCode = HDY_DIAG_CODE_NONE;
    fb->_lastRecordedDiagnosticSeverity = HDY_DIAG_SEVERITY_NONE;
    fb->_lastRecordedDiagnosticFlags = HDY_DIAG_FLAG_NONE;
    fb->_lastRecordedProtectionAction = HDY_PROTECTION_ACTION_NONE;
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_NONE);
    HDY_StateReporter_RefreshStandardOutputs(fb);
}

void HDY_StateReporter_ClearDiagnosticRetentionOnly(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC_LATCH);
    HDY_Diagnostics_ClearSnapshot(&fb->LAST_DIAGNOSTIC_SNAPSHOT);
    HDY_Diagnostics_ClearSnapshot(&fb->LAST_FAULT_SNAPSHOT);
    HDY_DiagnosticsHistory_Clear(&fb->DIAGNOSTIC_HISTORY);
}

void HDY_StateReporter_ResetDiagnosticRetention(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    HDY_StateReporter_ClearDiagnosticRetentionOnly(fb);
}

void HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(HDY_MotionControlFB* fb) {
    if (fb == NULL || fb->FAULT || fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        return;
    }

    HDY_StateReporter_ClearCurrentDiagnostic(fb);
}

static void HDY_StateReporter_UpdateRecordedDiagnosticSignature(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_lastRecordedDiagnosticCode = fb->DIAGNOSTIC.code;
    fb->_lastRecordedDiagnosticSeverity = fb->DIAGNOSTIC.severity;
    fb->_lastRecordedDiagnosticFlags = fb->DIAGNOSTIC.flags;
    fb->_lastRecordedProtectionAction = fb->DIAGNOSTIC.protectionAction;
}

static HDY_BOOL HDY_StateReporter_ShouldRecordDiagnosticEvent(const HDY_MotionControlFB* fb) {
    if (fb == NULL || fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        return false;
    }

    return (fb->DIAGNOSTIC.code != fb->_lastRecordedDiagnosticCode) ||
        (fb->DIAGNOSTIC.severity != fb->_lastRecordedDiagnosticSeverity) ||
        (fb->DIAGNOSTIC.flags != fb->_lastRecordedDiagnosticFlags) ||
        (fb->DIAGNOSTIC.protectionAction != fb->_lastRecordedProtectionAction);
}

static HDY_UINT8 HDY_StateReporter_ResolveDiagnosticSegmentIndex(const HDY_MotionControlFB* fb,
                                                                  const HDY_MotionSegment* segment) {
    if (fb == NULL || segment == NULL || fb->STATE.currentSegmentIndex >= HDY_MAX_SEGMENTS) {
        return (HDY_UINT8)HDY_MAX_SEGMENTS;
    }

    return (HDY_UINT8)fb->STATE.currentSegmentIndex;
}

static const char* HDY_StateReporter_ResolveDiagnosticSegmentName(const HDY_MotionControlFB* fb,
                                                                   const HDY_MotionSegment* segment) {
    if (segment != NULL && segment->name[0] != '\0') {
        return segment->name;
    }

    if (fb != NULL && fb->STATE.currentSegmentName[0] != '\0') {
        return fb->STATE.currentSegmentName;
    }

    return NULL;
}

void HDY_StateReporter_RecordDiagnosticEvent(HDY_MotionControlFB* fb,
                                              HDY_TIME eventTimestamp,
                                              const HDY_MotionSegment* segment,
                                              const HDY_ExecutionReference* references) {
    HDY_DiagnosticSnapshot snapshot;

    if (fb == NULL) {
        return;
    }

    if (fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        HDY_StateReporter_UpdateRecordedDiagnosticSignature(fb);
        return;
    }

    if (!HDY_StateReporter_ShouldRecordDiagnosticEvent(fb)) {
        HDY_StateReporter_UpdateRecordedDiagnosticSignature(fb);
        return;
    }

    HDY_Diagnostics_CaptureSnapshot(&snapshot,
                                    &fb->DIAGNOSTIC,
                                    &fb->AXIS_REF,
                                    references,
                                    eventTimestamp,
                                    HDY_StateReporter_ResolveDiagnosticSegmentIndex(fb, segment),
                                    HDY_StateReporter_ResolveDiagnosticSegmentName(fb, segment),
                                    fb->STATUS,
                                    fb->ACTIVE,
                                    fb->FINISHED,
                                    fb->FAULT);
    fb->DIAGNOSTIC_LATCH = fb->DIAGNOSTIC;
    fb->LAST_DIAGNOSTIC_SNAPSHOT = snapshot;
    if (fb->FAULT) {
        fb->LAST_FAULT_SNAPSHOT = snapshot;
    }
    HDY_DiagnosticsHistory_Push(&fb->DIAGNOSTIC_HISTORY, &snapshot);
    HDY_StateReporter_UpdateRecordedDiagnosticSignature(fb);
}
