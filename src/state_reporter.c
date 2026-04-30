#include "state_reporter.h"
#include "diagnostics.h"
#include "protection_manager.h"
#include <string.h>

static HYD_BOOL HYD_StateReporter_HasSelectedStartSource(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (fb->USE_RECIPE) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

static HYD_ControllerStatus HYD_StateReporter_ResolveIdleStatus(const HYD_MotionControlFB* fb,
                                                                HYD_BOOL finished,
                                                                HYD_BOOL segmentCompleted) {
    if (finished) {
        return HYD_STATUS_FINISHED;
    }

    if (segmentCompleted) {
        return HYD_STATUS_SEGMENT_COMPLETE;
    }

    if (HYD_StateReporter_HasSelectedStartSource(fb)) {
        return HYD_STATUS_READY;
    }

    return HYD_STATUS_IDLE;
}

static HYD_FbState HYD_StateReporter_ResolveIdleFbState(const HYD_MotionControlFB* fb,
                                                        HYD_BOOL finished,
                                                        HYD_BOOL segmentCompleted) {
    if (finished) {
        return HYD_FB_STATE_DONE;
    }

    if (segmentCompleted) {
        return HYD_FB_STATE_SEGMENT_COMPLETE;
    }

    if (HYD_StateReporter_HasSelectedStartSource(fb)) {
        return HYD_FB_STATE_READY;
    }

    return HYD_FB_STATE_IDLE;
}

static HYD_ControllerStatus HYD_StateReporter_ResolveExecutionStatus(const HYD_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return HYD_STATUS_RUNNING;
    }

    if (diagnostic->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        return HYD_STATUS_DEGRADED;
    }

    return HYD_STATUS_RUNNING;
}

static HYD_BOOL HYD_StateReporter_ResolveBusy(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    switch (fb->FB_STATE) {
        case HYD_FB_STATE_STARTING:
        case HYD_FB_STATE_RUNNING:
        case HYD_FB_STATE_SEGMENT_COMPLETE:
        case HYD_FB_STATE_HOLD:
            return true;
        case HYD_FB_STATE_DISABLED:
        case HYD_FB_STATE_IDLE:
        case HYD_FB_STATE_READY:
        case HYD_FB_STATE_DONE:
        case HYD_FB_STATE_ABORTED:
        case HYD_FB_STATE_FAULT:
        default:
            return false;
    }
}

static HYD_BOOL HYD_StateReporter_ResolveDone(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    return fb->FB_STATE == HYD_FB_STATE_DONE;
}

static HYD_BOOL HYD_StateReporter_ResolveError(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    return fb->STATE.faultActive || fb->FB_STATE == HYD_FB_STATE_FAULT ||
        fb->DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT;
}

static HYD_DiagnosticCode HYD_StateReporter_ResolveErrorId(const HYD_MotionControlFB* fb) {
    if (!HYD_StateReporter_ResolveError(fb)) {
        return HYD_DIAG_CODE_NONE;
    }

    if (fb->DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
        return fb->DIAGNOSTIC.code;
    }

    if (fb->LAST_FAULT_SNAPSHOT.valid &&
        fb->LAST_FAULT_SNAPSHOT.diagnostic.code != HYD_DIAG_CODE_NONE) {
        return fb->LAST_FAULT_SNAPSHOT.diagnostic.code;
    }

    if (fb->DIAGNOSTIC_HISTORY.hasRecord &&
        fb->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.severity == HYD_DIAG_SEVERITY_FAULT &&
        fb->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code != HYD_DIAG_CODE_NONE) {
        return fb->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code;
    }

    return HYD_DIAG_CODE_NONE;
}

static void HYD_StateReporter_ClearPressureLoopState(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->STATE.pressureLoop, 0, sizeof(fb->STATE.pressureLoop));
}

static void HYD_StateReporter_ClearExecutionReferences(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->STATE.references, 0, sizeof(fb->STATE.references));
    fb->STATE.pressureControllerApplied = HYD_PRESSURE_CONTROLLER_NONE;
    HYD_StateReporter_ClearPressureLoopState(fb);
}

static void HYD_StateReporter_SetExecutionReferences(HYD_MotionControlFB* fb,
                                                     const HYD_ExecutionReference* references,
                                                     HYD_PressureControllerType pressureControllerApplied) {
    if (fb == NULL) {
        return;
    }

    if (references == NULL) {
        HYD_StateReporter_ClearExecutionReferences(fb);
        return;
    }

    fb->STATE.references = *references;
    fb->STATE.pressureControllerApplied = pressureControllerApplied;
}

static void HYD_StateReporter_SetPressureLoopState(HYD_MotionControlFB* fb,
                                                   const HYD_PressureControllerOutput* pressureOutput) {
    if (fb == NULL) {
        return;
    }

    if (pressureOutput == NULL || pressureOutput->appliedStrategy == HYD_PRESSURE_CONTROLLER_NONE) {
        HYD_StateReporter_ClearPressureLoopState(fb);
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

void HYD_StateReporter_SetActive(HYD_MotionControlFB* fb, HYD_BOOL active) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.active = active;
}

void HYD_StateReporter_SetFinished(HYD_MotionControlFB* fb, HYD_BOOL finished) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.finished = finished;
}

void HYD_StateReporter_SetFault(HYD_MotionControlFB* fb, HYD_BOOL fault) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.faultActive = fault;
    HYD_StateReporter_RefreshStandardOutputs(fb);
}

void HYD_StateReporter_SetStatus(HYD_MotionControlFB* fb, HYD_ControllerStatus status) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.status = status;
}

void HYD_StateReporter_SetFbState(HYD_MotionControlFB* fb, HYD_FbState state) {
    if (fb == NULL) {
        return;
    }

    fb->FB_STATE = state;
    HYD_StateReporter_RefreshStandardOutputs(fb);
}

void HYD_StateReporter_SetProtectionAction(HYD_MotionControlFB* fb, HYD_ProtectionAction action) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.protectionAction = action;
}

void HYD_StateReporter_SetPlannedDirection(HYD_MotionControlFB* fb, HYD_MotionDirection direction) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.plannedDirection = direction;
}

void HYD_StateReporter_SetSegmentSource(HYD_MotionControlFB* fb, HYD_SegmentSource source) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.segmentSource = source;
}

void HYD_StateReporter_RefreshStandardOutputs(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->ERROR_ID = HYD_StateReporter_ResolveErrorId(fb);
}

void HYD_StateReporter_ResetTransitionFlags(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_segmentChangedFlag = false;
    fb->SEGMENT_CHANGED = false;
}

void HYD_StateReporter_ApplySafeOutputs(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->PUMP_SPEED = 0.0;
    fb->STATE.plannedVelocity = 0.0;
    fb->STATE.plannedFlow = 0.0;
    fb->STATE.commandedPumpSpeed = 0.0;
    HYD_StateReporter_ClearExecutionReferences(fb);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_NONE);
    HYD_StateReporter_SetPlannedDirection(fb, HYD_DIRECTION_HOLD);
    HYD_StateReporter_SetActive(fb, false);
}

void HYD_StateReporter_ClearSegmentTag(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.currentSegmentTag = 0;
}

void HYD_StateReporter_SetSegmentTag(HYD_MotionControlFB* fb, HYD_UINT8 tag) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.currentSegmentTag = tag;
}

void HYD_StateReporter_SetIdleState(HYD_MotionControlFB* fb,
                                    HYD_BOOL finished,
                                    HYD_BOOL segmentCompleted) {
    HYD_ControllerStatus status;
    HYD_FbState fbState;

    if (fb == NULL) {
        return;
    }

    status = HYD_StateReporter_ResolveIdleStatus(fb, finished, segmentCompleted);
    fbState = HYD_StateReporter_ResolveIdleFbState(fb, finished, segmentCompleted);
    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetFinished(fb, finished);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_NONE);
    fb->SEGMENT_COMPLETED = segmentCompleted;
    HYD_StateReporter_ResetTransitionFlags(fb);
    HYD_StateReporter_SetStatus(fb, status);
    HYD_StateReporter_SetFbState(fb, fbState);
}

void HYD_StateReporter_SetHoldState(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetFinished(fb, false);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_NONE);
    fb->SEGMENT_COMPLETED = false;
    HYD_StateReporter_ResetTransitionFlags(fb);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_HOLD);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_HOLD);
}

void HYD_StateReporter_EnterFaultState(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetFinished(fb, false);
    fb->SEGMENT_COMPLETED = false;
    HYD_StateReporter_ResetTransitionFlags(fb);
    HYD_StateReporter_SetFault(fb, true);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_STOP);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_FAULT);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_FAULT);
}

void HYD_StateReporter_ReportExecution(HYD_MotionControlFB* fb,
                                       const HYD_MotionPlannerOutput* plannerOutput,
                                       const HYD_PumpConverterOutput* pumpOutput,
                                       const HYD_ExecutionReference* references,
                                       HYD_PressureControllerType pressureControllerApplied,
                                       const HYD_PressureControllerOutput* pressureOutput,
                                       const HYD_DiagnosticInfo* diagnostic) {
    if (fb == NULL || plannerOutput == NULL || pumpOutput == NULL) {
        return;
    }

    fb->PUMP_SPEED = pumpOutput->pumpSpeed;
    fb->STATE.plannedVelocity = plannerOutput->targetVelocity;
    fb->STATE.plannedFlow = pumpOutput->commandFlow;
    fb->STATE.commandedPumpSpeed = pumpOutput->pumpSpeed;
    HYD_StateReporter_SetExecutionReferences(fb, references, pressureControllerApplied);
    HYD_StateReporter_SetPressureLoopState(fb, pressureOutput);
    HYD_StateReporter_SetProtectionAction(fb,
                                          (diagnostic != NULL) ? diagnostic->protectionAction
                                                               : HYD_PROTECTION_ACTION_NONE);
    HYD_StateReporter_SetPlannedDirection(fb, plannerOutput->direction);
    HYD_StateReporter_SetActive(fb, true);
    HYD_StateReporter_SetFinished(fb, false);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_SetStatus(fb, HYD_StateReporter_ResolveExecutionStatus(diagnostic));
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_RUNNING);
}

void HYD_StateReporter_ReportDiagnostic(HYD_MotionControlFB* fb,
                                        HYD_DiagnosticCode code,
                                        HYD_DiagnosticSeverity severity,
                                        HYD_TIME eventTimestamp,
                                        const HYD_MotionSegment* segment,
                                        const HYD_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HYD_Diagnostics_SetEvent(&fb->DIAGNOSTIC, code, severity);
    HYD_StateReporter_SetProtectionAction(fb, fb->DIAGNOSTIC.protectionAction);
    HYD_StateReporter_RefreshStandardOutputs(fb);
    HYD_StateReporter_RecordDiagnosticEvent(fb, eventTimestamp, segment, references);
}

void HYD_StateReporter_ReportFault(HYD_MotionControlFB* fb,
                                   HYD_DiagnosticCode code,
                                   HYD_TIME eventTimestamp,
                                   const HYD_MotionSegment* segment,
                                   const HYD_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HYD_ProtectionManager_EnterFaultStop(fb);
    HYD_StateReporter_ReportDiagnostic(fb, code, HYD_DIAG_SEVERITY_FAULT, eventTimestamp, segment, references);
}

void HYD_StateReporter_ClearCurrentDiagnostic(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_Diagnostics_Clear(&fb->DIAGNOSTIC);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_NONE);
    HYD_StateReporter_RefreshStandardOutputs(fb);
}

void HYD_StateReporter_ClearDiagnosticRetentionOnly(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_Diagnostics_ClearSnapshot(&fb->LAST_FAULT_SNAPSHOT);
    HYD_DiagnosticsHistory_Clear(&fb->DIAGNOSTIC_HISTORY);
}

void HYD_StateReporter_ResetDiagnosticRetention(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    HYD_StateReporter_ClearDiagnosticRetentionOnly(fb);
}

void HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(HYD_MotionControlFB* fb) {
    if (fb == NULL || fb->STATE.faultActive || fb->DIAGNOSTIC.code == HYD_DIAG_CODE_NONE) {
        return;
    }

    HYD_StateReporter_ClearCurrentDiagnostic(fb);
}

static HYD_BOOL HYD_StateReporter_ShouldRecordDiagnosticEvent(const HYD_MotionControlFB* fb) {
    if (fb == NULL || fb->DIAGNOSTIC.code == HYD_DIAG_CODE_NONE) {
        return false;
    }

    /* Deduplicate against the last recorded snapshot in history */
    if (fb->DIAGNOSTIC_HISTORY.hasRecord) {
        const HYD_DiagnosticInfo* last = &fb->DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic;
        if (fb->DIAGNOSTIC.code == last->code &&
            fb->DIAGNOSTIC.severity == last->severity &&
            fb->DIAGNOSTIC.flags == last->flags &&
            fb->DIAGNOSTIC.protectionAction == last->protectionAction) {
            return false;
        }
    }

    return true;
}

static HYD_UINT8 HYD_StateReporter_ResolveDiagnosticSegmentIndex(const HYD_MotionControlFB* fb,
                                                                  const HYD_MotionSegment* segment) {
    if (fb == NULL || segment == NULL || fb->STATE.currentSegmentIndex >= HYD_MAX_SEGMENTS) {
        return (HYD_UINT8)HYD_MAX_SEGMENTS;
    }

    return (HYD_UINT8)fb->STATE.currentSegmentIndex;
}

static HYD_UINT8 HYD_StateReporter_ResolveDiagnosticSegmentTag(const HYD_MotionControlFB* fb,
                                                                const HYD_MotionSegment* segment) {
    if (segment != NULL && segment->segmentTag != 0) {
        return segment->segmentTag;
    }

    if (fb != NULL && fb->STATE.currentSegmentTag != 0) {
        return fb->STATE.currentSegmentTag;
    }

    return 0;
}

void HYD_StateReporter_RecordDiagnosticEvent(HYD_MotionControlFB* fb,
                                              HYD_TIME eventTimestamp,
                                              const HYD_MotionSegment* segment,
                                              const HYD_ExecutionReference* references) {
    HYD_DiagnosticSnapshot snapshot;

    if (fb == NULL) {
        return;
    }

    if (fb->DIAGNOSTIC.code == HYD_DIAG_CODE_NONE) {
        return;
    }

    if (!HYD_StateReporter_ShouldRecordDiagnosticEvent(fb)) {
        return;
    }

    HYD_Diagnostics_CaptureSnapshot(&snapshot,
                                    &fb->DIAGNOSTIC,
                                    &fb->AXIS_REF,
                                    references,
                                    eventTimestamp,
                                    HYD_StateReporter_ResolveDiagnosticSegmentIndex(fb, segment),
                                    HYD_StateReporter_ResolveDiagnosticSegmentTag(fb, segment),
                                    fb->STATE.status,
                                    fb->STATE.active,
                                    fb->STATE.finished,
                                    fb->STATE.faultActive);
    if (fb->STATE.faultActive) {
        fb->LAST_FAULT_SNAPSHOT = snapshot;
    }
    HYD_DiagnosticsHistory_Push(&fb->DIAGNOSTIC_HISTORY, &snapshot);
}
