#include "motion_control.h"
#include "diagnostics.h"
#include "motion_planner.h"
#include "pressure_controller.h"
#include "protection_manager.h"
#include "pump_converter.h"
#include "ramp_controller.h"
#include "recipe_validator.h"
#include "segment_completion.h"
#include "state_reporter.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static HDY_BOOL HDY_QueuePendingCommand(HDY_MotionControlFB* fb,
                                        HDY_FbCommand command,
                                        HDY_UINT segmentIndex,
                                        HDY_TIME timestamp);
static void HDY_ReportDiagnostic(HDY_MotionControlFB* fb,
                                 HDY_DiagnosticCode code,
                                 HDY_DiagnosticSeverity severity,
                                 const char* message,
                                 HDY_TIME eventTimestamp,
                                 const HDY_MotionSegment* segment,
                                 const HDY_ExecutionReference* references);
static void HDY_MotionControlFB_RunRunningState(HDY_MotionControlFB* fb);
static void HDY_PrimeSegmentControllers(HDY_MotionControlFB* fb,
                                        const HDY_MotionSegment* segment,
                                        HDY_TIME timestamp,
                                        HDY_BOOL allowFlowCarryover);

static HDY_REAL HDY_MinReal(HDY_REAL left, HDY_REAL right) {
    return (left < right) ? left : right;
}

static HDY_REAL HDY_AbsReal(HDY_REAL value) {
    return (value < 0.0) ? -value : value;
}

static HDY_BOOL HDY_IsFiniteReal(HDY_REAL value) {
    return isfinite(value) ? true : false;
}

typedef HDY_UINT16 HDY_FbStateMask;

#define HDY_FB_STATE_MASK_BIT(state) ((HDY_FbStateMask)(1U << (state)))

static const HDY_FbStateMask HDY_COMMAND_ALLOWED_STATE_MASKS[HDY_CMD_ACK + 1U] = {
    [HDY_CMD_NONE] = (HDY_FbStateMask)0U,
    [HDY_CMD_START] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_IDLE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_READY) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_SEGMENT_COMPLETE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_DONE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_ABORTED),
    [HDY_CMD_NEXT] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_SEGMENT_COMPLETE),
    [HDY_CMD_STOP] = (HDY_FbStateMask)0U,
    [HDY_CMD_HOLD] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_STARTING) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_RUNNING),
    [HDY_CMD_RESUME] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_HOLD),
    [HDY_CMD_ABORT] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_STARTING) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_RUNNING) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_SEGMENT_COMPLETE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_HOLD),
    [HDY_CMD_RESET] = (HDY_FbStateMask)0U,
    [HDY_CMD_ACK] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_DISABLED) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_IDLE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_READY) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_SEGMENT_COMPLETE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_HOLD) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_DONE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_ABORTED)
};

static const char* HDY_CommandToString(HDY_FbCommand command) {
    switch (command) {
        case HDY_CMD_START:
            return "START";
        case HDY_CMD_NEXT:
            return "NEXT";
        case HDY_CMD_STOP:
            return "STOP";
        case HDY_CMD_HOLD:
            return "HOLD";
        case HDY_CMD_RESUME:
            return "RESUME";
        case HDY_CMD_ABORT:
            return "ABORT";
        case HDY_CMD_RESET:
            return "RESET";
        case HDY_CMD_ACK:
            return "ACK";
        case HDY_CMD_NONE:
        default:
            return "NONE";
    }
}

static const char* HDY_FbStateToString(HDY_FbState state) {
    switch (state) {
        case HDY_FB_STATE_DISABLED:
            return "DISABLED";
        case HDY_FB_STATE_IDLE:
            return "IDLE";
        case HDY_FB_STATE_READY:
            return "READY";
        case HDY_FB_STATE_STARTING:
            return "STARTING";
        case HDY_FB_STATE_RUNNING:
            return "RUNNING";
        case HDY_FB_STATE_SEGMENT_COMPLETE:
            return "SEGMENT_COMPLETE";
        case HDY_FB_STATE_HOLD:
            return "HOLD";
        case HDY_FB_STATE_DONE:
            return "DONE";
        case HDY_FB_STATE_ABORTED:
            return "ABORTED";
        case HDY_FB_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

static HDY_FbState HDY_ResolveEffectiveFbState(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return HDY_FB_STATE_IDLE;
    }

    if (!fb->EN) {
        return HDY_FB_STATE_DISABLED;
    }

    return fb->FB_STATE;
}

static HDY_BOOL HDY_UsesRecipeSource(const HDY_MotionControlFB* fb) {
    return (fb != NULL) ? fb->USE_RECIPE : true;
}

static HDY_BOOL HDY_HasSelectedStartSource(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (HDY_UsesRecipeSource(fb)) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

static void HDY_ResetReadyContextPreview(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    if (HDY_UsesRecipeSource(fb) && fb->RECIPE_SIZE > 0U) {
        fb->STATE.currentSegmentIndex = 0U;
    } else {
        fb->STATE.currentSegmentIndex = HDY_MAX_SEGMENTS;
    }
}

static const HDY_MotionSegment* HDY_ResolveStartSourceSegment(const HDY_MotionControlFB* fb,
                                                              size_t requestedSegmentIndex,
                                                              size_t* resolvedSegmentIndex,
                                                              HDY_SegmentSource* resolvedSource) {
    if (resolvedSegmentIndex != NULL) {
        *resolvedSegmentIndex = HDY_MAX_SEGMENTS;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HDY_SEGMENT_SOURCE_NONE;
    }

    if (fb == NULL) {
        return NULL;
    }

    if (HDY_UsesRecipeSource(fb)) {
        if (requestedSegmentIndex >= fb->RECIPE_SIZE) {
            return NULL;
        }
        if (resolvedSegmentIndex != NULL) {
            *resolvedSegmentIndex = requestedSegmentIndex;
        }
        if (resolvedSource != NULL) {
            *resolvedSource = HDY_SEGMENT_SOURCE_RECIPE;
        }
        return &fb->RECIPE[requestedSegmentIndex];
    }

    if (!fb->DIRECT_SEGMENT_VALID) {
        return NULL;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HDY_SEGMENT_SOURCE_DIRECT;
    }
    return &fb->DIRECT_SEGMENT;
}

static HDY_FbStateMask HDY_CommandAllowedStateMask(HDY_FbCommand command) {
    if ((HDY_UINT)command >= (sizeof(HDY_COMMAND_ALLOWED_STATE_MASKS) / sizeof(HDY_COMMAND_ALLOWED_STATE_MASKS[0]))) {
        return (HDY_FbStateMask)0U;
    }

    return HDY_COMMAND_ALLOWED_STATE_MASKS[(HDY_UINT)command];
}

static HDY_BOOL HDY_IsCommandAllowedInState(HDY_FbCommand command, HDY_FbState state) {
    HDY_FbStateMask mask;

    if ((HDY_UINT)state >= (sizeof(HDY_FbStateMask) * 8U)) {
        return false;
    }

    mask = HDY_CommandAllowedStateMask(command);
    return (mask & HDY_FB_STATE_MASK_BIT(state)) != 0U;
}

static const HDY_MotionSegment* HDY_ResolveCommandDiagnosticSegment(const HDY_MotionControlFB* fb,
                                                                    HDY_FbCommand command,
                                                                    HDY_UINT requestedSegmentIndex) {
    if (fb == NULL) {
        return NULL;
    }

    if (command == HDY_CMD_START) {
        return HDY_ResolveStartSourceSegment(fb, requestedSegmentIndex, NULL, NULL);
    }

    if (fb->_activeSegmentValid) {
        return &fb->_activeSegment;
    }

    if (fb->STATE.currentSegmentIndex < fb->RECIPE_SIZE) {
        return &fb->RECIPE[fb->STATE.currentSegmentIndex];
    }

    return NULL;
}

static void HDY_ReportCommandNotAllowed(HDY_MotionControlFB* fb,
                                        HDY_FbCommand command,
                                        HDY_FbState state,
                                        HDY_TIME eventTimestamp,
                                        HDY_UINT requestedSegmentIndex,
                                        const HDY_ExecutionReference* references) {
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return;
    }

    (void)snprintf(message,
                   sizeof(message),
                   "%s not allowed in %s",
                   HDY_CommandToString(command),
                   HDY_FbStateToString(state));
    HDY_ReportDiagnostic(fb,
                         HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                         HDY_DIAG_SEVERITY_WARNING,
                         message,
                         eventTimestamp,
                         HDY_ResolveCommandDiagnosticSegment(fb, command, requestedSegmentIndex),
                         references);
}

static void HDY_ReportPendingCommandConflict(HDY_MotionControlFB* fb,
                                             HDY_FbCommand command,
                                             HDY_TIME eventTimestamp,
                                             HDY_UINT requestedSegmentIndex,
                                             const HDY_ExecutionReference* references) {
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return;
    }

    (void)snprintf(message,
                   sizeof(message),
                   "%s rejected: %s pending",
                   HDY_CommandToString(command),
                   HDY_CommandToString(fb->_pendingCommand));
    HDY_ReportDiagnostic(fb,
                         HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                         HDY_DIAG_SEVERITY_WARNING,
                         message,
                         eventTimestamp,
                         HDY_ResolveCommandDiagnosticSegment(fb, command, requestedSegmentIndex),
                         references);
}

static HDY_BOOL HDY_RequestCommandQueue(HDY_MotionControlFB* fb,
                                        HDY_FbCommand command,
                                        HDY_UINT segmentIndex,
                                        HDY_TIME timestamp,
                                        const HDY_ExecutionReference* references) {
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(command, effectiveState)) {
        if (effectiveState != HDY_FB_STATE_FAULT) {
            HDY_ReportCommandNotAllowed(fb,
                                        command,
                                        effectiveState,
                                        timestamp,
                                        segmentIndex,
                                        references);
        }
        return false;
    }

    if (command != HDY_CMD_ABORT && fb->_pendingCommand != HDY_CMD_NONE) {
        HDY_ReportPendingCommandConflict(fb,
                                         command,
                                         timestamp,
                                         segmentIndex,
                                         references);
        return false;
    }

    return HDY_QueuePendingCommand(fb, command, segmentIndex, timestamp);
}

static void HDY_RecordFeedbackTimestamp(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    fb->_lastFeedbackTimestamp = timestamp;
    fb->_feedbackTimestampValid = true;
}

static HDY_BOOL HDY_AxisRefIsValid(const HDY_AxisRef* axisRef) {
    if (axisRef == NULL) {
        return false;
    }

    return HDY_IsFiniteReal(axisRef->position) &&
        HDY_IsFiniteReal(axisRef->velocity) &&
        HDY_IsFiniteReal(axisRef->flow) &&
        HDY_IsFiniteReal(axisRef->pressure) &&
        HDY_IsFiniteReal(axisRef->timestamp) &&
        (axisRef->pressure >= 0.0) &&
        (axisRef->timestamp >= 0.0);
}

static void HDY_ClearStartCommandInput(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->START_SEGMENT = false;
    fb->START_SEGMENT_INDEX = 0U;
}

static void HDY_ClearPendingCommand(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_pendingCommand = HDY_CMD_NONE;
    fb->_pendingCommandSegmentIndex = 0U;
    fb->_pendingCommandTimestamp = 0.0;
}

static HDY_BOOL HDY_QueuePendingCommand(HDY_MotionControlFB* fb,
                                        HDY_FbCommand command,
                                        HDY_UINT segmentIndex,
                                        HDY_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    if (command == HDY_CMD_ABORT) {
        fb->_pendingCommand = command;
        fb->_pendingCommandSegmentIndex = segmentIndex;
        fb->_pendingCommandTimestamp = timestamp;
        return true;
    }

    if (fb->_pendingCommand != HDY_CMD_NONE) {
        return false;
    }

    fb->_pendingCommand = command;
    fb->_pendingCommandSegmentIndex = segmentIndex;
    fb->_pendingCommandTimestamp = timestamp;
    return true;
}

static void HDY_ClearCurrentDiagnostic(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    fb->_lastRecordedDiagnosticCode = HDY_DIAG_CODE_NONE;
    fb->_lastRecordedDiagnosticSeverity = HDY_DIAG_SEVERITY_NONE;
    fb->_lastRecordedDiagnosticFlags = HDY_DIAG_FLAG_NONE;
    fb->_lastRecordedProtectionAction = HDY_PROTECTION_ACTION_NONE;
    HDY_StateReporter_RefreshStandardOutputs(fb);
}

static void HDY_ClearDiagnosticRetentionOnly(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC_LATCH);
    HDY_Diagnostics_ClearSnapshot(&fb->LAST_DIAGNOSTIC_SNAPSHOT);
    HDY_Diagnostics_ClearSnapshot(&fb->LAST_FAULT_SNAPSHOT);
    HDY_DiagnosticsHistory_Clear(&fb->DIAGNOSTIC_HISTORY);
}

static void HDY_ResetDiagnosticRetention(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_ClearCurrentDiagnostic(fb);
    HDY_ClearDiagnosticRetentionOnly(fb);
}

static void HDY_ClearLiveDiagnosticInNonFaultHold(HDY_MotionControlFB* fb) {
    if (fb == NULL || fb->FAULT || fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        return;
    }

    HDY_ClearCurrentDiagnostic(fb);
}

static void HDY_UpdateRecordedDiagnosticSignature(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_lastRecordedDiagnosticCode = fb->DIAGNOSTIC.code;
    fb->_lastRecordedDiagnosticSeverity = fb->DIAGNOSTIC.severity;
    fb->_lastRecordedDiagnosticFlags = fb->DIAGNOSTIC.flags;
    fb->_lastRecordedProtectionAction = fb->DIAGNOSTIC.protectionAction;
}

static HDY_BOOL HDY_ShouldRecordDiagnosticEvent(const HDY_MotionControlFB* fb) {
    if (fb == NULL || fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        return false;
    }

    return (fb->DIAGNOSTIC.code != fb->_lastRecordedDiagnosticCode) ||
        (fb->DIAGNOSTIC.severity != fb->_lastRecordedDiagnosticSeverity) ||
        (fb->DIAGNOSTIC.flags != fb->_lastRecordedDiagnosticFlags) ||
        (fb->DIAGNOSTIC.protectionAction != fb->_lastRecordedProtectionAction);
}

static HDY_UINT8 HDY_ResolveDiagnosticSegmentIndex(const HDY_MotionControlFB* fb,
                                                   const HDY_MotionSegment* segment) {
    if (fb == NULL || segment == NULL || fb->STATE.currentSegmentIndex >= HDY_MAX_SEGMENTS) {
        return (HDY_UINT8)HDY_MAX_SEGMENTS;
    }

    return (HDY_UINT8)fb->STATE.currentSegmentIndex;
}

static const char* HDY_ResolveDiagnosticSegmentName(const HDY_MotionControlFB* fb,
                                                    const HDY_MotionSegment* segment) {
    if (segment != NULL && segment->name[0] != '\0') {
        return segment->name;
    }

    if (fb != NULL && fb->STATE.currentSegmentName[0] != '\0') {
        return fb->STATE.currentSegmentName;
    }

    return NULL;
}

static void HDY_RecordDiagnosticEvent(HDY_MotionControlFB* fb,
                                      HDY_TIME eventTimestamp,
                                      const HDY_MotionSegment* segment,
                                      const HDY_ExecutionReference* references) {
    HDY_DiagnosticSnapshot snapshot;

    if (fb == NULL) {
        return;
    }

    if (fb->DIAGNOSTIC.code == HDY_DIAG_CODE_NONE) {
        HDY_UpdateRecordedDiagnosticSignature(fb);
        return;
    }

    if (!HDY_ShouldRecordDiagnosticEvent(fb)) {
        HDY_UpdateRecordedDiagnosticSignature(fb);
        return;
    }

    HDY_Diagnostics_CaptureSnapshot(&snapshot,
                                    &fb->DIAGNOSTIC,
                                    &fb->AXIS_REF,
                                    references,
                                    eventTimestamp,
                                    HDY_ResolveDiagnosticSegmentIndex(fb, segment),
                                    HDY_ResolveDiagnosticSegmentName(fb, segment),
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
    HDY_UpdateRecordedDiagnosticSignature(fb);
}

static void HDY_PrepareRecipeLoadState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_ResetReadyContextPreview(fb);
    fb->_segmentStartTime = 0.0;
    fb->_holdStateTime = 0.0;
    fb->_lastCommandedFlow = 0.0;
    fb->_activeSegmentValid = false;
    fb->_startSegmentSignalPrev = false;
    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    HDY_ClearPendingCommand(fb);
    HDY_ClearStartCommandInput(fb);
    HDY_StateReporter_SetIdleState(fb, false, false);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_ClearSegmentName(fb);
}

static void HDY_ReportDiagnostic(HDY_MotionControlFB* fb,
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
    HDY_StateReporter_RefreshStandardOutputs(fb);
    HDY_RecordDiagnosticEvent(fb, eventTimestamp, segment, references);
}

static void HDY_ReportFault(HDY_MotionControlFB* fb,
                            HDY_DiagnosticCode code,
                            const char* message,
                            HDY_TIME eventTimestamp,
                            const HDY_MotionSegment* segment,
                            const HDY_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_EnterFaultStop(fb);
    HDY_ReportDiagnostic(fb, code, HDY_DIAG_SEVERITY_FAULT, message, eventTimestamp, segment, references);
}

static HDY_BOOL HDY_ValidateStartRequest(const HDY_MotionControlFB* fb,
                                         size_t segmentIndex,
                                         HDY_DiagnosticCode* code,
                                         char* message,
                                         size_t messageSize) {
    const HDY_MotionSegment* segment;
    size_t resolvedSegmentIndex;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_HasSelectedStartSource(fb)) {
        if (code != NULL) {
            *code = HDY_UsesRecipeSource(fb) ? HDY_DIAG_CODE_NO_RECIPE
                                             : HDY_DIAG_CODE_NO_DIRECT_SEGMENT;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message,
                    HDY_UsesRecipeSource(fb) ? "No recipe loaded"
                                             : "No direct segment configured",
                    messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    segment = HDY_ResolveStartSourceSegment(fb, segmentIndex, &resolvedSegmentIndex, NULL);
    if (segment == NULL) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "Start segment index is out of range", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    return HDY_RecipeValidator_ValidateRuntimeConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                                     fb->PUMP_SPEED_LIMIT,
                                                     code,
                                                     message,
                                                     messageSize) &&
        HDY_RecipeValidator_ValidateSegment(segment,
                                            resolvedSegmentIndex,
                                            code,
                                            message,
                                            messageSize) &&
        HDY_RecipeValidator_ValidateStartContext(segment,
                                                 resolvedSegmentIndex,
                                                 &fb->AXIS_REF,
                                                 code,
                                                 message,
                                                 messageSize);
}

static HDY_BOOL HDY_ValidateNextRequest(const HDY_MotionControlFB* fb,
                                        HDY_DiagnosticCode* code,
                                        char* message,
                                        size_t messageSize) {
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_UsesRecipeSource(fb)) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "NEXT is not supported in direct mode", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    if (fb->RECIPE_SIZE == 0U) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_NO_RECIPE;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "No recipe loaded", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_DONE) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "Recipe is already finished", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_STARTING || effectiveState == HDY_FB_STATE_RUNNING) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "Current segment has not completed", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    if (!HDY_IsCommandAllowedInState(HDY_CMD_NEXT, effectiveState)) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        if (message != NULL && messageSize > 0U) {
            (void)snprintf(message,
                           messageSize,
                           "%s not allowed in %s",
                           HDY_CommandToString(HDY_CMD_NEXT),
                           HDY_FbStateToString(effectiveState));
        }
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        if (message != NULL && messageSize > 0U) {
            strncpy(message, "Current segment has not completed", messageSize - 1U);
            message[messageSize - 1U] = '\0';
        }
        return false;
    }

    return true;
}

static void HDY_PrimeSegmentControllers(HDY_MotionControlFB* fb,
                                        const HDY_MotionSegment* segment,
                                        HDY_TIME timestamp,
                                        HDY_BOOL allowFlowCarryover) {
    HDY_REAL initialPressureControlOutput;
    HDY_REAL trackingFlowReference;

    if (fb == NULL || segment == NULL) {
        return;
    }

    HDY_RecordFeedbackTimestamp(fb, timestamp);
    HDY_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp);

    trackingFlowReference = HDY_AbsReal(fb->AXIS_REF.flow);
    if (trackingFlowReference <= 0.0 && allowFlowCarryover) {
        trackingFlowReference = fb->_lastCommandedFlow;
    }

    initialPressureControlOutput = segment->targetFlow;
    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP && trackingFlowReference > 0.0) {
        initialPressureControlOutput = trackingFlowReference;
    }
    initialPressureControlOutput = HDY_MinReal(initialPressureControlOutput, segment->maxFlow);
    if (initialPressureControlOutput < 0.0) {
        initialPressureControlOutput = 0.0;
    }

    HDY_PressureController_InitState(&fb->_pressureController,
                                     fb->AXIS_REF.pressure,
                                     initialPressureControlOutput,
                                     timestamp);
    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP && trackingFlowReference > 0.0) {
        HDY_PressureController_RequestTracking(&fb->_pressureController, initialPressureControlOutput);
    }
}

static HDY_BOOL HDY_BeginSegment(HDY_MotionControlFB* fb,
                                 size_t segmentIndex,
                                 HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};
    HDY_BOOL preserveFlowCarryover;
    const HDY_MotionSegment* sourceSegment;
    size_t resolvedSegmentIndex;
    HDY_SegmentSource resolvedSource;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_ValidateStartRequest(fb, segmentIndex, &code, message, sizeof(message))) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ResetReadyContextPreview(fb);
        HDY_ReportDiagnostic(fb,
                             code,
                             HDY_DIAG_SEVERITY_WARNING,
                             message,
                             timestamp,
                             HDY_ResolveStartSourceSegment(fb, segmentIndex, NULL, NULL),
                             NULL);
        return false;
    }

    preserveFlowCarryover = fb->SEGMENT_COMPLETED;
    sourceSegment = HDY_ResolveStartSourceSegment(fb,
                                                  segmentIndex,
                                                  &resolvedSegmentIndex,
                                                  &resolvedSource);
    if (sourceSegment == NULL) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Start source resolution failed",
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return false;
    }

    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_ResetTransitionFlags(fb);

    fb->_activeSegment = *sourceSegment;
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = resolvedSource;
    fb->STATE.currentSegmentIndex = resolvedSegmentIndex;
    fb->_segmentStartTime = timestamp;
    fb->_holdStateTime = 0.0;
    fb->_segmentChangedFlag = true;
    fb->SEGMENT_COMPLETED = false;

    HDY_PrimeSegmentControllers(fb, &fb->_activeSegment, timestamp, preserveFlowCarryover);

    HDY_StateReporter_SetSegmentName(fb, fb->_activeSegment.name);
    HDY_StateReporter_SetSegmentSource(fb, resolvedSource);
    HDY_ClearCurrentDiagnostic(fb);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
    HDY_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);
    return true;
}

static HDY_BOOL HDY_AdvanceToNextSegment(HDY_MotionControlFB* fb,
                                         HDY_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    if (!HDY_UsesRecipeSource(fb)) {
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                             HDY_DIAG_SEVERITY_WARNING,
                             "NEXT is not supported in direct mode",
                             timestamp,
                             &fb->_activeSegment,
                             &fb->STATE.references);
        return false;
    }

    if (fb->STATE.currentSegmentIndex + 1U < fb->RECIPE_SIZE) {
        return HDY_BeginSegment(fb, fb->STATE.currentSegmentIndex + 1U, timestamp);
    }

    HDY_ProtectionManager_ApplyIdleState(fb, true, true);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_ResetReadyContextPreview(fb);
    HDY_ClearCurrentDiagnostic(fb);
    return true;
}

static void HDY_EnterHoldNow(HDY_MotionControlFB* fb,
                             HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Hold requested without active segment",
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    fb->_holdStateTime = timestamp;
    HDY_RecordFeedbackTimestamp(fb, timestamp);
    HDY_StateReporter_SetSegmentName(fb, fb->_activeSegment.name);
    HDY_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HDY_StateReporter_SetHoldState(fb);
}

static HDY_BOOL HDY_ResumeHeldSegment(HDY_MotionControlFB* fb,
                                      HDY_TIME timestamp) {
    HDY_TIME holdDuration;

    if (fb == NULL) {
        return false;
    }

    if (!fb->_activeSegmentValid) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Resume requested without held segment",
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return false;
    }

    if (!HDY_AxisRefIsValid(&fb->AXIS_REF)) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_SENSOR_FAULT,
                        "Axis feedback is invalid",
                        fb->AXIS_REF.timestamp,
                        &fb->_activeSegment,
                        &fb->STATE.references);
        return false;
    }

    if (fb->_feedbackTimestampValid && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
                        "Axis timestamp moved backwards",
                        fb->AXIS_REF.timestamp,
                        &fb->_activeSegment,
                        &fb->STATE.references);
        return false;
    }

    holdDuration = timestamp - fb->_holdStateTime;
    if (holdDuration < 0.0) {
        holdDuration = 0.0;
    }

    fb->_segmentStartTime += holdDuration;
    fb->_holdStateTime = 0.0;
    fb->SEGMENT_COMPLETED = false;
    HDY_PrimeSegmentControllers(fb, &fb->_activeSegment, timestamp, true);
    HDY_StateReporter_SetSegmentName(fb, fb->_activeSegment.name);
    HDY_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
    HDY_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);
    return true;
}

static void HDY_AbortNow(HDY_MotionControlFB* fb,
                         HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    HDY_ClearStartCommandInput(fb);
    HDY_ProtectionManager_ApplyIdleState(fb, true, false);
    fb->_holdStateTime = 0.0;
    fb->_lastCommandedFlow = 0.0;
    HDY_ResetReadyContextPreview(fb);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_ABORTED);
    HDY_ReportDiagnostic(fb,
                         HDY_DIAG_CODE_ABORTED,
                         HDY_DIAG_SEVERITY_INFO,
                         "Aborted by caller",
                         timestamp,
                         NULL,
                         &fb->STATE.references);
}

static void HDY_UpdateSegmentChangedPulse(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
    fb->_segmentChangedFlag = false;
}

static void HDY_MaintainNonExecutingState(HDY_MotionControlFB* fb,
                                          HDY_BOOL autoClearLiveDiagnostic) {
    HDY_FbState preservedState;

    if (fb == NULL) {
        return;
    }

    if (fb->FINISHED) {
        preservedState = fb->FB_STATE;
        HDY_ProtectionManager_ApplyIdleState(fb, true, fb->SEGMENT_COMPLETED);
        if (preservedState == HDY_FB_STATE_ABORTED) {
            HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_ABORTED);
        }
        if (autoClearLiveDiagnostic) {
            HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    if (fb->SEGMENT_COMPLETED) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, true);
        if (autoClearLiveDiagnostic) {
            HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    HDY_ProtectionManager_ApplyIdleState(fb, false, false);
    HDY_ResetReadyContextPreview(fb);
    if (autoClearLiveDiagnostic) {
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
    }
}

static void HDY_MaintainPausedHoldState(HDY_MotionControlFB* fb,
                                        HDY_BOOL autoClearLiveDiagnostic) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Hold state lost active segment context",
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    HDY_StateReporter_SetSegmentName(fb, fb->_activeSegment.name);
    HDY_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HDY_StateReporter_SetHoldState(fb);
    if (autoClearLiveDiagnostic) {
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
    }
}

static HDY_BOOL HDY_MotionControlFB_ConsumePendingCommand(HDY_MotionControlFB* fb,
                                                          HDY_FbCommand* processedCommand) {
    HDY_FbCommand command;
    HDY_UINT segmentIndex;
    HDY_TIME timestamp;
    HDY_FbState effectiveState;

    if (processedCommand != NULL) {
        *processedCommand = HDY_CMD_NONE;
    }

    if (fb == NULL || fb->_pendingCommand == HDY_CMD_NONE) {
        return true;
    }

    command = fb->_pendingCommand;
    segmentIndex = fb->_pendingCommandSegmentIndex;
    timestamp = fb->_pendingCommandTimestamp;
    HDY_ClearPendingCommand(fb);

    if (processedCommand != NULL) {
        *processedCommand = command;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(command, effectiveState)) {
        if (effectiveState != HDY_FB_STATE_FAULT) {
            HDY_ReportCommandNotAllowed(fb,
                                        command,
                                        effectiveState,
                                        timestamp,
                                        segmentIndex,
                                        &fb->STATE.references);
        }
        return true;
    }

    switch (command) {
        case HDY_CMD_START:
            (void)HDY_BeginSegment(fb, segmentIndex, timestamp);
            return true;
        case HDY_CMD_NEXT:
            (void)HDY_AdvanceToNextSegment(fb, timestamp);
            return true;
        case HDY_CMD_HOLD:
            HDY_EnterHoldNow(fb, timestamp);
            return false;
        case HDY_CMD_RESUME:
            return HDY_ResumeHeldSegment(fb, timestamp);
        case HDY_CMD_ABORT:
            HDY_AbortNow(fb, timestamp);
            return false;
        default:
            return true;
    }
}

static void HDY_MotionControlFB_RunStateMachine(HDY_MotionControlFB* fb,
                                                HDY_BOOL allowRunningExecution) {
    if (fb == NULL || !allowRunningExecution) {
        return;
    }

    switch (fb->FB_STATE) {
        case HDY_FB_STATE_STARTING:
        case HDY_FB_STATE_RUNNING:
            HDY_MotionControlFB_RunRunningState(fb);
            break;
        case HDY_FB_STATE_DISABLED:
        case HDY_FB_STATE_IDLE:
        case HDY_FB_STATE_READY:
        case HDY_FB_STATE_SEGMENT_COMPLETE:
        case HDY_FB_STATE_HOLD:
        case HDY_FB_STATE_DONE:
        case HDY_FB_STATE_ABORTED:
        case HDY_FB_STATE_FAULT:
        default:
            break;
    }
}

static void HDY_MotionControlFB_PublishOutputs(HDY_MotionControlFB* fb,
                                                HDY_BOOL autoClearLiveDiagnostic) {
    if (fb == NULL) {
        return;
    }

    if (fb->FAULT) {
        HDY_ProtectionManager_ApplyFaultHold(fb);
        HDY_UpdateSegmentChangedPulse(fb);
        return;
    }

    switch (fb->FB_STATE) {
        case HDY_FB_STATE_STARTING:
        case HDY_FB_STATE_RUNNING:
            break;
        case HDY_FB_STATE_HOLD:
            HDY_MaintainPausedHoldState(fb, autoClearLiveDiagnostic);
            break;
        case HDY_FB_STATE_DISABLED:
        case HDY_FB_STATE_IDLE:
        case HDY_FB_STATE_READY:
        case HDY_FB_STATE_SEGMENT_COMPLETE:
        case HDY_FB_STATE_DONE:
        case HDY_FB_STATE_ABORTED:
        case HDY_FB_STATE_FAULT:
        default:
            HDY_MaintainNonExecutingState(fb, autoClearLiveDiagnostic);
            break;
    }

    HDY_UpdateSegmentChangedPulse(fb);
}

static void HDY_MotionControlFB_RunRunningState(HDY_MotionControlFB* fb) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};
    const HDY_MotionSegment* segment;
    HDY_REAL elapsed;
    HDY_RampControllerInput rampInput;
    HDY_RampControllerOutput rampOutput;
    HDY_MotionPlannerInput plannerInput;
    HDY_MotionPlannerOutput plannerOutput;
    HDY_PressureControllerInput pressureInput;
    HDY_PressureControllerOutput pressureOutput;
    HDY_PumpConverterInput pumpInput;
    HDY_PumpConverterOutput pumpOutput;
    HDY_ExecutionReference executionReference;
    HDY_DiagnosticsContext diagnosticContext;
    HDY_SegmentCompletionContext completionContext;
    HDY_BOOL segmentCompleted;
    HDY_BOOL recipeFinished;
    HDY_SegmentSource completedSegmentSource;

    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Active segment context is invalid",
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    segment = &fb->_activeSegment;

    if ((fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_RECIPE &&
         fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) ||
        (fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_DIRECT &&
         fb->STATE.currentSegmentIndex != HDY_MAX_SEGMENTS) ||
        (fb->_activeSegmentSource == HDY_SEGMENT_SOURCE_NONE)) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        "Current segment index is out of range",
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    if (!HDY_AxisRefIsValid(&fb->AXIS_REF)) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_SENSOR_FAULT,
                        "Axis feedback is invalid",
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }

    if (fb->_feedbackTimestampValid && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
                        "Axis timestamp moved backwards",
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }
    HDY_RecordFeedbackTimestamp(fb, fb->AXIS_REF.timestamp);

    if (!HDY_RecipeValidator_ValidateRuntimeConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                                   fb->PUMP_SPEED_LIMIT,
                                                   &code,
                                                   message,
                                                   sizeof(message)) ||
        !HDY_RecipeValidator_ValidateSegment(segment,
                                             fb->STATE.currentSegmentIndex,
                                             &code,
                                             message,
                                             sizeof(message))) {
        HDY_ReportFault(fb,
                        code,
                        message,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }

    elapsed = fb->AXIS_REF.timestamp - fb->_segmentStartTime;
    if (elapsed < 0.0) {
        elapsed = 0.0;
    }

    rampInput.targetPressure = segment->targetPressure;
    rampInput.rampRate = segment->pressureRampRate;
    rampInput.currentTime = fb->AXIS_REF.timestamp;
    HDY_RampController_Execute(&fb->_rampController, &rampInput, &rampOutput);

    memset(&plannerOutput, 0, sizeof(plannerOutput));
    memset(&pressureOutput, 0, sizeof(pressureOutput));
    plannerOutput.direction = HDY_DIRECTION_HOLD;
    pressureOutput.appliedStrategy = HDY_PRESSURE_CONTROLLER_NONE;

    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
        pressureInput.targetPressure = rampOutput.rampedPressure;
        pressureInput.measuredPressure = fb->AXIS_REF.pressure;
        pressureInput.feedforwardFlow = segment->targetFlow;
        pressureInput.outputMin = 0.0;
        pressureInput.outputMax = segment->maxFlow;
        pressureInput.timestamp = fb->AXIS_REF.timestamp;
        HDY_PressureController_Execute(segment,
                                       &fb->_pressureController,
                                       &pressureInput,
                                       &pressureOutput);
        plannerOutput.targetFlow = pressureOutput.outputFlow;
        plannerOutput.direction = segment->direction;
    } else {
        plannerInput.axisRef = &fb->AXIS_REF;
        plannerInput.segment = segment;
        plannerInput.elapsedTime = elapsed;
        plannerInput.rampedPressure = rampOutput.rampedPressure;
        HDY_MotionPlanner_Execute(&plannerInput, &plannerOutput);
    }

    pumpInput.requestedFlow = plannerOutput.targetFlow;
    pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    pumpInput.direction = plannerOutput.direction;
    HDY_PumpConverter_Execute(&pumpInput, &pumpOutput);

    executionReference.elapsedTime = elapsed;
    executionReference.pressureReference = (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP)
        ? rampOutput.rampedPressure
        : segment->targetPressure;
    executionReference.flowReference = pumpOutput.commandFlow;
    executionReference.velocityReference = plannerOutput.targetVelocity;

    diagnosticContext.axisRef = &fb->AXIS_REF;
    diagnosticContext.segment = segment;
    diagnosticContext.references = &executionReference;
    HDY_Diagnostics_UpdateExecution(&fb->DIAGNOSTIC, &diagnosticContext);

    fb->_lastCommandedFlow = pumpOutput.commandFlow;

    if (fb->DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP) {
        HDY_ProtectionManager_EnterFaultStop(fb);
        HDY_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
        return;
    }

    completionContext.segment = segment;
    completionContext.axisRef = &fb->AXIS_REF;
    completionContext.references = &executionReference;
    segmentCompleted = HDY_SegmentCompletion_CheckWithContext(&completionContext);
    if (segmentCompleted) {
        completedSegmentSource = fb->_activeSegmentSource;
        recipeFinished = (completedSegmentSource == HDY_SEGMENT_SOURCE_DIRECT) ||
            (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HDY_ProtectionManager_ApplyIdleState(fb, recipeFinished, true);
        HDY_StateReporter_SetSegmentSource(fb, completedSegmentSource);
        HDY_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
        return;
    }

    HDY_StateReporter_ReportExecution(fb,
                                      &plannerOutput,
                                      &pumpOutput,
                                      &executionReference,
                                      pressureOutput.appliedStrategy,
                                      &pressureOutput,
                                      &fb->DIAGNOSTIC);
    HDY_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HDY_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
    fb->SEGMENT_COMPLETED = false;
}

static HDY_BOOL HDY_RequestStartCommand(HDY_MotionControlFB* fb,
                                        size_t segmentIndex,
                                        HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    if (fb->FAULT) {
        return false;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(HDY_CMD_START, effectiveState)) {
        HDY_ReportCommandNotAllowed(fb,
                                    HDY_CMD_START,
                                    effectiveState,
                                    timestamp,
                                    (HDY_UINT)segmentIndex,
                                    &fb->STATE.references);
        return false;
    }

    if (!HDY_ValidateStartRequest(fb, segmentIndex, &code, message, sizeof(message))) {
        HDY_ReportDiagnostic(fb,
                             code,
                             HDY_DIAG_SEVERITY_WARNING,
                             message,
                             timestamp,
                             HDY_ResolveStartSourceSegment(fb, segmentIndex, NULL, NULL),
                             NULL);
        return false;
    }

    return HDY_RequestCommandQueue(fb,
                                   HDY_CMD_START,
                                   (HDY_UINT)segmentIndex,
                                   timestamp,
                                   &fb->STATE.references);
}

static HDY_BOOL HDY_RequestHoldCommand(HDY_MotionControlFB* fb,
                                       HDY_TIME timestamp) {
    if (fb == NULL || fb->FAULT) {
        return false;
    }

    return HDY_RequestCommandQueue(fb,
                                   HDY_CMD_HOLD,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static HDY_BOOL HDY_RequestResumeCommand(HDY_MotionControlFB* fb,
                                         HDY_TIME timestamp) {
    if (fb == NULL || fb->FAULT) {
        return false;
    }

    return HDY_RequestCommandQueue(fb,
                                   HDY_CMD_RESUME,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static HDY_BOOL HDY_RequestAbortCommand(HDY_MotionControlFB* fb,
                                        HDY_TIME timestamp) {
    HDY_FbState effectiveState;

    if (fb == NULL || fb->FAULT) {
        return false;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(HDY_CMD_ABORT, effectiveState)) {
        HDY_ReportCommandNotAllowed(fb,
                                    HDY_CMD_ABORT,
                                    effectiveState,
                                    timestamp,
                                    0U,
                                    &fb->STATE.references);
        return false;
    }

    HDY_ClearStartCommandInput(fb);
    return HDY_RequestCommandQueue(fb,
                                   HDY_CMD_ABORT,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static void HDY_MotionControlFB_SampleCommands(HDY_MotionControlFB* fb) {
    HDY_BOOL startSignal;
    HDY_BOOL startEdge;
    HDY_UINT startSegmentIndex;

    if (fb == NULL) {
        return;
    }

    startSignal = fb->START_SEGMENT;
    startSegmentIndex = fb->START_SEGMENT_INDEX;
    startEdge = startSignal && !fb->_startSegmentSignalPrev;
    fb->_startSegmentSignalPrev = startSignal;
    HDY_ClearStartCommandInput(fb);

    if (!fb->EN) {
        return;
    }

    if (startEdge) {
        (void)HDY_RequestStartCommand(fb, startSegmentIndex, fb->AXIS_REF.timestamp);
    }
}

void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(fb, 0, sizeof(*fb));
    fb->ENO = true;
    fb->USE_RECIPE = true;
    fb->FB_STATE = HDY_FB_STATE_IDLE;
    fb->STATE.currentSegmentIndex = HDY_MAX_SEGMENTS;
    fb->_activeSegmentSource = HDY_SEGMENT_SOURCE_NONE;
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_ResetDiagnosticRetention(fb);
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
    HDY_PressureController_ClearState(&fb->_pressureController);
    HDY_ClearPendingCommand(fb);
    HDY_StateReporter_RefreshStandardOutputs(fb);
}

HDY_BOOL HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb,
                                        const HDY_MotionSegment* recipe,
                                        size_t recipeSize) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return false;
    }

    if (!HDY_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code, message, sizeof(message))) {
        memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
        fb->RECIPE_SIZE = 0U;
        HDY_PrepareRecipeLoadState(fb);
        HDY_ReportDiagnostic(fb,
                             code,
                             HDY_DIAG_SEVERITY_WARNING,
                             message,
                             fb->AXIS_REF.timestamp,
                             NULL,
                             NULL);
        return false;
    }

    memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
    memcpy(fb->RECIPE, recipe, recipeSize * sizeof(HDY_MotionSegment));
    fb->RECIPE_SIZE = recipeSize;
    HDY_PrepareRecipeLoadState(fb);
    HDY_ClearCurrentDiagnostic(fb);
    return true;
}

HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(HDY_MotionControlFB* fb,
                                              const HDY_MotionSegment* segment) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return false;
    }

    if (!HDY_RecipeValidator_ValidateSegment(segment,
                                             HDY_MAX_SEGMENTS,
                                             &code,
                                             message,
                                             sizeof(message))) {
        memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
        fb->DIRECT_SEGMENT_VALID = false;
        if (!fb->ACTIVE && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->FINISHED && !fb->SEGMENT_COMPLETED) {
            HDY_ResetReadyContextPreview(fb);
            HDY_StateReporter_SetIdleState(fb, false, false);
            HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
            HDY_StateReporter_ClearSegmentName(fb);
        }
        HDY_ReportDiagnostic(fb,
                             code,
                             HDY_DIAG_SEVERITY_WARNING,
                             message,
                             fb->AXIS_REF.timestamp,
                             NULL,
                             NULL);
        return false;
    }

    fb->DIRECT_SEGMENT = *segment;
    fb->DIRECT_SEGMENT_VALID = true;
    if (!fb->ACTIVE && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->FINISHED && !fb->SEGMENT_COMPLETED) {
        HDY_ResetReadyContextPreview(fb);
        HDY_StateReporter_SetIdleState(fb, false, false);
    }
    HDY_ClearCurrentDiagnostic(fb);
    return true;
}

void HDY_MotionControlFB_ClearDirectSegment(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
    fb->DIRECT_SEGMENT_VALID = false;
    if (!fb->ACTIVE && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->FINISHED && !fb->SEGMENT_COMPLETED) {
        HDY_ResetReadyContextPreview(fb);
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
        HDY_StateReporter_ClearSegmentName(fb);
    }
}

HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb,
                                          size_t segmentIndex,
                                          HDY_TIME timestamp) {
    return HDY_RequestStartCommand(fb, segmentIndex, timestamp);
}

HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return false;
    }

    if (fb->FAULT) {
        return false;
    }

    if (!HDY_ValidateNextRequest(fb, &code, message, sizeof(message))) {
        HDY_ReportDiagnostic(fb,
                             code,
                             (code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED)
                                 ? HDY_DIAG_SEVERITY_INFO
                                 : HDY_DIAG_SEVERITY_WARNING,
                             message,
                             timestamp,
                             (fb->STATE.currentSegmentIndex < fb->RECIPE_SIZE)
                                 ? &fb->RECIPE[fb->STATE.currentSegmentIndex]
                                 : NULL,
                             &fb->STATE.references);
        return false;
    }

    return HDY_RequestCommandQueue(fb,
                                   HDY_CMD_NEXT,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

HDY_BOOL HDY_MotionControlFB_Hold(HDY_MotionControlFB* fb) {
    return HDY_RequestHoldCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HDY_BOOL HDY_MotionControlFB_Resume(HDY_MotionControlFB* fb) {
    return HDY_RequestResumeCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HDY_BOOL HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb) {
    return HDY_RequestAbortCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HDY_BOOL HDY_MotionControlFB_AcknowledgeDiagnostics(HDY_MotionControlFB* fb) {
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HDY_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(HDY_CMD_ACK, effectiveState)) {
        return false;
    }

    if (fb->FAULT || fb->DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
        return false;
    }

    HDY_ClearCurrentDiagnostic(fb);
    HDY_ClearDiagnosticRetentionOnly(fb);
    return true;
}

void HDY_MotionControlFB_Cycle(HDY_MotionControlFB* fb) {
    HDY_FbCommand processedCommand;
    HDY_BOOL allowRunningExecution;

    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = false;

    if (!fb->EN) {
        fb->ENO = false;
        HDY_ClearPendingCommand(fb);
        HDY_ClearStartCommandInput(fb);
        HDY_ProtectionManager_ApplyDisabledState(fb);
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        return;
    }

    fb->ENO = true;
    if (fb->RESET) {
        HDY_MotionControlFB_Init(fb);
        return;
    }

    allowRunningExecution = HDY_MotionControlFB_ConsumePendingCommand(fb, &processedCommand);
    HDY_MotionControlFB_RunStateMachine(fb, allowRunningExecution);
    HDY_MotionControlFB_PublishOutputs(fb, processedCommand == HDY_CMD_NONE);
}

void HDY_MotionControlFB_Scan(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_SampleCommands(fb);
    HDY_MotionControlFB_Cycle(fb);
}

void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_Scan(fb);
}
