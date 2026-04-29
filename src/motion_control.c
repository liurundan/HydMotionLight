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
#include "motion_utils.h"
#include "motion_validator.h"
#include "segment_limits.h"
#include "hdy_config.h"
#include <math.h>
#include <string.h>



/* Internal function declarations */
static HDY_BOOL HDY_QueuePendingCommand(HDY_MotionControlFB* fb,
                                        HDY_FbCommand command,
                                        HDY_UINT segmentIndex,
                                        HDY_TIME timestamp);
static void HDY_MotionControlFB_RunRunningState(HDY_MotionControlFB* fb);
static void HDY_PrimeSegmentControllers(HDY_MotionControlFB* fb,
                                        const HDY_MotionSegment* segment,
                                        HDY_TIME timestamp,
                                        HDY_BOOL allowFlowCarryover);
static void HDY_ExecuteActiveSegmentControl(HDY_MotionControlFB* fb,
                                            const HDY_MotionSegment* segment,
                                            HDY_REAL elapsed,
                                            HDY_RampControllerOutput* rampOutput,
                                            HDY_MotionPlannerOutput* plannerOutput,
                                            HDY_PressureControllerOutput* pressureOutput,
                                            HDY_PumpConverterOutput* pumpOutput,
                                            HDY_ExecutionReference* executionReference);
static void HDY_UpdateExecutionDiagnostics(HDY_MotionControlFB* fb,
                                           const HDY_MotionSegment* segment,
                                           const HDY_ExecutionReference* executionReference,
                                           HDY_REAL elapsed);
static void HDY_ConfigureSegmentCriteria(HDY_DiagnosticCriteria* criteria,
                                          HDY_REAL baseThreshold,
                                          HDY_DiagnosticCode code,
                                          HDY_ProtectionAction action);

/* Diagnostic reporting moved to StateReporter: use
 * HDY_StateReporter_ReportDiagnostic / HDY_StateReporter_ReportFault
 * directly from motion_control to centralize behavior.
 */

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
    [HDY_CMD_ABORT] = HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_IDLE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_READY) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_STARTING) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_RUNNING) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_SEGMENT_COMPLETE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_DONE) |
        HDY_FB_STATE_MASK_BIT(HDY_FB_STATE_ABORTED) |
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

/* String conversion functions moved to motion_utils module
 * Use HDY_MotionUtils_CommandToString and HDY_MotionUtils_StateToString
 */

/* Helper functions moved to motion_validator module
 * Use HDY_MotionValidator_ResolveEffectiveFbState,
 * HDY_MotionValidator_UsesRecipeSource, and
 * HDY_MotionValidator_HasSelectedStartSource
 */

static void HDY_ResetReadyContextPreview(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    if (HDY_MotionValidator_UsesRecipeSource(fb) && fb->RECIPE_SIZE > 0U) {
        fb->STATE.currentSegmentIndex = 0U;
    } else {
        fb->STATE.currentSegmentIndex = HDY_MAX_SEGMENTS;
    }
}

static const HDY_MotionSegment* HDY_ResolveStartSourceSegment(const HDY_MotionControlFB* fb,
                                                              size_t requestedSegmentIndex,
                                                              size_t* resolvedSegmentIndex,
                                                              HDY_SegmentSource* resolvedSource) {
    return HDY_MotionValidator_ResolveStartSourceSegment(fb, requestedSegmentIndex,
                                                          resolvedSegmentIndex, resolvedSource);
}

static HDY_BOOL HDY_IsCommandAllowedInState(HDY_FbCommand command, HDY_FbState state) {
    HDY_FbStateMask mask;

    if ((HDY_UINT)state >= (sizeof(HDY_FbStateMask) * 8U)) {
        return false;
    }

    if ((HDY_UINT)command >= (sizeof(HDY_COMMAND_ALLOWED_STATE_MASKS) / sizeof(HDY_COMMAND_ALLOWED_STATE_MASKS[0]))) {
        return false;
    }

    mask = HDY_COMMAND_ALLOWED_STATE_MASKS[(HDY_UINT)command];
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
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_ReportDiagnostic(fb,
                                       HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                       HDY_DIAG_SEVERITY_WARNING,
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

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
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
        HDY_StateReporter_ReportDiagnostic(fb,
                                           HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HDY_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           HDY_ResolveCommandDiagnosticSegment(fb, command, segmentIndex),
                                           references);
        return false;
    }

    return HDY_QueuePendingCommand(fb, command, segmentIndex, timestamp);
}

/* Removed HDY_AxisRefIsValid - now using HDY_MotionUtils_AxisRefIsValid directly */

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
    HDY_StateReporter_ClearSegmentTag(fb);
}

/* Diagnostic reporting moved into StateReporter (see state_reporter.c). */

static HDY_BOOL HDY_ValidateNextRequest(const HDY_MotionControlFB* fb,
                                        HDY_DiagnosticCode* code) {
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HDY_MotionValidator_UsesRecipeSource(fb)) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (fb->RECIPE_SIZE == 0U) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_NO_RECIPE;
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_DONE) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED;
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_STARTING || effectiveState == HDY_FB_STATE_RUNNING) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    if (!HDY_IsCommandAllowedInState(HDY_CMD_NEXT, effectiveState)) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
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

    fb->_lastFeedbackTimestamp = timestamp;
    HDY_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp);

    trackingFlowReference = HDY_MotionUtils_AbsReal(fb->AXIS_REF.flow);
    if (trackingFlowReference <= 0.0 && allowFlowCarryover) {
        trackingFlowReference = fb->_lastCommandedFlow;
    }

    initialPressureControlOutput = segment->targetFlow;
    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP && trackingFlowReference > 0.0) {
        initialPressureControlOutput = trackingFlowReference;
    }
    initialPressureControlOutput = HDY_MotionUtils_MinReal(initialPressureControlOutput, segment->maxFlow);
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
    HDY_BOOL preserveFlowCarryover;
    const HDY_MotionSegment* sourceSegment;
    size_t resolvedSegmentIndex;
    HDY_SegmentSource resolvedSource;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_MotionValidator_ValidateStartRequest(fb, segmentIndex, &code)) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ResetReadyContextPreview(fb);
        HDY_StateReporter_ReportDiagnostic(fb,
                           code,
                           HDY_DIAG_SEVERITY_WARNING,
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
        HDY_StateReporter_ReportFault(fb,
                          HDY_DIAG_CODE_INTERNAL_ERROR,
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

    /* Reset diagnostic criteria layer for new segment */
    HDY_ErrorMonitor_Reset(&fb->_errorMonitor);
    HDY_DiagnosticCriteria_ResetState(&fb->_pressureCriteriaState);
    HDY_DiagnosticCriteria_ResetState(&fb->_flowCriteriaState);
    HDY_DiagnosticCriteria_ResetState(&fb->_velocityCriteriaState);
    HDY_DiagnosticCriteria_ResetState(&fb->_positionCriteriaState);
    HDY_DiagnosticCriteria_ResetState(&fb->_timeoutCriteriaState);
    fb->_isSwitchPhase = true;

    /* Configure criteria thresholds for this segment (once per segment, not every cycle).
     * The default suppress/debounce/hysteresis/escalation settings from
     * CreateDefaultXxxCriteria (Init) are preserved; only the threshold and
     * diagnostic code are overridden based on segment tolerances. */
    {
        HDY_REAL pTol = HDY_Segment_GetPressureTolerance(sourceSegment);
        HDY_REAL fTol = HDY_Segment_GetFlowTolerance(sourceSegment);
        HDY_REAL vTol = HDY_Segment_GetVelocityTolerance(sourceSegment);
        HDY_REAL posTol = HDY_Segment_GetPositionTolerance(sourceSegment);
        HDY_TIME tLim = HDY_Segment_GetTimeoutLimit(sourceSegment);

        if (sourceSegment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP && pTol > 0.0) {
            HDY_ConfigureSegmentCriteria(&fb->_pressureCriteria, pTol,
                                          HDY_DIAG_CODE_OVER_PRESSURE,
                                          HDY_PROTECTION_ACTION_DERATE);
        }
        if (fTol > 0.0) {
            HDY_ConfigureSegmentCriteria(&fb->_flowCriteria, fTol,
                                          HDY_DIAG_CODE_FLOW_DEVIATION,
                                          HDY_PROTECTION_ACTION_DERATE);
        }
        if (vTol > 0.0) {
            HDY_ConfigureSegmentCriteria(&fb->_velocityCriteria, vTol,
                                          HDY_DIAG_CODE_VELOCITY_DEVIATION,
                                          HDY_PROTECTION_ACTION_WARNING);
        }
        if (posTol > 0.0) {
            HDY_ConfigureSegmentCriteria(&fb->_positionCriteria, posTol,
                                          HDY_DIAG_CODE_POSITION_DEVIATION,
                                          HDY_PROTECTION_ACTION_WARNING);
        }
        if (tLim > 0.0) {
            HDY_ConfigureSegmentCriteria(&fb->_timeoutCriteria, tLim,
                                          HDY_DIAG_CODE_TIMEOUT,
                                          HDY_PROTECTION_ACTION_STOP);
            fb->_timeoutCriteria.severity = HDY_DIAG_SEVERITY_FAULT;
            /* Clamp startup/switch suppress times so they never exceed the
             * timeout limit itself — a timeout that is fully suppressed by
             * its own startup window would be undetectable. */
            if (fb->_timeoutCriteria.startupSuppressTime >= tLim) {
                fb->_timeoutCriteria.startupSuppressTime = tLim * 0.5;
            }
            if (fb->_timeoutCriteria.switchSuppressTime >= tLim) {
                fb->_timeoutCriteria.switchSuppressTime = 0.0;
            }
        }
    }

    /* Enable startup suppress for first 500ms, then switch suppress for next 300ms.
     * _switchSuppressEndTime uses the pressure criteria's startup+switch window
     * as a unified transition period; all criteria share this end time.
     * Must be computed AFTER HDY_ConfigureSegmentCriteria so that it reflects
     * the current segment's criteria, not the previous segment's. */
    fb->_switchSuppressEndTime = fb->_pressureCriteria.startupSuppressTime +
                                  fb->_pressureCriteria.switchSuppressTime;

    HDY_PrimeSegmentControllers(fb, &fb->_activeSegment, timestamp, preserveFlowCarryover);

    HDY_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HDY_StateReporter_SetSegmentSource(fb, resolvedSource);
    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFinished(fb, false);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
    HDY_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_STARTING);
    fb->_executionId++;
    return true;
}

static HDY_BOOL HDY_AdvanceToNextSegment(HDY_MotionControlFB* fb,
                                         HDY_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    if (!HDY_MotionValidator_UsesRecipeSource(fb)) {
        HDY_StateReporter_ReportDiagnostic(fb,
                                           HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HDY_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           &fb->_activeSegment,
                                           &fb->STATE.references);
        return false;
    }

    if (fb->STATE.currentSegmentIndex + 1U < fb->RECIPE_SIZE) {
        return HDY_BeginSegment(fb, fb->STATE.currentSegmentIndex + 1U, timestamp);
    }

    HDY_ProtectionManager_ApplyIdleState(fb, true, true);
    HDY_StateReporter_ClearSegmentTag(fb);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_ResetReadyContextPreview(fb);
    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

static void HDY_EnterHoldNow(HDY_MotionControlFB* fb,
                             HDY_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    fb->_holdStateTime = timestamp;
    fb->_lastFeedbackTimestamp = timestamp;
    HDY_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
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
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return false;
    }

    if (!HDY_MotionUtils_AxisRefIsValid(&fb->AXIS_REF)) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_SENSOR_FAULT,
                        fb->AXIS_REF.timestamp,
                        &fb->_activeSegment,
                        &fb->STATE.references);
        return false;
    }

    if (fb->_lastFeedbackTimestamp >= 0.0 && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
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
    HDY_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
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
    /* Ensure outputs are forced to a safe state immediately on abort */
    HDY_StateReporter_ApplySafeOutputs(fb);
    fb->_holdStateTime = 0.0;
    fb->_lastCommandedFlow = 0.0;
    HDY_ResetReadyContextPreview(fb);
    HDY_StateReporter_ClearSegmentTag(fb);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_ABORTED);
    HDY_StateReporter_ReportDiagnostic(fb,
                                       HDY_DIAG_CODE_ABORTED,
                                       HDY_DIAG_SEVERITY_INFO,
                                       timestamp,
                                       NULL,
                                       &fb->STATE.references);
    fb->_executionId++;
}

static void HDY_MaintainNonExecutingState(HDY_MotionControlFB* fb,
                                          HDY_BOOL autoClearLiveDiagnostic) {
    HDY_FbState preservedState;

    if (fb == NULL) {
        return;
    }

    if (fb->STATE.finished) {
        preservedState = fb->FB_STATE;
        HDY_ProtectionManager_ApplyIdleState(fb, true, fb->SEGMENT_COMPLETED);
        if (preservedState == HDY_FB_STATE_ABORTED) {
            HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_ABORTED);
        }
        if (autoClearLiveDiagnostic) {
            HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    if (fb->SEGMENT_COMPLETED) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, true);
        if (autoClearLiveDiagnostic) {
            HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    HDY_ProtectionManager_ApplyIdleState(fb, false, false);
    HDY_ResetReadyContextPreview(fb);
    if (autoClearLiveDiagnostic) {
        HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
    }
}

static void HDY_MaintainPausedHoldState(HDY_MotionControlFB* fb,
                                        HDY_BOOL autoClearLiveDiagnostic) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    HDY_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HDY_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HDY_StateReporter_SetHoldState(fb);
    if (autoClearLiveDiagnostic) {
        HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
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

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
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

    if (fb->STATE.faultActive) {
        HDY_ProtectionManager_ApplyFaultHold(fb);
        fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
        fb->_segmentChangedFlag = false;
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

    fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
        fb->_segmentChangedFlag = false;
}

/*
 * Configure a single HDY_DiagnosticCriteria from segment tolerances.
 * Called once per segment start (HDY_BeginSegment), not every cycle.
 * This preserves debounce/hysteresis/fault-escalation state across cycles
 * while keeping the criteria configuration stable for the segment lifetime.
 */
static void HDY_ConfigureSegmentCriteria(HDY_DiagnosticCriteria* criteria,
                                          HDY_REAL baseThreshold,
                                          HDY_DiagnosticCode code,
                                          HDY_ProtectionAction action) {
    if (criteria == NULL) {
        return;
    }

    /* Preserve existing suppress/escalation/debounce/hysteresis defaults that
     * were set by CreateDefaultXxxCriteria during Init. Only override the
     * fields that must adapt to the current segment's tolerances. */
    criteria->baseThreshold = baseThreshold;
    criteria->diagnosticCode = code;
    criteria->severity = HDY_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = action;
}

static void HDY_UpdateMonitorPositionError(HDY_ErrorMonitor* monitor,
                                           const HDY_MotionSegment* segment,
                                           const HDY_AxisRef* axisRef,
                                           HDY_TIME currentTime) {
    HDY_BOOL wasActive;
    HDY_REAL positionTolerance;

    if (monitor == NULL || segment == NULL || axisRef == NULL) {
        return;
    }

    positionTolerance = HDY_Segment_GetPositionTolerance(segment);
    wasActive = monitor->positionErrorActive;
    monitor->positionError = segment->targetPosition - axisRef->position;

    /* Position error is "active" only when it exceeds the configured tolerance.
     * Comparing against zero would make positionErrorActive almost always true
     * due to floating-point representation, causing false duration accumulation. */
    monitor->positionErrorActive =
        (positionTolerance > 0.0 && fabs(monitor->positionError) > positionTolerance) ? true : false;

    if (!monitor->positionErrorActive) {
        monitor->positionErrorDuration = 0.0;
        return;
    }

    if (!wasActive) {
        monitor->positionErrorStartTime = currentTime;
        monitor->positionErrorDuration = 0.0;
        return;
    }

    monitor->positionErrorDuration = currentTime - monitor->positionErrorStartTime;
}

static void HDY_ExecuteActiveSegmentControl(HDY_MotionControlFB* fb,
                                            const HDY_MotionSegment* segment,
                                            HDY_REAL elapsed,
                                            HDY_RampControllerOutput* rampOutput,
                                            HDY_MotionPlannerOutput* plannerOutput,
                                            HDY_PressureControllerOutput* pressureOutput,
                                            HDY_PumpConverterOutput* pumpOutput,
                                            HDY_ExecutionReference* executionReference) {
    HDY_RampControllerInput rampInput;
    HDY_MotionPlannerInput plannerInput;
    HDY_PressureControllerInput pressureInput;
    HDY_PumpConverterInput pumpInput;

    if (fb == NULL || segment == NULL || rampOutput == NULL || plannerOutput == NULL ||
        pressureOutput == NULL || pumpOutput == NULL || executionReference == NULL) {
        return;
    }

    rampInput.targetPressure = segment->targetPressure;
    rampInput.rampRate = segment->pressureRampRate;
    rampInput.currentTime = fb->AXIS_REF.timestamp;
    HDY_RampController_Execute(&fb->_rampController, &rampInput, rampOutput);

    memset(plannerOutput, 0, sizeof(*plannerOutput));
    memset(pressureOutput, 0, sizeof(*pressureOutput));
    plannerOutput->direction = HDY_DIRECTION_HOLD;
    pressureOutput->appliedStrategy = HDY_PRESSURE_CONTROLLER_NONE;

    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
        pressureInput.targetPressure = rampOutput->rampedPressure;
        pressureInput.measuredPressure = fb->AXIS_REF.pressure;
        pressureInput.feedforwardFlow = segment->targetFlow;
        pressureInput.outputMin = 0.0;
        pressureInput.outputMax = segment->maxFlow;
        pressureInput.timestamp = fb->AXIS_REF.timestamp;
        HDY_PressureController_Execute(segment,
                                       &fb->_pressureController,
                                       &pressureInput,
                                       pressureOutput);
        plannerOutput->targetFlow = pressureOutput->outputFlow;
        plannerOutput->direction = segment->direction;
    } else {
        plannerInput.axisRef = &fb->AXIS_REF;
        plannerInput.segment = segment;
        plannerInput.elapsedTime = elapsed;
        plannerInput.rampedPressure = rampOutput->rampedPressure;
        HDY_MotionPlanner_Execute(&plannerInput, plannerOutput);
    }

    pumpInput.requestedFlow = plannerOutput->targetFlow;
    pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    pumpInput.direction = plannerOutput->direction;
    HDY_PumpConverter_Execute(&pumpInput, pumpOutput);

    executionReference->elapsedTime = elapsed;
    executionReference->pressureReference = (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP)
        ? rampOutput->rampedPressure
        : segment->targetPressure;
    executionReference->flowReference = pumpOutput->commandFlow;
    executionReference->velocityReference = plannerOutput->targetVelocity;
}

static void HDY_UpdateExecutionDiagnostics(HDY_MotionControlFB* fb,
                                           const HDY_MotionSegment* segment,
                                           const HDY_ExecutionReference* executionReference,
                                           HDY_REAL elapsed) {
    HDY_REAL pressureTolerance;
    HDY_REAL flowTolerance;
    HDY_REAL positionTolerance;
    HDY_REAL velocityTolerance;
    HDY_REAL velocityReferenceAbs;
    HDY_REAL actualVelocityAbs;
    HDY_DiagnosticResult pressureResult;
    HDY_DiagnosticResult flowResult;
    HDY_DiagnosticResult velocityResult;
    HDY_DiagnosticResult positionResult;
    HDY_DiagnosticCode priorityCode = HDY_DIAG_CODE_NONE;
    HDY_DiagnosticSeverity prioritySeverity = HDY_DIAG_SEVERITY_NONE;
    HDY_BOOL overPressure = false;
    HDY_BOOL underPressure = false;
    HDY_BOOL flowDeviation = false;
    HDY_BOOL positionDeviation = false;
    HDY_BOOL velocityDeviation = false;
    HDY_BOOL timeout = false;
    HDY_REAL pressureError = 0.0;
    HDY_REAL flowError = 0.0;
    HDY_REAL velocityError = 0.0;

    /* Derive suppress flags from segment elapsed time and criteria configuration.
     * isStartupPhase: determined per-criteria from its own startupSuppressTime.
     * isSwitchPhase: shared flag cleared after startup suppress window elapses. */
    HDY_BOOL isStartupPhase = false;  /* Computed per-criteria below */
    HDY_BOOL isSwitchPhase = false;

    if (fb == NULL || segment == NULL || executionReference == NULL) {
        return;
    }

    pressureTolerance = HDY_Segment_GetPressureTolerance(segment);
    flowTolerance = HDY_Segment_GetFlowTolerance(segment);
    positionTolerance = HDY_Segment_GetPositionTolerance(segment);
    velocityTolerance = HDY_Segment_GetVelocityTolerance(segment);

    /* Determine suppress phases from each criteria's own configuration and elapsed time.
     * Each diagnostic channel independently determines its startup phase based on
     * its own startupSuppressTime. Switch phase is shared across all channels. */
    isSwitchPhase = fb->_isSwitchPhase;

    HDY_ErrorMonitor_Update(&fb->_errorMonitor,
                            &fb->AXIS_REF,
                            executionReference,
                            fb->AXIS_REF.timestamp);
    HDY_UpdateMonitorPositionError(&fb->_errorMonitor,
                                   segment,
                                   &fb->AXIS_REF,
                                   fb->AXIS_REF.timestamp);
    pressureError = fb->_errorMonitor.pressureError;
    flowError = fb->_errorMonitor.flowError;
    velocityError = fb->_errorMonitor.velocityError;

    /* --- Pressure diagnostics --- */
    /* Criteria was configured once in HDY_BeginSegment; only check here. */
    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP && pressureTolerance > 0.0) {
        isStartupPhase = HDY_IsStartupSuppressActive(elapsed, fb->_pressureCriteria.startupSuppressTime);
        if (HDY_DiagnosticCriteria_CheckPressure(&pressureResult,
                                                 &fb->_errorMonitor,
                                                 &fb->_pressureCriteria,
                                                 &fb->_pressureCriteriaState,
                                                 fb->AXIS_REF.timestamp,
                                                 elapsed,
                                                 fb->_pressureCriteria.enableStartupSuppress && isStartupPhase,
                                                 isSwitchPhase)) {
            pressureError = fb->_errorMonitor.pressureError;
            if (pressureError < -pressureTolerance) {
                overPressure = true;
            } else if (pressureError > pressureTolerance) {
                underPressure = true;
            }
            /* Check fault escalation: WARNING -> FAULT after sustained duration */
            HDY_DiagnosticCriteria_CheckFaultEscalation(&pressureResult,
                                                         &fb->_pressureCriteria,
                                                         &fb->_pressureCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (pressureResult.severity == HDY_DIAG_SEVERITY_FAULT) {
                overPressure = true;  /* Ensure fault is propagated */
            }
        }
    } else {
        HDY_DiagnosticCriteria_ResetState(&fb->_pressureCriteriaState);
    }

    /* --- Flow diagnostics --- */
    /* Removed hardcoded elapsed > 0.1 gate; the criteria layer's
     * startup/switch suppress mechanism handles transient suppression. */
    if (flowTolerance > 0.0 && executionReference->flowReference > 0.0) {
        isStartupPhase = HDY_IsStartupSuppressActive(elapsed, fb->_flowCriteria.startupSuppressTime);
        if (HDY_DiagnosticCriteria_CheckFlow(&flowResult,
                                             &fb->_errorMonitor,
                                             &fb->_flowCriteria,
                                             &fb->_flowCriteriaState,
                                             fb->AXIS_REF.timestamp,
                                             elapsed,
                                             fb->_flowCriteria.enableStartupSuppress && isStartupPhase,
                                             isSwitchPhase)) {
            flowDeviation = true;
            flowError = fb->_errorMonitor.flowError;
            /* Check fault escalation: WARNING -> FAULT after sustained duration */
            HDY_DiagnosticCriteria_CheckFaultEscalation(&flowResult,
                                                         &fb->_flowCriteria,
                                                         &fb->_flowCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (flowResult.severity == HDY_DIAG_SEVERITY_FAULT) {
                flowDeviation = true;
            }
        }
    } else {
        HDY_DiagnosticCriteria_ResetState(&fb->_flowCriteriaState);
    }

    /* --- Velocity diagnostics --- */
    /* No longer unconditionally reset; check when velocity tolerance is configured. */
    if (velocityTolerance > 0.0) {
        isStartupPhase = HDY_IsStartupSuppressActive(elapsed, fb->_velocityCriteria.startupSuppressTime);
        if (HDY_DiagnosticCriteria_CheckVelocity(&velocityResult,
                                                   &fb->_errorMonitor,
                                                   &fb->_velocityCriteria,
                                                   &fb->_velocityCriteriaState,
                                                   fb->AXIS_REF.timestamp,
                                                   elapsed,
                                                   fb->_velocityCriteria.enableStartupSuppress && isStartupPhase,
                                                   isSwitchPhase)) {
            velocityDeviation = true;
            velocityError = fb->_errorMonitor.velocityError;
            /* Check fault escalation */
            HDY_DiagnosticCriteria_CheckFaultEscalation(&velocityResult,
                                                         &fb->_velocityCriteria,
                                                         &fb->_velocityCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (velocityResult.severity == HDY_DIAG_SEVERITY_FAULT) {
                velocityDeviation = true;
            }
        }
    } else {
        HDY_DiagnosticCriteria_ResetState(&fb->_velocityCriteriaState);
    }

    /* --- Position diagnostics --- */
    velocityReferenceAbs = fabs(executionReference->velocityReference);
    actualVelocityAbs = fabs(fb->AXIS_REF.velocity);
    /* Position deviation is only meaningful when the actuator is near the
     * target (both reference and actual velocity are near-zero). During
     * motion the position error equals remaining travel, not a fault.
     * The velocityTolerance gate ensures we only check position when
     * the axis has settled, making the diagnostic meaningful.
     * The velocityTolerance > 0 requirement was removed - if no velocity
     * tolerance is configured, the check is still gated by actual velocity
     * being below the reference velocity threshold.
     * Apply to all HDY_MODE_POSITION segments (not just HDY_END_POSITION),
     * because position tracking deviation is relevant whenever the planner
     * is driving toward a target position, regardless of end condition. */
    if (segment->mode == HDY_MODE_POSITION &&
        positionTolerance > 0.0 &&
        velocityReferenceAbs < (velocityTolerance > 0.0 ? velocityTolerance : 1.0) &&
        actualVelocityAbs < (velocityTolerance > 0.0 ? velocityTolerance : 1.0)) {
        isStartupPhase = HDY_IsStartupSuppressActive(elapsed, fb->_positionCriteria.startupSuppressTime);
        if (HDY_DiagnosticCriteria_CheckPosition(&positionResult,
                                                 &fb->_errorMonitor,
                                                 &fb->_positionCriteria,
                                                 &fb->_positionCriteriaState,
                                                 fb->AXIS_REF.timestamp,
                                                 elapsed,
                                                 fb->_positionCriteria.enableStartupSuppress && isStartupPhase,
                                                 isSwitchPhase)) {
            positionDeviation = true;
            /* Check fault escalation */
            HDY_DiagnosticCriteria_CheckFaultEscalation(&positionResult,
                                                         &fb->_positionCriteria,
                                                         &fb->_positionCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (positionResult.severity == HDY_DIAG_SEVERITY_FAULT) {
                positionDeviation = true;
            }
        }
    } else {
        HDY_DiagnosticCriteria_ResetState(&fb->_positionCriteriaState);
    }

    /* --- Timeout diagnostics ---
     * Uses criteria layer for consistency with startup/switch suppress,
     * debounce, and unified diagnostic channel semantics. */
    {
        HDY_DiagnosticResult timeoutResult;
        HDY_BOOL isStartupPhaseTimeout = HDY_IsStartupSuppressActive(elapsed, fb->_timeoutCriteria.startupSuppressTime);
        if (HDY_DiagnosticCriteria_CheckTimeout(&timeoutResult,
                                                  &fb->_timeoutCriteria,
                                                  &fb->_timeoutCriteriaState,
                                                  fb->AXIS_REF.timestamp,
                                                  elapsed,
                                                  fb->_timeoutCriteria.enableStartupSuppress && isStartupPhaseTimeout,
                                                  isSwitchPhase)) {
            timeout = true;
        }
    }

    /* Build priority diagnostic from active conditions.
     * Fault-escalated results upgrade the severity from WARNING to FAULT. */
    if (timeout) {
        priorityCode = HDY_DIAG_CODE_TIMEOUT;
        prioritySeverity = HDY_DIAG_SEVERITY_FAULT;
    } else if (overPressure) {
        priorityCode = HDY_DIAG_CODE_OVER_PRESSURE;
        prioritySeverity = (pressureResult.severity == HDY_DIAG_SEVERITY_FAULT)
            ? HDY_DIAG_SEVERITY_FAULT : HDY_DIAG_SEVERITY_WARNING;
    } else if (underPressure) {
        priorityCode = HDY_DIAG_CODE_UNDER_PRESSURE;
        prioritySeverity = (pressureResult.severity == HDY_DIAG_SEVERITY_FAULT)
            ? HDY_DIAG_SEVERITY_FAULT : HDY_DIAG_SEVERITY_WARNING;
    } else if (flowDeviation) {
        priorityCode = HDY_DIAG_CODE_FLOW_DEVIATION;
        prioritySeverity = (flowResult.severity == HDY_DIAG_SEVERITY_FAULT)
            ? HDY_DIAG_SEVERITY_FAULT : HDY_DIAG_SEVERITY_WARNING;
    } else if (positionDeviation) {
        priorityCode = HDY_DIAG_CODE_POSITION_DEVIATION;
        prioritySeverity = (positionResult.severity == HDY_DIAG_SEVERITY_FAULT)
            ? HDY_DIAG_SEVERITY_FAULT : HDY_DIAG_SEVERITY_WARNING;
    } else if (velocityDeviation) {
        priorityCode = HDY_DIAG_CODE_VELOCITY_DEVIATION;
        prioritySeverity = (velocityResult.severity == HDY_DIAG_SEVERITY_FAULT)
            ? HDY_DIAG_SEVERITY_FAULT : HDY_DIAG_SEVERITY_WARNING;
    }

    if (priorityCode != HDY_DIAG_CODE_NONE) {
        HDY_Diagnostics_SetEvent(&fb->DIAGNOSTIC, priorityCode, prioritySeverity);
    } else {
        HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    }

    fb->DIAGNOSTIC.overPressure = overPressure;
    fb->DIAGNOSTIC.underPressure = underPressure;
    fb->DIAGNOSTIC.flowDeviation = flowDeviation;
    fb->DIAGNOSTIC.positionDeviation = positionDeviation;
    fb->DIAGNOSTIC.velocityDeviation = velocityDeviation;
    fb->DIAGNOSTIC.timeout = timeout;
    fb->DIAGNOSTIC.pressureError = pressureError;
    fb->DIAGNOSTIC.flowError = flowError;
    fb->DIAGNOSTIC.velocityError = velocityError;
    fb->DIAGNOSTIC.flags = HDY_Diagnostics_GetFlagMask(&fb->DIAGNOSTIC);

    /* Clear switch phase using the unified transition window end time,
     * not tied to any single criteria's startup suppress time. */
    if (fb->_isSwitchPhase && fb->_switchSuppressEndTime > 0.0 && elapsed >= fb->_switchSuppressEndTime) {
        fb->_isSwitchPhase = false;
    }
}

static void HDY_MotionControlFB_RunRunningState(HDY_MotionControlFB* fb) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    const HDY_MotionSegment* segment;
    HDY_REAL elapsed;
    HDY_RampControllerOutput rampOutput;
    HDY_MotionPlannerOutput plannerOutput;
    HDY_PressureControllerOutput pressureOutput;
    HDY_PumpConverterOutput pumpOutput;
    HDY_ExecutionReference executionReference;
    HDY_SegmentCompletionContext completionContext;
    HDY_BOOL segmentCompleted;
    HDY_BOOL recipeFinished;
    HDY_SegmentSource completedSegmentSource;

    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
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
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_INTERNAL_ERROR,
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    if (!HDY_MotionUtils_AxisRefIsValid(&fb->AXIS_REF)) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_SENSOR_FAULT,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }

    if (fb->_lastFeedbackTimestamp >= 0.0 && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HDY_StateReporter_ReportFault(fb,
                        HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }
    fb->_lastFeedbackTimestamp = fb->AXIS_REF.timestamp;

    if (!HDY_PumpConverter_ValidateConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                          fb->PUMP_SPEED_LIMIT,
                                          &code) ||
        !HDY_RecipeValidator_ValidateSegment(segment,
                                             fb->STATE.currentSegmentIndex,
                                             &code)) {
        HDY_StateReporter_ReportFault(fb,
                        code,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }

    elapsed = fb->AXIS_REF.timestamp - fb->_segmentStartTime;
    if (elapsed < 0.0) {
        elapsed = 0.0;
    }

    HDY_ExecuteActiveSegmentControl(fb,
                                    segment,
                                    elapsed,
                                    &rampOutput,
                                    &plannerOutput,
                                    &pressureOutput,
                                    &pumpOutput,
                                    &executionReference);
    HDY_UpdateExecutionDiagnostics(fb, segment, &executionReference, elapsed);

    fb->_lastCommandedFlow = pumpOutput.commandFlow;

    if (fb->DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP) {
        HDY_ProtectionManager_EnterFaultStop(fb);
        HDY_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
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
        HDY_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
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
    HDY_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);

    fb->_simFeedback.targetPosition = segment->targetPosition;
    fb->_simFeedback.targetVelocity = plannerOutput.targetVelocity;
    fb->_simFeedback.targetFlow     = pumpOutput.commandFlow;
    fb->_simFeedback.targetPressure = executionReference.pressureReference;
    fb->_simFeedback.valid          = true;

    fb->SEGMENT_COMPLETED = false;
}

static HDY_BOOL HDY_RequestStartCommand(HDY_MotionControlFB* fb,
                                        size_t segmentIndex,
                                        HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    if (fb->STATE.faultActive) {
        return false;
    }

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(HDY_CMD_START, effectiveState)) {
        HDY_ReportCommandNotAllowed(fb,
                                    HDY_CMD_START,
                                    effectiveState,
                                    timestamp,
                                    (HDY_UINT)segmentIndex,
                                    &fb->STATE.references);
        return false;
    }

    if (!HDY_MotionValidator_ValidateStartRequest(fb, segmentIndex, &code)) {
        HDY_StateReporter_ReportDiagnostic(fb,
                           code,
                           HDY_DIAG_SEVERITY_WARNING,
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
    if (fb == NULL || fb->STATE.faultActive) {
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
    if (fb == NULL || fb->STATE.faultActive) {
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

    if (fb == NULL || fb->STATE.faultActive) {
        return false;
    }

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
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

    if (startEdge) {
        (void)HDY_RequestStartCommand(fb, startSegmentIndex, fb->AXIS_REF.timestamp);
    }
}

void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(fb, 0, sizeof(*fb));
    fb->_lastFeedbackTimestamp = -1.0;  /* sentinel: not yet valid */
    fb->USE_RECIPE = false;
    fb->FB_STATE = HDY_FB_STATE_IDLE;
    fb->STATE.currentSegmentIndex = HDY_MAX_SEGMENTS;
    fb->_activeSegmentSource = HDY_SEGMENT_SOURCE_NONE;
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_ClearSegmentTag(fb);
    HDY_StateReporter_ResetDiagnosticRetention(fb);
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
    HDY_PressureController_ClearState(&fb->_pressureController);
    HDY_ClearPendingCommand(fb);
    HDY_StateReporter_RefreshStandardOutputs(fb);

    /* Initialize diagnostic criteria layer */
    HDY_ErrorMonitor_Init(&fb->_errorMonitor);
    HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&fb->_pressureCriteria);
    HDY_DiagnosticCriteria_InitState(&fb->_pressureCriteriaState);
    HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(&fb->_flowCriteria);
    HDY_DiagnosticCriteria_InitState(&fb->_flowCriteriaState);
    HDY_DiagnosticCriteria_CreateDefaultVelocityCriteria(&fb->_velocityCriteria);
    HDY_DiagnosticCriteria_InitState(&fb->_velocityCriteriaState);
    HDY_DiagnosticCriteria_CreateDefaultPositionCriteria(&fb->_positionCriteria);
    HDY_DiagnosticCriteria_InitState(&fb->_positionCriteriaState);
    HDY_DiagnosticCriteria_CreateDefaultTimeoutCriteria(&fb->_timeoutCriteria);
    HDY_DiagnosticCriteria_InitState(&fb->_timeoutCriteriaState);
    fb->_isSwitchPhase = false;
    fb->_switchSuppressEndTime = 0.0;
}

void HDY_MotionControlFB_SoftReset(HDY_MotionControlFB* fb) {
    /* Persistent fields saved across soft reset */
    HDY_MotionSegment savedRecipe[HDY_MAX_SEGMENTS];
    HDY_UINT savedRecipeSize;
    HDY_MotionSegment savedDirectSegment;
    HDY_BOOL savedDirectSegmentValid;
    HDY_REAL savedFlowToPumpSpeedGain;
    HDY_REAL savedPumpSpeedLimit;
    HDY_BOOL savedUseRecipe;
    HDY_DiagnosticCriteria savedPressureCriteria;
    HDY_DiagnosticCriteria savedFlowCriteria;
    HDY_DiagnosticCriteria savedVelocityCriteria;
    HDY_DiagnosticCriteria savedPositionCriteria;

    if (fb == NULL) {
        return;
    }

    /* 1. Save persistent configuration */
    (void)memcpy(savedRecipe, fb->RECIPE, sizeof(savedRecipe));
    savedRecipeSize = fb->RECIPE_SIZE;
    savedDirectSegment = fb->DIRECT_SEGMENT;
    savedDirectSegmentValid = fb->DIRECT_SEGMENT_VALID;
    savedFlowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    savedPumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    savedUseRecipe = fb->USE_RECIPE;
    savedPressureCriteria = fb->_pressureCriteria;
    savedFlowCriteria = fb->_flowCriteria;
    savedVelocityCriteria = fb->_velocityCriteria;
    savedPositionCriteria = fb->_positionCriteria;

    /* 2. Full memset to clear all runtime state cleanly */
    (void)memset(fb, 0, sizeof(*fb));
    fb->_lastFeedbackTimestamp = -1.0;  /* sentinel: not yet valid */

    /* 3. Restore persistent configuration */
    (void)memcpy(fb->RECIPE, savedRecipe, sizeof(fb->RECIPE));
    fb->RECIPE_SIZE = savedRecipeSize;
    fb->DIRECT_SEGMENT = savedDirectSegment;
    fb->DIRECT_SEGMENT_VALID = savedDirectSegmentValid;
    fb->FLOW_TO_PUMP_SPEED_GAIN = savedFlowToPumpSpeedGain;
    fb->PUMP_SPEED_LIMIT = savedPumpSpeedLimit;
    fb->USE_RECIPE = savedUseRecipe;
    fb->_pressureCriteria = savedPressureCriteria;
    fb->_flowCriteria = savedFlowCriteria;
    fb->_velocityCriteria = savedVelocityCriteria;
    fb->_positionCriteria = savedPositionCriteria;

    /* 4. Reinitialize framework-level state (same as Init) */
    fb->_activeSegmentSource = HDY_SEGMENT_SOURCE_NONE;
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_ClearSegmentTag(fb);
    HDY_StateReporter_ResetDiagnosticRetention(fb);
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
    HDY_PressureController_ClearState(&fb->_pressureController);
    HDY_ClearPendingCommand(fb);

    /* 5. Reinitialize diagnostic criteria states (but keep the criteria configs) */
    HDY_ErrorMonitor_Init(&fb->_errorMonitor);
    HDY_DiagnosticCriteria_InitState(&fb->_pressureCriteriaState);
    HDY_DiagnosticCriteria_InitState(&fb->_flowCriteriaState);
    HDY_DiagnosticCriteria_InitState(&fb->_velocityCriteriaState);
    HDY_DiagnosticCriteria_InitState(&fb->_positionCriteriaState);
    HDY_DiagnosticCriteria_InitState(&fb->_timeoutCriteriaState);
    fb->_isSwitchPhase = false;
    fb->_switchSuppressEndTime = 0.0;

    /* 6. Set state to READY (if recipe or direct segment is loaded) or IDLE */
    HDY_ResetReadyContextPreview(fb);
    HDY_StateReporter_SetIdleState(fb, false, false);
    HDY_StateReporter_RefreshStandardOutputs(fb);
}

HDY_BOOL HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb,
                                        const HDY_MotionSegment* recipe,
                                        size_t recipeSize) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code)) {
        memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
        fb->RECIPE_SIZE = 0U;
        HDY_PrepareRecipeLoadState(fb);
        HDY_StateReporter_ReportDiagnostic(fb,
                           code,
                           HDY_DIAG_SEVERITY_WARNING,
                           fb->AXIS_REF.timestamp,
                           NULL,
                           NULL);
        return false;
    }

    memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
    memcpy(fb->RECIPE, recipe, recipeSize * sizeof(HDY_MotionSegment));
    fb->RECIPE_SIZE = recipeSize;
    HDY_PrepareRecipeLoadState(fb);
    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(HDY_MotionControlFB* fb,
                                              const HDY_MotionSegment* segment) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_RecipeValidator_ValidateSegment(segment,
                                             HDY_MAX_SEGMENTS,
                                             &code)) {
        memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
        fb->DIRECT_SEGMENT_VALID = false;
        if (!fb->STATE.active && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
            HDY_ResetReadyContextPreview(fb);
            HDY_StateReporter_SetIdleState(fb, false, false);
            HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
            HDY_StateReporter_ClearSegmentTag(fb);
        }
        HDY_StateReporter_ReportDiagnostic(fb,
                           code,
                           HDY_DIAG_SEVERITY_WARNING,
                           fb->AXIS_REF.timestamp,
                           NULL,
                           NULL);
        return false;
    }

    fb->DIRECT_SEGMENT = *segment;
    fb->DIRECT_SEGMENT_VALID = true;
    if (!fb->STATE.active && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
        HDY_ResetReadyContextPreview(fb);
        HDY_StateReporter_SetIdleState(fb, false, false);
    }
    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

void HDY_MotionControlFB_ClearDirectSegment(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
    fb->DIRECT_SEGMENT_VALID = false;
    if (!fb->STATE.active && fb->FB_STATE != HDY_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
        HDY_ResetReadyContextPreview(fb);
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_StateReporter_SetSegmentSource(fb, HDY_SEGMENT_SOURCE_NONE);
        HDY_StateReporter_ClearSegmentTag(fb);
    }
}

HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb,
                                          size_t segmentIndex,
                                          HDY_TIME timestamp) {
    return HDY_RequestStartCommand(fb, segmentIndex, timestamp);
}

HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (fb->STATE.faultActive) {
        return false;
    }

    if (!HDY_ValidateNextRequest(fb, &code)) {
        HDY_StateReporter_ReportDiagnostic(fb,
                                           code,
                                           (code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED)
                                               ? HDY_DIAG_SEVERITY_INFO
                                               : HDY_DIAG_SEVERITY_WARNING,
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

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HDY_IsCommandAllowedInState(HDY_CMD_ACK, effectiveState)) {
        return false;
    }

    /* Only allow acknowledge when there are no active faults to ensure
     * the operator has resolved the root cause before clearing diagnostics.
     * This safety check prevents accidental dismissal of critical faults. */
    if (fb->STATE.faultActive) {
        return false;
    }

    /* Clear both current live diagnostic and retained history only when
     * we're in a safe state (no active faults). The diagnostic code
     * field is cleared as part of this operation. */
    HDY_StateReporter_ClearCurrentDiagnostic(fb);
    HDY_StateReporter_ClearDiagnosticRetentionOnly(fb);
    return true;
}

void HDY_MotionControlFB_Cycle(HDY_MotionControlFB* fb) {
    HDY_FbCommand processedCommand;
    HDY_BOOL allowRunningExecution;

    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = false;

    if (fb->RESET) {
        HDY_MotionControlFB_SoftReset(fb);
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
