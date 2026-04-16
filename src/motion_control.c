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
#include <string.h>

static HDY_REAL HDY_MinReal(HDY_REAL left, HDY_REAL right) {
    return (left < right) ? left : right;
}

static HDY_REAL HDY_AbsReal(HDY_REAL value) {
    return (value < 0.0) ? -value : value;
}

static HDY_BOOL HDY_IsFiniteReal(HDY_REAL value) {
    return isfinite(value) ? true : false;
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

static void HDY_ClearPendingStartCommand(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->START_SEGMENT = false;
    fb->START_SEGMENT_INDEX = 0U;
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

    fb->STATE.currentSegmentIndex = 0U;
    fb->_segmentStartTime = 0.0;
    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    HDY_ClearPendingStartCommand(fb);
    HDY_StateReporter_SetIdleState(fb, false, false);
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

void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(fb, 0, sizeof(*fb));
    fb->ENO = true;
    HDY_StateReporter_SetPlannedDirection(fb, HDY_DIRECTION_HOLD);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_ResetDiagnosticRetention(fb);
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
    HDY_PressureController_ClearState(&fb->_pressureController);
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

HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb,
                                          size_t segmentIndex,
                                          HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};
    HDY_REAL initialPressureControlOutput;
    HDY_REAL trackingFlowReference;
    const HDY_MotionSegment* segment;

    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);

    if (fb->RECIPE_SIZE == 0U) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_NO_RECIPE,
                             HDY_DIAG_SEVERITY_WARNING,
                             "No recipe loaded",
                             timestamp,
                             NULL,
                             NULL);
        return false;
    }

    if (segmentIndex >= fb->RECIPE_SIZE) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
                             HDY_DIAG_SEVERITY_WARNING,
                             "Start segment index is out of range",
                             timestamp,
                             NULL,
                             NULL);
        return false;
    }

    if (!HDY_RecipeValidator_ValidateRuntimeConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                                   fb->PUMP_SPEED_LIMIT,
                                                   &code,
                                                   message,
                                                   sizeof(message)) ||
        !HDY_RecipeValidator_ValidateSegment(&fb->RECIPE[segmentIndex],
                                             segmentIndex,
                                             &code,
                                             message,
                                             sizeof(message)) ||
        !HDY_RecipeValidator_ValidateStartContext(&fb->RECIPE[segmentIndex],
                                                  segmentIndex,
                                                  &fb->AXIS_REF,
                                                  &code,
                                                  message,
                                                  sizeof(message))) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb,
                             code,
                             HDY_DIAG_SEVERITY_WARNING,
                             message,
                             timestamp,
                             &fb->RECIPE[segmentIndex],
                             NULL);
        return false;
    }

    segment = &fb->RECIPE[segmentIndex];
    fb->STATE.currentSegmentIndex = segmentIndex;
    fb->_segmentStartTime = timestamp;
    HDY_ProtectionManager_ApplyIdleState(fb, false, false);
    HDY_RecordFeedbackTimestamp(fb, timestamp);
    fb->_segmentChangedFlag = true;
    HDY_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp);

    trackingFlowReference = HDY_AbsReal(fb->AXIS_REF.flow);
    if (trackingFlowReference <= 0.0) {
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
    HDY_StateReporter_SetSegmentName(fb, segment->name);
    HDY_ClearCurrentDiagnostic(fb);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
    HDY_StateReporter_SetPlannedDirection(fb, segment->direction);
    return true;
}

HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);

    if (fb->RECIPE_SIZE == 0U) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_NO_RECIPE,
                             HDY_DIAG_SEVERITY_WARNING,
                             "No recipe loaded",
                             timestamp,
                             NULL,
                             NULL);
        return false;
    }

    if (fb->FINISHED) {
        HDY_ProtectionManager_ApplyIdleState(fb, true, true);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED,
                             HDY_DIAG_SEVERITY_INFO,
                             "Recipe is already finished",
                             timestamp,
                             NULL,
                             &fb->STATE.references);
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        HDY_StateReporter_ResetTransitionFlags(fb);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED,
                             HDY_DIAG_SEVERITY_WARNING,
                             "Current segment has not completed",
                             timestamp,
                             (fb->STATE.currentSegmentIndex < fb->RECIPE_SIZE)
                                 ? &fb->RECIPE[fb->STATE.currentSegmentIndex]
                                 : NULL,
                             &fb->STATE.references);
        return false;
    }

    if (fb->STATE.currentSegmentIndex + 1U < fb->RECIPE_SIZE) {
        return HDY_MotionControlFB_StartSegment(fb, fb->STATE.currentSegmentIndex + 1U, timestamp);
    }

    HDY_ProtectionManager_ApplyIdleState(fb, true, true);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_ClearCurrentDiagnostic(fb);
    return true;
}

HDY_BOOL HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);
    HDY_ProtectionManager_ApplyIdleState(fb, true, false);
    fb->_segmentStartTime = 0.0;
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_ReportDiagnostic(fb,
                         HDY_DIAG_CODE_ABORTED,
                         HDY_DIAG_SEVERITY_INFO,
                         "Aborted by caller",
                         fb->AXIS_REF.timestamp,
                         NULL,
                         &fb->STATE.references);
    return true;
}

HDY_BOOL HDY_MotionControlFB_AcknowledgeDiagnostics(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (fb->FAULT || fb->DIAGNOSTIC.code != HDY_DIAG_CODE_NONE) {
        return false;
    }

    HDY_ClearCurrentDiagnostic(fb);
    HDY_ClearDiagnosticRetentionOnly(fb);
    return true;
}

void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb) {
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

    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = false;

    if (!fb->EN) {
        fb->ENO = false;
        HDY_ClearPendingStartCommand(fb);
        HDY_ProtectionManager_ApplyDisabledState(fb);
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        return;
    }

    fb->ENO = true;
    if (fb->RESET) {
        HDY_MotionControlFB_Init(fb);
        return;
    }

    if (fb->START_SEGMENT) {
        (void)HDY_MotionControlFB_StartSegment(fb, fb->START_SEGMENT_INDEX, fb->AXIS_REF.timestamp);
    }

    fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
    fb->_segmentChangedFlag = false;

    if (fb->RECIPE_SIZE == 0U) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, false);
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        return;
    }

    if (fb->FINISHED) {
        HDY_ProtectionManager_ApplyIdleState(fb, true, fb->SEGMENT_COMPLETED);
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        return;
    }

    if (fb->FAULT) {
        HDY_ProtectionManager_ApplyFaultHold(fb);
        return;
    }

    if (!fb->ACTIVE) {
        HDY_ProtectionManager_ApplyIdleState(fb, false, fb->SEGMENT_COMPLETED);
        HDY_ClearLiveDiagnosticInNonFaultHold(fb);
        return;
    }

    if (fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) {
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
                        &fb->RECIPE[fb->STATE.currentSegmentIndex],
                        &fb->STATE.references);
        return;
    }

    if (fb->_feedbackTimestampValid && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HDY_ReportFault(fb,
                        HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
                        "Axis timestamp moved backwards",
                        fb->AXIS_REF.timestamp,
                        &fb->RECIPE[fb->STATE.currentSegmentIndex],
                        &fb->STATE.references);
        return;
    }
    HDY_RecordFeedbackTimestamp(fb, fb->AXIS_REF.timestamp);

    if (!HDY_RecipeValidator_ValidateRuntimeConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                                   fb->PUMP_SPEED_LIMIT,
                                                   &code,
                                                   message,
                                                   sizeof(message)) ||
        !HDY_RecipeValidator_ValidateSegment(&fb->RECIPE[fb->STATE.currentSegmentIndex],
                                             fb->STATE.currentSegmentIndex,
                                             &code,
                                             message,
                                             sizeof(message))) {
        HDY_ReportFault(fb,
                        code,
                        message,
                        fb->AXIS_REF.timestamp,
                        &fb->RECIPE[fb->STATE.currentSegmentIndex],
                        &fb->STATE.references);
        return;
    }

    segment = &fb->RECIPE[fb->STATE.currentSegmentIndex];
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
        recipeFinished = (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HDY_ProtectionManager_ApplyIdleState(fb, recipeFinished, true);
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
    HDY_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);
    fb->SEGMENT_COMPLETED = false;
}
