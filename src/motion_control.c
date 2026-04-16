#include "motion_control.h"
#include "diagnostics.h"
#include "motion_planner.h"
#include "pump_converter.h"
#include "ramp_controller.h"
#include "recipe_validator.h"
#include "segment_completion.h"
#include "state_reporter.h"
#include <string.h>

static void HDY_ClearPendingStartCommand(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->START_SEGMENT = false;
    fb->START_SEGMENT_INDEX = 0U;
}

static void HDY_PrepareRecipeLoadState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->STATE.currentSegmentIndex = 0U;
    fb->_segmentStartTime = 0.0;
    HDY_ClearPendingStartCommand(fb);
    HDY_StateReporter_SetIdleState(fb, false, false);
    HDY_StateReporter_ClearSegmentName(fb);
}

static void HDY_ReportDiagnostic(HDY_MotionControlFB* fb,
                                 HDY_DiagnosticCode code,
                                 HDY_DiagnosticSeverity severity,
                                 const char* message) {
    if (fb == NULL) {
        return;
    }

    HDY_Diagnostics_SetEvent(&fb->DIAGNOSTIC, code, severity, message);
}

static void HDY_ReportFault(HDY_MotionControlFB* fb,
                            HDY_DiagnosticCode code,
                            const char* message) {
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_EnterFaultState(fb);
    HDY_ReportDiagnostic(fb, code, HDY_DIAG_SEVERITY_FAULT, message);
}

static void HDY_UpdateDisabledState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_ApplySafeOutputs(fb);
    fb->SEGMENT_COMPLETED = false;

    if (fb->FAULT) {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
        return;
    }

    if (fb->FINISHED) {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FINISHED);
    } else if (fb->RECIPE_SIZE > 0U) {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_READY);
    } else {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
    }
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
    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    HDY_RampController_Init(&fb->_rampController, 0.0, 0.0);
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
        HDY_ReportDiagnostic(fb, code, HDY_DIAG_SEVERITY_WARNING, message);
        return false;
    }

    memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
    memcpy(fb->RECIPE, recipe, recipeSize * sizeof(HDY_MotionSegment));
    fb->RECIPE_SIZE = recipeSize;
    HDY_PrepareRecipeLoadState(fb);
    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    return true;
}

HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb,
                                          size_t segmentIndex,
                                          HDY_TIME timestamp) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);

    if (fb->RECIPE_SIZE == 0U) {
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb, HDY_DIAG_CODE_NO_RECIPE, HDY_DIAG_SEVERITY_WARNING, "No recipe loaded");
        return false;
    }

    if (segmentIndex >= fb->RECIPE_SIZE) {
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
                             HDY_DIAG_SEVERITY_WARNING,
                             "Start segment index is out of range");
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
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb, code, HDY_DIAG_SEVERITY_WARNING, message);
        return false;
    }

    fb->STATE.currentSegmentIndex = segmentIndex;
    fb->_segmentStartTime = timestamp;
    HDY_StateReporter_SetIdleState(fb, false, false);
    fb->_segmentChangedFlag = true;
    HDY_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp);
    HDY_StateReporter_SetSegmentName(fb, fb->RECIPE[segmentIndex].name);
    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    HDY_StateReporter_SetActive(fb, true);
    HDY_StateReporter_SetFault(fb, false);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_RUNNING);
    HDY_StateReporter_SetPlannedDirection(fb, fb->RECIPE[segmentIndex].direction);
    return true;
}

HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);

    if (fb->RECIPE_SIZE == 0U) {
        HDY_StateReporter_SetIdleState(fb, false, false);
        HDY_ReportDiagnostic(fb, HDY_DIAG_CODE_NO_RECIPE, HDY_DIAG_SEVERITY_WARNING, "No recipe loaded");
        return false;
    }

    if (fb->FINISHED) {
        HDY_StateReporter_SetIdleState(fb, true, true);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED,
                             HDY_DIAG_SEVERITY_INFO,
                             "Recipe is already finished");
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        HDY_StateReporter_ResetTransitionFlags(fb);
        HDY_ReportDiagnostic(fb,
                             HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED,
                             HDY_DIAG_SEVERITY_WARNING,
                             "Current segment has not completed");
        return false;
    }

    if (fb->STATE.currentSegmentIndex + 1U < fb->RECIPE_SIZE) {
        return HDY_MotionControlFB_StartSegment(fb, fb->STATE.currentSegmentIndex + 1U, timestamp);
    }

    HDY_StateReporter_SetIdleState(fb, true, true);
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_Diagnostics_Clear(&fb->DIAGNOSTIC);
    return true;
}

HDY_BOOL HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    HDY_ClearPendingStartCommand(fb);
    HDY_StateReporter_SetIdleState(fb, true, false);
    fb->_segmentStartTime = 0.0;
    HDY_StateReporter_ClearSegmentName(fb);
    HDY_ReportDiagnostic(fb, HDY_DIAG_CODE_ABORTED, HDY_DIAG_SEVERITY_INFO, "Aborted by caller");
    return true;
}

void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};
    const HDY_MotionSegment* segment;
    HDY_REAL elapsed;
    HDY_REAL pressureReference;
    HDY_RampControllerInput rampInput;
    HDY_RampControllerOutput rampOutput;
    HDY_MotionPlannerInput plannerInput;
    HDY_MotionPlannerOutput plannerOutput;
    HDY_PumpConverterInput pumpInput;
    HDY_PumpConverterOutput pumpOutput;
    HDY_DiagnosticsContext diagnosticContext;
    HDY_BOOL segmentCompleted;
    HDY_BOOL recipeFinished;

    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = false;

    if (!fb->EN) {
        fb->ENO = false;
        HDY_ClearPendingStartCommand(fb);
        HDY_UpdateDisabledState(fb);
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
        HDY_StateReporter_ApplySafeOutputs(fb);
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
        return;
    }

    if (fb->FINISHED) {
        HDY_StateReporter_ApplySafeOutputs(fb);
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FINISHED);
        return;
    }

    if (fb->FAULT) {
        HDY_StateReporter_ApplySafeOutputs(fb);
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
        return;
    }

    if (!fb->ACTIVE) {
        HDY_StateReporter_ApplySafeOutputs(fb);
        return;
    }

    if (fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) {
        HDY_ReportFault(fb, HDY_DIAG_CODE_INTERNAL_ERROR, "Current segment index is out of range");
        return;
    }

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
        HDY_ReportFault(fb, code, message);
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

    plannerInput.axisRef = &fb->AXIS_REF;
    plannerInput.segment = segment;
    plannerInput.elapsedTime = elapsed;
    plannerInput.rampedPressure = rampOutput.rampedPressure;
    HDY_MotionPlanner_Execute(&plannerInput, &plannerOutput);

    pumpInput.requestedFlow = plannerOutput.targetFlow;
    pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    pumpInput.direction = plannerOutput.direction;
    HDY_PumpConverter_Execute(&pumpInput, &pumpOutput);

    pressureReference = (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP)
        ? rampOutput.rampedPressure
        : segment->targetPressure;

    diagnosticContext.axisRef = &fb->AXIS_REF;
    diagnosticContext.segment = segment;
    diagnosticContext.plannerOutput = &plannerOutput;
    diagnosticContext.commandedFlow = pumpOutput.commandFlow;
    diagnosticContext.pressureReference = pressureReference;
    diagnosticContext.elapsedTime = elapsed;
    HDY_Diagnostics_UpdateExecution(&fb->DIAGNOSTIC, &diagnosticContext);

    if (fb->DIAGNOSTIC.timeout) {
        HDY_StateReporter_EnterFaultState(fb);
        return;
    }

    segmentCompleted = HDY_SegmentCompletion_Check(segment, &fb->AXIS_REF, elapsed);
    if (segmentCompleted) {
        recipeFinished = (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HDY_StateReporter_SetIdleState(fb, recipeFinished, true);
        return;
    }

    HDY_StateReporter_ReportExecution(fb, &plannerOutput, &pumpOutput);
    fb->SEGMENT_COMPLETED = false;
}
