#include "motion_control.h"
#include "diagnostics.h"
#include "motion_planner.h"
#include "pressure_controller.h"
#include "safety_state_manager.h"
#include "pump_converter.h"
#include "ramp_controller.h"
#include "recipe_validator.h"
#include "segment_completion.h"
#include "state_reporter.h"
#include "velocity_controller.h"
#include "motion_utils.h"
#include "motion_validator.h"
#include "output_limiter.h"
#include "segment_limits.h"
#include "vp_transfer.h"
#include "hyd_config.h"
#include <math.h>
#include <string.h>



/* Internal function declarations */
static HYD_BOOL HYD_QueuePendingCommand(HYD_MotionControlFB* fb,
                                        HYD_FbCommand command,
                                        HYD_UINT segmentIndex,
                                        HYD_TIME timestamp);
static void HYD_MotionControlFB_RunRunningState(HYD_MotionControlFB* fb);
static void HYD_RunRunningStateStopping(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_MotionPlannerOutput* plannerOutput,
                                        HYD_PumpConverterOutput* pumpOutput,
                                        HYD_ExecutionReference* executionReference,
                                        HYD_PressureControllerOutput* pressureOutput);
static HYD_BOOL HYD_RunRunningStateBlendCutover(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                const HYD_ExecutionReference* executionReference);
static HYD_BOOL HYD_RunRunningStateCompletion(HYD_MotionControlFB* fb,
                                              const HYD_MotionSegment* segment,
                                              const HYD_MotionPlannerOutput* plannerOutput,
                                              const HYD_ExecutionReference* executionReference);
static HYD_DirectCommandKind HYD_InferDirectCommandKindFromSegment(const HYD_MotionSegment* segment);
static void HYD_PrimeSegmentControllers(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_TIME timestamp,
                                        HYD_BOOL allowFlowCarryover);
static void HYD_ExecuteActiveSegmentControl(HYD_MotionControlFB* fb,
                                            const HYD_MotionSegment* segment,
                                            HYD_REAL elapsed,
                                            HYD_REAL deltaTime,
                                            HYD_RampControllerOutput* rampOutput,
                                            HYD_MotionPlannerOutput* plannerOutput,
                                            HYD_PressureControllerOutput* pressureOutput,
                                            HYD_PumpConverterOutput* pumpOutput,
                                            HYD_ExecutionReference* executionReference);
static void HYD_UpdateExecutionDiagnostics(HYD_MotionControlFB* fb,
                                           const HYD_MotionSegment* segment,
                                           const HYD_ExecutionReference* executionReference,
                                           HYD_REAL elapsed);
static void HYD_ConfigureSegmentCriteria(HYD_DiagnosticCriteria* criteria,
                                          HYD_REAL baseThreshold,
                                          HYD_DiagnosticCode code,
                                          HYD_ProtectionAction action);

/* Diagnostic reporting moved to StateReporter: use
 * HYD_StateReporter_ReportDiagnostic / HYD_StateReporter_ReportFault
 * directly from motion_control to centralize behavior.
 */

typedef HYD_UINT16 HYD_FbStateMask;

#define HYD_FB_STATE_MASK_BIT(state) ((HYD_FbStateMask)(1U << (state)))

static const HYD_FbStateMask HYD_COMMAND_ALLOWED_STATE_MASKS[HYD_CMD_ACK + 1U] = {
    [HYD_CMD_NONE] = (HYD_FbStateMask)0U,
    [HYD_CMD_START] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_IDLE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_READY) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_SEGMENT_COMPLETE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_DONE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_ABORTED),
    [HYD_CMD_NEXT] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_SEGMENT_COMPLETE),
    [HYD_CMD_STOP] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_STARTING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_RUNNING),
    [HYD_CMD_HOLD] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_STARTING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_RUNNING),
    [HYD_CMD_RESUME] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_HOLD),
    [HYD_CMD_ABORT] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_IDLE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_READY) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_STARTING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_RUNNING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_SEGMENT_COMPLETE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_DONE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_ABORTED) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_HOLD) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_FAULT),
    [HYD_CMD_RESET] = (HYD_FbStateMask)0U,
    [HYD_CMD_ACK] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_DISABLED) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_IDLE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_READY) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_SEGMENT_COMPLETE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_HOLD) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_DONE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_ABORTED)
};

/* String conversion functions moved to motion_utils module
 * Use HYD_MotionUtils_CommandToString and HYD_MotionUtils_StateToString
 */

/* Helper functions moved to motion_validator module
 * Use HYD_MotionValidator_ResolveEffectiveFbState,
 * HYD_MotionValidator_UsesRecipeSource, and
 * HYD_MotionValidator_HasSelectedStartSource
 */

static void HYD_ResetReadyContextPreview(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    if (HYD_MotionValidator_UsesRecipeSource(fb) && fb->RECIPE_SIZE > 0U) {
        fb->STATE.currentSegmentIndex = 0U;
    } else {
        fb->STATE.currentSegmentIndex = HYD_MAX_SEGMENTS;
    }
}

static const HYD_MotionSegment* HYD_ResolveStartSourceSegment(const HYD_MotionControlFB* fb,
                                                              size_t requestedSegmentIndex,
                                                              size_t* resolvedSegmentIndex,
                                                              HYD_SegmentSource* resolvedSource) {
    return HYD_MotionValidator_ResolveStartSourceSegment(fb, requestedSegmentIndex,
                                                          resolvedSegmentIndex, resolvedSource);
}

static HYD_BOOL HYD_IsCommandAllowedInState(HYD_FbCommand command, HYD_FbState state) {
    HYD_FbStateMask mask;

    if ((HYD_UINT)state >= (sizeof(HYD_FbStateMask) * 8U)) {
        return false;
    }

    if ((HYD_UINT)command >= (sizeof(HYD_COMMAND_ALLOWED_STATE_MASKS) / sizeof(HYD_COMMAND_ALLOWED_STATE_MASKS[0]))) {
        return false;
    }

    mask = HYD_COMMAND_ALLOWED_STATE_MASKS[(HYD_UINT)command];
    return (mask & HYD_FB_STATE_MASK_BIT(state)) != 0U;
}

static const HYD_MotionSegment* HYD_ResolveCommandDiagnosticSegment(const HYD_MotionControlFB* fb,
                                                                    HYD_FbCommand command,
                                                                    HYD_UINT requestedSegmentIndex) {
    if (fb == NULL) {
        return NULL;
    }

    if (command == HYD_CMD_START) {
        return HYD_ResolveStartSourceSegment(fb, requestedSegmentIndex, NULL, NULL);
    }

    if (fb->_activeSegmentValid) {
        return &fb->_activeSegment;
    }

    if (fb->STATE.currentSegmentIndex < fb->RECIPE_SIZE) {
        return &fb->RECIPE[fb->STATE.currentSegmentIndex];
    }

    return NULL;
}

static void HYD_ReportCommandNotAllowed(HYD_MotionControlFB* fb,
                                        HYD_FbCommand command,
                                        HYD_FbState state,
                                        HYD_TIME eventTimestamp,
                                        HYD_UINT requestedSegmentIndex,
                                        const HYD_ExecutionReference* references) {
    if (fb == NULL) {
        return;
    }

    HYD_StateReporter_ReportDiagnostic(fb,
                                       HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                       HYD_DIAG_SEVERITY_WARNING,
                                       eventTimestamp,
                                       HYD_ResolveCommandDiagnosticSegment(fb, command, requestedSegmentIndex),
                                       references);
}

static HYD_BOOL HYD_RequestCommandQueue(HYD_MotionControlFB* fb,
                                        HYD_FbCommand command,
                                        HYD_UINT segmentIndex,
                                        HYD_TIME timestamp,
                                        const HYD_ExecutionReference* references) {
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_IsCommandAllowedInState(command, effectiveState)) {
        if (effectiveState != HYD_FB_STATE_FAULT) {
            HYD_ReportCommandNotAllowed(fb,
                                        command,
                                        effectiveState,
                                        timestamp,
                                        segmentIndex,
                                        references);
        }
        return false;
    }

    if (command != HYD_CMD_ABORT && fb->_pendingCommand != HYD_CMD_NONE) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           HYD_ResolveCommandDiagnosticSegment(fb, command, segmentIndex),
                                           references);
        return false;
    }

    return HYD_QueuePendingCommand(fb, command, segmentIndex, timestamp);
}

/* Removed HYD_AxisRefIsValid - now using HYD_MotionUtils_AxisRefIsValid directly */

static void HYD_ClearStartCommandInput(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->START_SEGMENT = false;
    fb->START_SEGMENT_INDEX = 0U;
}

static void HYD_ClearPendingCommand(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_pendingCommand = HYD_CMD_NONE;
    fb->_pendingCommandSegmentIndex = 0U;
    fb->_pendingCommandTimestamp = 0.0;
}

static HYD_REAL HYD_ResolveSegmentBrakingAccelerationForBlend(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->maxDeceleration > 0.0)
        ? segment->maxDeceleration
        : segment->maxAcceleration;
}

static void HYD_ClearDirectBlendContext(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    memset(&fb->_directBlendContext, 0, sizeof(fb->_directBlendContext));
    fb->_directBlendContext.bufferMode = HYD_BUFFER_MODE_ABORT;
}

static HYD_BOOL HYD_IsDirectBlendMode(HYD_BufferMode bufferMode) {
    return bufferMode >= HYD_BUFFER_MODE_BLENDING_LOW &&
           bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH;
}

static HYD_BOOL HYD_IsFinitePositionSegment(const HYD_MotionSegment* segment) {
    return segment != NULL &&
           segment->mode == HYD_MODE_POSITION &&
           segment->endCondition == HYD_END_POSITION &&
           segment->maxVelocity > 0.0 &&
           segment->maxAcceleration > 0.0 &&
           HYD_ResolveSegmentBrakingAccelerationForBlend(segment) > 0.0;
}

static HYD_BOOL HYD_AreBlendDirectionsCompatible(const HYD_MotionControlFB* fb,
                                                 const HYD_MotionSegment* activeSegment,
                                                 const HYD_MotionSegment* pendingSegment) {
    HYD_MotionDirection activeDirection;
    HYD_MotionDirection pendingDirection;

    if (fb == NULL || activeSegment == NULL || pendingSegment == NULL) {
        return false;
    }

    activeDirection = HYD_Segment_ResolveDirection(activeSegment, &fb->AXIS_REF);
    pendingDirection = HYD_Segment_ResolveDirection(pendingSegment, &fb->AXIS_REF);

    return (activeDirection == HYD_DIRECTION_EXTEND ||
            activeDirection == HYD_DIRECTION_RETRACT) &&
           activeDirection == pendingDirection;
}

static HYD_REAL HYD_SelectDirectBlendVelocity(HYD_BufferMode bufferMode,
                                              const HYD_MotionSegment* activeSegment,
                                              const HYD_MotionSegment* pendingSegment) {
    HYD_REAL previousVelocity;
    HYD_REAL nextVelocity;

    if (activeSegment == NULL || pendingSegment == NULL) {
        return 0.0;
    }

    previousVelocity = activeSegment->maxVelocity;
    nextVelocity = pendingSegment->maxVelocity;

    switch (bufferMode) {
        case HYD_BUFFER_MODE_BLENDING_LOW:
            return HYD_MotionUtils_MinReal(previousVelocity, nextVelocity);
        case HYD_BUFFER_MODE_BLENDING_PREVIOUS:
            return previousVelocity;
        case HYD_BUFFER_MODE_BLENDING_NEXT:
            return nextVelocity;
        case HYD_BUFFER_MODE_BLENDING_HIGH:
            return (previousVelocity > nextVelocity) ? previousVelocity : nextVelocity;
        default:
            return 0.0;
    }
}

static HYD_BOOL HYD_TryCreateDirectBlendContext(HYD_MotionControlFB* fb,
                                                HYD_BufferMode bufferMode,
                                                const HYD_MotionSegment* pendingSegment) {
    HYD_REAL selectedVelocity;
    HYD_REAL tolerance;

    if (fb == NULL || pendingSegment == NULL) {
        return false;
    }

    HYD_ClearDirectBlendContext(fb);

    if (!HYD_IsDirectBlendMode(bufferMode) ||
        !fb->_activeSegmentValid ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != HYD_DIRECT_CMD_MOVE_ABSOLUTE ||
        HYD_InferDirectCommandKindFromSegment(pendingSegment) != HYD_DIRECT_CMD_MOVE_ABSOLUTE ||
        !HYD_IsFinitePositionSegment(&fb->_activeSegment) ||
        !HYD_IsFinitePositionSegment(pendingSegment) ||
        !HYD_AreBlendDirectionsCompatible(fb, &fb->_activeSegment, pendingSegment)) {
        return false;
    }

    selectedVelocity = HYD_SelectDirectBlendVelocity(bufferMode,
                                                    &fb->_activeSegment,
                                                    pendingSegment);
    if (selectedVelocity <= 0.0) {
        return false;
    }

    tolerance = HYD_Segment_GetPositionTolerance(&fb->_activeSegment);

    fb->_directBlendContext.active = true;
    fb->_directBlendContext.bufferMode = bufferMode;
    fb->_directBlendContext.blendVelocity = selectedVelocity;
    fb->_directBlendContext.switchPosition = fb->_activeSegment.targetPosition;
    fb->_directBlendContext.switchTolerance = tolerance;
    return true;
}

static void HYD_RecordDirectExecutionCompleted(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_lastCompletedExecutionId = fb->_directOwnerExecutionId;
    fb->_lastCompletedKind = fb->_directOwnerKind;
}

static HYD_BOOL HYD_ShouldCutoverDirectBlend(const HYD_MotionControlFB* fb,
                                             const HYD_MotionSegment* segment) {
    HYD_MotionDirection direction;
    HYD_REAL tolerance;

    if (fb == NULL || segment == NULL ||
        !fb->_directBlendContext.active ||
        !fb->_directPendingValid ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
        return false;
    }

    direction = HYD_Segment_ResolveDirection(segment, &fb->AXIS_REF);
    tolerance = fb->_directBlendContext.switchTolerance;
    if (tolerance <= 0.0) {
        tolerance = HYD_Segment_GetPositionTolerance(segment);
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            return fb->AXIS_REF.position >=
                fb->_directBlendContext.switchPosition - tolerance;
        case HYD_DIRECTION_RETRACT:
            return fb->AXIS_REF.position <=
                fb->_directBlendContext.switchPosition + tolerance;
        default:
            return false;
    }
}

void HYD_ClearDirectPendingSlot(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_directPendingValid = false;
    memset(&fb->_directPendingSegment, 0, sizeof(fb->_directPendingSegment));
    fb->_directPendingKind = HYD_DIRECT_CMD_NONE;
    fb->_directPendingBufferMode = HYD_BUFFER_MODE_ABORT;
    HYD_ClearDirectBlendContext(fb);
}

static HYD_BOOL HYD_IsSegmentEndlessForBuffering(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return true;
    }

    if (segment->mode == HYD_MODE_SPEED_RAMP &&
        segment->endCondition == HYD_END_MANUAL) {
        return true;
    }

    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP &&
        segment->endCondition == HYD_END_MANUAL) {
        return true;
    }

    return false;
}

static HYD_BOOL HYD_QueuePendingCommand(HYD_MotionControlFB* fb,
                                        HYD_FbCommand command,
                                        HYD_UINT segmentIndex,
                                        HYD_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    if (command == HYD_CMD_ABORT) {
        fb->_pendingCommand = command;
        fb->_pendingCommandSegmentIndex = segmentIndex;
        fb->_pendingCommandTimestamp = timestamp;
        return true;
    }

    if (fb->_pendingCommand != HYD_CMD_NONE) {
        return false;
    }

    fb->_pendingCommand = command;
    fb->_pendingCommandSegmentIndex = segmentIndex;
    fb->_pendingCommandTimestamp = timestamp;
    return true;
}


static void HYD_PrepareRecipeLoadState(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_ResetReadyContextPreview(fb);
    fb->_segmentStartTime = 0.0;
    fb->_holdStateTime = 0.0;
    fb->_lastCommandedFlow = 0.0;
    fb->_activeSegmentValid = false;
    fb->_startSegmentSignalPrev = false;
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_ClearPendingCommand(fb);
    HYD_ClearDirectPendingSlot(fb);
    HYD_ClearStartCommandInput(fb);
    HYD_StateReporter_SetIdleState(fb, false, false);
    HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
    HYD_StateReporter_ClearSegmentTag(fb);
}

static HYD_DirectCommandKind HYD_InferDirectCommandKindFromSegment(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DIRECT_CMD_NONE;
    }

    if (segment->mode == HYD_MODE_POSITION &&
        segment->endCondition == HYD_END_POSITION) {
        return HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    }

    if (segment->mode == HYD_MODE_SPEED_RAMP) {
        return HYD_DIRECT_CMD_MOVE_VELOCITY;
    }

    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        return HYD_DIRECT_CMD_PRESSURE_HANDLE;
    }

    return HYD_DIRECT_CMD_NONE;
}

/* Diagnostic reporting moved into StateReporter (see state_reporter.c). */

static HYD_BOOL HYD_ValidateNextRequest(const HYD_MotionControlFB* fb,
                                        HYD_DiagnosticCode* code) {
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_MotionValidator_UsesRecipeSource(fb)) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (fb->RECIPE_SIZE == 0U) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_NO_RECIPE;
        }
        return false;
    }

    if (effectiveState == HYD_FB_STATE_DONE) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED;
        }
        return false;
    }

    if (effectiveState == HYD_FB_STATE_STARTING || effectiveState == HYD_FB_STATE_RUNNING) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    if (!HYD_IsCommandAllowedInState(HYD_CMD_NEXT, effectiveState)) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    return true;
}

static void HYD_PrimeSegmentControllers(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_TIME timestamp,
                                        HYD_BOOL allowFlowCarryover) {
    HYD_REAL initialPressureControlOutput;
    HYD_REAL trackingFlowReference;

    if (fb == NULL || segment == NULL) {
        return;
    }

    fb->_isDecelerating = false;
    fb->_decelStartTime = 0.0;
    fb->_decelStartVel = 0.0;
    fb->_completionCandidateStartTime = 0.0;
    fb->_completionCandidateActive = false;
    fb->STATE.vpTransferReady = false;
    fb->STATE.vpTransferReason = (HYD_UINT8)HYD_VP_TRANSFER_REASON_NONE;
    fb->_lastFeedbackTimestamp = timestamp;
    HYD_RampController_Init(&fb->_rampController, fb->AXIS_REF.pressure, timestamp);
    /* Sprint 2: Carry over velocity state for bumpless transitions.
     * P->V: seed with _lastCommandedFlow / velocityToFlowGain
     * S->S: retain lastTargetVelocity from previous segment */
    {
        HYD_REAL carriedVelocity = 0.0;
        HYD_REAL carriedFlow = 0.0;
        HYD_BOOL doCarryover = false;

        if (segment->mode == HYD_MODE_SPEED_RAMP) {
            if (fb->_previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
                HYD_REAL gain = segment->velocityToFlowGain;
                if (gain <= 0.0) { gain = 1.0; }
                if (fb->_lastCommandedFlow > 0.0) {
                    carriedVelocity = fb->_lastCommandedFlow / gain;
                    doCarryover = true;
                }
            } else if (fb->_previousSegmentMode == HYD_MODE_SPEED_RAMP) {
                if (fabs(fb->_plannerState.lastTargetVelocity) > 0.0) {
                    carriedVelocity = fabs(fb->_plannerState.lastTargetVelocity);
                    carriedFlow = fb->_plannerState.lastTargetFlow;
                    doCarryover = true;
                }
            }
        }

        memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));

        if (doCarryover) {
            fb->_plannerState.lastTargetVelocity = carriedVelocity;
            fb->_plannerState.lastTargetFlow = carriedFlow;
            fb->_plannerState.initialized = true;
        }
    }

    trackingFlowReference = HYD_MotionUtils_AbsReal(fb->AXIS_REF.flow);
    if (trackingFlowReference <= 0.0 && allowFlowCarryover) {
        trackingFlowReference = fb->_lastCommandedFlow;
    }

    initialPressureControlOutput = segment->targetFlow;
    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP && trackingFlowReference > 0.0) {
        initialPressureControlOutput = trackingFlowReference;
    }
    initialPressureControlOutput = HYD_MotionUtils_MinReal(initialPressureControlOutput, segment->maxFlow);
    if (initialPressureControlOutput < 0.0) {
        initialPressureControlOutput = 0.0;
    }

    HYD_PressureController_InitState(&fb->_pressureController,
                                     fb->AXIS_REF.pressure,
                                     initialPressureControlOutput,
                                     timestamp);
    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP && trackingFlowReference > 0.0) {
        HYD_PressureController_RequestTracking(&fb->_pressureController, initialPressureControlOutput);
    }
}

static HYD_BOOL HYD_BeginSegment(HYD_MotionControlFB* fb,
                                 size_t segmentIndex,
                                 HYD_TIME timestamp) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    HYD_BOOL preserveFlowCarryover;
    const HYD_MotionSegment* sourceSegment;
    size_t resolvedSegmentIndex;
    HYD_SegmentSource resolvedSource;

    if (fb == NULL) {
        return false;
    }

    if (!HYD_MotionValidator_ValidateStartRequest(fb, segmentIndex, &code)) {
        HYD_SafetyStateManager_ApplyIdleState(fb, false, false);
        HYD_ResetReadyContextPreview(fb);
        HYD_StateReporter_ReportDiagnostic(fb,
                           code,
                           HYD_DIAG_SEVERITY_WARNING,
                           timestamp,
                           HYD_ResolveStartSourceSegment(fb, segmentIndex, NULL, NULL),
                           NULL);
        return false;
    }

    preserveFlowCarryover = fb->SEGMENT_COMPLETED;
    sourceSegment = HYD_ResolveStartSourceSegment(fb,
                                                  segmentIndex,
                                                  &resolvedSegmentIndex,
                                                  &resolvedSource);
    if (sourceSegment == NULL) {
        HYD_StateReporter_ReportFault(fb,
                          HYD_DIAG_CODE_INTERNAL_ERROR,
                          timestamp,
                          NULL,
                          &fb->STATE.references);
        return false;
    }

    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_ResetTransitionFlags(fb);

    /* Save previous segment mode for bumpless carry-over before overwriting */
    if (fb->_activeSegmentValid) {
        fb->_previousSegmentMode = fb->_activeSegment.mode;
    }

    fb->_activeSegment = *sourceSegment;
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = resolvedSource;

    /* Resolve velocityToFlowGain: segment explicit > cylinderConfig > existing fallback */
    if (fb->_activeSegment.velocityToFlowGain <= 0.0f &&
        HYD_CylinderConfig_IsValid(&fb->cylinderConfig)) {
        fb->_activeSegment.velocityToFlowGain =
            HYD_CylinderConfig_GetVelocityToFlowGain(
                &fb->cylinderConfig, fb->_activeSegment.direction);
    }
    fb->STATE.currentSegmentIndex = resolvedSegmentIndex;
    fb->_segmentStartTime = timestamp;
    fb->_holdStateTime = 0.0;
    fb->_segmentChangedFlag = true;
    fb->SEGMENT_COMPLETED = false;

    /* Reset diagnostic criteria layer for new segment */
    HYD_ErrorMonitor_Reset(&fb->_errorMonitor);
    HYD_DiagnosticCriteria_ResetState(&fb->_pressureCriteriaState);
    HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
    HYD_DiagnosticCriteria_ResetState(&fb->_flowCriteriaState);
    HYD_DiagnosticCriteria_ResetState(&fb->_velocityCriteriaState);
    HYD_DiagnosticCriteria_ResetState(&fb->_positionCriteriaState);
    HYD_DiagnosticCriteria_ResetState(&fb->_timeoutCriteriaState);
    HYD_OutputLimiter_ResetState(&fb->_limiterState);
    fb->_isSwitchPhase = true;

    /* Configure criteria thresholds for this segment (once per segment, not every cycle).
     * The default suppress/debounce/hysteresis/escalation settings from
     * CreateDefaultXxxCriteria (Init) are preserved; only the threshold and
     * diagnostic code are overridden based on segment tolerances. */
    {
        HYD_REAL pTol = HYD_Segment_GetPressureTolerance(sourceSegment);
        HYD_REAL fTol = HYD_Segment_GetFlowTolerance(sourceSegment);
        HYD_REAL vTol = HYD_Segment_GetVelocityTolerance(sourceSegment);
        HYD_REAL posTol = HYD_Segment_GetPositionTolerance(sourceSegment);
        HYD_TIME tLim = HYD_Segment_GetTimeoutLimit(sourceSegment);

        if (sourceSegment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP && pTol > 0.0) {
            HYD_ConfigureSegmentCriteria(&fb->_pressureCriteria, pTol,
                                          HYD_DIAG_CODE_OVER_PRESSURE,
                                          HYD_PROTECTION_ACTION_DERATE);
        }
        if (fTol > 0.0) {
            HYD_ConfigureSegmentCriteria(&fb->_flowCriteria, fTol,
                                          HYD_DIAG_CODE_FLOW_DEVIATION,
                                          HYD_PROTECTION_ACTION_DERATE);
        }
        if (vTol > 0.0) {
            HYD_ConfigureSegmentCriteria(&fb->_velocityCriteria, vTol,
                                          HYD_DIAG_CODE_VELOCITY_DEVIATION,
                                          HYD_PROTECTION_ACTION_WARNING);
        }
        if (posTol > 0.0) {
            HYD_ConfigureSegmentCriteria(&fb->_positionCriteria, posTol,
                                          HYD_DIAG_CODE_POSITION_DEVIATION,
                                          HYD_PROTECTION_ACTION_WARNING);
        }
        if (tLim > 0.0) {
            HYD_ConfigureSegmentCriteria(&fb->_timeoutCriteria, tLim,
                                          HYD_DIAG_CODE_TIMEOUT,
                                          HYD_PROTECTION_ACTION_STOP);
            fb->_timeoutCriteria.severity = HYD_DIAG_SEVERITY_FAULT;
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
     * Must be computed AFTER HYD_ConfigureSegmentCriteria so that it reflects
     * the current segment's criteria, not the previous segment's. */
    fb->_switchSuppressEndTime = fb->_pressureCriteria.startupSuppressTime +
                                  fb->_pressureCriteria.switchSuppressTime;

    HYD_PrimeSegmentControllers(fb, &fb->_activeSegment, timestamp, preserveFlowCarryover);

    HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HYD_StateReporter_SetSegmentSource(fb, resolvedSource);
    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    HYD_StateReporter_SetActive(fb, true);
    HYD_StateReporter_SetFinished(fb, false);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_RUNNING);
    HYD_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_STARTING);
    fb->_executionId++;
    if (resolvedSource == HYD_SEGMENT_SOURCE_DIRECT) {
        fb->_directOwnerKind = HYD_InferDirectCommandKindFromSegment(sourceSegment);
        fb->_directSessionState = HYD_DIRECT_SESSION_RUNNING;
        fb->_directOwnerExecutionId = fb->_executionId;
    }
    return true;
}

static HYD_BOOL HYD_AdvanceToNextSegment(HYD_MotionControlFB* fb,
                                         HYD_TIME timestamp) {
    if (fb == NULL) {
        return false;
    }

    if (!HYD_MotionValidator_UsesRecipeSource(fb)) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           &fb->_activeSegment,
                                           &fb->STATE.references);
        return false;
    }

    if (fb->STATE.currentSegmentIndex + 1U < fb->RECIPE_SIZE) {
        return HYD_BeginSegment(fb, fb->STATE.currentSegmentIndex + 1U, timestamp);
    }

    HYD_SafetyStateManager_ApplyIdleState(fb, true, true);
    HYD_StateReporter_ClearSegmentTag(fb);
    HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
    HYD_ResetReadyContextPreview(fb);
    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

static HYD_BOOL HYD_StartPendingDirectSlot(HYD_MotionControlFB* fb,
                                           HYD_TIME timestamp,
                                           HYD_BOOL preservePlannerState) {
    HYD_MotionSegment segment;
    HYD_BOOL savedUseRecipe;
    HYD_MotionPlannerState preservedPlannerState;

    if (fb == NULL || !fb->_directPendingValid) {
        return false;
    }

    segment = fb->_directPendingSegment;
    preservedPlannerState = fb->_plannerState;
    HYD_ClearDirectPendingSlot(fb);

    savedUseRecipe = fb->USE_RECIPE;
    fb->DIRECT_SEGMENT = segment;
    fb->DIRECT_SEGMENT_VALID = true;
    fb->USE_RECIPE = false;
    /* Direct takeover invalidates any outer recipe MoveProfile's ownership
     * batch — bump _recipeBatchId BEFORE BeginSegment so that the IEC adapter
     * sees the recipe batch id change in the same scan that the source
     * transitions to DIRECT. */
    fb->_recipeBatchId++;
    if (!HYD_BeginSegment(fb, 0U, timestamp)) {
        fb->USE_RECIPE = savedUseRecipe;
        return false;
    }
    if (preservePlannerState) {
        fb->_plannerState = preservedPlannerState;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return true;
}

static void HYD_EnterHoldNow(HYD_MotionControlFB* fb,
                             HYD_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_INTERNAL_ERROR,
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    fb->_holdStateTime = timestamp;
    fb->_lastFeedbackTimestamp = timestamp;
    HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HYD_StateReporter_SetHoldState(fb);
}

static HYD_BOOL HYD_ResumeHeldSegment(HYD_MotionControlFB* fb,
                                      HYD_TIME timestamp) {
    HYD_TIME holdDuration;

    if (fb == NULL) {
        return false;
    }

    if (!fb->_activeSegmentValid) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_INTERNAL_ERROR,
                        timestamp,
                        NULL,
                        &fb->STATE.references);
        return false;
    }

    if (!HYD_MotionUtils_AxisRefIsValid(&fb->AXIS_REF)) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_SENSOR_FAULT,
                        fb->AXIS_REF.timestamp,
                        &fb->_activeSegment,
                        &fb->STATE.references);
        return false;
    }

    if (fb->_lastFeedbackTimestamp >= 0.0 && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_TIMESTAMP_ROLLBACK,
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
    HYD_PrimeSegmentControllers(fb, &fb->_activeSegment, timestamp, true);
    HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HYD_StateReporter_SetActive(fb, true);
    HYD_StateReporter_SetFinished(fb, false);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_RUNNING);
    HYD_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_STARTING);
    return true;
}

static void HYD_AbortNow(HYD_MotionControlFB* fb,
                         HYD_TIME timestamp) {
    if (fb == NULL) {
        return;
    }

    HYD_ClearStartCommandInput(fb);
    HYD_SafetyStateManager_ApplyIdleState(fb, true, false);
    /* Ensure outputs are forced to a safe state immediately on abort */
    HYD_StateReporter_ApplySafeOutputs(fb);
    fb->_holdStateTime = 0.0;
    fb->_lastCommandedFlow = 0.0;
    HYD_ResetReadyContextPreview(fb);
    HYD_StateReporter_ClearSegmentTag(fb);
    HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
    fb->_directSessionState = HYD_DIRECT_SESSION_ABORTED;
    HYD_ClearDirectPendingSlot(fb);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_ABORTED);
    HYD_StateReporter_ReportDiagnostic(fb,
                                       HYD_DIAG_CODE_ABORTED,
                                       HYD_DIAG_SEVERITY_INFO,
                                       timestamp,
                                       NULL,
                                       &fb->STATE.references);
    fb->_executionId++;
    /* External ABORT terminates the current recipe batch — bump
     * _recipeBatchId so the IEC adapter detects ownership loss on any
     * outer MoveProfile FB. */
    fb->_recipeBatchId++;
}

static void HYD_MaintainNonExecutingState(HYD_MotionControlFB* fb,
                                          HYD_BOOL autoClearLiveDiagnostic) {
    HYD_FbState preservedState;

    if (fb == NULL) {
        return;
    }

    if (fb->STATE.finished) {
        preservedState = fb->FB_STATE;
        HYD_SafetyStateManager_ApplyIdleState(fb, true, fb->SEGMENT_COMPLETED);
        if (preservedState == HYD_FB_STATE_ABORTED) {
            HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_ABORTED);
        }
        if (autoClearLiveDiagnostic) {
            HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    if (fb->SEGMENT_COMPLETED) {
        HYD_SafetyStateManager_ApplyIdleState(fb, false, true);
        if (autoClearLiveDiagnostic) {
            HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
        }
        return;
    }

    HYD_SafetyStateManager_ApplyIdleState(fb, false, false);
    HYD_ResetReadyContextPreview(fb);
    if (autoClearLiveDiagnostic) {
        HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
    }
}

static void HYD_MaintainPausedHoldState(HYD_MotionControlFB* fb,
                                        HYD_BOOL autoClearLiveDiagnostic) {
    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_INTERNAL_ERROR,
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HYD_StateReporter_SetHoldState(fb);
    if (autoClearLiveDiagnostic) {
        HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(fb);
    }
}

static HYD_BOOL HYD_MotionControlFB_ConsumePendingCommand(HYD_MotionControlFB* fb,
                                                          HYD_FbCommand* processedCommand) {
    HYD_FbCommand command;
    HYD_UINT segmentIndex;
    HYD_TIME timestamp;
    HYD_FbState effectiveState;

    if (processedCommand != NULL) {
        *processedCommand = HYD_CMD_NONE;
    }

    if (fb == NULL || fb->_pendingCommand == HYD_CMD_NONE) {
        return true;
    }

    command = fb->_pendingCommand;
    segmentIndex = fb->_pendingCommandSegmentIndex;
    timestamp = fb->_pendingCommandTimestamp;
    HYD_ClearPendingCommand(fb);

    if (processedCommand != NULL) {
        *processedCommand = command;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_IsCommandAllowedInState(command, effectiveState)) {
        if (effectiveState != HYD_FB_STATE_FAULT) {
            HYD_ReportCommandNotAllowed(fb,
                                        command,
                                        effectiveState,
                                        timestamp,
                                        segmentIndex,
                                        &fb->STATE.references);
        }
        return true;
    }

    switch (command) {
        case HYD_CMD_START:
            /* Initial Start opens a fresh recipe batch (or a fresh direct
             * session). Bump _recipeBatchId so a re-issued MoveProfile after
             * Reset / Done observes a distinct batch identity from the
             * previous run. NextSegment (HYD_CMD_NEXT) deliberately does
             * NOT bump _recipeBatchId — multi-segment recipe progress must
             * not look like external takeover. */
            fb->_recipeBatchId++;
            (void)HYD_BeginSegment(fb, segmentIndex, timestamp);
            return true;
        case HYD_CMD_NEXT:
            (void)HYD_AdvanceToNextSegment(fb, timestamp);
            return true;
        case HYD_CMD_STOP:
            if (fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
                fb->_lastPreemptedExecutionId = fb->_directOwnerExecutionId;
                fb->_lastPreemptedKind = fb->_directOwnerKind;
            } else {
                fb->_lastPreemptedExecutionId = 0U;
                fb->_lastPreemptedKind = HYD_DIRECT_CMD_NONE;
            }
            fb->_executionId++;
            /* STOP preempts any active recipe segment — bump
             * _recipeBatchId so the outer MoveProfile observes ownership
             * loss even though _activeSegmentSource stays RECIPE. */
            fb->_recipeBatchId++;
            fb->_directOwnerExecutionId = fb->_executionId;
            fb->_directOwnerKind = HYD_DIRECT_CMD_STOP;
            fb->_directSessionState = HYD_DIRECT_SESSION_STOPPING;
            fb->_isStopping = true;
            fb->_stopStartTime = timestamp;
            fb->_stopStartVel = fb->AXIS_REF.velocity;
            if (fabs(fb->_stopStartVel) < 0.001f) {
                fb->_stopStartVel = fb->STATE.plannedVelocity;
            }
            return true;
        case HYD_CMD_HOLD:
            HYD_EnterHoldNow(fb, timestamp);
            return false;
        case HYD_CMD_RESUME:
            return HYD_ResumeHeldSegment(fb, timestamp);
        case HYD_CMD_ABORT:
            /* C-3: clear faultActive so FAULT->ABORTED transition is clean.
             * Diagnostic retention is preserved (LAST_FAULT_SNAPSHOT + history). */
            if (fb->STATE.faultActive) {
                fb->STATE.faultActive = false;
            }
            HYD_AbortNow(fb, timestamp);
            return false;
        default:
            return true;
    }
}

static void HYD_MotionControlFB_RunStateMachine(HYD_MotionControlFB* fb,
                                                HYD_BOOL allowRunningExecution) {
    if (fb == NULL || !allowRunningExecution) {
        return;
    }

    switch (fb->FB_STATE) {
        case HYD_FB_STATE_STARTING:
        case HYD_FB_STATE_RUNNING:
            HYD_MotionControlFB_RunRunningState(fb);
            break;
        case HYD_FB_STATE_DISABLED:
        case HYD_FB_STATE_IDLE:
        case HYD_FB_STATE_READY:
        case HYD_FB_STATE_SEGMENT_COMPLETE:
        case HYD_FB_STATE_HOLD:
        case HYD_FB_STATE_DONE:
        case HYD_FB_STATE_ABORTED:
        case HYD_FB_STATE_FAULT:
        default:
            break;
    }
}

static void HYD_MotionControlFB_PublishOutputs(HYD_MotionControlFB* fb,
                                                HYD_BOOL autoClearLiveDiagnostic) {
    if (fb == NULL) {
        return;
    }

    if (fb->STATE.faultActive) {
        HYD_SafetyStateManager_ApplyFaultHold(fb);
        fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
        fb->_segmentChangedFlag = false;
        return;
    }

    switch (fb->FB_STATE) {
        case HYD_FB_STATE_STARTING:
        case HYD_FB_STATE_RUNNING:
            break;
        case HYD_FB_STATE_HOLD:
            HYD_MaintainPausedHoldState(fb, autoClearLiveDiagnostic);
            break;
        case HYD_FB_STATE_DISABLED:
        case HYD_FB_STATE_IDLE:
        case HYD_FB_STATE_READY:
        case HYD_FB_STATE_SEGMENT_COMPLETE:
        case HYD_FB_STATE_DONE:
        case HYD_FB_STATE_ABORTED:
        case HYD_FB_STATE_FAULT:
        default:
            HYD_MaintainNonExecutingState(fb, autoClearLiveDiagnostic);
            break;
    }

    fb->SEGMENT_CHANGED = fb->_segmentChangedFlag;
        fb->_segmentChangedFlag = false;
}

/*
 * Configure a single HYD_DiagnosticCriteria from segment tolerances.
 * Called once per segment start (HYD_BeginSegment), not every cycle.
 * This preserves debounce/hysteresis/fault-escalation state across cycles
 * while keeping the criteria configuration stable for the segment lifetime.
 */
static void HYD_ConfigureSegmentCriteria(HYD_DiagnosticCriteria* criteria,
                                          HYD_REAL baseThreshold,
                                          HYD_DiagnosticCode code,
                                          HYD_ProtectionAction action) {
    if (criteria == NULL) {
        return;
    }

    /* Preserve existing suppress/escalation/debounce/hysteresis defaults that
     * were set by CreateDefaultXxxCriteria during Init. Only override the
     * fields that must adapt to the current segment's tolerances. */
    criteria->baseThreshold = baseThreshold;
    criteria->diagnosticCode = code;
    criteria->severity = HYD_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = action;
}

static void HYD_UpdateMonitorPositionError(HYD_ErrorMonitor* monitor,
                                           const HYD_MotionSegment* segment,
                                           const HYD_AxisRef* axisRef,
                                           HYD_TIME currentTime) {
    HYD_BOOL wasActive;
    HYD_REAL positionTolerance;

    if (monitor == NULL || segment == NULL || axisRef == NULL) {
        return;
    }

    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
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

static void HYD_ExecuteActiveSegmentControl(HYD_MotionControlFB* fb,
                                            const HYD_MotionSegment* segment,
                                            HYD_REAL elapsed,
                                            HYD_REAL deltaTime,
                                            HYD_RampControllerOutput* rampOutput,
                                            HYD_MotionPlannerOutput* plannerOutput,
                                            HYD_PressureControllerOutput* pressureOutput,
                                            HYD_PumpConverterOutput* pumpOutput,
                                            HYD_ExecutionReference* executionReference) {
    HYD_RampControllerInput rampInput;
    HYD_MotionPlannerInput plannerInput;
    HYD_PressureControllerInput pressureInput;
    HYD_PumpConverterInput pumpInput;
    HYD_VelocityControllerInput velocityInput;
    HYD_VelocityControllerOutput velocityOutput;

    if (fb == NULL || segment == NULL || rampOutput == NULL || plannerOutput == NULL ||
        pressureOutput == NULL || pumpOutput == NULL || executionReference == NULL) {
        return;
    }

    rampInput.targetPressure = segment->targetPressure;
    rampInput.rampRate = segment->pressureRampRate;
    rampInput.currentTime = fb->AXIS_REF.timestamp;
    HYD_RampController_Execute(&fb->_rampController, &rampInput, rampOutput);

    memset(plannerOutput, 0, sizeof(*plannerOutput));
    memset(pressureOutput, 0, sizeof(*pressureOutput));
    plannerOutput->direction = HYD_DIRECTION_HOLD;
    pressureOutput->appliedStrategy = HYD_PRESSURE_CONTROLLER_NONE;

    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        pressureInput.targetPressure = rampOutput->rampedPressure;
        pressureInput.measuredPressure = fb->AXIS_REF.pressure;
        pressureInput.feedforwardFlow = segment->targetFlow;
        pressureInput.outputMin = 0.0;

        pressureInput.outputMax = segment->maxFlow;
        pressureInput.timestamp = fb->AXIS_REF.timestamp;
        HYD_PressureController_Execute(segment,
                                       &fb->_pressureController,
                                       &pressureInput,
                                       pressureOutput);
        plannerOutput->targetFlow = pressureOutput->outputFlow;
        plannerOutput->direction = segment->direction;
    } else {
        memset(&plannerInput, 0, sizeof(plannerInput));
        plannerInput.axisRef = &fb->AXIS_REF;
        plannerInput.segment = segment;
        plannerInput.elapsedTime = elapsed;
        plannerInput.deltaTime = (deltaTime > 0.0) ? deltaTime : 0.0;
        plannerInput.rampedPressure = rampOutput->rampedPressure;
        if (fb->_isDecelerating) {
            plannerInput.decelElapsed = fb->AXIS_REF.timestamp - fb->_decelStartTime;
            plannerInput.decelStartVel = fb->_decelStartVel;
        } else {
            plannerInput.decelElapsed = 0.0;
            plannerInput.decelStartVel = 0.0;
        }
        plannerInput.state = &fb->_plannerState;
        plannerInput.blend = fb->_directBlendContext.active
            ? &fb->_directBlendContext
            : NULL;
        HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);

        if (segment->mode == HYD_MODE_SPEED_RAMP && segment->velocityKp > 0.0) {
            velocityInput.targetVelocity = plannerOutput->targetVelocity;
            velocityInput.actualVelocity = fb->AXIS_REF.velocity;
            velocityInput.feedforwardFlow = plannerOutput->targetFlow;
            velocityInput.kp = segment->velocityKp;
            velocityInput.deadband = segment->velocityDeadband;
            velocityInput.correctionLimit = segment->velocityCorrectionLimit;
            velocityInput.outputMin = 0.0;
            velocityInput.outputMax = segment->maxFlow;

            HYD_VelocityController_Execute(&velocityInput, &velocityOutput);
            plannerOutput->targetFlow = velocityOutput.correctedFlow;
        }
    }

    pumpInput.requestedFlow = plannerOutput->targetFlow;
    if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
        pumpInput.flowToPumpSpeedGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
        pumpInput.pumpSpeedLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
    } else {
        pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    }
    pumpInput.direction = plannerOutput->direction;
    HYD_PumpConverter_Execute(&pumpInput, pumpOutput);

    executionReference->elapsedTime = elapsed;
    executionReference->pressureReference = (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP)
        ? rampOutput->rampedPressure
        : segment->targetPressure;
    executionReference->flowReference = pumpOutput->commandFlow;
    executionReference->velocityReference = plannerOutput->targetVelocity;
}

static void HYD_UpdateExecutionDiagnostics(HYD_MotionControlFB* fb,
                                           const HYD_MotionSegment* segment,
                                           const HYD_ExecutionReference* executionReference,
                                           HYD_REAL elapsed) {
    HYD_REAL pressureTolerance;
    HYD_REAL flowTolerance;
    HYD_REAL positionTolerance;
    HYD_REAL velocityTolerance;
    HYD_REAL velocityReferenceAbs;
    HYD_REAL actualVelocityAbs;
    HYD_DiagnosticResult pressureResult;
    HYD_DiagnosticResult flowResult;
    HYD_DiagnosticResult velocityResult;
    HYD_DiagnosticResult positionResult;
    HYD_DiagnosticResult ceilingResult;
    HYD_DiagnosticCode priorityCode = HYD_DIAG_CODE_NONE;
    HYD_DiagnosticSeverity prioritySeverity = HYD_DIAG_SEVERITY_NONE;
    HYD_BOOL overPressure = false;
    HYD_BOOL underPressure = false;
    HYD_BOOL flowDeviation = false;
    HYD_BOOL positionDeviation = false;
    HYD_BOOL velocityDeviation = false;
    HYD_BOOL timeout = false;
    HYD_BOOL pressureCeilingExceeded = false;
    HYD_BOOL pressureCeilingViolated = false;
    HYD_REAL pressureError = 0.0;
    HYD_REAL flowError = 0.0;
    HYD_REAL velocityError = 0.0;
    HYD_REAL pressureCeilingValue = 0.0;
    HYD_REAL pressureCeilingTolerance = 0.0;
    HYD_REAL ceilingErrorValue = 0.0;  /* actual - (ceiling + tol); only meaningful when > 0 */
    HYD_BOOL ceilingActive = false;

    /* Derive suppress flags from segment elapsed time and criteria configuration.
     * isStartupPhase: determined per-criteria from its own startupSuppressTime.
     * isSwitchPhase: shared flag cleared after startup suppress window elapses. */
    HYD_BOOL isStartupPhase = false;  /* Computed per-criteria below */
    HYD_BOOL isSwitchPhase = false;

    if (fb == NULL || segment == NULL || executionReference == NULL) {
        return;
    }

    pressureTolerance = HYD_Segment_GetPressureTolerance(segment);
    flowTolerance = HYD_Segment_GetFlowTolerance(segment);
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);
    velocityTolerance = HYD_Segment_GetVelocityTolerance(segment);

    /* Determine suppress phases from each criteria's own configuration and elapsed time.
     * Each diagnostic channel independently determines its startup phase based on
     * its own startupSuppressTime. Switch phase is shared across all channels. */
    isSwitchPhase = fb->_isSwitchPhase;

    {
        HYD_ErrorMonitorTolerances monitorTolerances;
        monitorTolerances.position = positionTolerance;
        monitorTolerances.velocity = velocityTolerance;
        monitorTolerances.flow     = flowTolerance;
        monitorTolerances.pressure = pressureTolerance;
        HYD_ErrorMonitor_Update(&fb->_errorMonitor,
                                &fb->AXIS_REF,
                                executionReference,
                                &monitorTolerances,
                                fb->AXIS_REF.timestamp);
    }
    HYD_UpdateMonitorPositionError(&fb->_errorMonitor,
                                   segment,
                                   &fb->AXIS_REF,
                                   fb->AXIS_REF.timestamp);
    pressureError = fb->_errorMonitor.pressureError;
    flowError = fb->_errorMonitor.flowError;
    velocityError = fb->_errorMonitor.velocityError;

    /* --- Pressure diagnostics --- */
    /* Criteria was configured once in HYD_BeginSegment; only check here. */
    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP && pressureTolerance > 0.0) {
        isStartupPhase = HYD_IsStartupSuppressActive(elapsed, fb->_pressureCriteria.startupSuppressTime);
        if (HYD_DiagnosticCriteria_CheckPressure(&pressureResult,
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
            HYD_DiagnosticCriteria_CheckFaultEscalation(&pressureResult,
                                                         &fb->_pressureCriteria,
                                                         &fb->_pressureCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (pressureResult.severity == HYD_DIAG_SEVERITY_FAULT) {
                overPressure = true;  /* Ensure fault is propagated */
            }
        }
    } else {
        HYD_DiagnosticCriteria_ResetState(&fb->_pressureCriteriaState);
    }

    /* --- Flow diagnostics --- */
    /* Removed hardcoded elapsed > 0.1 gate; the criteria layer's
     * startup/switch suppress mechanism handles transient suppression. */
    if (flowTolerance > 0.0 && executionReference->flowReference > 0.0) {
        isStartupPhase = HYD_IsStartupSuppressActive(elapsed, fb->_flowCriteria.startupSuppressTime);
        if (HYD_DiagnosticCriteria_CheckFlow(&flowResult,
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
            HYD_DiagnosticCriteria_CheckFaultEscalation(&flowResult,
                                                         &fb->_flowCriteria,
                                                         &fb->_flowCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (flowResult.severity == HYD_DIAG_SEVERITY_FAULT) {
                flowDeviation = true;
            }
        }
    } else {
        HYD_DiagnosticCriteria_ResetState(&fb->_flowCriteriaState);
    }

    /* --- Velocity diagnostics --- */
    /* No longer unconditionally reset; check when velocity tolerance is configured. */
    if (velocityTolerance > 0.0) {
        isStartupPhase = HYD_IsStartupSuppressActive(elapsed, fb->_velocityCriteria.startupSuppressTime);
        if (HYD_DiagnosticCriteria_CheckVelocity(&velocityResult,
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
            HYD_DiagnosticCriteria_CheckFaultEscalation(&velocityResult,
                                                         &fb->_velocityCriteria,
                                                         &fb->_velocityCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (velocityResult.severity == HYD_DIAG_SEVERITY_FAULT) {
                velocityDeviation = true;
            }
        }
    } else {
        HYD_DiagnosticCriteria_ResetState(&fb->_velocityCriteriaState);
    }

    /* --- Pressure ceiling diagnostics (Sprint 1 spec 1.3) ---
     * Evaluated under ALL HYD_ControlMode values whenever the segment
     * configures pressureCeiling > 0 AND the current position lies inside
     * the [pressureCeilingPositionStart, pressureCeilingPositionEnd] window
     * (or the window is degenerate, in which case always-on).
     *
     * The legacy pressure criteria pipeline (CheckPressure) measures error
     * against the closed-loop reference and only runs in PRESSURE_CLOSED_LOOP
     * mode; it cannot serve as a fixed-ceiling watchdog. We drive the criteria
     * state machine directly here using the same debounce + fault-escalation
     * primitives as the standard channels:
     *   - debounce via state->lastTriggered + state->triggerStartTime
     *   - WARNING -> FAULT escalation via HYD_DiagnosticCriteria_CheckFaultEscalation
     *     (state->warningActive + state->warningStartTime)
     *
     * IMPORTANT: We do NOT pass _pressureCeilingCriteria through
     * HYD_ConfigureSegmentCriteria; that helper would clobber the
     * diagnosticCode / protectionAction / severity fields that Task 2
     * carefully configured. Ceiling criteria are static across segments.
     */
    pressureCeilingValue = HYD_Segment_GetPressureCeiling(segment);
    if (pressureCeilingValue > 0.0) {
        pressureCeilingTolerance = HYD_Segment_GetPressureCeilingTolerance(segment);
        ceilingActive = HYD_Segment_PressureCeilingActiveAt(segment, fb->AXIS_REF.position);
        if (ceilingActive) {
            HYD_DiagnosticCriteriaState* ceilingState = &fb->_pressureCeilingCriteriaState;
            HYD_DiagnosticCriteria* ceilingCriteria = &fb->_pressureCeilingCriteria;
            HYD_BOOL ceilingStartupActive = HYD_IsStartupSuppressActive(elapsed, ceilingCriteria->startupSuppressTime);
            HYD_BOOL ceilingSwitchActive = HYD_IsSwitchSuppressActive(isSwitchPhase, elapsed, ceilingCriteria->switchSuppressTime);
            HYD_BOOL ceilingSuppressed =
                (ceilingCriteria->enableStartupSuppress && ceilingStartupActive) ||
                (ceilingCriteria->enableSwitchSuppress && ceilingSwitchActive);

            /* Latch: once the criteria has escalated to FAULT, keep the
             * pressureCeilingViolated BOOL set regardless of where the
             * current cycle's pressure sits relative to ceiling+tol. This
             * must be hoisted OUT of the breach branch below: if pressure
             * dips into the hysteresis band [ceiling, ceiling+tol] while
             * already escalated, control falls into the "hold" branch
             * which does not set the BOOL — without this hoist the latch
             * would drop on those cycles. Today the FAULT freeze in
             * PublishOutputs short-circuits UpdateExecutionDiagnostics
             * after the transition cycle, but the latch is still claimed
             * as an invariant (defense-in-depth).
             *
             * The criteria state's faultEscalated flag is cleared by the
             * ResetState calls in the strict-below-ceiling branch (line
             * below), the window-exit branch, and the no-ceiling branch —
             * so this latch correctly clears on recovery. ceilingResult
             * is a transient I/O struct passed to CheckFaultEscalation;
             * downstream priority logic reads pressureCeilingViolated and
             * the spec table in HYD_Diagnostics_SetEvent, not
             * ceilingResult, so we don't write FAULT severity here. */
            if (ceilingState->faultEscalated) {
                pressureCeilingViolated = true;
            }

            ceilingErrorValue = fb->AXIS_REF.pressure - (pressureCeilingValue + pressureCeilingTolerance);

            memset(&ceilingResult, 0, sizeof(ceilingResult));

            if (ceilingSuppressed) {
                /* Within startup/switch suppression window: do not arm the
                 * trigger but also do not reset prior state. Just hold. */
            } else if (ceilingErrorValue > 0.0) {
                /* Pressure exceeds ceiling+tol — apply debounce semantics
                 * matching the standard Check* functions. */
                if (!ceilingState->lastTriggered) {
                    ceilingState->lastTriggered = true;
                    ceilingState->triggerStartTime = fb->AXIS_REF.timestamp;
                }
                if ((fb->AXIS_REF.timestamp - ceilingState->triggerStartTime) >= ceilingCriteria->debounceTime) {
                    /* Debounce passed: WARNING (DERATE) latched. */
                    pressureCeilingExceeded = true;
                    /* Seed ceilingResult as INPUT to CheckFaultEscalation:
                     * the helper reads result->triggered (criteria.c:597)
                     * and result->severity (criteria.c:604) to decide
                     * whether to advance the warning timer. */
                    ceilingResult.triggered = true;
                    ceilingResult.severity = HYD_DIAG_SEVERITY_WARNING;

                    /* WARNING -> FAULT escalation after faultEscalationTime
                     * of sustained breach. The call updates state->
                     * faultEscalated on the transition cycle; the hoisted
                     * latch above turns that into the sticky
                     * pressureCeilingViolated BOOL across subsequent
                     * cycles (including hysteresis-band hold). */
                    (void)HYD_DiagnosticCriteria_CheckFaultEscalation(&ceilingResult,
                                                                       ceilingCriteria,
                                                                       ceilingState,
                                                                       fb->AXIS_REF.timestamp);
                }
            } else if (fb->AXIS_REF.pressure < pressureCeilingValue) {
                /* Pressure dipped strictly below ceiling (not just into the
                 * tolerance band). Use this stricter threshold for hysteresis
                 * so jitter near ceiling+tol does not toggle the diagnostic.
                 * ResetState clears state->faultEscalated, which in turn
                 * drops the hoisted pressureCeilingViolated latch on the
                 * next cycle. */
                HYD_DiagnosticCriteria_ResetState(ceilingState);
            }
            /* If pressure is between ceiling and ceiling+tol, we hold state:
             * an already-armed debounce does not reset, but we also do not
             * count toward escalation. This produces the desired hysteresis
             * band [ceiling, ceiling+tol]. The hoisted latch above keeps
             * pressureCeilingViolated set on these hold cycles when the
             * criteria is already escalated. */
        } else {
            /* Position is outside the configured window: reset state so the
             * next entry starts fresh and does not carry stale duration. */
            HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
        }
    } else {
        /* Segment did not configure a ceiling: keep state clean. */
        HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
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
     * Apply to all HYD_MODE_POSITION segments (not just HYD_END_POSITION),
     * because position tracking deviation is relevant whenever the planner
     * is driving toward a target position, regardless of end condition. */
    if (segment->mode == HYD_MODE_POSITION &&
        positionTolerance > 0.0 &&
        velocityReferenceAbs < (velocityTolerance > 0.0 ? velocityTolerance : 1.0) &&
        actualVelocityAbs < (velocityTolerance > 0.0 ? velocityTolerance : 1.0)) {
        isStartupPhase = HYD_IsStartupSuppressActive(elapsed, fb->_positionCriteria.startupSuppressTime);
        if (HYD_DiagnosticCriteria_CheckPosition(&positionResult,
                                                 &fb->_errorMonitor,
                                                 &fb->_positionCriteria,
                                                 &fb->_positionCriteriaState,
                                                 fb->AXIS_REF.timestamp,
                                                 elapsed,
                                                 fb->_positionCriteria.enableStartupSuppress && isStartupPhase,
                                                 isSwitchPhase)) {
            positionDeviation = true;
            /* Check fault escalation */
            HYD_DiagnosticCriteria_CheckFaultEscalation(&positionResult,
                                                         &fb->_positionCriteria,
                                                         &fb->_positionCriteriaState,
                                                         fb->AXIS_REF.timestamp);
            if (positionResult.severity == HYD_DIAG_SEVERITY_FAULT) {
                positionDeviation = true;
            }
        }
    } else {
        HYD_DiagnosticCriteria_ResetState(&fb->_positionCriteriaState);
    }

    /* --- Timeout diagnostics ---
     * Uses criteria layer for consistency with startup/switch suppress,
     * debounce, and unified diagnostic channel semantics. */
    {
        HYD_DiagnosticResult timeoutResult;
        HYD_BOOL isStartupPhaseTimeout = HYD_IsStartupSuppressActive(elapsed, fb->_timeoutCriteria.startupSuppressTime);
        if (HYD_DiagnosticCriteria_CheckTimeout(&timeoutResult,
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
     * Ordering rule: FAULT-level codes outrank WARNING-level codes; among
     * the same severity, ceiling violations outrank legacy pressure
     * deviations.  Concretely:
     *   1. timeout (FAULT)
     *   2. pressureCeilingViolated (FAULT)
     *   3. overPressure when fault-escalated (FAULT)
     *   4. pressureCeilingExceeded (WARNING)
     *   5. overPressure (WARNING)
     *   6. underPressure (WARNING or FAULT depending on escalation)
     *   7. flowDeviation
     *   8. positionDeviation
     *   9. velocityDeviation
     */
    if (timeout) {
        priorityCode = HYD_DIAG_CODE_TIMEOUT;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (pressureCeilingViolated) {
        priorityCode = HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (overPressure && pressureResult.severity == HYD_DIAG_SEVERITY_FAULT) {
        priorityCode = HYD_DIAG_CODE_OVER_PRESSURE;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (pressureCeilingExceeded) {
        priorityCode = HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED;
        prioritySeverity = HYD_DIAG_SEVERITY_WARNING;
    } else if (overPressure) {
        priorityCode = HYD_DIAG_CODE_OVER_PRESSURE;
        prioritySeverity = HYD_DIAG_SEVERITY_WARNING;
    } else if (underPressure) {
        priorityCode = HYD_DIAG_CODE_UNDER_PRESSURE;
        prioritySeverity = (pressureResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (flowDeviation) {
        priorityCode = HYD_DIAG_CODE_FLOW_DEVIATION;
        prioritySeverity = (flowResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (positionDeviation) {
        priorityCode = HYD_DIAG_CODE_POSITION_DEVIATION;
        prioritySeverity = (positionResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (velocityDeviation) {
        priorityCode = HYD_DIAG_CODE_VELOCITY_DEVIATION;
        prioritySeverity = (velocityResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    }

    if (priorityCode != HYD_DIAG_CODE_NONE) {
        HYD_Diagnostics_SetEvent(&fb->DIAGNOSTIC, priorityCode, prioritySeverity);
    } else {
        HYD_Diagnostics_Clear(&fb->DIAGNOSTIC);
    }

    fb->DIAGNOSTIC.overPressure = overPressure;
    fb->DIAGNOSTIC.underPressure = underPressure;
    fb->DIAGNOSTIC.flowDeviation = flowDeviation;
    fb->DIAGNOSTIC.positionDeviation = positionDeviation;
    fb->DIAGNOSTIC.velocityDeviation = velocityDeviation;
    fb->DIAGNOSTIC.timeout = timeout;
    fb->DIAGNOSTIC.pressureCeilingExceeded = pressureCeilingExceeded;
    fb->DIAGNOSTIC.pressureCeilingViolated = pressureCeilingViolated;
    fb->DIAGNOSTIC.pressureError = pressureError;
    fb->DIAGNOSTIC.flowError = flowError;
    fb->DIAGNOSTIC.velocityError = velocityError;
    fb->DIAGNOSTIC.flags = HYD_Diagnostics_GetFlagMask(&fb->DIAGNOSTIC);

    /* Clear switch phase using the unified transition window end time,
     * not tied to any single criteria's startup suppress time. */
    if (fb->_isSwitchPhase && fb->_switchSuppressEndTime > 0.0 && elapsed >= fb->_switchSuppressEndTime) {
        fb->_isSwitchPhase = false;
    }
}

static void HYD_MotionControlFB_RunRunningState(HYD_MotionControlFB* fb) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsed;
    HYD_REAL deltaTime;
    HYD_RampControllerOutput rampOutput;
    HYD_MotionPlannerOutput plannerOutput;
    HYD_PressureControllerOutput pressureOutput;
    HYD_PumpConverterOutput pumpOutput;
    HYD_OutputLimiterInput limiterInput;
    HYD_OutputLimiterOutput limiterOutput;
    HYD_ExecutionReference executionReference;

    if (fb == NULL) {
        return;
    }

    if (!fb->_activeSegmentValid) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_INTERNAL_ERROR,
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    segment = &fb->_activeSegment;

    if ((fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_RECIPE &&
         fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) ||
        (fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
         fb->STATE.currentSegmentIndex != HYD_MAX_SEGMENTS) ||
        (fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_NONE)) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_INTERNAL_ERROR,
                        fb->AXIS_REF.timestamp,
                        NULL,
                        &fb->STATE.references);
        return;
    }

    if (!HYD_MotionUtils_AxisRefIsValid(&fb->AXIS_REF)) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_SENSOR_FAULT,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }

    if (fb->_lastFeedbackTimestamp >= 0.0 && fb->AXIS_REF.timestamp < fb->_lastFeedbackTimestamp) {
        HYD_StateReporter_ReportFault(fb,
                        HYD_DIAG_CODE_TIMESTAMP_ROLLBACK,
                        fb->AXIS_REF.timestamp,
                        segment,
                        &fb->STATE.references);
        return;
    }
    deltaTime = (fb->_lastFeedbackTimestamp >= 0.0)
        ? fb->AXIS_REF.timestamp - fb->_lastFeedbackTimestamp
        : 0.0;

    {
        HYD_REAL effectiveGain, effectiveLimit;
        if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
            effectiveGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
            effectiveLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
        } else {
            effectiveGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
            effectiveLimit = fb->PUMP_SPEED_LIMIT;
        }
        if (!HYD_PumpConverter_ValidateConfig(effectiveGain,
                                              effectiveLimit,
                                              &code) ||
            !HYD_RecipeValidator_ValidateSegment(segment,
                                                 fb->STATE.currentSegmentIndex,
                                                 &code,
                                                 &fb->cylinderConfig)) {
            HYD_StateReporter_ReportFault(fb,
                            code,
                            fb->AXIS_REF.timestamp,
                            segment,
                            &fb->STATE.references);
            return;
        }
    }

    elapsed = fb->AXIS_REF.timestamp - fb->_segmentStartTime;
    if (elapsed < 0.0) {
        elapsed = 0.0;
    }

    HYD_ExecuteActiveSegmentControl(fb,
                                    segment,
                                    elapsed,
                                    deltaTime,
                                    &rampOutput,
                                    &plannerOutput,
                                    &pressureOutput,
                                    &pumpOutput,
                                    &executionReference);
    fb->_lastFeedbackTimestamp = fb->AXIS_REF.timestamp;
    HYD_UpdateExecutionDiagnostics(fb, segment, &executionReference, elapsed);
    {
        HYD_VpTransferResult vpResult;
        HYD_VpTransfer_Evaluate(segment, &fb->AXIS_REF, &executionReference, &vpResult);
        if (segment->vpTransferLatch && fb->STATE.vpTransferReady) {
            /* Latch mode: once triggered, hold until segment end (PrimeSegmentControllers clears it). */
        } else {
            fb->STATE.vpTransferReady = vpResult.ready;
            fb->STATE.vpTransferReason = (HYD_UINT8)vpResult.reason;
        }
    }

    limiterInput.requestedFlow = pumpOutput.commandFlow;
    limiterInput.requestedPumpSpeed = pumpOutput.pumpSpeed;
    if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
        limiterInput.flowToPumpSpeedGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
        limiterInput.pumpSpeedLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
    } else {
        limiterInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        limiterInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    }
    limiterInput.protectionAction = fb->DIAGNOSTIC.protectionAction;
    limiterInput.derateRatio = HYD_Segment_GetDerateRatio(segment);

    /*
     * 压力限制参数：
     * effectiveMaxPressure = 取 segment.maxPressure 和 fb.PRESSURE_LIMIT 中较小的非零值。
     * 两者都为 0 时不启用压力限制。
     */
    {
        HYD_REAL segMax = segment->maxPressure;
        HYD_REAL fbMax = fb->PRESSURE_LIMIT;
        HYD_REAL effective = 0.0;
        if (segMax > 0.0 && fbMax > 0.0) {
            effective = (segMax < fbMax) ? segMax : fbMax;
        } else if (segMax > 0.0) {
            effective = segMax;
        } else if (fbMax > 0.0) {
            effective = fbMax;
        }
        limiterInput.effectiveMaxPressure = effective;
    }
    limiterInput.actualPressure = fb->AXIS_REF.pressure;

    /* 软限位参数（使用电子尺位置反馈） */
    limiterInput.actualPosition = fb->AXIS_REF.position;
    limiterInput.strokeMm = fb->cylinderConfig.strokeMm;
    limiterInput.softLimitRetractMm = fb->cylinderConfig.softLimitRetractMm;
    limiterInput.softLimitBandMm = fb->cylinderConfig.softLimitBandMm;
    limiterInput.direction = segment->direction;
    limiterInput.currentTime = fb->AXIS_REF.timestamp;

    /* 使用带保护状态的扩展版本（支持压力限制 + 软限位 + debounce + 故障升级） */
    HYD_OutputLimiter_ExecuteWithProtection(&limiterInput, &fb->_limiterState, &limiterOutput);

    pumpOutput.commandFlow = limiterOutput.commandFlow;
    pumpOutput.pumpSpeed = limiterOutput.pumpSpeed;
    plannerOutput.targetFlow = limiterOutput.commandFlow;
    executionReference.flowReference = limiterOutput.commandFlow;

    /* 如果 limiter 报告了 FAULT 级诊断，升级 protectionAction 为 STOP */
    if (limiterOutput.diagnosticCode == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT ||
        limiterOutput.diagnosticCode == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED) {
        fb->DIAGNOSTIC.protectionAction = HYD_PROTECTION_ACTION_STOP;
        fb->DIAGNOSTIC.code = limiterOutput.diagnosticCode;
        fb->DIAGNOSTIC.severity = HYD_DIAG_SEVERITY_FAULT;
    } else if (limiterOutput.diagnosticCode != HYD_DIAG_CODE_NONE &&
               fb->DIAGNOSTIC.protectionAction < HYD_PROTECTION_ACTION_DERATE) {
        /* WARNING 级：仅在当前无更高优先级保护时设置 */
        fb->DIAGNOSTIC.code = limiterOutput.diagnosticCode;
        fb->DIAGNOSTIC.severity = HYD_DIAG_SEVERITY_WARNING;
    }

    fb->_lastCommandedFlow = pumpOutput.commandFlow;

    if (fb->_isStopping) {
        HYD_RunRunningStateStopping(fb, segment, &plannerOutput, &pumpOutput,
                                    &executionReference, &pressureOutput);
        return;
    }

    if (HYD_RunRunningStateBlendCutover(fb, segment, &executionReference)) {
        return;
    }

    if (HYD_RunRunningStateCompletion(fb, segment, &plannerOutput, &executionReference)) {
        return;
    }

    HYD_StateReporter_ReportExecution(fb,
                                      &plannerOutput,
                                      &pumpOutput,
                                      &executionReference,
                                      pressureOutput.appliedStrategy,
                                      &pressureOutput,
                                      &fb->DIAGNOSTIC);
    HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, &executionReference);

    fb->_simFeedback.targetPosition = segment->targetPosition;
    fb->_simFeedback.targetVelocity = plannerOutput.targetVelocity;
    fb->_simFeedback.targetFlow     = pumpOutput.commandFlow;
    fb->_simFeedback.targetPressure = executionReference.pressureReference;
    fb->_simFeedback.valid          = true;

    fb->SEGMENT_COMPLETED = false;
}

static void HYD_RunRunningStateStopping(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* segment,
                                        HYD_MotionPlannerOutput* plannerOutput,
                                        HYD_PumpConverterOutput* pumpOutput,
                                        HYD_ExecutionReference* executionReference,
                                        HYD_PressureControllerOutput* pressureOutput) {
    HYD_REAL stopElapsed = fb->AXIS_REF.timestamp - fb->_stopStartTime;
    HYD_REAL stopMag = fabs(fb->_stopStartVel);
    HYD_REAL stopSign = (fb->_stopStartVel >= 0.0f) ? 1.0f : -1.0f;
    HYD_REAL stopDeceleration = (fb->_stopDeceleration > 0.0f)
        ? fb->_stopDeceleration
        : ((segment->maxDeceleration > 0.0f) ? segment->maxDeceleration : segment->maxAcceleration);
    HYD_REAL decelMag = stopMag - stopDeceleration * stopElapsed;

    if (decelMag < 0.0f) {
        decelMag = 0.0f;
    }

    plannerOutput->targetVelocity = decelMag * stopSign;
    plannerOutput->targetFlow = HYD_ClampReal(decelMag * segment->velocityToFlowGain,
                                              0.0f,
                                              segment->maxFlow);
    pumpOutput->commandFlow = plannerOutput->targetFlow;
    {
        HYD_REAL effectiveGain, effectiveLimit;
        if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
            effectiveGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
            effectiveLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
        } else {
            effectiveGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
            effectiveLimit = fb->PUMP_SPEED_LIMIT;
        }
        pumpOutput->pumpSpeed = HYD_ClampReal(plannerOutput->targetFlow * effectiveGain,
                                              0.0f,
                                              effectiveLimit);
    }
    executionReference->flowReference = pumpOutput->commandFlow;
    executionReference->velocityReference = plannerOutput->targetVelocity;

    HYD_StateReporter_ReportExecution(fb,
                                      plannerOutput,
                                      pumpOutput,
                                      executionReference,
                                      pressureOutput->appliedStrategy,
                                      pressureOutput,
                                      &fb->DIAGNOSTIC);

    fb->_simFeedback.targetPosition = segment->targetPosition;
    fb->_simFeedback.targetVelocity = plannerOutput->targetVelocity;
    fb->_simFeedback.targetFlow = pumpOutput->commandFlow;
    fb->_simFeedback.targetPressure = executionReference->pressureReference;
    fb->_simFeedback.valid = true;

    if (decelMag < HYD_THRESH_STOP_DECEL_DONE_MAG &&
        fabs(fb->AXIS_REF.velocity) < HYD_THRESH_STOP_VEL_DONE_MAG) {
        fb->_isStopping = false;
        fb->_stopStartVel = 0.0f;
        fb->_stopDeceleration = 0.0f;
        fb->_directSessionState = HYD_DIRECT_SESSION_DONE;
        HYD_ClearDirectPendingSlot(fb);
        HYD_SafetyStateManager_ApplyIdleState(fb, true, false);
        HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_DONE);
    } else if (stopDeceleration > 0.0f) {
        /* C-4: stop-timeout safety net.
         * If the commanded deceleration ramp completes (decelMag ~= 0)
         * but feedback velocity never drops below the completion threshold
         * (stuck encoder / actuator), this branch would hang forever.
         * Threshold = HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT * ideal-stop + slack. */
        HYD_REAL idealStopTime = stopMag / stopDeceleration;
        HYD_REAL stopTimeoutLimit = HYD_THRESH_STOP_TIMEOUT_IDEAL_MULT * idealStopTime +
                                    HYD_THRESH_STOP_TIMEOUT_SLACK_S;
        if (stopElapsed > stopTimeoutLimit) {
            fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
            HYD_StateReporter_ReportFault(fb,
                                          HYD_DIAG_CODE_TIMEOUT,
                                          fb->AXIS_REF.timestamp,
                                          segment,
                                          &fb->STATE.references);
            HYD_SafetyStateManager_EnterFaultStop(fb);
        }
    }
}

/* Returns true if the caller should immediately return (cutover/fault transition occurred). */
static HYD_BOOL HYD_RunRunningStateBlendCutover(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                const HYD_ExecutionReference* executionReference) {
    if (fb->DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP) {
        fb->_directSessionState = HYD_DIRECT_SESSION_FAULT;
        HYD_SafetyStateManager_EnterFaultStop(fb);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }

    if (HYD_ShouldCutoverDirectBlend(fb, segment)) {
        HYD_RecordDirectExecutionCompleted(fb);
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, true);
        return true;
    }

    return false;
}

/* Returns true if the segment completed and the caller should return immediately. */
static HYD_BOOL HYD_RunRunningStateCompletion(HYD_MotionControlFB* fb,
                                              const HYD_MotionSegment* segment,
                                              const HYD_MotionPlannerOutput* plannerOutput,
                                              const HYD_ExecutionReference* executionReference) {
    HYD_SegmentCompletionContext completionContext;
    HYD_BOOL segmentCompleted;
    HYD_BOOL recipeFinished;
    HYD_SegmentSource completedSegmentSource;

    if (fb->_isDecelerating &&
        fabs(plannerOutput->targetVelocity) < HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        completedSegmentSource = fb->_activeSegmentSource;
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
            fb->_directPendingValid) {
            (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
            return true;
        }
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
            HYD_RecordDirectExecutionCompleted(fb);
        }
        recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
            (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
        HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }

    completionContext.segment = segment;
    completionContext.axisRef = &fb->AXIS_REF;
    completionContext.references = executionReference;
    completionContext.timestamp = fb->AXIS_REF.timestamp;
    completionContext.candidateStartTime = &fb->_completionCandidateStartTime;
    completionContext.candidateActive = &fb->_completionCandidateActive;
    segmentCompleted = HYD_SegmentCompletion_CheckWithContext(&completionContext);
    if (!segmentCompleted) {
        return false;
    }

    if (!fb->_isDecelerating &&
        segment->mode == HYD_MODE_SPEED_RAMP &&
        segment->endCondition != HYD_END_POSITION) {
        fb->_isDecelerating = true;
        fb->_decelStartTime = fb->AXIS_REF.timestamp;
        fb->_decelStartVel = fabs(plannerOutput->targetVelocity);
        return false;   /* continue execution to let deceleration take effect */
    }

    completedSegmentSource = fb->_activeSegmentSource;
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
        fb->_directPendingValid) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
        return true;
    }
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
        HYD_RecordDirectExecutionCompleted(fb);
    }
    recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
        (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
    HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
    HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
    HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
    return true;
}

static HYD_BOOL HYD_RequestStartCommand(HYD_MotionControlFB* fb,
                                        size_t segmentIndex,
                                        HYD_TIME timestamp) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    if (fb->STATE.faultActive) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_IsCommandAllowedInState(HYD_CMD_START, effectiveState)) {
        HYD_ReportCommandNotAllowed(fb,
                                    HYD_CMD_START,
                                    effectiveState,
                                    timestamp,
                                    (HYD_UINT)segmentIndex,
                                    &fb->STATE.references);
        return false;
    }

    if (!HYD_MotionValidator_ValidateStartRequest(fb, segmentIndex, &code)) {
        HYD_StateReporter_ReportDiagnostic(fb,
                           code,
                           HYD_DIAG_SEVERITY_WARNING,
                           timestamp,
                           HYD_ResolveStartSourceSegment(fb, segmentIndex, NULL, NULL),
                           NULL);
        return false;
    }

    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_START,
                                   (HYD_UINT)segmentIndex,
                                   timestamp,
                                   &fb->STATE.references);
}

static HYD_BOOL HYD_RequestHoldCommand(HYD_MotionControlFB* fb,
                                       HYD_TIME timestamp) {
    if (fb == NULL || fb->STATE.faultActive) {
        return false;
    }

    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_HOLD,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static HYD_BOOL HYD_RequestStopCommand(HYD_MotionControlFB* fb,
                                       HYD_TIME timestamp,
                                       HYD_REAL deceleration) {
    if (fb == NULL || fb->STATE.faultActive) {
        return false;
    }

    fb->_stopDeceleration = (deceleration > 0.0f) ? deceleration : 0.0f;
    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_STOP,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static HYD_BOOL HYD_RequestResumeCommand(HYD_MotionControlFB* fb,
                                         HYD_TIME timestamp) {
    if (fb == NULL || fb->STATE.faultActive) {
        return false;
    }

    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_RESUME,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static HYD_BOOL HYD_RequestAbortCommand(HYD_MotionControlFB* fb,
                                        HYD_TIME timestamp) {
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_IsCommandAllowedInState(HYD_CMD_ABORT, effectiveState)) {
        HYD_ReportCommandNotAllowed(fb,
                                    HYD_CMD_ABORT,
                                    effectiveState,
                                    timestamp,
                                    0U,
                                    &fb->STATE.references);
        return false;
    }

    HYD_ClearStartCommandInput(fb);
    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_ABORT,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

static void HYD_MotionControlFB_SampleCommands(HYD_MotionControlFB* fb) {
    HYD_BOOL startSignal;
    HYD_BOOL startEdge;
    HYD_UINT startSegmentIndex;

    if (fb == NULL) {
        return;
    }

    startSignal = fb->START_SEGMENT;
    startSegmentIndex = fb->START_SEGMENT_INDEX;
    startEdge = startSignal && !fb->_startSegmentSignalPrev;
    fb->_startSegmentSignalPrev = startSignal;
    HYD_ClearStartCommandInput(fb);

    if (startEdge) {
        (void)HYD_RequestStartCommand(fb, startSegmentIndex, fb->AXIS_REF.timestamp);
    }
}

void HYD_MotionControlFB_Init(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(fb, 0, sizeof(*fb));
    fb->_lastFeedbackTimestamp = -1.0;  /* sentinel: not yet valid */
    fb->USE_RECIPE = false;
    fb->FB_STATE = HYD_FB_STATE_IDLE;
    fb->STATE.currentSegmentIndex = HYD_MAX_SEGMENTS;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
    HYD_StateReporter_SetPlannedDirection(fb, HYD_DIRECTION_HOLD);
    HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_IDLE);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_ClearSegmentTag(fb);
    HYD_StateReporter_ResetDiagnosticRetention(fb);
    HYD_RampController_Init(&fb->_rampController, 0.0, 0.0);
    memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
    HYD_PressureController_ClearState(&fb->_pressureController);
    HYD_ClearPendingCommand(fb);
    HYD_ClearDirectPendingSlot(fb);
    HYD_StateReporter_RefreshStandardOutputs(fb);

    /* Initialize diagnostic criteria layer */
    HYD_ErrorMonitor_Init(&fb->_errorMonitor);
    HYD_DiagnosticCriteria_CreateDefaultPressureCriteria(&fb->_pressureCriteria);
    HYD_DiagnosticCriteria_InitState(&fb->_pressureCriteriaState);
    /* Pressure ceiling criteria mirrors _pressureCriteria but uses a shorter
     * debounce + faster fault escalation, since ceiling violations are
     * already "above the safe envelope" and should react more promptly. */
    HYD_DiagnosticCriteria_CreateDefaultPressureCriteria(&fb->_pressureCeilingCriteria);
    fb->_pressureCeilingCriteria.debounceTime = HYD_THRESH_PRESSURE_CEILING_DEBOUNCE_S;          /* 50 ms — react faster than normal pressure deviation */
    fb->_pressureCeilingCriteria.startupSuppressTime = HYD_THRESH_PRESSURE_CEILING_STARTUP_SUPPRESS_S;
    fb->_pressureCeilingCriteria.enableStartupSuppress = true;
    fb->_pressureCeilingCriteria.faultEscalationTime = HYD_THRESH_PRESSURE_CEILING_FAULT_ESCALATION_S;   /* 300 ms above ceiling -> escalate to FAULT/STOP */
    fb->_pressureCeilingCriteria.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    fb->_pressureCeilingCriteria.diagnosticCode = HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED;
    fb->_pressureCeilingCriteria.faultCode = HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED;
    HYD_DiagnosticCriteria_InitState(&fb->_pressureCeilingCriteriaState);
    HYD_DiagnosticCriteria_CreateDefaultFlowCriteria(&fb->_flowCriteria);
    HYD_DiagnosticCriteria_InitState(&fb->_flowCriteriaState);
    HYD_DiagnosticCriteria_CreateDefaultVelocityCriteria(&fb->_velocityCriteria);
    HYD_DiagnosticCriteria_InitState(&fb->_velocityCriteriaState);
    HYD_DiagnosticCriteria_CreateDefaultPositionCriteria(&fb->_positionCriteria);
    HYD_DiagnosticCriteria_InitState(&fb->_positionCriteriaState);
    HYD_DiagnosticCriteria_CreateDefaultTimeoutCriteria(&fb->_timeoutCriteria);
    HYD_DiagnosticCriteria_InitState(&fb->_timeoutCriteriaState);
    fb->_isSwitchPhase = false;
    fb->_switchSuppressEndTime = 0.0;

    /* Set parameter defaults (matching previous hardcoded values in motion_interface.c) */
    fb->_params.positionTolerance = 0.1f;
    fb->_params.velocityTolerance = 5.0f;
    fb->_params.flowTolerance = 1.0f;
    fb->_params.pressureTolerance = 0.5f;
    fb->_params.timeoutLimit = 0.0f; // default diabled, since not all recipes may have a meaningful timeout condition
    fb->_params.velocityToFlowGain = 0.2f;
    fb->_params.maxVelocity = 100.0f;
    fb->_params.maxAcceleration = 500.0f;
    fb->_params.maxDeceleration = 500.0f;
    fb->_params.maxFlow = 50.0f;
    fb->_params.pressureRampRate = 10.0f;
    fb->_params.pressureKp = 0.5f;
    fb->_params.pressureKpHigh = 0.0f;
    fb->_params.pressureGainBand = 0.2f;
    fb->_params.pressureKi = 0.1f;
    fb->_params.pressureKd = 0.01f;
    fb->_params.pressureIntegralLimit = 10.0f;
    fb->_params.pressureDeadband = 0.5f;
    fb->_params.pressureFilterAlpha = 0.5f;
    fb->_params.pressureDerivativeFilterAlpha = 0.5f;
    fb->_params.velocityKp = 0.0f;
    fb->_params.velocityDeadband = 0.0f;
    fb->_params.velocityCorrectionLimit = 0.0f;
    fb->_params.flowToPumpSpeedGain = 20.0f;
    fb->_params.pumpSpeedLimit = 1800.0f;
    fb->_params.pressureControllerType = (HYD_REAL)HYD_PRESSURE_CONTROLLER_PI;
    fb->_params.defaultTargetFlow = 5.0f;
    fb->_params.useSimulation = false;

    /* Legacy defaults — used when pumpConfig/cylinderConfig are not configured.
     * pumpConfig and cylinderConfig are zero after memset — inactive by default. */
    fb->FLOW_TO_PUMP_SPEED_GAIN = fb->_params.flowToPumpSpeedGain;
    fb->PUMP_SPEED_LIMIT = fb->_params.pumpSpeedLimit;
    fb->_useSimulation = fb->_params.useSimulation;
    fb->_configuredUseRecipe = false;
    fb->_directOwnerKind = HYD_DIRECT_CMD_NONE;
    fb->_directSessionState = HYD_DIRECT_SESSION_IDLE;
    fb->_directOwnerExecutionId = 0U;
    fb->_lastPreemptedExecutionId = 0U;
    fb->_lastPreemptedKind = HYD_DIRECT_CMD_NONE;
    fb->_lastCompletedExecutionId = 0U;
    fb->_lastCompletedKind = HYD_DIRECT_CMD_NONE;
    fb->_isStopping = false;
    fb->_stopStartTime = 0.0;
    fb->_stopStartVel = 0.0;
    fb->_stopDeceleration = 0.0;

}

void HYD_MotionControlFB_SoftReset(HYD_MotionControlFB* fb) {
    /* Persistent fields saved across soft reset */
    HYD_MotionSegment savedRecipe[HYD_MAX_SEGMENTS];
    HYD_UINT savedRecipeSize;
    HYD_MotionSegment savedDirectSegment;
    HYD_BOOL savedDirectSegmentValid;
    HYD_REAL savedFlowToPumpSpeedGain;
    HYD_REAL savedPumpSpeedLimit;
    HYD_BOOL savedConfiguredUseRecipe;
    HYD_BOOL savedUseSimulation;
    HYD_MotionFBParams savedParams;
    HYD_DiagnosticCriteria savedPressureCriteria;
    HYD_DiagnosticCriteria savedPressureCeilingCriteria;
    HYD_DiagnosticCriteria savedFlowCriteria;
    HYD_DiagnosticCriteria savedVelocityCriteria;
    HYD_DiagnosticCriteria savedPositionCriteria;

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
    savedConfiguredUseRecipe = fb->_configuredUseRecipe;
    savedUseSimulation = fb->_useSimulation;
    savedParams = fb->_params;
    savedPressureCriteria = fb->_pressureCriteria;
    savedPressureCeilingCriteria = fb->_pressureCeilingCriteria;
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
    fb->USE_RECIPE = savedConfiguredUseRecipe;
    fb->_params = savedParams;
    fb->FLOW_TO_PUMP_SPEED_GAIN = savedParams.flowToPumpSpeedGain;
    fb->PUMP_SPEED_LIMIT = savedParams.pumpSpeedLimit;
    fb->_useSimulation = savedUseSimulation;
    fb->_configuredUseRecipe = savedConfiguredUseRecipe;
    fb->_pressureCriteria = savedPressureCriteria;
    fb->_pressureCeilingCriteria = savedPressureCeilingCriteria;
    fb->_flowCriteria = savedFlowCriteria;
    fb->_velocityCriteria = savedVelocityCriteria;
    fb->_positionCriteria = savedPositionCriteria;
    fb->_directOwnerKind = HYD_DIRECT_CMD_NONE;
    fb->_directSessionState = HYD_DIRECT_SESSION_IDLE;
    fb->_directOwnerExecutionId = 0U;
    fb->_lastPreemptedExecutionId = 0U;
    fb->_lastPreemptedKind = HYD_DIRECT_CMD_NONE;
    fb->_lastCompletedExecutionId = 0U;
    fb->_lastCompletedKind = HYD_DIRECT_CMD_NONE;
    fb->_isStopping = false;
    fb->_stopStartTime = 0.0;
    fb->_stopStartVel = 0.0;
    fb->_stopDeceleration = 0.0;

    /* 4. Reinitialize framework-level state (same as Init) */
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
    HYD_StateReporter_SetPlannedDirection(fb, HYD_DIRECTION_HOLD);
    HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
    HYD_StateReporter_SetFault(fb, false);
    HYD_StateReporter_ClearSegmentTag(fb);
    HYD_StateReporter_ResetDiagnosticRetention(fb);
    HYD_RampController_Init(&fb->_rampController, 0.0, 0.0);
    memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
    HYD_PressureController_ClearState(&fb->_pressureController);
    HYD_ClearPendingCommand(fb);
    HYD_ClearDirectPendingSlot(fb);

    /* 5. Reinitialize diagnostic criteria states (but keep the criteria configs) */
    HYD_ErrorMonitor_Init(&fb->_errorMonitor);
    HYD_DiagnosticCriteria_InitState(&fb->_pressureCriteriaState);
    HYD_DiagnosticCriteria_InitState(&fb->_pressureCeilingCriteriaState);
    HYD_DiagnosticCriteria_InitState(&fb->_flowCriteriaState);
    HYD_DiagnosticCriteria_InitState(&fb->_velocityCriteriaState);
    HYD_DiagnosticCriteria_InitState(&fb->_positionCriteriaState);
    HYD_DiagnosticCriteria_InitState(&fb->_timeoutCriteriaState);
    fb->_isSwitchPhase = false;
    fb->_switchSuppressEndTime = 0.0;

    /* 6. Set state to READY (if recipe or direct segment is loaded) or IDLE */
    HYD_ResetReadyContextPreview(fb);
    HYD_StateReporter_SetIdleState(fb, false, false);
    HYD_StateReporter_RefreshStandardOutputs(fb);
}

HYD_BOOL HYD_MotionControlFB_LoadRecipe(HYD_MotionControlFB* fb,
                                        const HYD_MotionSegment* recipe,
                                        size_t recipeSize) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (!HYD_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code)) {
        memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
        fb->RECIPE_SIZE = 0U;
        HYD_PrepareRecipeLoadState(fb);
        HYD_StateReporter_ReportDiagnostic(fb,
                           code,
                           HYD_DIAG_SEVERITY_WARNING,
                           fb->AXIS_REF.timestamp,
                           NULL,
                           NULL);
        return false;
    }

    memset(fb->RECIPE, 0, sizeof(fb->RECIPE));
    memcpy(fb->RECIPE, recipe, recipeSize * sizeof(HYD_MotionSegment));
    fb->RECIPE_SIZE = recipeSize;
    HYD_PrepareRecipeLoadState(fb);
    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

HYD_BOOL HYD_MotionControlFB_LoadDirectSegment(HYD_MotionControlFB* fb,
                                              const HYD_MotionSegment* segment) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (!HYD_RecipeValidator_ValidateSegment(segment,
                                             HYD_MAX_SEGMENTS,
                                             &code,
                                             &fb->cylinderConfig)) {
        memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
        fb->DIRECT_SEGMENT_VALID = false;
        if (!fb->STATE.active && fb->FB_STATE != HYD_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
            HYD_ResetReadyContextPreview(fb);
            HYD_StateReporter_SetIdleState(fb, false, false);
            HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
            HYD_StateReporter_ClearSegmentTag(fb);
        }
        HYD_StateReporter_ReportDiagnostic(fb,
                           code,
                           HYD_DIAG_SEVERITY_WARNING,
                           fb->AXIS_REF.timestamp,
                           NULL,
                           NULL);
        return false;
    }

    fb->DIRECT_SEGMENT = *segment;
    fb->DIRECT_SEGMENT_VALID = true;
    if (!fb->STATE.active && fb->FB_STATE != HYD_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
        HYD_ResetReadyContextPreview(fb);
        HYD_StateReporter_SetIdleState(fb, false, false);
    }
    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    return true;
}

HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    HYD_BOOL savedUseRecipe;
    HYD_BOOL activeDirect;
    HYD_BOOL shouldAbort;

    if (fb == NULL || segment == NULL) {
        return false;
    }

    if (!HYD_RecipeValidator_ValidateSegment(segment, HYD_MAX_SEGMENTS, &code, &fb->cylinderConfig)) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           code,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           NULL,
                                           NULL);
        return false;
    }

    if (fb->_isStopping && bufferMode != HYD_BUFFER_MODE_ABORT) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                           &fb->STATE.references);
        return false;
    }

    activeDirect = fb->STATE.active &&
                   fb->_activeSegmentValid &&
                   fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT;
    shouldAbort = (bufferMode == HYD_BUFFER_MODE_ABORT &&
                   (fb->STATE.active || HYD_MotionControlFB_IsBusy(fb))) ||
                  (activeDirect && HYD_IsSegmentEndlessForBuffering(&fb->_activeSegment));

    if (shouldAbort) {
        if (activeDirect) {
            fb->_lastPreemptedExecutionId = fb->_directOwnerExecutionId;
            fb->_lastPreemptedKind = fb->_directOwnerKind;
        } else {
            fb->_lastPreemptedExecutionId = 0U;
            fb->_lastPreemptedKind = HYD_DIRECT_CMD_NONE;
        }
        HYD_ClearDirectPendingSlot(fb);
        fb->DIRECT_SEGMENT = *segment;
        fb->DIRECT_SEGMENT_VALID = true;
        savedUseRecipe = fb->USE_RECIPE;
        fb->USE_RECIPE = false;
        if (activeDirect || fb->STATE.active || HYD_MotionControlFB_IsBusy(fb)) {
            HYD_AbortNow(fb, timestamp);
        }
        (void)HYD_BeginSegment(fb, 0U, timestamp);
        fb->USE_RECIPE = savedUseRecipe;
        return true;
    }

    if (bufferMode != HYD_BUFFER_MODE_ABORT &&
        (activeDirect || fb->STATE.active || HYD_MotionControlFB_IsBusy(fb))) {
        if (fb->_directPendingValid) {
            HYD_StateReporter_ReportDiagnostic(fb,
                                               HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                               HYD_DIAG_SEVERITY_WARNING,
                                               timestamp,
                                               &fb->_activeSegment,
                                               &fb->STATE.references);
            return false;
        }
        fb->_directPendingSegment = *segment;
        fb->_directPendingKind = HYD_InferDirectCommandKindFromSegment(segment);
        fb->_directPendingBufferMode = bufferMode;
        fb->_directPendingValid = true;
        (void)HYD_TryCreateDirectBlendContext(fb, bufferMode, segment);
        return true;
    }

    fb->DIRECT_SEGMENT = *segment;
    fb->DIRECT_SEGMENT_VALID = true;
    savedUseRecipe = fb->USE_RECIPE;
    fb->USE_RECIPE = false;
    if (!HYD_MotionControlFB_StartSegment(fb, 0U, timestamp)) {
        fb->USE_RECIPE = savedUseRecipe;
        return false;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return true;
}

void HYD_MotionControlFB_ClearDirectSegment(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    memset(&fb->DIRECT_SEGMENT, 0, sizeof(fb->DIRECT_SEGMENT));
    fb->DIRECT_SEGMENT_VALID = false;
    if (!fb->STATE.active && fb->FB_STATE != HYD_FB_STATE_HOLD && !fb->STATE.finished && !fb->SEGMENT_COMPLETED) {
        HYD_ResetReadyContextPreview(fb);
        HYD_StateReporter_SetIdleState(fb, false, false);
        HYD_StateReporter_SetSegmentSource(fb, HYD_SEGMENT_SOURCE_NONE);
        HYD_StateReporter_ClearSegmentTag(fb);
    }
}

HYD_BOOL HYD_MotionControlFB_StartSegment(HYD_MotionControlFB* fb,
                                          size_t segmentIndex,
                                          HYD_TIME timestamp) {
    return HYD_RequestStartCommand(fb, segmentIndex, timestamp);
}

HYD_BOOL HYD_MotionControlFB_NextSegment(HYD_MotionControlFB* fb, HYD_TIME timestamp) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    if (fb == NULL) {
        return false;
    }

    if (fb->STATE.faultActive) {
        return false;
    }

    if (!HYD_ValidateNextRequest(fb, &code)) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           code,
                                           (code == HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED)
                                               ? HYD_DIAG_SEVERITY_INFO
                                               : HYD_DIAG_SEVERITY_WARNING,
                                           timestamp,
                                           (fb->STATE.currentSegmentIndex < fb->RECIPE_SIZE)
                                               ? &fb->RECIPE[fb->STATE.currentSegmentIndex]
                                               : NULL,
                                           &fb->STATE.references);
        return false;
    }

    return HYD_RequestCommandQueue(fb,
                                   HYD_CMD_NEXT,
                                   0U,
                                   timestamp,
                                   &fb->STATE.references);
}

HYD_BOOL HYD_MotionControlFB_Hold(HYD_MotionControlFB* fb) {
    return HYD_RequestHoldCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HYD_BOOL HYD_MotionControlFB_Resume(HYD_MotionControlFB* fb) {
    return HYD_RequestResumeCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HYD_BOOL HYD_MotionControlFB_Stop(HYD_MotionControlFB* fb,
                                  HYD_TIME timestamp,
                                  HYD_REAL deceleration) {
    return HYD_RequestStopCommand(fb, timestamp, deceleration);
}

HYD_BOOL HYD_MotionControlFB_Abort(HYD_MotionControlFB* fb) {
    return HYD_RequestAbortCommand(fb, (fb != NULL) ? fb->AXIS_REF.timestamp : 0.0);
}

HYD_BOOL HYD_MotionControlFB_AcknowledgeDiagnostics(HYD_MotionControlFB* fb) {
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_IsCommandAllowedInState(HYD_CMD_ACK, effectiveState)) {
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
    HYD_StateReporter_ClearCurrentDiagnostic(fb);
    HYD_StateReporter_ClearDiagnosticRetentionOnly(fb);
    return true;
}

void HYD_MotionControlFB_Cycle(HYD_MotionControlFB* fb) {
    HYD_FbCommand processedCommand;
    HYD_BOOL allowRunningExecution;

    if (fb == NULL) {
        return;
    }

    fb->SEGMENT_CHANGED = false;

    if (fb->RESET) {
        HYD_MotionControlFB_SoftReset(fb);
        return;
    }

    allowRunningExecution = HYD_MotionControlFB_ConsumePendingCommand(fb, &processedCommand);
    HYD_MotionControlFB_RunStateMachine(fb, allowRunningExecution);
    HYD_MotionControlFB_PublishOutputs(fb, processedCommand == HYD_CMD_NONE);
}

void HYD_MotionControlFB_Scan(HYD_MotionControlFB* fb) {
    HYD_MotionControlFB_SampleCommands(fb);
    HYD_MotionControlFB_Cycle(fb);
}

void HYD_MotionControlFB_Execute(HYD_MotionControlFB* fb) {
    HYD_MotionControlFB_Scan(fb);
}

HYD_BOOL HYD_MotionControlFB_ReadParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_REAL* value)
{
    if (fb == NULL || value == NULL) return false;
    if (paramNumber < 0 || paramNumber >= HYD_PARAM_COUNT) return false;
    if (paramNumber == HYD_PARAM_USE_SIMULATION) return false;

    switch ((HYD_ParameterNumber)paramNumber) {
        case HYD_PARAM_POSITION_TOLERANCE:             *value = fb->_params.positionTolerance; break;
        case HYD_PARAM_VELOCITY_TOLERANCE:             *value = fb->_params.velocityTolerance; break;
        case HYD_PARAM_FLOW_TOLERANCE:                 *value = fb->_params.flowTolerance; break;
        case HYD_PARAM_PRESSURE_TOLERANCE:             *value = fb->_params.pressureTolerance; break;
        case HYD_PARAM_TIMEOUT_LIMIT:                  *value = fb->_params.timeoutLimit; break;
        case HYD_PARAM_VELOCITY_TO_FLOW_GAIN:          *value = fb->_params.velocityToFlowGain; break;
        case HYD_PARAM_MAX_VELOCITY:                   *value = fb->_params.maxVelocity; break;
        case HYD_PARAM_MAX_ACCELERATION:               *value = fb->_params.maxAcceleration; break;
        case HYD_PARAM_MAX_DECELERATION:               *value = fb->_params.maxDeceleration; break;
        case HYD_PARAM_MAX_FLOW:                       *value = fb->_params.maxFlow; break;
        case HYD_PARAM_PRESSURE_RAMP_RATE:             *value = fb->_params.pressureRampRate; break;
        case HYD_PARAM_PRESSURE_KP:                    *value = fb->_params.pressureKp; break;
        case HYD_PARAM_PRESSURE_KP_HIGH:               *value = fb->_params.pressureKpHigh; break;
        case HYD_PARAM_PRESSURE_GAIN_BAND:             *value = fb->_params.pressureGainBand; break;
        case HYD_PARAM_PRESSURE_KI:                    *value = fb->_params.pressureKi; break;
        case HYD_PARAM_PRESSURE_KD:                    *value = fb->_params.pressureKd; break;
        case HYD_PARAM_PRESSURE_INTEGRAL_LIMIT:        *value = fb->_params.pressureIntegralLimit; break;
        case HYD_PARAM_PRESSURE_DEADBAND:              *value = fb->_params.pressureDeadband; break;
        case HYD_PARAM_PRESSURE_FILTER_ALPHA:          *value = fb->_params.pressureFilterAlpha; break;
        case HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA: *value = fb->_params.pressureDerivativeFilterAlpha; break;
        case HYD_PARAM_VELOCITY_KP:                    *value = fb->_params.velocityKp; break;
        case HYD_PARAM_VELOCITY_DEADBAND:              *value = fb->_params.velocityDeadband; break;
        case HYD_PARAM_VELOCITY_CORRECTION_LIMIT:      *value = fb->_params.velocityCorrectionLimit; break;
        case HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN:        *value = fb->_params.flowToPumpSpeedGain; break;
        case HYD_PARAM_PUMP_SPEED_LIMIT:               *value = fb->_params.pumpSpeedLimit; break;
        case HYD_PARAM_PRESSURE_CONTROLLER_TYPE:       *value = fb->_params.pressureControllerType; break;
        case HYD_PARAM_DEFAULT_TARGET_FLOW:            *value = fb->_params.defaultTargetFlow; break;
        case HYD_PARAM_PUMP_DISPLACEMENT:              *value = fb->pumpConfig.displacementMlRev; break;
        case HYD_PARAM_PUMP_VOLUMETRIC_EFF:            *value = fb->pumpConfig.volumetricEfficiency; break;
        case HYD_PARAM_PUMP_MAX_SPEED:                 *value = fb->pumpConfig.maxSpeedRpm; break;
        case HYD_PARAM_CYLINDER_AREA_EXTEND:           *value = fb->cylinderConfig.areaExtendMm2; break;
        case HYD_PARAM_CYLINDER_AREA_RETRACT:          *value = fb->cylinderConfig.areaRetractMm2; break;
        case HYD_PARAM_CYLINDER_STROKE:                *value = fb->cylinderConfig.strokeMm; break;
        default: return false;
    }
    return true;
}

HYD_BOOL HYD_MotionControlFB_WriteParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_REAL value)
{
    if (fb == NULL) return false;
    if (paramNumber < 0 || paramNumber >= HYD_PARAM_COUNT) return false;
    if (paramNumber == HYD_PARAM_USE_SIMULATION) return false;

    switch ((HYD_ParameterNumber)paramNumber) {
        case HYD_PARAM_POSITION_TOLERANCE:             fb->_params.positionTolerance = value; break;
        case HYD_PARAM_VELOCITY_TOLERANCE:             fb->_params.velocityTolerance = value; break;
        case HYD_PARAM_FLOW_TOLERANCE:                 fb->_params.flowTolerance = value; break;
        case HYD_PARAM_PRESSURE_TOLERANCE:             fb->_params.pressureTolerance = value; break;
        case HYD_PARAM_TIMEOUT_LIMIT:                  fb->_params.timeoutLimit = value; break;
        case HYD_PARAM_VELOCITY_TO_FLOW_GAIN:          fb->_params.velocityToFlowGain = value; break;
        case HYD_PARAM_MAX_VELOCITY:                   fb->_params.maxVelocity = value; break;
        case HYD_PARAM_MAX_ACCELERATION:               fb->_params.maxAcceleration = value; break;
        case HYD_PARAM_MAX_DECELERATION:               fb->_params.maxDeceleration = value; break;
        case HYD_PARAM_MAX_FLOW:                       fb->_params.maxFlow = value; break;
        case HYD_PARAM_PRESSURE_RAMP_RATE:             fb->_params.pressureRampRate = value; break;
        case HYD_PARAM_PRESSURE_KP:                    fb->_params.pressureKp = value; break;
        case HYD_PARAM_PRESSURE_KP_HIGH:               fb->_params.pressureKpHigh = value; break;
        case HYD_PARAM_PRESSURE_GAIN_BAND:             fb->_params.pressureGainBand = value; break;
        case HYD_PARAM_PRESSURE_KI:                    fb->_params.pressureKi = value; break;
        case HYD_PARAM_PRESSURE_KD:                    fb->_params.pressureKd = value; break;
        case HYD_PARAM_PRESSURE_INTEGRAL_LIMIT:        fb->_params.pressureIntegralLimit = value; break;
        case HYD_PARAM_PRESSURE_DEADBAND:              fb->_params.pressureDeadband = value; break;
        case HYD_PARAM_PRESSURE_FILTER_ALPHA:          fb->_params.pressureFilterAlpha = value; break;
        case HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA: fb->_params.pressureDerivativeFilterAlpha = value; break;
        case HYD_PARAM_VELOCITY_KP:                    fb->_params.velocityKp = value; break;
        case HYD_PARAM_VELOCITY_DEADBAND:              fb->_params.velocityDeadband = value; break;
        case HYD_PARAM_VELOCITY_CORRECTION_LIMIT:      fb->_params.velocityCorrectionLimit = value; break;
        case HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN:
            fb->_params.flowToPumpSpeedGain = value;
            fb->FLOW_TO_PUMP_SPEED_GAIN = value;
            break;
        case HYD_PARAM_PUMP_SPEED_LIMIT:
            fb->_params.pumpSpeedLimit = value;
            fb->PUMP_SPEED_LIMIT = value;
            break;
        case HYD_PARAM_PRESSURE_CONTROLLER_TYPE:       fb->_params.pressureControllerType = value; break;
        case HYD_PARAM_DEFAULT_TARGET_FLOW:            fb->_params.defaultTargetFlow = value; break;
        case HYD_PARAM_PUMP_DISPLACEMENT:              fb->pumpConfig.displacementMlRev = value; break;
        case HYD_PARAM_PUMP_VOLUMETRIC_EFF:            fb->pumpConfig.volumetricEfficiency = value; break;
        case HYD_PARAM_PUMP_MAX_SPEED:                 fb->pumpConfig.maxSpeedRpm = value; break;
        case HYD_PARAM_CYLINDER_AREA_EXTEND:           fb->cylinderConfig.areaExtendMm2 = value; break;
        case HYD_PARAM_CYLINDER_AREA_RETRACT:          fb->cylinderConfig.areaRetractMm2 = value; break;
        case HYD_PARAM_CYLINDER_STROKE:                fb->cylinderConfig.strokeMm = value; break;
        default: return false;
    }
    return true;
}

HYD_DirectCommandKind HYD_MotionControlFB_GetDirectOwnerKind(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directOwnerKind : HYD_DIRECT_CMD_NONE;
}

HYD_DirectSessionState HYD_MotionControlFB_GetDirectSessionState(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directSessionState : HYD_DIRECT_SESSION_IDLE;
}

uint16_t HYD_MotionControlFB_GetDirectOwnerExecutionId(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directOwnerExecutionId : 0U;
}

HYD_BOOL HYD_MotionControlFB_WasExecutionPreempted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind) {
    return (fb != NULL) &&
           fb->_lastPreemptedExecutionId == executionId &&
           fb->_lastPreemptedKind == kind;
}

HYD_BOOL HYD_MotionControlFB_WasExecutionCompleted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind) {
    return (fb != NULL) &&
           fb->_lastCompletedExecutionId == executionId &&
           fb->_lastCompletedKind == kind;
}

HYD_BOOL HYD_MotionControlFB_ConsumeExecutionCompleted(HYD_MotionControlFB* fb,
                                                       uint16_t executionId,
                                                       HYD_DirectCommandKind kind) {
    if (fb == NULL ||
        fb->_lastCompletedExecutionId != executionId ||
        fb->_lastCompletedKind != kind) {
        return false;
    }

    fb->_lastCompletedExecutionId = 0U;
    fb->_lastCompletedKind = HYD_DIRECT_CMD_NONE;
    return true;
}

HYD_BOOL HYD_MotionControlFB_ApplyLiveUpdate(HYD_MotionControlFB* fb,
                                             const HYD_LiveUpdateRequest* request) {
    HYD_MotionSegment updated;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    if (fb == NULL || request == NULL || request->flags == 0U) {
        return false;
    }

    if (!fb->_activeSegmentValid ||
        !fb->STATE.active ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != request->ownerKind ||
        fb->_directOwnerExecutionId != request->ownerExecutionId) {
        if (fb->STATE.finished &&
            fb->_directOwnerKind == request->ownerKind &&
            fb->_directOwnerExecutionId == request->ownerExecutionId) {
            return true;
        }
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           fb->AXIS_REF.timestamp,
                                           fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                           &fb->STATE.references);
        return false;
    }

    updated = fb->_activeSegment;

    if ((request->flags & HYD_LIVE_UPDATE_TARGET_POSITION) != 0U) {
        if (updated.mode != HYD_MODE_POSITION) {
            return false;
        }
        updated.targetPosition = request->targetPosition;
    }

    if ((request->flags & HYD_LIVE_UPDATE_MAX_VELOCITY) != 0U) {
        if (updated.mode != HYD_MODE_POSITION &&
            updated.mode != HYD_MODE_SPEED_RAMP) {
            return false;
        }
        updated.maxVelocity = request->maxVelocity;
        updated.maxFlow = (request->maxVelocity > 0.0f)
            ? request->maxVelocity * updated.velocityToFlowGain
            : updated.maxFlow;
    }

    if ((request->flags & HYD_LIVE_UPDATE_ACCELERATION) != 0U) {
        if (updated.mode != HYD_MODE_POSITION &&
            updated.mode != HYD_MODE_SPEED_RAMP) {
            return false;
        }
        updated.maxAcceleration = request->maxAcceleration;
    }

    if ((request->flags & HYD_LIVE_UPDATE_DECELERATION) != 0U) {
        if (updated.mode != HYD_MODE_POSITION &&
            updated.mode != HYD_MODE_SPEED_RAMP) {
            return false;
        }
        updated.maxDeceleration = (request->maxDeceleration > 0.0f)
            ? request->maxDeceleration
            : updated.maxAcceleration;
    }

    if ((request->flags & HYD_LIVE_UPDATE_TARGET_PRESSURE) != 0U) {
        if (updated.mode != HYD_MODE_PRESSURE_CLOSED_LOOP) {
            return false;
        }
        updated.targetPressure = request->targetPressure;
    }

    if ((request->flags & HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE) != 0U) {
        if (updated.mode != HYD_MODE_PRESSURE_CLOSED_LOOP) {
            return false;
        }
        updated.pressureRampRate = request->pressureRampRate;
    }

    if (!HYD_RecipeValidator_ValidateSegment(&updated,
                                             fb->STATE.currentSegmentIndex,
                                             &code,
                                             &fb->cylinderConfig)) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           code,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           fb->AXIS_REF.timestamp,
                                           &fb->_activeSegment,
                                           &fb->STATE.references);
        return false;
    }

    fb->_activeSegment = updated;
    HYD_StateReporter_SetPlannedDirection(fb, fb->_activeSegment.direction);
    HYD_StateReporter_SetSegmentTag(fb, fb->_activeSegment.segmentTag);
    HYD_StateReporter_SetSegmentSource(fb, fb->_activeSegmentSource);
    HYD_StateReporter_ClearCurrentDiagnostic(fb);

    if (fb->_directPendingValid && fb->_directBlendContext.active) {
        (void)HYD_TryCreateDirectBlendContext(fb,
                                              fb->_directPendingBufferMode,
                                              &fb->_directPendingSegment);
    }

    return true;
}

HYD_BOOL HYD_MotionControlFB_ReadBoolParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL* value)
{
    if (fb == NULL || value == NULL) return false;
    if (paramNumber != HYD_PARAM_USE_SIMULATION) return false;

    *value = fb->_params.useSimulation;
    return true;
}

HYD_BOOL HYD_MotionControlFB_WriteBoolParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL value)
{
    if (fb == NULL) return false;
    if (paramNumber != HYD_PARAM_USE_SIMULATION) return false;

    fb->_params.useSimulation = value;
    fb->_useSimulation = value;
    return true;
}
