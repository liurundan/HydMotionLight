#include "safety_state_manager.h"
#include "state_reporter.h"
#include "pressure_controller.h"

extern void HYD_ClearDirectPendingSlot(HYD_MotionControlFB* fb);

static HYD_BOOL HYD_SafetyStateManager_HasSelectedStartSource(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (fb->USE_RECIPE) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

void HYD_SafetyStateManager_ResetRuntimeActuation(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    /* Sprint 2: Capture previous segment mode before clearing runtime state,
     * so that PrimeSegmentControllers can carry over velocity for bumpless
     * P->V / S->S transitions. Must be done here because _activeSegmentValid
     * is cleared by this function before the next HYD_BeginSegment call. */
    if (fb->_activeSegmentValid) {
        fb->_previousSegmentMode = fb->_activeSegment.mode;
    }

    HYD_PressureController_ClearState(&fb->_pressureController);
    fb->_lastFeedbackTimestamp = -1.0;  /* negative sentinel: not yet valid */
    fb->_segmentStartTime = 0.0;
    fb->_holdStateTime = 0.0;
    fb->_activeSegmentValid = false;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
}

void HYD_SafetyStateManager_ApplyIdleState(HYD_MotionControlFB* fb,
                                          HYD_BOOL finished,
                                          HYD_BOOL segmentCompleted) {
    if (fb == NULL) {
        return;
    }

    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_StateReporter_SetIdleState(fb, finished, segmentCompleted);
}

void HYD_SafetyStateManager_ApplyDisabledState(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ResetTransitionFlags(fb);
    HYD_StateReporter_ApplySafeOutputs(fb);
    fb->SEGMENT_COMPLETED = false;
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_DISABLED);

    if (fb->STATE.faultActive) {
        HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_STOP);
        HYD_StateReporter_SetStatus(fb, HYD_STATUS_FAULT);
        return;
    }

    if (fb->STATE.finished) {
        HYD_StateReporter_SetStatus(fb, HYD_STATUS_FINISHED);
    } else if (HYD_SafetyStateManager_HasSelectedStartSource(fb)) {
        HYD_StateReporter_SetStatus(fb, HYD_STATUS_READY);
    } else {
        HYD_StateReporter_SetStatus(fb, HYD_STATUS_IDLE);
    }
}

void HYD_SafetyStateManager_ApplyFaultHold(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_STOP);
    HYD_StateReporter_SetStatus(fb, HYD_STATUS_FAULT);
    HYD_StateReporter_SetFbState(fb, HYD_FB_STATE_FAULT);
}

void HYD_SafetyStateManager_EnterFaultStop(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_ClearDirectPendingSlot(fb);
    HYD_StateReporter_EnterFaultState(fb);
}
