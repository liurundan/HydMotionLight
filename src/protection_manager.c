#include "protection_manager.h"
#include "state_reporter.h"
#include "pressure_controller.h"

static HDY_BOOL HDY_ProtectionManager_HasSelectedStartSource(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (fb->USE_RECIPE) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

void HDY_ProtectionManager_ResetRuntimeActuation(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_PressureController_ClearState(&fb->_pressureController);
    fb->_lastFeedbackTimestamp = 0.0;
    fb->_feedbackTimestampValid = false;
    fb->_segmentStartTime = 0.0;
    fb->_holdStateTime = 0.0;
    fb->_activeSegmentValid = false;
    fb->_activeSegmentSource = HDY_SEGMENT_SOURCE_NONE;
}

void HDY_ProtectionManager_ApplyIdleState(HDY_MotionControlFB* fb,
                                          HDY_BOOL finished,
                                          HDY_BOOL segmentCompleted) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    HDY_StateReporter_SetIdleState(fb, finished, segmentCompleted);
}

void HDY_ProtectionManager_ApplyDisabledState(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HDY_StateReporter_ResetTransitionFlags(fb);
    HDY_StateReporter_ApplySafeOutputs(fb);
    fb->SEGMENT_COMPLETED = false;
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_DISABLED);

    if (fb->FAULT) {
        HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_STOP);
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
        return;
    }

    if (fb->FINISHED) {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_FINISHED);
    } else if (HDY_ProtectionManager_HasSelectedStartSource(fb)) {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_READY);
    } else {
        HDY_StateReporter_SetStatus(fb, HDY_STATUS_IDLE);
    }
}

void HDY_ProtectionManager_ApplyFaultHold(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HDY_StateReporter_ApplySafeOutputs(fb);
    HDY_StateReporter_SetProtectionAction(fb, HDY_PROTECTION_ACTION_STOP);
    HDY_StateReporter_SetStatus(fb, HDY_STATUS_FAULT);
    HDY_StateReporter_SetFbState(fb, HDY_FB_STATE_FAULT);
}

void HDY_ProtectionManager_EnterFaultStop(HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    HDY_ProtectionManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HDY_StateReporter_EnterFaultState(fb);
}
