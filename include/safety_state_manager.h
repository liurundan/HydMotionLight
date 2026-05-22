#ifndef HYD_SAFETY_STATE_MANAGER_H
#define HYD_SAFETY_STATE_MANAGER_H

#include "motion_control.h"

/* Clears runtime actuation/controller state while preserving loaded recipe/configuration. */
void HYD_SafetyStateManager_ResetRuntimeActuation(HYD_MotionControlFB* fb);

/* Applies a non-executing safe state with status resolved from finished/segmentCompleted. */
void HYD_SafetyStateManager_ApplyIdleState(HYD_MotionControlFB* fb,
                                          HYD_BOOL finished,
                                          HYD_BOOL segmentCompleted);

/* Applies safe outputs when EN=false while preserving ready/finished/fault status semantics. */
void HYD_SafetyStateManager_ApplyDisabledState(HYD_MotionControlFB* fb);

/* Re-applies the already-latched protected-stop hold state on subsequent cycles. */
void HYD_SafetyStateManager_ApplyFaultHold(HYD_MotionControlFB* fb);

/* Enters protected-stop state immediately from the current executing context. */
void HYD_SafetyStateManager_EnterFaultStop(HYD_MotionControlFB* fb);

#endif /* HYD_SAFETY_STATE_MANAGER_H */
