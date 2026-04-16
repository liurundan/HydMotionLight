#ifndef HDY_PROTECTION_MANAGER_H
#define HDY_PROTECTION_MANAGER_H

#include "motion_control.h"

/* Clears runtime actuation/controller state while preserving loaded recipe/configuration. */
void HDY_ProtectionManager_ResetRuntimeActuation(HDY_MotionControlFB* fb);

/* Applies a non-executing safe state with status resolved from finished/segmentCompleted. */
void HDY_ProtectionManager_ApplyIdleState(HDY_MotionControlFB* fb,
                                          HDY_BOOL finished,
                                          HDY_BOOL segmentCompleted);

/* Applies safe outputs when EN=false while preserving ready/finished/fault status semantics. */
void HDY_ProtectionManager_ApplyDisabledState(HDY_MotionControlFB* fb);

/* Re-applies the already-latched protected-stop hold state on subsequent cycles. */
void HDY_ProtectionManager_ApplyFaultHold(HDY_MotionControlFB* fb);

/* Enters protected-stop state immediately from the current executing context. */
void HDY_ProtectionManager_EnterFaultStop(HDY_MotionControlFB* fb);

#endif /* HDY_PROTECTION_MANAGER_H */
