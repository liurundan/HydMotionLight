#ifndef HYD_TOGGLE_MECHANISM_POOL_H
#define HYD_TOGGLE_MECHANISM_POOL_H

#include <stddef.h>
#include <stdint.h>

#include "toggle_kinematics.h"

#define HYD_TOGGLE_SLOT_NONE UINT8_MAX
#define HYD_TOGGLE_VALIDATION_NONE UINT8_MAX

#if (HYD_MAX_TOGGLE_MECHANISMS < 1) || \
    (HYD_MAX_TOGGLE_MECHANISMS > UINT8_MAX) || \
    (HYD_MAX_TOGGLE_MECHANISMS > HYD_MAX_AXIS_MOTION)
#error "HYD_MAX_TOGGLE_MECHANISMS must fit HYD_UINT8 and axis capacity"
#endif

#if (HYD_MAX_TOGGLE_VALIDATIONS < 1) || \
    (HYD_MAX_TOGGLE_VALIDATIONS > UINT8_MAX)
#error "HYD_MAX_TOGGLE_VALIDATIONS must fit in a non-sentinel HYD_UINT8"
#endif

/*
 * This pool is owned by the single cyclic motion-control context. Calls must
 * not race with an ISR or another task. Slot/token handles and returned
 * pointers become invalid immediately when their resource is released.
 */

void HYD_ToggleMechanismPool_Reset(void);
HYD_BOOL HYD_ToggleMechanismPool_Reserve(HYD_UINT8 ownerAxis,
                                         HYD_UINT8 *slot);
void HYD_ToggleMechanismPool_Release(HYD_UINT8 slot);
HYD_BOOL HYD_ToggleMechanismPool_Commit(
    HYD_UINT8 slot,
    const HYD_TogglePreparedConfig *prepared,
    HYD_BOOL usingDefaults);
const HYD_TogglePreparedConfig *HYD_ToggleMechanismPool_GetPrepared(
    HYD_UINT8 slot);
const HYD_ToggleGeometryConfig *HYD_ToggleMechanismPool_GetRaw(
    HYD_UINT8 slot);
HYD_UINT16 HYD_ToggleMechanismPool_GetVersion(HYD_UINT8 slot);
HYD_BOOL HYD_ToggleMechanismPool_UsingDefaults(HYD_UINT8 slot);
size_t HYD_ToggleMechanismPool_SlotSize(void);

HYD_BOOL HYD_ToggleMechanismPool_AcquireValidation(HYD_UINT8 *token);
HYD_ToggleValidation *HYD_ToggleMechanismPool_GetValidation(HYD_UINT8 token);
void HYD_ToggleMechanismPool_ReleaseValidation(HYD_UINT8 token);

#endif
