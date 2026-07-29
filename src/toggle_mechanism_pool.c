#include "toggle_mechanism_pool.h"

#include <string.h>

typedef struct {
    HYD_BOOL used;
    HYD_BOOL valid;
    HYD_BOOL usingDefaults;
    HYD_UINT8 ownerAxis;
    HYD_UINT16 configVersion;
    HYD_TogglePreparedConfig prepared;
} HYD_ToggleMechanismSlot;

typedef struct {
    HYD_BOOL used;
    HYD_ToggleValidation validation;
} HYD_ToggleValidationWorkspace;

static HYD_ToggleMechanismSlot
    toggle_slots[HYD_MAX_TOGGLE_MECHANISMS];
static HYD_ToggleValidationWorkspace
    validation_workspaces[HYD_MAX_TOGGLE_VALIDATIONS];

static HYD_BOOL slot_index_valid(HYD_UINT8 slot)
{
    return slot < HYD_MAX_TOGGLE_MECHANISMS;
}

static HYD_BOOL validation_token_valid(HYD_UINT8 token)
{
    return token < HYD_MAX_TOGGLE_VALIDATIONS;
}

void HYD_ToggleMechanismPool_Reset(void)
{
    memset(toggle_slots, 0, sizeof(toggle_slots));
    memset(validation_workspaces, 0, sizeof(validation_workspaces));
}

HYD_BOOL HYD_ToggleMechanismPool_Reserve(HYD_UINT8 ownerAxis,
                                         HYD_UINT8 *slot)
{
    HYD_UINT8 index;

    if ((slot == NULL) || (ownerAxis >= HYD_MAX_AXIS_MOTION)) {
        return 0;
    }

    for (index = 0u; index < HYD_MAX_TOGGLE_MECHANISMS; ++index) {
        if (toggle_slots[index].used &&
            (toggle_slots[index].ownerAxis == ownerAxis)) {
            return 0;
        }
    }

    for (index = 0u; index < HYD_MAX_TOGGLE_MECHANISMS; ++index) {
        if (!toggle_slots[index].used) {
            memset(&toggle_slots[index], 0, sizeof(toggle_slots[index]));
            toggle_slots[index].used = 1;
            toggle_slots[index].ownerAxis = ownerAxis;
            *slot = index;
            return 1;
        }
    }

    return 0;
}

void HYD_ToggleMechanismPool_Release(HYD_UINT8 slot)
{
    if (!slot_index_valid(slot)) {
        return;
    }

    memset(&toggle_slots[slot], 0, sizeof(toggle_slots[slot]));
}

HYD_BOOL HYD_ToggleMechanismPool_Commit(
    HYD_UINT8 slot,
    const HYD_TogglePreparedConfig *prepared,
    HYD_BOOL usingDefaults)
{
    HYD_TogglePreparedConfig next;
    HYD_UINT16 next_version;

    if (!slot_index_valid(slot) || !toggle_slots[slot].used ||
        (prepared == NULL)) {
        return 0;
    }

    next = *prepared;
    next_version = (HYD_UINT16)(toggle_slots[slot].configVersion + 1u);
    if (next_version == 0u) {
        next_version = 1u;
    }

    toggle_slots[slot].prepared = next;
    toggle_slots[slot].usingDefaults = usingDefaults ? 1 : 0;
    toggle_slots[slot].valid = 1;
    toggle_slots[slot].configVersion = next_version;
    return 1;
}

const HYD_TogglePreparedConfig *HYD_ToggleMechanismPool_GetPrepared(
    HYD_UINT8 slot)
{
    if (!slot_index_valid(slot) || !toggle_slots[slot].used ||
        !toggle_slots[slot].valid) {
        return NULL;
    }

    return &toggle_slots[slot].prepared;
}

const HYD_ToggleGeometryConfig *HYD_ToggleMechanismPool_GetRaw(
    HYD_UINT8 slot)
{
    const HYD_TogglePreparedConfig *prepared =
        HYD_ToggleMechanismPool_GetPrepared(slot);

    return (prepared == NULL) ? NULL : &prepared->raw;
}

HYD_UINT16 HYD_ToggleMechanismPool_GetVersion(HYD_UINT8 slot)
{
    if (!slot_index_valid(slot) || !toggle_slots[slot].used ||
        !toggle_slots[slot].valid) {
        return 0u;
    }

    return toggle_slots[slot].configVersion;
}

HYD_BOOL HYD_ToggleMechanismPool_UsingDefaults(HYD_UINT8 slot)
{
    if (!slot_index_valid(slot) || !toggle_slots[slot].used ||
        !toggle_slots[slot].valid) {
        return 0;
    }

    return toggle_slots[slot].usingDefaults;
}

size_t HYD_ToggleMechanismPool_SlotSize(void)
{
    return sizeof(HYD_ToggleMechanismSlot);
}

HYD_BOOL HYD_ToggleMechanismPool_AcquireValidation(HYD_UINT8 *token)
{
    HYD_UINT8 index;

    if (token == NULL) {
        return 0;
    }

    for (index = 0u; index < HYD_MAX_TOGGLE_VALIDATIONS; ++index) {
        if (!validation_workspaces[index].used) {
            memset(&validation_workspaces[index], 0,
                   sizeof(validation_workspaces[index]));
            validation_workspaces[index].used = 1;
            *token = index;
            return 1;
        }
    }

    return 0;
}

HYD_ToggleValidation *HYD_ToggleMechanismPool_GetValidation(HYD_UINT8 token)
{
    if (!validation_token_valid(token) ||
        !validation_workspaces[token].used) {
        return NULL;
    }

    return &validation_workspaces[token].validation;
}

void HYD_ToggleMechanismPool_ReleaseValidation(HYD_UINT8 token)
{
    if (!validation_token_valid(token)) {
        return;
    }

    memset(&validation_workspaces[token], 0,
           sizeof(validation_workspaces[token]));
}
