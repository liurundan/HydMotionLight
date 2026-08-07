#include <assert.h>
#include <stdio.h>

#include "motion_control.h"
#include "toggle_mechanism_pool.h"

#define HYD_BASELINE_MOTION_FB_BYTES 3176U /* host ABI at commit f8bc1b9 */

static HYD_TogglePreparedConfig validated_default(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    return prepared;
}

static void test_all_slots_are_bounded_and_reusable(void)
{
    HYD_UINT8 slots[HYD_MAX_TOGGLE_MECHANISMS];
    HYD_UINT8 rejected = 0xa5u;
    HYD_UINT8 reused = HYD_TOGGLE_SLOT_NONE;
    HYD_UINT8 release_index = HYD_MAX_TOGGLE_MECHANISMS / 2u;
    HYD_UINT8 i;

    HYD_ToggleMechanismPool_Reset();

    assert(HYD_ToggleMechanismPool_Reserve(0u, &slots[0]));
    assert(slots[0] == 0u);
    assert(!HYD_ToggleMechanismPool_Reserve(0u, &rejected));
    assert(rejected == 0xa5u);

    for (i = 1u; i < HYD_MAX_TOGGLE_MECHANISMS; ++i) {
        assert(HYD_ToggleMechanismPool_Reserve(i, &slots[i]));
        assert(slots[i] == i);
    }

    for (i = 0u; i < HYD_MAX_TOGGLE_MECHANISMS; ++i) {
        assert(HYD_ToggleMechanismPool_GetPrepared(slots[i]) == NULL);
        assert(HYD_ToggleMechanismPool_GetRaw(slots[i]) == NULL);
        assert(HYD_ToggleMechanismPool_GetVersion(slots[i]) == 0u);
        assert(!HYD_ToggleMechanismPool_UsingDefaults(slots[i]));
    }

    assert(!HYD_ToggleMechanismPool_Reserve(0u, &rejected));
    assert(rejected == 0xa5u);

    HYD_ToggleMechanismPool_Release(slots[release_index]);
    assert(HYD_ToggleMechanismPool_Reserve(release_index, &reused));
    assert(reused == slots[release_index]);
}

static void test_atomic_commit_and_failed_candidate_isolation(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared = validated_default();
    HYD_TogglePreparedConfig before;
    HYD_TogglePreparedConfig invalid_output;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_UINT8 slot = HYD_TOGGLE_SLOT_NONE;

    HYD_ToggleMechanismPool_Reset();
    assert(HYD_ToggleMechanismPool_Reserve(3u, &slot));
    assert(HYD_ToggleMechanismPool_Commit(slot, &prepared, 1));
    assert(HYD_ToggleMechanismPool_GetPrepared(slot) != NULL);
    assert(HYD_ToggleMechanismPool_GetRaw(slot) ==
           &HYD_ToggleMechanismPool_GetPrepared(slot)->raw);
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 1u);
    assert(HYD_ToggleMechanismPool_UsingDefaults(slot));
    before = *HYD_ToggleMechanismPool_GetPrepared(slot);

    raw.dc = 400.0f;
    assert(!HYD_ToggleKinematics_ValidateBlocking(&raw, &invalid_output,
                                                  &error));
    assert(HYD_ToggleMechanismPool_GetPrepared(slot)->raw.dc == before.raw.dc);
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 1u);

    raw = HYD_ToggleKinematics_DefaultConfig();
    raw.dc = 377.5f;
    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    assert(HYD_ToggleMechanismPool_Commit(slot, &prepared, 0));
    assert(HYD_ToggleMechanismPool_GetRaw(slot)->dc == 377.5f);
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 2u);
    assert(!HYD_ToggleMechanismPool_UsingDefaults(slot));

    assert(!HYD_ToggleMechanismPool_Commit(slot, NULL, 0));
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 2u);
    assert(HYD_ToggleMechanismPool_SlotSize() >= sizeof(prepared));

    HYD_ToggleMechanismPool_Release(slot);
    assert(HYD_ToggleMechanismPool_GetPrepared(slot) == NULL);
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 0u);
}

static void test_validation_workspace_is_shared_and_reusable(void)
{
    HYD_UINT8 tokens[HYD_MAX_TOGGLE_VALIDATIONS];
    HYD_UINT8 rejected = 0x5au;
    HYD_UINT8 reused = HYD_TOGGLE_VALIDATION_NONE;
    HYD_UINT8 release_index = HYD_MAX_TOGGLE_VALIDATIONS / 2u;
    HYD_UINT8 i;
    HYD_ToggleValidation *workspace;

    HYD_ToggleMechanismPool_Reset();
    for (i = 0u; i < HYD_MAX_TOGGLE_VALIDATIONS; ++i) {
        assert(HYD_ToggleMechanismPool_AcquireValidation(&tokens[i]));
        assert(tokens[i] == i);
    }

    workspace = HYD_ToggleMechanismPool_GetValidation(tokens[release_index]);
    assert(workspace != NULL);
    workspace->nextPoint = 123u;

    assert(!HYD_ToggleMechanismPool_AcquireValidation(&rejected));
    assert(rejected == 0x5au);

    HYD_ToggleMechanismPool_ReleaseValidation(tokens[release_index]);
    assert(HYD_ToggleMechanismPool_GetValidation(tokens[release_index]) == NULL);
    assert(HYD_ToggleMechanismPool_AcquireValidation(&reused));
    assert(reused == tokens[release_index]);
    assert(HYD_ToggleMechanismPool_GetValidation(reused)->nextPoint == 0u);
}

static void test_version_wrap_never_publishes_reserved_zero(void)
{
    HYD_TogglePreparedConfig prepared = validated_default();
    HYD_UINT8 slot = HYD_TOGGLE_SLOT_NONE;
    unsigned int commit_count;

    HYD_ToggleMechanismPool_Reset();
    assert(HYD_ToggleMechanismPool_Reserve(0u, &slot));
    for (commit_count = 0u; commit_count < UINT16_MAX; ++commit_count) {
        assert(HYD_ToggleMechanismPool_Commit(slot, &prepared, 1));
    }
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == UINT16_MAX);

    prepared.raw.dc = 377.0f;
    assert(HYD_ToggleMechanismPool_Commit(slot, &prepared, 0));
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 1u);
    assert(HYD_ToggleMechanismPool_GetRaw(slot)->dc == 377.0f);
    assert(!HYD_ToggleMechanismPool_UsingDefaults(slot));
}

static void test_invalid_arguments_preserve_outputs(void)
{
    HYD_UINT8 slot = 0x33u;
    HYD_UINT8 token = 0x44u;

    HYD_ToggleMechanismPool_Reset();
    assert(!HYD_ToggleMechanismPool_Reserve(0u, NULL));
    assert(!HYD_ToggleMechanismPool_AcquireValidation(NULL));
    assert(HYD_ToggleMechanismPool_GetPrepared(HYD_TOGGLE_SLOT_NONE) == NULL);
    assert(HYD_ToggleMechanismPool_GetValidation(
               HYD_TOGGLE_VALIDATION_NONE) == NULL);
    assert(!HYD_ToggleMechanismPool_Reserve(HYD_MAX_AXIS_MOTION, &slot));
    assert(slot == 0x33u);

    while (HYD_ToggleMechanismPool_AcquireValidation(&token)) {
    }
    assert(token != HYD_TOGGLE_VALIDATION_NONE);
}

static void test_resource_budget(void)
{
    size_t slot_bytes = HYD_ToggleMechanismPool_SlotSize();
    size_t validation_bytes = sizeof(HYD_ToggleValidation);
    size_t motion_fb_bytes = sizeof(HYD_MotionControlFB);

    printf("toggle slot bytes=%zu validation bytes=%zu motion fb bytes=%zu\n",
           slot_bytes, validation_bytes, motion_fb_bytes);
    assert(slot_bytes <= 112U);
    assert(validation_bytes <= 160U);
    assert(motion_fb_bytes <= HYD_BASELINE_MOTION_FB_BYTES +
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
           56U
#else
           32U
#endif
    );
}

int main(void)
{
    test_all_slots_are_bounded_and_reusable();
    test_atomic_commit_and_failed_candidate_isolation();
    test_validation_workspace_is_shared_and_reusable();
    test_version_wrap_never_publishes_reserved_zero();
    test_invalid_arguments_preserve_outputs();
    test_resource_budget();
    printf("toggle mechanism pool tests passed\n");
    return 0;
}
