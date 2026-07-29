#include <assert.h>
#include <math.h>
#include <string.h>

#include "motion_interface.h"
#include "toggle_mechanism_pool.h"

#define IEC_VAL(variable) ((variable).value)

extern HYD_MotionControlFB *__MK_GetPublic_MotionControlFB(int index);

static void create_until_terminal(HYD_CREATEMOTION *create)
{
    unsigned int scans = 0u;

    while (!IEC_VAL(create->DONE) && !IEC_VAL(create->ERROR)) {
        __mcl_cmd_CreateMotion(create);
        ++scans;
        assert(scans < 128u);
    }
}

static void test_zero_initialized_create_remains_direct(void)
{
    HYD_CREATEMOTION create;
    HYD_MotionControlFB *axis;

    __HydMotion_framework_Init();
    memset(&create, 0, sizeof(create));

    __mcl_cmd_CreateMotion(&create);

    assert(IEC_VAL(create.DONE));
    assert(!IEC_VAL(create.BUSY));
    assert(!IEC_VAL(create.ERROR));
    assert(IEC_VAL(create.AXISID) == 0);
    axis = __MK_GetPublic_MotionControlFB(0);
    assert(axis != NULL);
    assert(axis->mechanismType == HYD_MECHANISM_DIRECT);
    assert(axis->mechanismSlot == HYD_TOGGLE_SLOT_NONE);
}

static void test_toggle_create_validates_before_activation(void)
{
    HYD_CREATEMOTION create;
    HYD_MotionControlFB *axis;
    const HYD_ToggleGeometryConfig *raw;

    __HydMotion_framework_Init();
    memset(&create, 0, sizeof(create));
    IEC_VAL(create.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;

    __mcl_cmd_CreateMotion(&create);

    assert(!IEC_VAL(create.DONE));
    assert(IEC_VAL(create.BUSY));
    assert(!IEC_VAL(create.ERROR));
    assert(__MK_GetPublic_MotionControlFB(0) == NULL);

    create_until_terminal(&create);

    assert(IEC_VAL(create.DONE));
    assert(!IEC_VAL(create.BUSY));
    assert(!IEC_VAL(create.ERROR));
    assert(IEC_VAL(create.AXISID) == 0);
    axis = __MK_GetPublic_MotionControlFB(0);
    assert(axis != NULL);
    assert(axis->mechanismType == HYD_MECHANISM_FIVE_POINT_TOGGLE);
    assert(axis->mechanismSlot != HYD_TOGGLE_SLOT_NONE);
    raw = HYD_ToggleMechanismPool_GetRaw(axis->mechanismSlot);
    assert(raw != NULL);
    assert(fabsf(raw->dc - 378.0f) < 1e-5f);
    assert(HYD_ToggleMechanismPool_UsingDefaults(axis->mechanismSlot));
    assert(HYD_ToggleMechanismPool_GetVersion(axis->mechanismSlot) == 1u);
}

static void test_invalid_mechanism_type_does_not_consume_axis(void)
{
    HYD_CREATEMOTION invalid;
    HYD_CREATEMOTION direct;

    __HydMotion_framework_Init();
    memset(&invalid, 0, sizeof(invalid));
    IEC_VAL(invalid.MECHANISM_TYPE) = 99;

    __mcl_cmd_CreateMotion(&invalid);

    assert(IEC_VAL(invalid.ERROR));
    assert(!IEC_VAL(invalid.BUSY));
    assert(IEC_VAL(invalid.ERRORID) == HYD_DIAG_CODE_MECHANISM_TYPE_INVALID);
    assert(__MK_GetPublic_MotionControlFB(0) == NULL);

    memset(&direct, 0, sizeof(direct));
    __mcl_cmd_CreateMotion(&direct);
    assert(IEC_VAL(direct.DONE));
    assert(IEC_VAL(direct.AXISID) == 0);
}

static void test_pool_exhaustion_does_not_consume_axis(void)
{
    HYD_CREATEMOTION toggle;
    HYD_CREATEMOTION direct;
    HYD_UINT8 slots[HYD_MAX_TOGGLE_MECHANISMS];
    HYD_UINT8 index;

    __HydMotion_framework_Init();
    for (index = 0u; index < HYD_MAX_TOGGLE_MECHANISMS; ++index) {
        assert(HYD_ToggleMechanismPool_Reserve(index, &slots[index]));
    }

    memset(&toggle, 0, sizeof(toggle));
    IEC_VAL(toggle.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    __mcl_cmd_CreateMotion(&toggle);
    assert(IEC_VAL(toggle.ERROR));
    assert(IEC_VAL(toggle.ERRORID) == HYD_DIAG_CODE_MECHANISM_POOL_EXHAUSTED);
    assert(__MK_GetPublic_MotionControlFB(0) == NULL);

    memset(&direct, 0, sizeof(direct));
    __mcl_cmd_CreateMotion(&direct);
    assert(IEC_VAL(direct.DONE));
    assert(IEC_VAL(direct.AXISID) == 0);
}

static void test_validation_workspace_exhaustion_rolls_back(void)
{
    HYD_CREATEMOTION toggle;
    HYD_CREATEMOTION direct;
    HYD_UINT8 token;

    __HydMotion_framework_Init();
    assert(HYD_ToggleMechanismPool_AcquireValidation(&token));

    memset(&toggle, 0, sizeof(toggle));
    IEC_VAL(toggle.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    __mcl_cmd_CreateMotion(&toggle);
    assert(IEC_VAL(toggle.ERROR));
    assert(IEC_VAL(toggle.ERRORID) == HYD_DIAG_CODE_MECHANISM_VALIDATION_BUSY);

    memset(&direct, 0, sizeof(direct));
    __mcl_cmd_CreateMotion(&direct);
    assert(IEC_VAL(direct.DONE));
    assert(IEC_VAL(direct.AXISID) == 0);
}

static void test_framework_reinit_releases_inflight_resources(void)
{
    HYD_CREATEMOTION first;
    HYD_CREATEMOTION second;

    __HydMotion_framework_Init();
    memset(&first, 0, sizeof(first));
    IEC_VAL(first.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    __mcl_cmd_CreateMotion(&first);
    assert(IEC_VAL(first.BUSY));

    __HydMotion_framework_Init();
    memset(&second, 0, sizeof(second));
    IEC_VAL(second.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    create_until_terminal(&second);

    assert(IEC_VAL(second.DONE));
    assert(IEC_VAL(second.AXISID) == 0);
}

static void test_stale_transaction_cannot_alias_reinitialized_resources(void)
{
    HYD_CREATEMOTION stale;
    HYD_CREATEMOTION current;

    __HydMotion_framework_Init();
    memset(&stale, 0, sizeof(stale));
    IEC_VAL(stale.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    __mcl_cmd_CreateMotion(&stale);
    assert(IEC_VAL(stale.BUSY));

    __HydMotion_framework_Init();
    memset(&current, 0, sizeof(current));
    IEC_VAL(current.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    __mcl_cmd_CreateMotion(&current);
    assert(IEC_VAL(current.BUSY));

    __mcl_cmd_CreateMotion(&stale);
    assert(IEC_VAL(stale.ERROR));
    assert(!IEC_VAL(stale.DONE));
    assert(IEC_VAL(stale.ERRORID) == HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY);

    create_until_terminal(&current);
    assert(IEC_VAL(current.DONE));
    assert(IEC_VAL(current.AXISID) == 0);
}

int main(void)
{
    test_zero_initialized_create_remains_direct();
    test_toggle_create_validates_before_activation();
    test_invalid_mechanism_type_does_not_consume_axis();
    test_pool_exhaustion_does_not_consume_axis();
    test_validation_workspace_exhaustion_rolls_back();
    test_framework_reinit_releases_inflight_resources();
    test_stale_transaction_cannot_alias_reinitialized_resources();
    return 0;
}
