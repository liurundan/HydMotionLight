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

static int create_toggle_axis(void)
{
    HYD_CREATEMOTION create;

    memset(&create, 0, sizeof(create));
    IEC_VAL(create.MECHANISM_TYPE) = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    create_until_terminal(&create);
    assert(IEC_VAL(create.DONE));
    return (int)IEC_VAL(create.AXISID);
}

static void set_config_inputs(HYD_CONFIGURETOGGLEMECHANISM *configure,
                              int axis,
                              const HYD_ToggleGeometryConfig *raw)
{
    memset(configure, 0, sizeof(*configure));
    IEC_VAL(configure->AXISID) = (IEC_SINT)axis;
    IEC_VAL(configure->EXECUTE) = true;
    IEC_VAL(configure->LR) = raw->lr;
    IEC_VAL(configure->LF) = raw->lf;
    IEC_VAL(configure->LPF) = raw->lpf;
    IEC_VAL(configure->LPK) = raw->lpk;
    IEC_VAL(configure->LD) = raw->ld;
    IEC_VAL(configure->HF) = raw->hf;
    IEC_VAL(configure->HM) = raw->hm;
    IEC_VAL(configure->DC) = raw->dc;
    IEC_VAL(configure->SM) = raw->sm;
    IEC_VAL(configure->XHANDOFF) = raw->xHandoff;
    IEC_VAL(configure->SIGMA_K) = raw->sigmaK;
    IEC_VAL(configure->SIGN_B) = raw->signB;
    IEC_VAL(configure->TAU_S) = raw->tauS;
    IEC_VAL(configure->SIGMA_C) = raw->sigmaC;
}

static void configure_until_terminal(HYD_CONFIGURETOGGLEMECHANISM *configure)
{
    unsigned int scans = 0u;

    while (!IEC_VAL(configure->DONE) && !IEC_VAL(configure->ERROR)) {
        __mcl_cmd_ConfigureToggleMechanism(configure);
        ++scans;
        assert(scans < 128u);
    }
}

static HYD_READTOGGLEMECHANISM read_toggle(int axis)
{
    HYD_READTOGGLEMECHANISM read;

    memset(&read, 0, sizeof(read));
    IEC_VAL(read.AXISID) = (IEC_SINT)axis;
    IEC_VAL(read.ENABLE) = true;
    __mcl_cmd_ReadToggleMechanism(&read);
    assert(IEC_VAL(read.VALID));
    assert(!IEC_VAL(read.ERROR));
    return read;
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
    assert(raw->sigmaC == (int8_t)-1);
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

static void test_configure_latches_inputs_and_reads_committed_state(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_CONFIGURETOGGLEMECHANISM configure;
    HYD_READTOGGLEMECHANISM read;
    int axis;

    __HydMotion_framework_Init();
    axis = create_toggle_axis();
    raw.dc = 377.5f;
    raw.sigmaC = (int8_t)1;
    set_config_inputs(&configure, axis, &raw);

    __mcl_cmd_ConfigureToggleMechanism(&configure);
    assert(IEC_VAL(configure.BUSY));
    assert(!IEC_VAL(configure.DONE));
    assert(IEC_VAL(configure.CONFIG_VERSION) == 1u);

    read = read_toggle(axis);
    assert(fabs(IEC_VAL(read.DC) - 378.0) < 1e-9);
    assert(IEC_VAL(read.SIGMA_C) == -1);
    assert(IEC_VAL(read.CONFIG_VERSION) == 1u);

    IEC_VAL(configure.DC) = 400.0;
    IEC_VAL(configure.AXISID) = 99;
    configure_until_terminal(&configure);
    assert(IEC_VAL(configure.DONE));
    assert(IEC_VAL(configure.CONFIG_VERSION) == 2u);

    read = read_toggle(axis);
    assert(fabs(IEC_VAL(read.DC) - 377.5) < 1e-9);
    assert(IEC_VAL(read.SIGMA_C) == 1);
    assert(IEC_VAL(read.CONFIG_VERSION) == 2u);
    assert(!IEC_VAL(read.USING_DEFAULTS));
}

static void test_invalid_configure_preserves_committed_geometry(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_CONFIGURETOGGLEMECHANISM configure;
    HYD_READTOGGLEMECHANISM before;
    HYD_READTOGGLEMECHANISM after;
    int axis;

    __HydMotion_framework_Init();
    axis = create_toggle_axis();
    before = read_toggle(axis);

    raw.dc = 400.0f;
    set_config_inputs(&configure, axis, &raw);
    configure_until_terminal(&configure);
    assert(IEC_VAL(configure.ERROR));
    assert(IEC_VAL(configure.ERRORID) ==
           HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID);
    assert(IEC_VAL(configure.CONFIG_VERSION) ==
           IEC_VAL(before.CONFIG_VERSION));

    after = read_toggle(axis);
    assert(fabs(IEC_VAL(after.DC) - IEC_VAL(before.DC)) < 1e-9);
    assert(IEC_VAL(after.CONFIG_VERSION) == IEC_VAL(before.CONFIG_VERSION));
}

static void test_handoff_validation_and_automatic_mode(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_CONFIGURETOGGLEMECHANISM configure;
    HYD_READTOGGLEMECHANISM read;
    HYD_UINT16 version;
    int axis;

    __HydMotion_framework_Init();
    axis = create_toggle_axis();
    read = read_toggle(axis);
    version = (HYD_UINT16)IEC_VAL(read.CONFIG_VERSION);

    raw.xHandoff = -0.5f;
    set_config_inputs(&configure, axis, &raw);
    configure_until_terminal(&configure);
    assert(IEC_VAL(configure.ERROR));
    assert(IEC_VAL(configure.CONFIG_VERSION) == version);
    read = read_toggle(axis);
    assert(IEC_VAL(read.CONFIG_VERSION) == version);

    raw.xHandoff = 0.0f;
    set_config_inputs(&configure, axis, &raw);
    configure_until_terminal(&configure);
    assert(IEC_VAL(configure.DONE));
    read = read_toggle(axis);
    assert(fabs(IEC_VAL(read.XHANDOFF)) < 1e-9);
    assert(fabs(IEC_VAL(read.X_HANDOFF_EFFECTIVE) -
                IEC_VAL(read.X_GEOMETRY_MIN)) < 1e-9);
    assert(IEC_VAL(read.CONFIG_VERSION) == (HYD_UINT16)(version + 1u));
}

static void test_active_axis_rejects_configuration(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_CONFIGURETOGGLEMECHANISM configure;
    HYD_READTOGGLEMECHANISM before;
    HYD_READTOGGLEMECHANISM after;
    HYD_MotionControlFB *fb;
    int axis;

    __HydMotion_framework_Init();
    axis = create_toggle_axis();
    fb = __MK_GetPublic_MotionControlFB(axis);
    assert(fb != NULL);
    before = read_toggle(axis);
    fb->STATE.active = true;
    fb->FB_STATE = HYD_FB_STATE_RUNNING;

    raw.dc = 377.5f;
    set_config_inputs(&configure, axis, &raw);
    __mcl_cmd_ConfigureToggleMechanism(&configure);
    assert(IEC_VAL(configure.ERROR));
    assert(IEC_VAL(configure.ERRORID) ==
           HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY);
    assert(IEC_VAL(configure.CONFIG_VERSION) ==
           IEC_VAL(before.CONFIG_VERSION));

    after = read_toggle(axis);
    assert(IEC_VAL(after.CONFIG_VERSION) == IEC_VAL(before.CONFIG_VERSION));
    assert(fabs(IEC_VAL(after.DC) - IEC_VAL(before.DC)) < 1e-9);
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
    test_configure_latches_inputs_and_reads_committed_state();
    test_invalid_configure_preserves_committed_geometry();
    test_handoff_validation_and_automatic_mode();
    test_active_axis_rejects_configuration();
    return 0;
}
