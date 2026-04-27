#include "hydro_sim_fb.h"

#include <string.h>

#define HDY_SIM_DEFAULT_CYCLE_TIME_S (0.001f)

static HydraulicSimEnv g_shared_env;
static HDY_HydraulicSimFB _sim_fb[HDY_MAX_HYDRAULIC_SIM_FB];
static int g_axis_slot_by_id[HDY_MAX_HYDRAULIC_SIM_FB];
static unsigned int NextAllocatedHydraulicSimFB = 0U;

static int Hdy_IsValidAxisType(int axis_type) {
    return (axis_type == (int)SIM_AXIS_CLAMP) || (axis_type == (int)SIM_AXIS_INJECT);
}

static int Hdy_NormalizeDirection(int direction) {
    if (direction > 0) return 1;
    if (direction < 0) return -1;
    return 0;
}

static void Hdy_ResetAxisMapping(void) {
    int i;
    for (i = 0; i < HDY_MAX_HYDRAULIC_SIM_FB; ++i) {
        g_axis_slot_by_id[i] = -1;
    }
}

static int Hdy_GetSlotByAxisId(int axis_id) {
    if (axis_id < 0 || axis_id >= HDY_MAX_HYDRAULIC_SIM_FB) {
        return -1;
    }
    return g_axis_slot_by_id[axis_id];
}

static void Hdy_ResetHandle(HDY_HydraulicSimFB* fb) {
    if (fb == NULL) return;
    memset(fb, 0, sizeof(*fb));
    fb->axis_id = -1;
}

static void Hdy_CopyAxisFeedbackToHandle(HDY_HydraulicSimFB* fb) {
    AxisFeedback feedback;

    if (fb == NULL || fb->_env == NULL || fb->axis_id < 0) return;
    if (!HydraulicSim_ReadAxis(fb->_env, fb->axis_id, &feedback)) return;

    fb->pos_mm = feedback.position_mm;
    fb->vel_mm_s = feedback.velocity_mm_s;
    fb->pressure_bar = feedback.pressure_bar;
    fb->active = fb->enable && (fb->_env->pump_owner_axis_id == fb->axis_id);
}

static void Hdy_InitStandaloneHandle(HDY_HydraulicSimFB* fb,
                                     int axis_id,
                                     HDY_UINT8 axis_type) {
    if (fb == NULL) return;

    Hdy_ResetHandle(fb);
    fb->allocated = true;
    fb->axis_id = axis_id;
    fb->axis_type = axis_type;
    fb->enable = false;
    fb->direction = 0;
    fb->cmd_rpm = 0.0;
    fb->_env = &fb->_env_storage;
    HydraulicSim_Init(fb->_env);
    HydraulicSim_RegisterAxis(fb->_env, axis_id, (SimAxisKind)axis_type);
    HydraulicSim_ConfigureAxis(fb->_env, axis_id, 0.0f, 0.0f, 0.0f);
    Hdy_CopyAxisFeedbackToHandle(fb);
    fb->_initialized = true;
}

static void Hdy_InitSharedHandle(HDY_HydraulicSimFB* fb,
                                 int axis_id,
                                 HDY_UINT8 axis_type,
                                 HDY_REAL max_vel,
                                 HDY_REAL max_acc,
                                 HDY_REAL max_dec) {
    if (fb == NULL) return;

    Hdy_ResetHandle(fb);
    fb->allocated = true;
    fb->axis_id = axis_id;
    fb->axis_type = axis_type;
    fb->maxVel = max_vel;
    fb->maxAcc = max_acc;
    fb->maxDec = max_dec;
    fb->enable = false;
    fb->direction = 0;
    fb->cmd_rpm = 0.0;
    fb->_env = &g_shared_env;
    fb->_initialized = true;
    Hdy_CopyAxisFeedbackToHandle(fb);
}

static int Hdy_AllocateAxisId(void) {
    int axis_id;

    if (NextAllocatedHydraulicSimFB >= HDY_MAX_HYDRAULIC_SIM_FB) {
        return -1;
    }

    axis_id = (int)NextAllocatedHydraulicSimFB;
    g_axis_slot_by_id[axis_id] = axis_id;
    NextAllocatedHydraulicSimFB += 1U;
    return axis_id;
}

HDY_HydraulicSimFB* __MK_GetPublic_HydraulicSimFB(int index) {
    int slot = Hdy_GetSlotByAxisId(index);
    if (slot < 0 || slot >= (int)HDY_MAX_HYDRAULIC_SIM_FB) {
        return NULL;
    }
    if (!_sim_fb[slot].allocated) {
        return NULL;
    }
    return &_sim_fb[slot];
}

void HDY_HydraulicSimFB_Init(HDY_HydraulicSimFB* fb) {
    if (fb == NULL) return;
    Hdy_InitStandaloneHandle(fb, 0, (HDY_UINT8)SIM_AXIS_CLAMP);
}

void HDY_HydraulicSimFB_Cycle(HDY_HydraulicSimFB* fb) {
    if (fb == NULL || !fb->_initialized || fb->_env == NULL) return;

    if (fb->_env == &g_shared_env) {
        Hdy_CopyAxisFeedbackToHandle(fb);
        return;
    }

    HydraulicSim_SetAxisCommand(fb->_env,
                                fb->axis_id,
                                fb->enable,
                                (float)fb->cmd_rpm,
                                Hdy_NormalizeDirection((int)fb->direction));
    HydraulicSim_Step(fb->_env, HDY_SIM_DEFAULT_CYCLE_TIME_S);
    Hdy_CopyAxisFeedbackToHandle(fb);
}

int __HdySimulator_framework_Init() {
    int i;

    HydraulicSim_Init(&g_shared_env);
    NextAllocatedHydraulicSimFB = 0U;
    Hdy_ResetAxisMapping();

    for (i = 0; i < HDY_MAX_HYDRAULIC_SIM_FB; ++i) {
        Hdy_ResetHandle(&_sim_fb[i]);
    }

    return 1;
}

void __HdySimulator_framework_Cleanup() {
}

void __HdySimulator_framework_Retrieve() {
}

void __HdySimulator_framework_Publish() {
    unsigned int i;

    HydraulicSim_Step(&g_shared_env, HDY_SIM_DEFAULT_CYCLE_TIME_S);
    for (i = 0; i < NextAllocatedHydraulicSimFB; ++i) {
        if (_sim_fb[i].allocated) {
            Hdy_CopyAxisFeedbackToHandle(&_sim_fb[i]);
        }
    }
}

void __mcl_cmd_createSimAxis(HDY_CREATESIMAXIS *data__) {
    int axis_type;
    int axis_id;

    if (data__ == NULL) return;

    __SET_VAR(data__->, DONE,, 0);
    axis_type = (int)__GET_VAR(data__->AXISTYPE);
    if (!Hdy_IsValidAxisType(axis_type)) {
        return;
    }
    if (HydraulicSim_FindAxisByKind(&g_shared_env, (SimAxisKind)axis_type) != NULL) {
        return;
    }

    axis_id = Hdy_AllocateAxisId();
    if (axis_id < 0) {
        return;
    }

    if (!HydraulicSim_RegisterAxis(&g_shared_env, axis_id, (SimAxisKind)axis_type)) {
        g_axis_slot_by_id[axis_id] = -1;
        NextAllocatedHydraulicSimFB -= 1U;
        return;
    }

    HydraulicSim_ConfigureAxis(&g_shared_env,
                               axis_id,
                               (float)__GET_VAR(data__->MAXVEL),
                               (float)__GET_VAR(data__->MAXACC),
                               (float)__GET_VAR(data__->MAXDEC));

    Hdy_InitSharedHandle(&_sim_fb[axis_id],
                         axis_id,
                         (HDY_UINT8)axis_type,
                         (HDY_REAL)__GET_VAR(data__->MAXVEL),
                         (HDY_REAL)__GET_VAR(data__->MAXACC),
                         (HDY_REAL)__GET_VAR(data__->MAXDEC));

    __SET_VAR(data__->, AXISID,, axis_id);
    __SET_VAR(data__->, DONE,, 1);
    __SET_VAR(data__->, ENO,, 1);
}

void __mcl_cmd_moveSimAxis(HDY_MOVESIMAXIS *data__) {
    int axis_id;
    HDY_HydraulicSimFB* fb;

    if (data__ == NULL) return;

    axis_id = (int)__GET_VAR(data__->AXISID);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);
    if (fb == NULL) {
        return;
    }

    fb->enable = __GET_VAR(data__->ENABLE);
    fb->cmd_rpm = (HDY_REAL)__GET_VAR(data__->CMD_RPM);
    fb->direction = Hdy_NormalizeDirection((int)__GET_VAR(data__->DIRECTION));

    HydraulicSim_SetAxisCommand(&g_shared_env,
                                axis_id,
                                fb->enable,
                                (float)fb->cmd_rpm,
                                (int)fb->direction);
    Hdy_CopyAxisFeedbackToHandle(fb);

    __SET_VAR(data__->, BUSY,, fb->active);
    __SET_VAR(data__->, ENO,, 1);
}

void __mcl_cmd_readSimAxis(HDY_READSIMAXIS *data__) {
    int axis_id;
    HDY_HydraulicSimFB* fb;

    if (data__ == NULL) return;
    if (!__GET_VAR(data__->ENABLE)) {
        return;
    }

    axis_id = (int)__GET_VAR(data__->AXISID);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);
    if (fb == NULL) {
        return;
    }

    Hdy_CopyAxisFeedbackToHandle(fb);

    __SET_VAR(data__->, ACTIVE,, fb->active);
    __SET_VAR(data__->, POS_MM,, fb->pos_mm);
    __SET_VAR(data__->, VEL_MM_S,, fb->vel_mm_s);
    __SET_VAR(data__->, PRESSURE_BAR,, fb->pressure_bar);
    __SET_VAR(data__->, BUSY,, fb->active);
    __SET_VAR(data__->, ENO,, 1);
}
