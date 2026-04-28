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
    fb->_isSharedEnv = true;
    fb->_initialized = true;
    Hdy_CopyAxisFeedbackToHandle(fb);
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

void HDY_HydraulicSimFB_Cycle(HDY_HydraulicSimFB* fb) {
    if (fb == NULL || !fb->_initialized || fb->_env == NULL) return;

    /* 共享模式下只读取快照，步进由 __HdySimulator_framework_Publish 统一完成 */
    if (fb->_isSharedEnv) {
        Hdy_CopyAxisFeedbackToHandle(fb);
        return;
    }

    /* 非共享模式（离线仿真）：自行写入命令并步进 */
    HydraulicSim_SetAxisCommand(fb->_env,
                                fb->axis_id,
                                fb->enable,
                                (float)fb->cmd_rpm,
                                HydraulicSim_NormalizeDirection((int)fb->direction));
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

    return 0;
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

    if(!__GET_VAR(data__->DONE)) {

		axis_type = (int) __GET_VAR(data__->AXISTYPE);
		if (!Hdy_IsValidAxisType(axis_type)) {
			return;
		}

		/* 预分配 axis_id，但不提交计数器；注册失败时无需回退 */
		if (NextAllocatedHydraulicSimFB >= HDY_MAX_HYDRAULIC_SIM_FB) {
			return;
		}
		axis_id = (int) NextAllocatedHydraulicSimFB;
		g_axis_slot_by_id[axis_id] = axis_id;

		/* RegisterAxis 内部已包含 FindAxisByKind 同类型重复检查 */
		if (!HydraulicSim_RegisterAxis(&g_shared_env, axis_id,
				(SimAxisKind) axis_type)) {
			g_axis_slot_by_id[axis_id] = -1;
			return;
		}

		/* 注册成功，提交分配计数器 */
		NextAllocatedHydraulicSimFB += 1U;

		HydraulicSim_ConfigureAxis(&g_shared_env, axis_id,
				(float) __GET_VAR(data__->MAXVEL),
				(float) __GET_VAR(data__->MAXACC),
				(float) __GET_VAR(data__->MAXDEC));

		Hdy_InitSharedHandle(&_sim_fb[axis_id], axis_id, (HDY_UINT8) axis_type,
				(HDY_REAL) __GET_VAR(data__->MAXVEL),
				(HDY_REAL) __GET_VAR(data__->MAXACC),
				(HDY_REAL) __GET_VAR(data__->MAXDEC));

		__SET_VAR(data__->, AXISID,, axis_id);
		__SET_VAR(data__->, DONE,, 1);
		__SET_VAR(data__->, ENO,, 1);
    }
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
    fb->direction = HydraulicSim_NormalizeDirection((int)__GET_VAR(data__->DIRECTION));

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
