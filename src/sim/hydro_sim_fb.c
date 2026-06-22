#include "hydro_sim_fb.h"
#include "pressure_model.h"

#include <string.h>

#define HYD_SIM_DEFAULT_CYCLE_TIME_S (0.001f)

static HydraulicSimEnv g_shared_env;
static HYD_HydraulicSimFB _sim_fb[HYD_MAX_HYDRAULIC_SIM_FB];
static int g_axis_slot_by_id[HYD_MAX_HYDRAULIC_SIM_FB];
static unsigned int NextAllocatedHydraulicSimFB = 0U;
static PressureModelParams g_pressure_model_params;
static PressureModelState g_pressure_model_state;
static int g_pressure_model_initialized = 0;
static int g_pressure_model_have_time = 0;
static float g_pressure_model_last_time_s = 0.0f;
static const unsigned int kPressureModelSeed = 0x13572468u;

static void PressureModelFb_ResetOutputs(HYD_PRESSUREMODEL *data__) {
    __SET_VAR(data__->, REAL_PRESSURE_BAR,, 0.0f);
    __SET_VAR(data__->, MEASURED_PRESSURE_BAR,, 0.0f);
    __SET_VAR(data__->, ACTUAL_MOTOR_RPM,, 0.0f);
    __SET_VAR(data__->, ACTIVE,, 0);
}

static void PressureModelFb_ResetState(void) {
    PressureModel_Reset(&g_pressure_model_state, kPressureModelSeed);
    g_pressure_model_have_time = 0;
    g_pressure_model_last_time_s = 0.0f;
}

static void PressureModelFb_EnsureInitialized(void) {
    if (g_pressure_model_initialized) {
        return;
    }

    PressureModel_InitParams(&g_pressure_model_params);
    PressureModelFb_ResetState();
    g_pressure_model_initialized = 1;
}

static int Hyd_IsValidAxisType(int axis_type) {
    return (axis_type == (int)SIM_AXIS_CLAMP) || (axis_type == (int)SIM_AXIS_INJECT);
}

static void Hyd_ResetAxisMapping(void) {
    int i;
    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        g_axis_slot_by_id[i] = -1;
    }
}

static int Hyd_GetSlotByAxisId(int axis_id) {
    if (axis_id < 0 || axis_id >= HYD_MAX_HYDRAULIC_SIM_FB) {
        return -1;
    }
    return g_axis_slot_by_id[axis_id];
}

static void Hyd_ResetHandle(HYD_HydraulicSimFB* fb) {
    if (fb == NULL) return;
    memset(fb, 0, sizeof(*fb));
    fb->axis_id = -1;
}

static void Hyd_CopyAxisFeedbackToHandle(HYD_HydraulicSimFB* fb) {
    AxisFeedback feedback;

    if (fb == NULL || fb->_env == NULL || fb->axis_id < 0) return;
    if (!HydraulicSim_ReadAxis(fb->_env, fb->axis_id, &feedback)) return;

    fb->pos_mm = feedback.position_mm;
    fb->vel_mm_s = feedback.velocity_mm_s;
    fb->pressure_bar = feedback.pressure_bar;
    fb->active = fb->enable && (fb->_env->pump_owner_axis_id == fb->axis_id);
}

static void Hyd_InitSharedHandle(HYD_HydraulicSimFB* fb,
                                 int axis_id,
                                 HYD_UINT8 axis_type,
                                 HYD_REAL max_vel,
                                 HYD_REAL max_acc,
                                 HYD_REAL max_dec) {
    if (fb == NULL) return;

    Hyd_ResetHandle(fb);
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
    Hyd_CopyAxisFeedbackToHandle(fb);
}

HYD_HydraulicSimFB* __MK_GetPublic_HydraulicSimFB(int index) {
    int slot = Hyd_GetSlotByAxisId(index);
    if (slot < 0 || slot >= (int)HYD_MAX_HYDRAULIC_SIM_FB) {
        return NULL;
    }
    if (!_sim_fb[slot].allocated) {
        return NULL;
    }
    return &_sim_fb[slot];
}

void HYD_HydraulicSimFB_Cycle(HYD_HydraulicSimFB* fb) {
    if (fb == NULL || !fb->_initialized || fb->_env == NULL) return;

    /* 共享模式下只读取快照，步进由 __HydSimulator_framework_Publish 统一完成 */
    if (fb->_isSharedEnv) {
        Hyd_CopyAxisFeedbackToHandle(fb);
        return;
    }

    /* 非共享模式（离线仿真）：自行写入命令并步进 */
    HydraulicSim_SetAxisCommand(fb->_env,
                                fb->axis_id,
                                fb->enable,
                                (float)fb->cmd_rpm,
                                HydraulicSim_NormalizeDirection((int)fb->direction));
    HydraulicSim_Step(fb->_env, HYD_SIM_DEFAULT_CYCLE_TIME_S);
    Hyd_CopyAxisFeedbackToHandle(fb);
}

int __HydSimulator_framework_Init() {
    int i;

    HydraulicSim_Init(&g_shared_env);
    NextAllocatedHydraulicSimFB = 0U;
    Hyd_ResetAxisMapping();

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        Hyd_ResetHandle(&_sim_fb[i]);
    }

    if (g_pressure_model_initialized) {
        PressureModelFb_ResetState();
    }

    return 0;
}

void __HydSimulator_framework_Cleanup() {
}

void __HydSimulator_framework_Retrieve() {
}

void __HydSimulator_framework_Publish() {
    unsigned int i;

    HydraulicSim_Step(&g_shared_env, HYD_SIM_DEFAULT_CYCLE_TIME_S);
    for (i = 0; i < NextAllocatedHydraulicSimFB; ++i) {
        if (_sim_fb[i].allocated) {
            Hyd_CopyAxisFeedbackToHandle(&_sim_fb[i]);
        }
    }
}

void __mcl_cmd_createSimAxis(HYD_CREATESIMAXIS *data__) {
    int axis_type;
    int axis_id;

    if (data__ == NULL) return;

    if(!__GET_VAR(data__->DONE)) {

		axis_type = (int) __GET_VAR(data__->AXISTYPE);
		if (!Hyd_IsValidAxisType(axis_type)) {
			return;
		}

		/* 预分配 axis_id，但不提交计数器；注册失败时无需回退 */
		if (NextAllocatedHydraulicSimFB >= HYD_MAX_HYDRAULIC_SIM_FB) {
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

		Hyd_InitSharedHandle(&_sim_fb[axis_id], axis_id, (HYD_UINT8) axis_type,
				(HYD_REAL) __GET_VAR(data__->MAXVEL),
				(HYD_REAL) __GET_VAR(data__->MAXACC),
				(HYD_REAL) __GET_VAR(data__->MAXDEC));

		__SET_VAR(data__->, AXISID,, axis_id);
		__SET_VAR(data__->, DONE,, 1);
		__SET_VAR(data__->, ENO,, 1);
    }
}

void __mcl_cmd_moveSimAxis(HYD_MOVESIMAXIS *data__) {
    int axis_id;
    HYD_HydraulicSimFB* fb;

    if (data__ == NULL) return;

    axis_id = (int)__GET_VAR(data__->AXISID);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);
    if (fb == NULL) {
        return;
    }

    fb->enable = __GET_VAR(data__->ENABLE);
    fb->cmd_rpm = (HYD_REAL)__GET_VAR(data__->CMD_RPM);
    fb->direction = HydraulicSim_NormalizeDirection((int)__GET_VAR(data__->DIRECTION));

    HydraulicSim_SetAxisCommand(&g_shared_env,
                                axis_id,
                                fb->enable,
                                (float)fb->cmd_rpm,
                                (int)fb->direction);
    Hyd_CopyAxisFeedbackToHandle(fb);

    __SET_VAR(data__->, BUSY,, fb->active);
    __SET_VAR(data__->, ENO,, 1);
}

void __mcl_cmd_readSimAxis(HYD_READSIMAXIS *data__) {
    int axis_id;
    HYD_HydraulicSimFB* fb;

    if (data__ == NULL) return;
    if (!__GET_VAR(data__->ENABLE)) {
        return;
    }

    axis_id = (int)__GET_VAR(data__->AXISID);
    fb = __MK_GetPublic_HydraulicSimFB(axis_id);
    if (fb == NULL) {
        return;
    }

    Hyd_CopyAxisFeedbackToHandle(fb);

    __SET_VAR(data__->, ACTIVE,, fb->active);
    __SET_VAR(data__->, POS_MM,, fb->pos_mm);
    __SET_VAR(data__->, VEL_MM_S,, fb->vel_mm_s);
    __SET_VAR(data__->, PRESSURE_BAR,, fb->pressure_bar);
    __SET_VAR(data__->, BUSY,, fb->active);
    __SET_VAR(data__->, ENO,, 1);
}

void __mcl_cmd_updatePressureModel(HYD_PRESSUREMODEL *data__)
{
    float current_time;
    float dt_s;
    float target_motor_speed;
    PressureModelOutput out;

    if (data__ == NULL) return;
    PressureModelFb_EnsureInitialized();

    if (!__GET_VAR(data__->ENABLE)) {
        PressureModelFb_ResetState();
        PressureModelFb_ResetOutputs(data__);
        return;
    }

    current_time = __GET_VAR(data__->TIME_S);
    target_motor_speed = __GET_VAR(data__->MOTOR_RPM);
    g_pressure_model_params.model_type = (unsigned char)__GET_VAR(data__->MODEL_TYPE);
    g_pressure_model_params.first_order_k_bar_per_rpm = (float)__GET_VAR(data__->K_NUM);
    g_pressure_model_params.first_order_tau_s = (float)__GET_VAR(data__->TTAU);
    g_pressure_model_params.first_order_delay_s = (float)__GET_VAR(data__->DELAYTIME);
    if (g_pressure_model_have_time && current_time > g_pressure_model_last_time_s) {
        dt_s = current_time - g_pressure_model_last_time_s;
    } else {
        dt_s = 0.001f;
    }
    g_pressure_model_last_time_s = current_time;
    g_pressure_model_have_time = 1;

    memset(&out, 0, sizeof(out));
    PressureModel_Step(&g_pressure_model_params,
                       &g_pressure_model_state,
                       target_motor_speed,
                       dt_s,
                       &out);

    __SET_VAR(data__->, REAL_PRESSURE_BAR,, out.real_pressure_bar);
    __SET_VAR(data__->, MEASURED_PRESSURE_BAR,, out.measured_pressure_bar);
    __SET_VAR(data__->, ACTUAL_MOTOR_RPM,, out.actual_motor_rpm);
    __SET_VAR(data__->, ACTIVE,, 1);
}
