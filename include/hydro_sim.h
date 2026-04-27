#ifndef HYDRO_SIM_H
#define HYDRO_SIM_H

#include "hydro_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HDY_MAX_HYDRAULIC_SIM_FB
#define HDY_MAX_HYDRAULIC_SIM_FB (2)
#endif

/* ==================================================================
 * L2: 液压离线仿真器 (Hydraulic Simulator)
 *
 * 设计目标：
 * - HydraulicSimEnv 表示系统级共享仿真环境
 * - SimAxisState 表示通用轴表中的单轴状态
 * - 单泵模型：同一扫描周期只允许 pump_owner_axis_id 指向的轴参与流量换算
 * - 纯 C99 / 静态有界内存 / 无动态分配
 * ================================================================== */

typedef struct HydraulicSimEnv HydraulicSimEnv;

typedef enum {
    SIM_AXIS_NONE = -1,
    SIM_AXIS_CLAMP = 0,
    SIM_AXIS_INJECT = 1
} SimAxisKind;

typedef struct {
    bool valve_fwd;
    bool valve_bwd;
} SimValveCommandState;

typedef struct {
    bool  interlock_ok;
    bool  servo_ready;
    bool  motion_stalled;
    float pressure_bias_bar;
    float pressure_scale;
    bool  pressure_stuck_enabled;
    float pressure_stuck_bar;
    bool  pressure_invalid;
} SimAxisFeedbackInjection;

typedef struct {
    HydraulicSimEnv* env;
    int axis_id;
} SimBackendCtx;

typedef struct {
    float area_fwd_mm2;
    float area_bwd_mm2;
    float stroke_mm;

    float current_pos_mm;
    float current_vel_mm_s;

    float close_pos_mm;
    float tie_bar_stiffness_N_mm;
    float base_friction_N;
} SimCylinder;

typedef struct {
    bool allocated;
    int axis_id;
    SimAxisKind axis_type;

    SimCylinder cylinder;
    SimValveCommandState valve_cmd;
    SimAxisFeedbackInjection feedback_inj;

    float branch_pressure_bar;
    float last_cmd_rpm;
    int direction_cmd;
    bool enabled;

    float max_vel_mm_s;
    float max_acc_mm_s2;
    float max_dec_mm_s2;

    AxisFeedback last_feedback;

    ISensorBackend backend;
    SimBackendCtx backend_ctx;
} SimAxisState;

struct HydraulicSimEnv {
    float pump_displacement_ml_r;
    float pump_vol_efficiency;
    float melt_stiffness_N_mm;

    bool  inject_mold_obstacle;
    float obstacle_pos_mm;
    float obstacle_stiffness_N_mm;

    float sim_system_pressure_bar;
    float sim_time_s;

    int pump_owner_axis_id;
    float cmd_rpm;

    SimAxisState axes[HDY_MAX_HYDRAULIC_SIM_FB];
    int axis_count;
    char _initialized;
};

void HydraulicSim_Init(HydraulicSimEnv* env);

int HydraulicSim_RegisterAxis(HydraulicSimEnv* env, int axis_id, SimAxisKind axis_kind);
int HydraulicSim_ConfigureAxis(HydraulicSimEnv* env,
                               int axis_id,
                               float max_vel,
                               float max_acc,
                               float max_dec);
SimAxisState* HydraulicSim_FindAxisById(HydraulicSimEnv* env, int axis_id);
const SimAxisState* HydraulicSim_FindAxisByIdConst(const HydraulicSimEnv* env, int axis_id);
SimAxisState* HydraulicSim_FindAxisByKind(HydraulicSimEnv* env, SimAxisKind axis_kind);
const SimAxisState* HydraulicSim_FindAxisByKindConst(const HydraulicSimEnv* env, SimAxisKind axis_kind);
int HydraulicSim_SetAxisCommand(HydraulicSimEnv* env,
                                int axis_id,
                                bool enable,
                                float cmd_rpm,
                                int direction);
int HydraulicSim_ReadAxis(HydraulicSimEnv* env, int axis_id, AxisFeedback* fb);

ISensorBackend* HydraulicSim_GetAxisBackend(HydraulicSimEnv* env, int axis_id);

void HydraulicSim_SetValveSwitchDelay(HydraulicSimEnv* env, float delay_s);
void HydraulicSim_SetAxisServoReady(HydraulicSimEnv* env, int axis_id, bool ready);
void HydraulicSim_SetAxisInterlock(HydraulicSimEnv* env, int axis_id, bool interlock_ok);
void HydraulicSim_SetAxisMotionStall(HydraulicSimEnv* env, int axis_id, bool stalled);
void HydraulicSim_SetPressureSensorBias(HydraulicSimEnv* env, int axis_id, float bias_bar);
void HydraulicSim_SetPressureSensorScale(HydraulicSimEnv* env, int axis_id, float scale);
void HydraulicSim_SetPressureSensorStuck(HydraulicSimEnv* env,
                                         int axis_id,
                                         bool enabled,
                                         float stuck_bar);
void HydraulicSim_SetPressureSensorInvalid(HydraulicSimEnv* env, int axis_id, bool invalid);

void HydraulicSim_Step(HydraulicSimEnv* env, float dt_s);

int HydraulicSim_NormalizeDirection(int direction);

#ifdef __cplusplus
}
#endif

#endif // HYDRO_SIM_H
