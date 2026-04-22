#ifndef HYDRO_SIM_H
#define HYDRO_SIM_H

#include "hydro_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================================
 * L2: 液压离线仿真器 (Hydraulic Simulator)
 * 基于集总参数法的离线仿真模型，用于在 PC 端验证工艺与运动逻辑
 * ================================================================== */

typedef struct HydraulicSimEnv HydraulicSimEnv;

typedef enum {
    SIM_AXIS_NONE = -1,
    SIM_AXIS_CLAMP = 0,
    SIM_AXIS_INJECT = 1
} SimAxisKind;

/**
 * @brief 仿真用的阀门指令状态（简化版）
 * @note 在离线仿真中，valve_fwd/bwd仅表示运动方向，不模拟阀门动力学
 *       - valve_fwd=true: 前进方向（正速度）
 *       - valve_bwd=true: 后退方向（负速度）
 *       - 两者不能同时为true，同时为false时停止运动
 */
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
    SimAxisKind axis_kind;
} SimBackendCtx;

/**
 * @brief 仿真用的单油缸模型 (主要针对合模缸)
 */
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
    SimAxisKind axis_kind;
    SimCylinder* cyl;
    SimValveCommandState* cmd;
    SimAxisFeedbackInjection* feedback_inj;
} SimAxisEntry;

/**
 * @brief 仿真环境主上下文句柄
 */
struct HydraulicSimEnv {
    // 1. 物理配置
    float pump_displacement_ml_r;
    float pump_vol_efficiency;
    SimCylinder clamp_cyl;
    SimCylinder inject_cyl;
    float melt_stiffness_N_mm;
    float valve_switch_delay_s;

    // 2. 接收自控制层的最终提交结果
    float cmd_rpm;
    SimAxisKind pump_owner_axis;
    SimValveCommandState clamp_cmd;
    SimValveCommandState inject_cmd;

    // 3. 故障注入与环境干扰 (Unit Test 用)
    bool  inject_mold_obstacle;
    float obstacle_pos_mm;
    float obstacle_stiffness_N_mm;
    SimAxisFeedbackInjection clamp_feedback_injection;
    SimAxisFeedbackInjection inject_feedback_injection;

    // 4. 仿真输出状态
    float sim_system_pressure_bar;
    float sim_time_s;

    // 5. 绑定的 L2 后端接口 (针对不同的轴提供专用的接口)
    ISensorBackend clamp_backend;
    ISensorBackend inject_backend;
    SimBackendCtx clamp_backend_ctx;
    SimBackendCtx inject_backend_ctx;

    // 6. 动态轴支持 (扩展性)
    SimAxisEntry axes[2];  // 当前支持2个轴，未来可扩展
    int axis_count;
};

/**
 * @brief 初始化仿真环境
 */
void HydraulicSim_Init(HydraulicSimEnv* env);

/**
 * @brief 获取可以注入给 L4 轴控层的硬件后端接口
 */
ISensorBackend* HydraulicSim_GetClampBackend(HydraulicSimEnv* env);
ISensorBackend* HydraulicSim_GetInjectBackend(HydraulicSimEnv* env);

/**
 * @brief 设置阀切换延时（秒）
 * @note 在简化版仿真中，此参数不起作用，保留仅为接口兼容性
 */
void HydraulicSim_SetValveSwitchDelay(HydraulicSimEnv* env, float delay_s);

/**
 * @brief 设置指定轴的伺服 ready 注入状态
 */
void HydraulicSim_SetAxisServoReady(HydraulicSimEnv* env, SimAxisKind axis_kind, bool ready);

/**
 * @brief 设置指定轴的互锁注入状态
 */
void HydraulicSim_SetAxisInterlock(HydraulicSimEnv* env, SimAxisKind axis_kind, bool interlock_ok);

/**
 * @brief 设置指定轴的动作超时注入（启用后开阀也不产生位移）
 */
void HydraulicSim_SetAxisMotionStall(HydraulicSimEnv* env, SimAxisKind axis_kind, bool stalled);

/**
 * @brief 设置指定轴的压力传感器偏置
 */
void HydraulicSim_SetPressureSensorBias(HydraulicSimEnv* env, SimAxisKind axis_kind, float bias_bar);

/**
 * @brief 设置指定轴的压力传感器比例因子
 */
void HydraulicSim_SetPressureSensorScale(HydraulicSimEnv* env, SimAxisKind axis_kind, float scale);

/**
 * @brief 设置指定轴的压力传感器卡死值
 */
void HydraulicSim_SetPressureSensorStuck(HydraulicSimEnv* env,
                                         SimAxisKind axis_kind,
                                         bool enabled,
                                         float stuck_bar);

/**
 * @brief 设置指定轴的压力传感器无效注入
 */
void HydraulicSim_SetPressureSensorInvalid(HydraulicSimEnv* env, SimAxisKind axis_kind, bool invalid);

/**
 * @brief 仿真环境时间步进 (必须在控制周期之后调用)
 * @param env 仿真环境句柄
 * @param dt_s 步长 (秒，如 0.001 代表 1ms)
 */
void HydraulicSim_Step(HydraulicSimEnv* env, float dt_s);

#ifdef __cplusplus
}
#endif

#endif // HYDRO_SIM_H
