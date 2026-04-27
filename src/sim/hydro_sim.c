#include "hydro_sim.h"
#include <math.h>
#include <string.h>

#define SIM_MAX_ACTIVE_BRANCHES 4  // 支持最多4个活动分支，当前使用2个

typedef struct {
    SimAxisKind axis_kind;
    SimCylinder* cyl;
    SimValveCommandState* cmd;
    float area_mm2;
    float dir_sign;
} SimActiveBranch;

static HydraulicSimEnv* Sim_GetEnv(void* ctx) {
    SimBackendCtx* backend = (SimBackendCtx*)ctx;
    return (backend != NULL) ? backend->env : NULL;
}

static SimAxisKind Sim_GetAxisKind(void* ctx) {
    SimBackendCtx* backend = (SimBackendCtx*)ctx;
    return (backend != NULL) ? backend->axis_kind : SIM_AXIS_NONE;
}

static SimCylinder* Sim_GetCylinder(HydraulicSimEnv* env, SimAxisKind axis_kind) {
    if (env == NULL) return NULL;

    switch (axis_kind) {
        case SIM_AXIS_CLAMP:
            return &env->clamp_cyl;
        case SIM_AXIS_INJECT:
            return &env->inject_cyl;
        case SIM_AXIS_NONE:
        default:
            return NULL;
    }
}

static SimValveCommandState* Sim_GetValveCommand(HydraulicSimEnv* env, SimAxisKind axis_kind) {
    if (env == NULL) return NULL;

    switch (axis_kind) {
        case SIM_AXIS_CLAMP:
            return &env->clamp_cmd;
        case SIM_AXIS_INJECT:
            return &env->inject_cmd;
        case SIM_AXIS_NONE:
        default:
            return NULL;
    }
}

static SimAxisFeedbackInjection* Sim_GetFeedbackInjection(HydraulicSimEnv* env, SimAxisKind axis_kind) {
    if (env == NULL) return NULL;

    switch (axis_kind) {
        case SIM_AXIS_CLAMP:
            return &env->clamp_feedback_injection;
        case SIM_AXIS_INJECT:
            return &env->inject_feedback_injection;
        case SIM_AXIS_NONE:
        default:
            return NULL;
    }
}

static void Sim_InitFeedbackInjection(SimAxisFeedbackInjection* inj) {
    if (inj == NULL) return;
    memset(inj, 0, sizeof(*inj));
    inj->interlock_ok = true;
    inj->servo_ready = true;
    inj->pressure_scale = 1.0f;
}

static int Sim_AppendActiveBranch(SimActiveBranch* branches,
                                  int branch_count,
                                  int max_count,
                                  SimAxisKind axis_kind,
                                  SimCylinder* cyl,
                                  SimValveCommandState* cmd) {
    if (branches == NULL || cyl == NULL || cmd == NULL || branch_count >= max_count) {
        return branch_count;
    }

    // 简化版：直接使用valve_fwd/valve_bwd标志位，不再模拟阀门动力学
    if (cmd->valve_fwd == cmd->valve_bwd) {
        // 两者同时为true或同时为false时不产生运动
        return branch_count;
    }

    branches[branch_count].axis_kind = axis_kind;
    branches[branch_count].cyl = cyl;
    branches[branch_count].cmd = cmd;
    if (cmd->valve_fwd) {
        branches[branch_count].area_mm2 = cyl->area_fwd_mm2;
        branches[branch_count].dir_sign = 1.0f;
    } else {
        branches[branch_count].area_mm2 = cyl->area_bwd_mm2;
        branches[branch_count].dir_sign = -1.0f;
    }
    return branch_count + 1;
}

static int Sim_CollectActiveBranches(HydraulicSimEnv* env, SimActiveBranch* branches, int max_count) {
    int branch_count = 0;

    if (env == NULL || branches == NULL || max_count <= 0) {
        return 0;
    }

    for (int i = 0; i < env->axis_count && branch_count < max_count; ++i) {
        SimAxisEntry* axis = &env->axes[i];
        branch_count = Sim_AppendActiveBranch(branches,
                                              branch_count,
                                              max_count,
                                              axis->axis_kind,
                                              axis->cyl,
                                              axis->cmd);
    }
    return branch_count;
}

static float Sim_ComputeLoadForce(const HydraulicSimEnv* env, SimAxisKind axis_kind, float dir_sign) {
    if (env == NULL) return 0.0f;

    if (axis_kind == SIM_AXIS_CLAMP) {
        float total_force_N = env->clamp_cyl.base_friction_N;
        if (env->inject_mold_obstacle && env->clamp_cyl.current_pos_mm > env->obstacle_pos_mm) {
            total_force_N += env->obstacle_stiffness_N_mm * (env->clamp_cyl.current_pos_mm - env->obstacle_pos_mm);
        }
        if (env->clamp_cyl.current_pos_mm > env->clamp_cyl.close_pos_mm) {
            total_force_N += env->clamp_cyl.tie_bar_stiffness_N_mm * (env->clamp_cyl.current_pos_mm - env->clamp_cyl.close_pos_mm);
        }
        return total_force_N;
    }

    if (axis_kind == SIM_AXIS_INJECT) {
        float total_force_N = env->inject_cyl.base_friction_N;
        if (dir_sign > 0.0f) {
            total_force_N += env->melt_stiffness_N_mm * env->inject_cyl.current_pos_mm;
        }
        return total_force_N;
    }

    return 0.0f;
}

/* ==================================================================
 * 仿真器的 ISensorBackend 回调实现
 * ================================================================== */

static void Sim_ReadFeedback(void* ctx, AxisFeedback* fb) {
    if (fb == NULL) return;
    memset(fb, 0, sizeof(*fb));

    HydraulicSimEnv* env = Sim_GetEnv(ctx);
    SimAxisKind axis_kind = Sim_GetAxisKind(ctx);
    SimCylinder* cyl = Sim_GetCylinder(env, axis_kind);
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (env == NULL || cyl == NULL) {
        return;
    }

    fb->position_mm = cyl->current_pos_mm;
    fb->velocity_mm_s = cyl->current_vel_mm_s;

    if (inj != NULL && inj->pressure_invalid) {
        fb->pressure_bar = NAN;
    } else {
        float pressure_bar = env->sim_system_pressure_bar;
        if (inj != NULL) {
            if (inj->pressure_stuck_enabled) {
                pressure_bar = inj->pressure_stuck_bar;
            } else {
                pressure_bar = (pressure_bar * inj->pressure_scale) + inj->pressure_bias_bar;
            }
            fb->interlock_ok = inj->interlock_ok;
            fb->servo_ready = inj->servo_ready;
        } else {
            fb->interlock_ok = true;
            fb->servo_ready = true;
        }
        fb->pressure_bar = pressure_bar;
    }

    if (inj == NULL) {
        fb->interlock_ok = true;
        fb->servo_ready = true;
    }
}

static void Sim_WriteValves(void* ctx, HydroValve** valves, int count) {
    HydraulicSimEnv* env = Sim_GetEnv(ctx);
    SimValveCommandState* cmd = Sim_GetValveCommand(env, Sim_GetAxisKind(ctx));
    if (cmd == NULL) return;

    cmd->valve_fwd = false;
    cmd->valve_bwd = false;

    for (int i = 0; i < count; ++i) {
        HydroValve* valve = (valves != NULL) ? valves[i] : NULL;
        if (valve == NULL) continue;

        switch (valve->role) {
            case VALVE_ROLE_FWD:
                cmd->valve_fwd = valve->cmd_state;
                break;
            case VALVE_ROLE_BWD:
                cmd->valve_bwd = valve->cmd_state;
                break;
            default:
                break;
        }
    }
}

static void Sim_WritePump(void* ctx, HydroPump* pump) {
    HydraulicSimEnv* env = Sim_GetEnv(ctx);
    SimAxisKind axis_kind = Sim_GetAxisKind(ctx);
    if (env == NULL || pump == NULL) return;

    if (pump->grant_valid && pump->granted_rpm > 0.0f) {
        env->cmd_rpm = pump->granted_rpm;
        env->pump_owner_axis = axis_kind;
    } else {
        env->cmd_rpm = 0.0f;
        env->pump_owner_axis = SIM_AXIS_NONE;
    }
}

/* ==================================================================
 * 仿真器核心：初始化与单步积分计算
 * ================================================================== */

void HydraulicSim_Init(HydraulicSimEnv* env) {
    if (env == NULL) return;
    memset(env, 0, sizeof(HydraulicSimEnv));

    // 仿真器限制：最多支持2个轴（clamp和inject），每个轴在任意时刻只能有一个活动分支（fwd或bwd阀门之一）
    // SIM_MAX_ACTIVE_BRANCHES定义为4以提供扩展余量，但当前实现假设最多2个并发活动分支
    env->pump_displacement_ml_r = 28.0f;
    env->pump_vol_efficiency = 0.95f;
    env->pump_owner_axis = SIM_AXIS_NONE;
    env->valve_switch_delay_s = 0.0f;

    env->clamp_cyl.area_fwd_mm2 = 12500.0f;
    env->clamp_cyl.area_bwd_mm2 = 8500.0f;
    env->clamp_cyl.stroke_mm = 500.0f;
    env->clamp_cyl.close_pos_mm = 495.0f;
    env->clamp_cyl.tie_bar_stiffness_N_mm = 500000.0f;
    env->clamp_cyl.base_friction_N = 1000.0f;

    env->inject_cyl.area_fwd_mm2 = 8000.0f;
    env->inject_cyl.area_bwd_mm2 = 4500.0f;
    env->inject_cyl.stroke_mm = 300.0f;
    env->inject_cyl.base_friction_N = 800.0f;
    env->melt_stiffness_N_mm = 150.0f;

    env->obstacle_pos_mm = 494.0f;
    env->obstacle_stiffness_N_mm = 80000.0f;
    Sim_InitFeedbackInjection(&env->clamp_feedback_injection);
    Sim_InitFeedbackInjection(&env->inject_feedback_injection);

    env->clamp_backend_ctx.env = env;
    env->clamp_backend_ctx.axis_kind = SIM_AXIS_CLAMP;
    env->clamp_backend.ctx = &env->clamp_backend_ctx;
    env->clamp_backend.read_feedback = Sim_ReadFeedback;
    env->clamp_backend.write_valves = Sim_WriteValves;
    env->clamp_backend.write_pump = Sim_WritePump;

    env->inject_backend_ctx.env = env;
    env->inject_backend_ctx.axis_kind = SIM_AXIS_INJECT;
    env->inject_backend.ctx = &env->inject_backend_ctx;
    env->inject_backend.read_feedback = Sim_ReadFeedback;
    env->inject_backend.write_valves = Sim_WriteValves;
    env->inject_backend.write_pump = Sim_WritePump;

    // 初始化动态轴列表
    env->axes[0].axis_kind = SIM_AXIS_CLAMP;
    env->axes[0].cyl = &env->clamp_cyl;
    env->axes[0].cmd = &env->clamp_cmd;
    env->axes[0].feedback_inj = &env->clamp_feedback_injection;
    env->axes[1].axis_kind = SIM_AXIS_INJECT;
    env->axes[1].cyl = &env->inject_cyl;
    env->axes[1].cmd = &env->inject_cmd;
    env->axes[1].feedback_inj = &env->inject_feedback_injection;
    env->axis_count = 2;

    env->_initialized = true;
}

ISensorBackend* HydraulicSim_GetClampBackend(HydraulicSimEnv* env) {
    return (env != NULL) ? &env->clamp_backend : NULL;
}

ISensorBackend* HydraulicSim_GetInjectBackend(HydraulicSimEnv* env) {
    return (env != NULL) ? &env->inject_backend : NULL;
}

void HydraulicSim_SetValveSwitchDelay(HydraulicSimEnv* env, float delay_s) {
    // 简化版：此函数保留仅为接口兼容性，不再实际影响仿真行为
    if (env == NULL) return;
    env->valve_switch_delay_s = 0.0f;  // 强制为0，不再使用延迟
}

void HydraulicSim_SetAxisServoReady(HydraulicSimEnv* env, SimAxisKind axis_kind, bool ready) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->servo_ready = ready;
}

void HydraulicSim_SetAxisInterlock(HydraulicSimEnv* env, SimAxisKind axis_kind, bool interlock_ok) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->interlock_ok = interlock_ok;
}

void HydraulicSim_SetAxisMotionStall(HydraulicSimEnv* env, SimAxisKind axis_kind, bool stalled) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->motion_stalled = stalled;
}

void HydraulicSim_SetPressureSensorBias(HydraulicSimEnv* env, SimAxisKind axis_kind, float bias_bar) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->pressure_bias_bar = bias_bar;
}

void HydraulicSim_SetPressureSensorScale(HydraulicSimEnv* env, SimAxisKind axis_kind, float scale) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->pressure_scale = scale;
}

void HydraulicSim_SetPressureSensorStuck(HydraulicSimEnv* env,
                                         SimAxisKind axis_kind,
                                         bool enabled,
                                         float stuck_bar) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->pressure_stuck_enabled = enabled;
    inj->pressure_stuck_bar = stuck_bar;
}

void HydraulicSim_SetPressureSensorInvalid(HydraulicSimEnv* env, SimAxisKind axis_kind, bool invalid) {
    SimAxisFeedbackInjection* inj = Sim_GetFeedbackInjection(env, axis_kind);
    if (inj == NULL) return;
    inj->pressure_invalid = invalid;
}

void HydraulicSim_Step(HydraulicSimEnv* env, float dt_s) {
    SimActiveBranch branches[SIM_MAX_ACTIVE_BRANCHES];
    if (env == NULL) return;

    const float flow_lpm = (env->pump_displacement_ml_r * env->cmd_rpm / 1000.0f) * env->pump_vol_efficiency;
    const float flow_mm3_s = flow_lpm * 16666.667f;

    env->clamp_cyl.current_vel_mm_s = 0.0f;
    env->inject_cyl.current_vel_mm_s = 0.0f;
    env->sim_system_pressure_bar = 0.0f;

    // 简化版：不再模拟阀门动力学，直接使用当前方向指令
    const int branch_count = Sim_CollectActiveBranches(env, branches, SIM_MAX_ACTIVE_BRANCHES);
    if (branch_count > 0) {
        const float flow_share_mm3_s = (env->cmd_rpm > 0.0f)
                                     ? (flow_mm3_s / (float)branch_count)
                                     : 0.0f;

        for (int i = 0; i < branch_count; ++i) {
            SimActiveBranch* branch = &branches[i];
            SimAxisFeedbackInjection* inj;
            if (branch->cyl == NULL || branch->area_mm2 <= 0.1f) {
                continue;
            }

            inj = Sim_GetFeedbackInjection(env, branch->axis_kind);
            if (env->cmd_rpm > 0.0f && !(inj != NULL && inj->motion_stalled)) {
                const float vel = branch->dir_sign * (flow_share_mm3_s / branch->area_mm2);
                branch->cyl->current_vel_mm_s = vel;
                branch->cyl->current_pos_mm += vel * dt_s;

                if (branch->cyl->current_pos_mm < 0.0f) {
                    branch->cyl->current_pos_mm = 0.0f;
                    branch->cyl->current_vel_mm_s = 0.0f;
                } else if (branch->cyl->current_pos_mm > branch->cyl->stroke_mm) {
                    branch->cyl->current_pos_mm = branch->cyl->stroke_mm;
                    branch->cyl->current_vel_mm_s = 0.0f;
                }
            } else {
                branch->cyl->current_vel_mm_s = 0.0f;
            }

            const float total_force_N = Sim_ComputeLoadForce(env, branch->axis_kind, branch->dir_sign);
            const float pressure_bar = (total_force_N / branch->area_mm2) * 10.0f;
            if (pressure_bar > env->sim_system_pressure_bar) {
                env->sim_system_pressure_bar = pressure_bar;
            }
        }
    }

    env->sim_time_s += dt_s;
}
