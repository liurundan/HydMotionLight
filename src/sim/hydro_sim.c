#include "hydro_sim.h"

#include <math.h>
#include <string.h>

static HydraulicSimEnv* Sim_GetEnv(void* ctx) {
    SimBackendCtx* backend = (SimBackendCtx*)ctx;
    return (backend != NULL) ? backend->env : NULL;
}

static int Sim_GetAxisId(void* ctx) {
    SimBackendCtx* backend = (SimBackendCtx*)ctx;
    return (backend != NULL) ? backend->axis_id : -1;
}

int HydraulicSim_NormalizeDirection(int direction) {
    if (direction > 0) return 1;
    if (direction < 0) return -1;
    return 0;
}

static void Sim_InitFeedbackInjection(SimAxisFeedbackInjection* inj) {
    if (inj == NULL) return;

    memset(inj, 0, sizeof(*inj));
    inj->interlock_ok = true;
    inj->servo_ready = true;
    inj->pressure_scale = 1.0f;
}

static void Sim_InitAxisFeedback(SimAxisState* axis) {
    if (axis == NULL) return;

    memset(&axis->last_feedback, 0, sizeof(axis->last_feedback));
    axis->last_feedback.interlock_ok = axis->feedback_inj.interlock_ok;
    axis->last_feedback.servo_ready = axis->feedback_inj.servo_ready;
}

static void Sim_InitAxisByType(SimAxisState* axis) {
    if (axis == NULL) return;

    memset(&axis->cylinder, 0, sizeof(axis->cylinder));
    axis->branch_pressure_bar = 0.0f;
    axis->last_cmd_rpm = 0.0f;
    axis->direction_cmd = 0;
    axis->enabled = false;
    axis->valve_cmd.valve_fwd = false;
    axis->valve_cmd.valve_bwd = false;
    axis->max_vel_mm_s = 0.0f;
    axis->max_acc_mm_s2 = 0.0f;
    axis->max_dec_mm_s2 = 0.0f;
    Sim_InitFeedbackInjection(&axis->feedback_inj);

    switch (axis->axis_type) {
        case SIM_AXIS_CLAMP:
            axis->cylinder.area_fwd_mm2 = 12500.0f;
            axis->cylinder.area_bwd_mm2 = 8500.0f;
            axis->cylinder.stroke_mm = 500.0f;
            axis->cylinder.close_pos_mm = 495.0f;
            axis->cylinder.tie_bar_stiffness_N_mm = 500000.0f;
            axis->cylinder.base_friction_N = 1000.0f;
            break;
        case SIM_AXIS_INJECT:
            axis->cylinder.area_fwd_mm2 = 8000.0f;
            axis->cylinder.area_bwd_mm2 = 4500.0f;
            axis->cylinder.stroke_mm = 300.0f;
            axis->cylinder.close_pos_mm = 0.0f;
            axis->cylinder.tie_bar_stiffness_N_mm = 0.0f;
            axis->cylinder.base_friction_N = 800.0f;
            break;
        case SIM_AXIS_NONE:
        default:
            break;
    }

    Sim_InitAxisFeedback(axis);
}

static float Sim_GetAreaForDirection(const SimAxisState* axis, int direction) {
    if (axis == NULL) return 0.0f;

    if (direction > 0) {
        return axis->cylinder.area_fwd_mm2;
    }
    if (direction < 0) {
        return axis->cylinder.area_bwd_mm2;
    }
    return 0.0f;
}

static float Sim_ClampPosition(const SimAxisState* axis, float position_mm) {
    if (axis == NULL) return 0.0f;
    if (position_mm < 0.0f) return 0.0f;
    if (position_mm > axis->cylinder.stroke_mm) return axis->cylinder.stroke_mm;
    return position_mm;
}

static float Sim_ComputeLoadForceByType(const HydraulicSimEnv* env,
                                        const SimAxisState* axis,
                                        float dir_sign) {
    float total_force_N;

    if (env == NULL || axis == NULL) return 0.0f;

    total_force_N = axis->cylinder.base_friction_N;

    switch (axis->axis_type) {
        case SIM_AXIS_CLAMP:
            if (env->inject_mold_obstacle &&
                axis->cylinder.current_pos_mm > env->obstacle_pos_mm) {
                total_force_N += env->obstacle_stiffness_N_mm *
                                 (axis->cylinder.current_pos_mm - env->obstacle_pos_mm);
            }
            if (axis->cylinder.current_pos_mm > axis->cylinder.close_pos_mm) {
                total_force_N += axis->cylinder.tie_bar_stiffness_N_mm *
                                 (axis->cylinder.current_pos_mm - axis->cylinder.close_pos_mm);
            }
            break;
        case SIM_AXIS_INJECT:
            if (dir_sign > 0.0f) {
                total_force_N += env->melt_stiffness_N_mm * axis->cylinder.current_pos_mm;
            }
            break;
        case SIM_AXIS_NONE:
        default:
            break;
    }

    return total_force_N;
}

static void Sim_UpdateAxisFeedback(SimAxisState* axis) {
    float pressure_bar;

    if (axis == NULL) return;

    axis->last_feedback.position_mm = axis->cylinder.current_pos_mm;
    axis->last_feedback.velocity_mm_s = axis->cylinder.current_vel_mm_s;
    axis->last_feedback.interlock_ok = axis->feedback_inj.interlock_ok;
    axis->last_feedback.servo_ready = axis->feedback_inj.servo_ready;

    if (axis->feedback_inj.pressure_invalid) {
        axis->last_feedback.pressure_bar = NAN;
        return;
    }

    pressure_bar = axis->branch_pressure_bar;
    if (axis->feedback_inj.pressure_stuck_enabled) {
        pressure_bar = axis->feedback_inj.pressure_stuck_bar;
    } else {
        pressure_bar = (pressure_bar * axis->feedback_inj.pressure_scale) +
                       axis->feedback_inj.pressure_bias_bar;
    }

    axis->last_feedback.pressure_bar = pressure_bar;
}

static SimAxisState* Sim_FindFreeAxisSlot(HydraulicSimEnv* env) {
    int i;

    if (env == NULL) return NULL;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (!env->axes[i].allocated) {
            return &env->axes[i];
        }
    }

    return NULL;
}

static void Sim_ReadFeedback(void* ctx, AxisFeedback* fb) {
    HydraulicSimEnv* env;
    SimAxisState* axis;

    if (fb == NULL) return;
    memset(fb, 0, sizeof(*fb));

    env = Sim_GetEnv(ctx);
    axis = HydraulicSim_FindAxisById(env, Sim_GetAxisId(ctx));
    if (axis == NULL) return;

    *fb = axis->last_feedback;
}

static void Sim_WriteValves(void* ctx, HydroValve** valves, int count) {
    HydraulicSimEnv* env;
    SimAxisState* axis;
    int i;

    env = Sim_GetEnv(ctx);
    axis = HydraulicSim_FindAxisById(env, Sim_GetAxisId(ctx));
    if (axis == NULL) return;

    axis->valve_cmd.valve_fwd = false;
    axis->valve_cmd.valve_bwd = false;

    for (i = 0; i < count; ++i) {
        HydroValve* valve = (valves != NULL) ? valves[i] : NULL;
        if (valve == NULL) continue;

        switch (valve->role) {
            case VALVE_ROLE_FWD:
                axis->valve_cmd.valve_fwd = valve->cmd_state;
                break;
            case VALVE_ROLE_BWD:
                axis->valve_cmd.valve_bwd = valve->cmd_state;
                break;
            default:
                break;
        }
    }

    if (axis->valve_cmd.valve_fwd == axis->valve_cmd.valve_bwd) {
        axis->direction_cmd = 0;
    } else if (axis->valve_cmd.valve_fwd) {
        axis->direction_cmd = 1;
    } else {
        axis->direction_cmd = -1;
    }
}

static void Sim_WritePump(void* ctx, HydroPump* pump) {
    HydraulicSimEnv* env;
    SimAxisState* axis;

    env = Sim_GetEnv(ctx);
    axis = HydraulicSim_FindAxisById(env, Sim_GetAxisId(ctx));
    if (env == NULL || axis == NULL || pump == NULL) return;

    if (pump->grant_valid && pump->granted_rpm > 0.0f) {
        env->cmd_rpm = pump->granted_rpm;
        env->pump_owner_axis_id = axis->axis_id;
        axis->last_cmd_rpm = pump->granted_rpm;
        axis->enabled = true;
    } else if (env->pump_owner_axis_id == axis->axis_id) {
        env->cmd_rpm = 0.0f;
        env->pump_owner_axis_id = -1;
        axis->last_cmd_rpm = 0.0f;
    }
}

void HydraulicSim_Init(HydraulicSimEnv* env) {
    int i;

    if (env == NULL) return;

    memset(env, 0, sizeof(*env));
    env->pump_displacement_ml_r = 28.0f;
    env->pump_vol_efficiency = 0.95f;
    env->melt_stiffness_N_mm = 150.0f;
    env->obstacle_pos_mm = 494.0f;
    env->obstacle_stiffness_N_mm = 80000.0f;
    env->pump_owner_axis_id = -1;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        env->axes[i].axis_id = -1;
        env->axes[i].axis_type = SIM_AXIS_NONE;
    }

    env->axis_count = 0;
    env->_initialized = true;
}

int HydraulicSim_RegisterAxis(HydraulicSimEnv* env, int axis_id, SimAxisKind axis_kind) {
    SimAxisState* axis;

    if (env == NULL || !env->_initialized) return 0;
    if (axis_id < 0 || axis_id >= HYD_MAX_HYDRAULIC_SIM_FB) return 0;
    if (axis_kind != SIM_AXIS_CLAMP && axis_kind != SIM_AXIS_INJECT) return 0;
    if (HydraulicSim_FindAxisById(env, axis_id) != NULL) return 0;
    if (HydraulicSim_FindAxisByKind(env, axis_kind) != NULL) return 0;

    axis = Sim_FindFreeAxisSlot(env);
    if (axis == NULL) return 0;

    memset(axis, 0, sizeof(*axis));
    axis->allocated = true;
    axis->axis_id = axis_id;
    axis->axis_type = axis_kind;
    axis->backend.ctx = &axis->backend_ctx;
    axis->backend.read_feedback = Sim_ReadFeedback;
    axis->backend.write_valves = Sim_WriteValves;
    axis->backend.write_pump = Sim_WritePump;
    axis->backend_ctx.env = env;
    axis->backend_ctx.axis_id = axis_id;
    Sim_InitAxisByType(axis);

    env->axis_count += 1;
    return 1;
}

int HydraulicSim_ConfigureAxis(HydraulicSimEnv* env,
                               int axis_id,
                               float max_vel,
                               float max_acc,
                               float max_dec) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return 0;

    axis->max_vel_mm_s = max_vel;
    axis->max_acc_mm_s2 = max_acc;
    axis->max_dec_mm_s2 = max_dec;
    return 1;
}

SimAxisState* HydraulicSim_FindAxisById(HydraulicSimEnv* env, int axis_id) {
    int i;

    if (env == NULL) return NULL;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (env->axes[i].allocated && env->axes[i].axis_id == axis_id) {
            return &env->axes[i];
        }
    }

    return NULL;
}

const SimAxisState* HydraulicSim_FindAxisByIdConst(const HydraulicSimEnv* env, int axis_id) {
    int i;

    if (env == NULL) return NULL;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (env->axes[i].allocated && env->axes[i].axis_id == axis_id) {
            return &env->axes[i];
        }
    }

    return NULL;
}

SimAxisState* HydraulicSim_FindAxisByKind(HydraulicSimEnv* env, SimAxisKind axis_kind) {
    int i;

    if (env == NULL) return NULL;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (env->axes[i].allocated && env->axes[i].axis_type == axis_kind) {
            return &env->axes[i];
        }
    }

    return NULL;
}

const SimAxisState* HydraulicSim_FindAxisByKindConst(const HydraulicSimEnv* env, SimAxisKind axis_kind) {
    int i;

    if (env == NULL) return NULL;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (env->axes[i].allocated && env->axes[i].axis_type == axis_kind) {
            return &env->axes[i];
        }
    }

    return NULL;
}

int HydraulicSim_SetAxisCommand(HydraulicSimEnv* env,
                                int axis_id,
                                bool enable,
                                float cmd_rpm,
                                int direction) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return 0;

    axis->enabled = enable;
    axis->last_cmd_rpm = (cmd_rpm > 0.0f) ? cmd_rpm : 0.0f;
    axis->direction_cmd = HydraulicSim_NormalizeDirection(direction);
    axis->valve_cmd.valve_fwd = (axis->direction_cmd > 0);
    axis->valve_cmd.valve_bwd = (axis->direction_cmd < 0);

    if (enable) {
        env->cmd_rpm = axis->last_cmd_rpm;
        env->pump_owner_axis_id = axis->axis_id;
    } else if (env->pump_owner_axis_id == axis->axis_id) {
        env->cmd_rpm = 0.0f;
        env->pump_owner_axis_id = -1;
    }

    return 1;
}

int HydraulicSim_ReadAxis(HydraulicSimEnv* env, int axis_id, AxisFeedback* fb) {
    SimAxisState* axis;

    if (fb == NULL) return 0;

    axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return 0;

    *fb = axis->last_feedback;
    return 1;
}

ISensorBackend* HydraulicSim_GetAxisBackend(HydraulicSimEnv* env, int axis_id) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    return (axis != NULL) ? &axis->backend : NULL;
}

void HydraulicSim_SetValveSwitchDelay(HydraulicSimEnv* env, float delay_s) {
    (void)env;
    (void)delay_s;
}

void HydraulicSim_SetAxisServoReady(HydraulicSimEnv* env, int axis_id, bool ready) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.servo_ready = ready;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_SetAxisInterlock(HydraulicSimEnv* env, int axis_id, bool interlock_ok) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.interlock_ok = interlock_ok;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_SetAxisMotionStall(HydraulicSimEnv* env, int axis_id, bool stalled) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.motion_stalled = stalled;
}

void HydraulicSim_SetPressureSensorBias(HydraulicSimEnv* env, int axis_id, float bias_bar) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.pressure_bias_bar = bias_bar;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_SetPressureSensorScale(HydraulicSimEnv* env, int axis_id, float scale) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.pressure_scale = scale;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_SetPressureSensorStuck(HydraulicSimEnv* env,
                                         int axis_id,
                                         bool enabled,
                                         float stuck_bar) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.pressure_stuck_enabled = enabled;
    axis->feedback_inj.pressure_stuck_bar = stuck_bar;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_SetPressureSensorInvalid(HydraulicSimEnv* env, int axis_id, bool invalid) {
    SimAxisState* axis = HydraulicSim_FindAxisById(env, axis_id);
    if (axis == NULL) return;

    axis->feedback_inj.pressure_invalid = invalid;
    Sim_UpdateAxisFeedback(axis);
}

void HydraulicSim_Step(HydraulicSimEnv* env, float dt_s) {
    SimAxisState* owner_axis;
    float effective_dt_s;
    float flow_lpm;
    float flow_mm3_s;
    int i;

    if (env == NULL || !env->_initialized) return;

    effective_dt_s = (dt_s > 0.0f) ? dt_s : 0.0f;
    flow_lpm = (env->cmd_rpm > 0.0f)
             ? ((env->pump_displacement_ml_r * env->cmd_rpm / 1000.0f) * env->pump_vol_efficiency)
             : 0.0f;
    flow_mm3_s = flow_lpm * 16666.667f;

    env->sim_system_pressure_bar = 0.0f;

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (!env->axes[i].allocated) continue;

        env->axes[i].cylinder.current_vel_mm_s = 0.0f;
        env->axes[i].branch_pressure_bar = 0.0f;
    }

    owner_axis = HydraulicSim_FindAxisById(env, env->pump_owner_axis_id);
    if (owner_axis != NULL) {
        const int direction = HydraulicSim_NormalizeDirection(owner_axis->direction_cmd);
        const float dir_sign = (float)direction;
        const float area_mm2 = Sim_GetAreaForDirection(owner_axis, direction);

        if (direction != 0 && area_mm2 > 0.1f) {
            if (env->cmd_rpm > 0.0f &&
                owner_axis->enabled &&
                owner_axis->feedback_inj.interlock_ok &&
                owner_axis->feedback_inj.servo_ready &&
                !owner_axis->feedback_inj.motion_stalled) {
                float velocity_mm_s = dir_sign * (flow_mm3_s / area_mm2);
                float next_pos_mm = owner_axis->cylinder.current_pos_mm + (velocity_mm_s * effective_dt_s);
                float clamped_pos_mm = Sim_ClampPosition(owner_axis, next_pos_mm);

                if (fabsf(clamped_pos_mm - next_pos_mm) > 1e-6f) {
                    velocity_mm_s = 0.0f;
                }

                owner_axis->cylinder.current_pos_mm = clamped_pos_mm;
                owner_axis->cylinder.current_vel_mm_s = velocity_mm_s;
            }

            owner_axis->branch_pressure_bar =
                (Sim_ComputeLoadForceByType(env, owner_axis, dir_sign) / area_mm2) * 10.0f;
            env->sim_system_pressure_bar = owner_axis->branch_pressure_bar;
        }
    }

    for (i = 0; i < HYD_MAX_HYDRAULIC_SIM_FB; ++i) {
        if (!env->axes[i].allocated) continue;
        Sim_UpdateAxisFeedback(&env->axes[i]);
    }

    env->sim_time_s += effective_dt_s;
}
