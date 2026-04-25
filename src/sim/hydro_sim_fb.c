/**
 * @file hydro_sim_fb.c
 * @brief 液压仿真器 PLCopen 功能块实现
 *
 * 简洁的高电平使能模式:
 *   EN=true  → 每周期同步输入 → 步进仿真 → 同步输出
 *   EN=false → 输出归零冻结, 不步进
 */

#include "hydro_sim_fb.h"
#include <string.h>

/* ==================================================================
 * 内部辅助: 将 FB 输入引脚同步到内部 HydraulicSimEnv
 * ================================================================== */
static void SyncInputsToEnv(HDY_HydraulicSimFB* fb) {
    HydraulicSimEnv* env = &fb->_env;

    /* 泵指令 */
    env->cmd_rpm = (float)fb->CMD_RPM;
    if (fb->PUMP_OWNER_AXIS == 0) {
        env->pump_owner_axis = SIM_AXIS_CLAMP;
    } else if (fb->PUMP_OWNER_AXIS == 1) {
        env->pump_owner_axis = SIM_AXIS_INJECT;
    } else {
        env->pump_owner_axis = SIM_AXIS_NONE;
    }

    /* 合模轴阀门 */
    env->clamp_cmd.valve_fwd = fb->CLAMP_VALVE_FWD;
    env->clamp_cmd.valve_bwd = fb->CLAMP_VALVE_BWD;

    /* 射胶轴阀门 */
    env->inject_cmd.valve_fwd = fb->INJECT_VALVE_FWD;
    env->inject_cmd.valve_bwd = fb->INJECT_VALVE_BWD;

    /* 合模轴故障注入 */
    env->clamp_feedback_injection.servo_ready   = fb->CLAMP_SERVO_READY;
    env->clamp_feedback_injection.interlock_ok  = fb->CLAMP_INTERLOCK_OK;
    env->clamp_feedback_injection.motion_stalled = fb->CLAMP_MOTION_STALL;
    env->clamp_feedback_injection.pressure_bias_bar = (float)fb->CLAMP_PRESSURE_BIAS;
    env->clamp_feedback_injection.pressure_scale    = (float)fb->CLAMP_PRESSURE_SCALE;

    /* 射胶轴故障注入 */
    env->inject_feedback_injection.servo_ready   = fb->INJECT_SERVO_READY;
    env->inject_feedback_injection.interlock_ok  = fb->INJECT_INTERLOCK_OK;
    env->inject_feedback_injection.motion_stalled = fb->INJECT_MOTION_STALL;
    env->inject_feedback_injection.pressure_bias_bar = (float)fb->INJECT_PRESSURE_BIAS;
    env->inject_feedback_injection.pressure_scale    = (float)fb->INJECT_PRESSURE_SCALE;

    /* 模具障碍物 */
    env->inject_mold_obstacle    = fb->MOLD_OBSTACLE;
    env->obstacle_pos_mm         = (float)fb->OBSTACLE_POS_MM;
    env->obstacle_stiffness_N_mm = (float)fb->OBSTACLE_STIFFNESS;
}

/* ==================================================================
 * 内部辅助: 将内部 HydraulicSimEnv 状态拷贝到 FB 输出引脚
 * ================================================================== */
static void SyncEnvToOutputs(HDY_HydraulicSimFB* fb) {
    const HydraulicSimEnv* env = &fb->_env;

    /* 合模轴 */
    fb->CLAMP_POS_MM       = (HDY_REAL)env->clamp_cyl.current_pos_mm;
    fb->CLAMP_VEL_MM_S     = (HDY_REAL)env->clamp_cyl.current_vel_mm_s;
    fb->CLAMP_PRESSURE_BAR = (HDY_REAL)env->sim_system_pressure_bar;

    /* 射胶轴 */
    fb->INJECT_POS_MM       = (HDY_REAL)env->inject_cyl.current_pos_mm;
    fb->INJECT_VEL_MM_S     = (HDY_REAL)env->inject_cyl.current_vel_mm_s;
    fb->INJECT_PRESSURE_BAR = (HDY_REAL)env->sim_system_pressure_bar;

    /* 系统级 */
    fb->SYSTEM_PRESSURE_BAR = (HDY_REAL)env->sim_system_pressure_bar;
    fb->SIM_TIME_S          = (HDY_REAL)env->sim_time_s;
}

/* ==================================================================
 * 内部辅助: EN=false 时归零所有输出引脚
 * ================================================================== */
static void ZeroOutputs(HDY_HydraulicSimFB* fb) {
    fb->ENO                = false;
    fb->ACTIVE             = false;
    fb->CLAMP_POS_MM       = 0.0;
    fb->CLAMP_VEL_MM_S     = 0.0;
    fb->CLAMP_PRESSURE_BAR = 0.0;
    fb->INJECT_POS_MM       = 0.0;
    fb->INJECT_VEL_MM_S     = 0.0;
    fb->INJECT_PRESSURE_BAR = 0.0;
    fb->SYSTEM_PRESSURE_BAR = 0.0;
    fb->SIM_TIME_S          = 0.0;
}

/* ==================================================================
 * 公共 API
 * ================================================================== */

void HDY_HydraulicSimFB_Init(HDY_HydraulicSimFB* fb) {
    if (fb == NULL) return;

    /* 清零整个结构体 */
    memset(fb, 0, sizeof(HDY_HydraulicSimFB));

    /* 初始化内部仿真环境 (设置物理参数默认值) */
    HydraulicSim_Init(&fb->_env);

    /* 设置故障注入默认值 — 与 HydraulicSim_Init 一致 */
    fb->CLAMP_SERVO_READY    = true;
    fb->CLAMP_INTERLOCK_OK   = true;
    fb->CLAMP_MOTION_STALL   = false;
    fb->INJECT_SERVO_READY   = true;
    fb->INJECT_INTERLOCK_OK  = true;
    fb->INJECT_MOTION_STALL  = false;
    fb->CLAMP_PRESSURE_SCALE  = 1.0;
    fb->INJECT_PRESSURE_SCALE = 1.0;

    /* EN 默认 false — 调用者必须显式置 EN=true 才会步进 */
    fb->EN           = false;
    fb->ENO          = false;
    fb->ACTIVE       = false;
    fb->_initialized = true;
}

void HDY_HydraulicSimFB_Cycle(HDY_HydraulicSimFB* fb) {
    if (fb == NULL || !fb->_initialized) return;

    /* ---- EN=false: 冻结输出, 不步进 ---- */
    if (!fb->EN) {
        ZeroOutputs(fb);
        return;
    }

    /* ---- EN=true: 正常运行 ---- */
    fb->ENO    = true;
    fb->ACTIVE = true;

    /* 1. 同步输入引脚 → 内部 env */
    SyncInputsToEnv(fb);

    /* 2. 步进仿真 */
    HydraulicSim_Step(&fb->_env, (float)fb->CYCLE_TIME);

    /* 3. 同步内部 env → 输出引脚 */
    SyncEnvToOutputs(fb);
}

ISensorBackend* HDY_HydraulicSimFB_GetClampBackend(HDY_HydraulicSimFB* fb) {
    return (fb != NULL) ? HydraulicSim_GetClampBackend(&fb->_env) : NULL;
}

ISensorBackend* HDY_HydraulicSimFB_GetInjectBackend(HDY_HydraulicSimFB* fb) {
    return (fb != NULL) ? HydraulicSim_GetInjectBackend(&fb->_env) : NULL;
}

HydraulicSimEnv* HDY_HydraulicSimFB_GetEnv(HDY_HydraulicSimFB* fb) {
    return (fb != NULL) ? &fb->_env : NULL;
}

void __mcl_cmd_injectsimulator(INJECTSIMULATOR *data__)
{
    // 这里可以实现对 INJECTSIMULATOR 功能块的周期性处理逻辑
    // 例如：读取输入参数, 更新仿真状态, 输出结果等
    IEC_BOOL bInit = __GET_VAR(data__->INIT);
    
    if (!bInit)
    {
        HDY_HydraulicSimFB_Init(&data__->_sim_fb);
        __SET_VAR(data__->,INIT, ,true);
    }
    else
    {
        IEC_BOOL Enable = __GET_VAR(data__->ENABLE);
        data__->_sim_fb.EN = Enable;  // 使能仿真器
        if (Enable)
        {
            // 同步输入参数到内部仿真环境
            IEC_REAL cycletime = __GET_VAR(data__->CYCLE_TIME);
            IEC_REAL cmdrpm = __GET_VAR(data__->CMD_RPM);
            IEC_USINT axisid = __GET_VAR(data__->PUMP_OWNER_AXIS);
            IEC_SINT direction = __GET_VAR(data__->DIRECTION);
            IEC_REAL pressure_bias = __GET_VAR(data__->PRESSURE_BIAS);
            IEC_REAL pressure_scale = __GET_VAR(data__->PRESSURE_SCALE);

            data__->_sim_fb.CYCLE_TIME = cycletime;
            data__->_sim_fb.CMD_RPM = cmdrpm;
            data__->_sim_fb.PUMP_OWNER_AXIS = axisid;

            switch (axisid)
            {
            case 0:
                data__->_sim_fb.CLAMP_PRESSURE_BIAS = pressure_bias;
                data__->_sim_fb.CLAMP_PRESSURE_SCALE = pressure_scale;
                switch (direction)
                {
                case 0:
                    data__->_sim_fb.CLAMP_VALVE_FWD = false;
                    data__->_sim_fb.CLAMP_VALVE_BWD = false;
                    break;
                case 1:
                    data__->_sim_fb.CLAMP_VALVE_FWD = true;
                    data__->_sim_fb.CLAMP_VALVE_BWD = false;
                    break;
                case -1:
                    data__->_sim_fb.CLAMP_VALVE_FWD = false;
                    data__->_sim_fb.CLAMP_VALVE_BWD = true;
                default:
                    break;
                }
                break;
            case 1:
                data__->_sim_fb.INJECT_PRESSURE_BIAS = pressure_bias;
                data__->_sim_fb.INJECT_PRESSURE_SCALE = pressure_scale;
                switch (direction)
                {
                case 0:
                    data__->_sim_fb.INJECT_VALVE_FWD = false;
                    data__->_sim_fb.INJECT_VALVE_BWD = false;
                    break;
                case 1:
                    data__->_sim_fb.INJECT_VALVE_FWD = true;
                    data__->_sim_fb.INJECT_VALVE_BWD = false;
                    break;
                case -1:
                    data__->_sim_fb.INJECT_VALVE_FWD = false;
                    data__->_sim_fb.INJECT_VALVE_BWD = true;
                default:
                    break;
                }
                break;
            default:
                break;
            }

            HDY_HydraulicSimFB_Cycle(&data__->_sim_fb);

            // 这里可以将仿真结果写回到输出变量
            __SET_VAR(data__->,ACTIVE, ,data__->_sim_fb.ACTIVE);
            switch (axisid)
            {
                case 0:                  
                    __SET_VAR(data__->,POS_MM, ,data__->_sim_fb.CLAMP_POS_MM);
                    __SET_VAR(data__->,VEL_MM_S, ,data__->_sim_fb.CLAMP_VEL_MM_S);
                    __SET_VAR(data__->,PRESSURE_BAR, ,data__->_sim_fb.CLAMP_PRESSURE_BAR);
                    break;
                case 1:
                    __SET_VAR(data__->,POS_MM, ,data__->_sim_fb.INJECT_POS_MM);
                    __SET_VAR(data__->,VEL_MM_S, ,data__->_sim_fb.INJECT_VEL_MM_S);
                    __SET_VAR(data__->,PRESSURE_BAR, ,data__->_sim_fb.INJECT_PRESSURE_BAR);
                    break;
                default:
                    break;
            }

        }
        else
        {
            // EN=false 时, FB 内部会自动归零输出引脚, 这里可以选择是否需要额外处理
        }
    }
}