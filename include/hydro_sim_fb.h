#ifndef HYDRO_SIM_FB_H
#define HYDRO_SIM_FB_H

#include "common_types.h"
#include "hydro_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================================
 * 液压仿真器 PLCopen 功能块 (HydraulicSimFB)
 *
 * 设计原则：
 * - 高电平使能模式：EN=true 步进仿真，EN=false 立即冻结输出并复位
 * - 无状态机 / 无边沿检测：简洁直接，适合 PLC 周期调用
 * - 纯 C99 / 静态有界内存 / 无动态分配
 *
 * 调用模式：
 *   1. HDY_HydraulicSimFB_Init(&fb)       — 一次性初始化
 *   2. 每周期设置输入引脚（EN, CMD_RPM, 阀门, 故障注入等）
 *   3. HDY_HydraulicSimFB_Cycle(&fb)      — 周期调用
 *   4. 读取输出引脚（位置/速度/压力, ENO, ACTIVE 等）
 *
 * EN 语义：
 *   EN=true  → ENO=true, ACTIVE=true, 每周期步进仿真
 *   EN=false → ENO=false, ACTIVE=false, 输出归零复位，不再步进
 *   EN 从 false→true：仿真从当前物理状态继续（不重新初始化）
 * ================================================================== */

typedef struct {
    /* ---- 输入引脚 ---- */

    HDY_BOOL EN;                    /* 高电平使能: true=运行, false=冻结复位 */

    HDY_REAL CYCLE_TIME;            /* 周期步长(秒), 如 0.001 = 1ms */

    /* 泵指令输入 */
    HDY_REAL CMD_RPM;               /* 泵指令转速 (rpm) */
    HDY_UINT8 PUMP_OWNER_AXIS;      /* 泵归属轴: 0=CLAMP, 1=INJECT */

    /* 合模轴阀门指令 */
    HDY_BOOL CLAMP_VALVE_FWD;       /* 合模前进阀 */
    HDY_BOOL CLAMP_VALVE_BWD;       /* 合模后退阀 */

    /* 射胶轴阀门指令 */
    HDY_BOOL INJECT_VALVE_FWD;      /* 射胶前进阀 */
    HDY_BOOL INJECT_VALVE_BWD;      /* 射胶后退阀 */

    /* 故障注入输入 (PLC 可在线设置, 每次 Cycle 同步到内部 env) */
    HDY_BOOL CLAMP_SERVO_READY;     /* 合模伺服就绪 (默认 true) */
    HDY_BOOL CLAMP_INTERLOCK_OK;    /* 合模互锁正常 (默认 true) */
    HDY_BOOL CLAMP_MOTION_STALL;    /* 合模动作卡死 (默认 false) */
    HDY_BOOL INJECT_SERVO_READY;    /* 射胶伺服就绪 (默认 true) */
    HDY_BOOL INJECT_INTERLOCK_OK;   /* 射胶互锁正常 (默认 true) */
    HDY_BOOL INJECT_MOTION_STALL;   /* 射胶动作卡死 (默认 false) */

    HDY_REAL CLAMP_PRESSURE_BIAS;   /* 合模压力偏置 (bar) */
    HDY_REAL CLAMP_PRESSURE_SCALE;  /* 合模压力比例因子 (默认 1.0) */
    HDY_REAL INJECT_PRESSURE_BIAS;  /* 射胶压力偏置 (bar) */
    HDY_REAL INJECT_PRESSURE_SCALE; /* 射胶压力比例因子 (默认 1.0) */

    HDY_BOOL MOLD_OBSTACLE;         /* 模具障碍物使能 */
    HDY_REAL OBSTACLE_POS_MM;       /* 障碍物位置 (mm) */
    HDY_REAL OBSTACLE_STIFFNESS;    /* 障碍物刚度 (N/mm) */

    /* ---- 输出引脚 ---- */

    HDY_BOOL ENO;                   /* 使能输出, 跟随 EN */
    HDY_BOOL ACTIVE;                /* 仿真正在运行 (EN=true 时为 true) */

    /* 合模轴反馈输出 */
    HDY_REAL CLAMP_POS_MM;          /* 合模位置 (mm) */
    HDY_REAL CLAMP_VEL_MM_S;        /* 合模速度 (mm/s) */
    HDY_REAL CLAMP_PRESSURE_BAR;    /* 合模压力 (bar) */

    /* 射胶轴反馈输出 */
    HDY_REAL INJECT_POS_MM;         /* 射胶位置 (mm) */
    HDY_REAL INJECT_VEL_MM_S;       /* 射胶速度 (mm/s) */
    HDY_REAL INJECT_PRESSURE_BAR;   /* 射胶压力 (bar) */

    /* 系统级输出 */
    HDY_REAL SYSTEM_PRESSURE_BAR;   /* 系统压力 (bar) */
    HDY_REAL SIM_TIME_S;            /* 仿真累计时间 (s) */

    /* ---- 内部状态 ---- */
    HydraulicSimEnv _env;           /* 内嵌仿真环境 */
    HDY_BOOL _initialized;          /* 是否已初始化 (Init 后为 true) */
} HDY_HydraulicSimFB;

/**
 * @brief 完整初始化仿真器功能块（含物理参数默认值）
 * @param fb 功能块指针
 * @note 调用后 EN=false, 输出全部归零, 物理参数为默认值
 */
void HDY_HydraulicSimFB_Init(HDY_HydraulicSimFB* fb);

/**
 * @brief PLC 周期调用入口
 *
 * 行为:
 *   EN=false → 输出归零, ENO=false, ACTIVE=false, 不步进
 *   EN=true  → 同步输入到内部 env, 步进仿真, 拷贝输出, ENO=true, ACTIVE=true
 *
 * @param fb 功能块指针
 */
void HDY_HydraulicSimFB_Cycle(HDY_HydraulicSimFB* fb);

/**
 * @brief 获取内部 ISensorBackend (合模轴)
 * @note 供旧代码 / 高级用途使用, 通过 backend 可与控制器轴直接对接
 */
ISensorBackend* HDY_HydraulicSimFB_GetClampBackend(HDY_HydraulicSimFB* fb);

/**
 * @brief 获取内部 ISensorBackend (射胶轴)
 * @note 供旧代码 / 高级用途使用, 通过 backend 可与控制器轴直接对接
 */
ISensorBackend* HDY_HydraulicSimFB_GetInjectBackend(HDY_HydraulicSimFB* fb);

/**
 * @brief 获取内部 HydraulicSimEnv 指针
 * @note 高级用途: 可直接修改物理参数 (泵排量, 油缸面积, 摩擦力等)
 *       Cycle() 内部始终基于 _env 执行, 修改后下一周期自动生效
 */
HydraulicSimEnv* HDY_HydraulicSimFB_GetEnv(HDY_HydraulicSimFB* fb);

#ifdef __cplusplus
}
#endif

#endif /* HYDRO_SIM_FB_H */
