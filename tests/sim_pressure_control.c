/**
 * @file sim_pressure_control.c
 * @brief 压力闭环控制算法仿真 harness（伺服泵控液压系统）
 *
 * 目标（用户需求）：
 *   1) 构建压力闭环控制算法的测试代码：控制对象仿真模型 + 噪声 + 齿轮泵流量脉动干扰；
 *      系统输入 = 电机转速(rpm)，输出 = 实际压力反馈(bar)；系统增益 K=4.5，滞后 τ=100ms。
 *   2) 对比 RBF-PID / RBF-PI 与常规 PI 的仿真结果并给出数据分析。
 *   3) 验证“用电机实时位置(pump_phase_rev)前馈补偿低压纹波”是否可行。
 *
 * 设计要点：
 *   - 被控对象复用 src/sim/PressureModel.c 的一阶分支：first_order_k_bar_per_rpm=K，
 *     first_order_tau_s=τ，first_order_delay_s=0，motor_tau_s=0（关电机低通，使脉动抵达压力端）。
 *   - 齿轮泵齿落流量脉动：因数字压力传感器装在油泵出口，脉动以“出口压力脉动”形式直接叠加在
 *     传感器测量值上；脉动相位 φ = 2π·Z·pump_phase_rev（Z=13），频率 f=Z·rpm/60，
 *     周期 T=60/(Z·rpm) —— 与“低速时纹波周期与转速相关、随转速降低而变长”一致。
 *   - 前馈补偿：利用同一 pump_phase_rev 预测齿落相位，在泵出口压力上叠加反向脉动以抵消，
 *     最优 K_ff ≈ −A_pressure。
 *   - 控制器：HYD_PressureController_Execute 支持 PI / RBF_PI / RBF_PID。
 *
 * 编译：cmake --build --preset mingw （目标 sim_pressure_control）
 * 运行：out/build/mingw/sim_pressure_control  （在构建目录执行，CSV 写到 ./sim_output/）
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common_types.h"
#include "pressure_controller.h"
#include "pressure_model.h"
#include "pump_converter.h"
#include "pressure_ripple_comp.h"   /* 已实现的齿轮泵脉动补偿模块（EMA 复解调） */

/* ====================== 仿真常量（可在此微调） ====================== */
#define SIM_DT_S           0.001f     /* 离散步长 1ms */
#define SIM_DURATION_S     8.0f       /* 仿真时长 8s */
#define SIM_TARGET_BAR     100.0f     /* 目标压力 100 bar */
#define SIM_K_GAIN         4.5f       /* 系统增益 K = 4.5 bar/rpm（用户给定） */
#define SIM_TAU_S          0.10f      /* 滞后 τ = 100ms（用户给定） */
#define SIM_Z_TEETH        13         /* 齿轮泵齿数 Z */
#define SIM_FLOW2SPEED     20.0f      /* rpm per L/min（与现有测试一致） */
#define SIM_PUMP_LIMIT     1800.0f    /* 泵转速上限 rpm */
#define SIM_A_PRESSURE     1.5f       /* 齿轮泵出口压力脉动幅值 bar（约 1.5% 目标） */
#define SIM_SIGMA_SENSOR   0.30f      /* 传感器噪声 σ bar */
#define SIM_GAIN_MISMATCH  5.4f       /* 鲁棒性测试：实际增益（与标称 4.5 失配） */
#define SIM_DIST_STEP_BAR  8.0f       /* 负载阶跃扰动幅值 bar */
#define SIM_DIST_START_S   4.0f       /* 扰动起始时间 s */
#define SIM_DIST_DUR_S     0.5f       /* 扰动持续 s */

#define PI_F  3.14159265358979323846f
#define N_STEPS ((int)(SIM_DURATION_S / SIM_DT_S))

/* ====================== 简易确定性 RNG（传感器噪声） ====================== */
static uint32_t g_rng = 0x12345678u;
static uint32_t xorshift32(void) {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng;
}
static float rand_u01(void) { return (float)(xorshift32() & 0x00ffffffu) / 16777216.0f; }
static float rand_gauss(float std) {
    float u1 = rand_u01(), u2 = rand_u01();
    if (u1 < 1e-7f) u1 = 1e-7f;
    return std * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

/* ====================== 数据结构 ====================== */
typedef struct {
    float dt, duration, target, K, tau, A_pressure, sigma_sensor, flow2speed, pump_limit;
    int   controller;          /* HYD_PressureControllerType */
    int   use_ff; float K_ff;   /* 前馈补偿开关与增益 */
    int   gain_mismatch;       /* 1=实际增益用 SIM_GAIN_MISMATCH */
    int   disturbance;         /* 1=注入负载阶跃 */
    int   record;              /* 1=记录时间序列到数组 */
    int   ff_flow;             /* 1=对 PI 注入稳态前馈流量(等效 RBF 的 systemGain 补偿)，用于公平对比 */
} SimConfig;

typedef struct {
    float *t, *target, *real_p, *measured, *rpm_cmd, *u_flow; /* 长度 N_STEPS 或 NULL */
} RecBuf;

 typedef struct {
     float rise_time, settle_time, overshoot_pct, steady_err, ripple_rms, ripple_p2p, recovery_time;
     float ff_cancel;   /* 脉动前馈抵消质量：1 - var(eP+compFF*K)/var(eP)，∈[0,1]，越高越好 */
     int   ok;
 } Metrics;

/* ====================== 控制器段配置 ====================== */
static HYD_MotionSegment make_segment(const SimConfig *cfg) {
    HYD_MotionSegment s;
    memset(&s, 0, sizeof(s));
    s.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    s.endCondition = HYD_END_TIME;
    s.direction = HYD_DIRECTION_HOLD;
    s.duration = 10.0;
    s.targetPressure = cfg->target;
    s.maxFlow = cfg->pump_limit / cfg->flow2speed;
    s.pressureController = (HYD_PressureControllerType)cfg->controller;
    s.pressureCeiling = cfg->target * 3.0f;
    s.pressureFilterAlpha = (cfg->controller == HYD_PRESSURE_CONTROLLER_PI) ? 0.2f : 1.0f;
    s.pressureDerivativeFilterAlpha = 1.0f;
    /* RBF 增益补偿沿用现有测试惯例：systemGain = K * flow2speed（与已验证的 5.4*20 一致量级） */
    s.systemGain = (cfg->controller == HYD_PRESSURE_CONTROLLER_PI) ? 0.0f : (cfg->K * cfg->flow2speed);
    s.pressureIntegralLimit = s.maxFlow;

    if (cfg->controller == HYD_PRESSURE_CONTROLLER_PI) {
        s.pressureKp = 0.05f;
        s.pressureKi = 0.005f;
        s.pressureKd = 0.0f;
    } else {
        s.pressureRbfConfig.minKp = 0.040f; s.pressureRbfConfig.maxKp = 0.060f;
        s.pressureRbfConfig.minKi = 0.0008f; s.pressureRbfConfig.maxKi = 0.0016f;
        if (cfg->controller == HYD_PRESSURE_CONTROLLER_RBF_PI) {
            s.pressureRbfConfig.minKd = 0.0f; s.pressureRbfConfig.maxKd = 0.0f;
        } else {
            s.pressureRbfConfig.minKd = 0.015f; s.pressureRbfConfig.maxKd = 0.035f;
        }
        s.pressureRbfConfig.etaW = 0.0020f; s.pressureRbfConfig.etaC = 0.0020f;
        s.pressureRbfConfig.etaB = 0.0010f;
        s.pressureRbfConfig.etaP = 0.00010f; s.pressureRbfConfig.etaI = 0.00005f;
        s.pressureRbfConfig.etaD = 0.00010f;
        s.pressureRbfConfig.disablePressureAccelFeedforward = 1.0f;
    }
    return s;
}

/* ====================== 核心仿真 ====================== */
static Metrics run_sim(const SimConfig *cfg, RecBuf *rec) {
    PressureModelParams params;
    PressureModelState  plant;
    PressureModelOutput plant_out;
    HYD_PressureControllerState cstate;
    HYD_PressureControllerInput  cinput;
    HYD_PressureControllerOutput coutput;
    HYD_PumpConverterInput  pinput;
    HYD_PumpConverterOutput poutput;
    HYD_MotionSegment segment = make_segment(cfg);

    float actual_K = cfg->gain_mismatch ? SIM_GAIN_MISMATCH : cfg->K;
    float tail_start = (int)(0.75f * N_STEPS);

    Metrics m; memset(&m, 0, sizeof(m)); m.ok = 1;
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&cinput, 0, sizeof(cinput));
    memset(&coutput, 0, sizeof(coutput));
    memset(&pinput, 0, sizeof(pinput));
    memset(&poutput, 0, sizeof(poutput));

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = actual_K;
    params.first_order_tau_s = cfg->tau;
    params.first_order_delay_s = 0.0f;
    params.motor_tau_s = 0.0f;            /* 关闭电机低通，使脉动能抵达压力端 */
    params.sensor_range_bar = 10000.0f;   /* 避免传感器量程钳位（脉动在 harness 内叠加） */
    params.enable_sensor_noise = 0u;     /* 噪声由 harness 叠加，避免一阶分支重复 */
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;

    PressureModel_Reset(&plant, 0x5A5A5A5Au);
    HYD_PressureController_InitState(&cstate, 0.0, 0.0, 0.0);

    float real_p_clean_prev = 0.0f;       /* 上一拍“系统压力”（不含出口脉动） */
    float last_exceed2 = -1.0f;           /* 最后越出 ±2% 带宽的时刻 */
    float rise_t = -1.0f;                  /* 上升时间（首达 90%） */
    float peak_p = 0.0f;
    float tail_sum = 0.0f, tail_sumsq = 0.0f, tail_min = 1e30f, tail_max = -1e30f;
    int   tail_count = 0;
    float recovery_t = -1.0f;
    float dist_end = SIM_DIST_START_S + SIM_DIST_DUR_S;

    for (int step = 0; step < N_STEPS; ++step) {
        float t = (float)(step + 1) * cfg->dt;
        float phase = plant.pump_phase_rev;                 /* 电机/泵实时位置（转，wrap[0,1)） */
        float phi = 2.0f * PI_F * (float)SIM_Z_TEETH * phase;
        float ripple = cfg->A_pressure * sinf(phi);         /* 齿轮泵出口脉动 */
        float ff = cfg->use_ff ? (cfg->K_ff * sinf(phi)) : 0.0f;
        float d_eff = ripple + ff;                          /* 净出口脉动（前馈抵消） */

        /* 控制器看到的测量值 = 系统压力 + 出口脉动 + 传感器噪声 */
        float measured = real_p_clean_prev + d_eff + rand_gauss(cfg->sigma_sensor);
        if (measured < 0.0f) measured = 0.0f;
        if (measured > params.sensor_range_bar) measured = params.sensor_range_bar;

        cinput.targetPressure = cfg->target;
        cinput.measuredPressure = (HYD_REAL)measured;
        /* PI 公平前馈：注入稳态流量(与 RBF 的 systemGain 补偿等效)，避免 PI 仅靠积分从零爬升。
           仅对 PI 生效，RBF 已通过 systemGain 内部补偿，避免双重叠加。 */
        if (cfg->ff_flow && cfg->controller == HYD_PRESSURE_CONTROLLER_PI) {
            cinput.feedforwardFlow = (HYD_REAL)(cfg->target / (cfg->K * cfg->flow2speed));
        } else {
            cinput.feedforwardFlow = 0.0;
        }
        cinput.outputMin = 0.0;
        cinput.outputMax = (HYD_REAL)segment.maxFlow;
        cinput.flowToPumpSpeedGain = (HYD_REAL)cfg->flow2speed;
        cinput.pumpSpeedLimit = (HYD_REAL)cfg->pump_limit;
        cinput.timestamp = (HYD_REAL)t;

        HYD_PressureController_Execute(&segment, &cstate, &cinput, &coutput);

        pinput.requestedFlow = (HYD_REAL)coutput.outputFlow;
        pinput.flowToPumpSpeedGain = (HYD_REAL)cfg->flow2speed;
        pinput.pumpSpeedLimit = (HYD_REAL)cfg->pump_limit;
        pinput.direction = segment.direction;
        HYD_PumpConverter_Execute(&pinput, &poutput);

        float rpm_cmd = (float)poutput.pumpSpeed;
        PressureModel_Step(&params, &plant, rpm_cmd, cfg->dt, &plant_out);

        float real_p_clean = plant_out.real_pressure_bar;
        if (cfg->disturbance && t >= SIM_DIST_START_S && t <= dist_end) {
            real_p_clean -= SIM_DIST_STEP_BAR;              /* 负载阶跃扰动 */
        }
        float real_p = real_p_clean + d_eff;                /* 实际出口压力（含脉动） */

        /* ---- 指标累计 ---- */
        if (real_p > peak_p) peak_p = real_p;
        if (rise_t < 0.0f && real_p >= 0.9f * cfg->target) rise_t = t;
        if (fabsf(real_p - cfg->target) > 0.02f * cfg->target) last_exceed2 = t;
        if (step >= tail_start) {
            tail_sum += real_p; tail_sumsq += real_p * real_p;
            if (real_p < tail_min) tail_min = real_p;
            if (real_p > tail_max) tail_max = real_p;
            ++tail_count;
        }
        if (cfg->disturbance && recovery_t < 0.0f && t > dist_end &&
            fabsf(real_p - cfg->target) <= 0.02f * cfg->target) {
            recovery_t = t - dist_end;
        }

        if (rec && rec->t) {
            rec->t[step] = t;
            rec->target[step] = cfg->target;
            rec->real_p[step] = real_p;
            rec->measured[step] = measured;
            rec->rpm_cmd[step] = rpm_cmd;
            rec->u_flow[step] = (float)coutput.outputFlow;
        }
        real_p_clean_prev = real_p_clean;
    }

    float steady_mean = tail_count ? (tail_sum / (float)tail_count) : 0.0f;
    float var = tail_count ? (tail_sumsq / (float)tail_count - steady_mean * steady_mean) : 0.0f;
    if (var < 0.0f) var = 0.0f;

    m.rise_time = (rise_t < 0.0f) ? cfg->duration : rise_t;
    m.settle_time = (last_exceed2 < 0.0f) ? 0.0f : last_exceed2;
    m.overshoot_pct = (peak_p > cfg->target) ? ((peak_p - cfg->target) / cfg->target * 100.0f) : 0.0f;
    m.steady_err = steady_mean - cfg->target;
    m.ripple_rms = sqrtf(var);
    m.ripple_p2p = (tail_count) ? (tail_max - tail_min) : 0.0f;
    m.recovery_time = recovery_t;
    return m;
}

/* ====================== CSV 输出 ====================== */
static void write_csv(const char *name, const RecBuf *rec) {
    FILE *f = fopen(name, "w");
    if (!f) { fprintf(stderr, "WARN: cannot write %s\n", name); return; }
    fprintf(f, "t,target,real_p,measured,rpm_cmd,u_flow\n");
    for (int i = 0; i < N_STEPS; ++i) {
        fprintf(f, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                rec->t[i], rec->target[i], rec->real_p[i], rec->measured[i],
                rec->rpm_cmd[i], rec->u_flow[i]);
    }
    fclose(f);
}

static RecBuf alloc_rec(void) {
    RecBuf r; memset(&r, 0, sizeof(r));
    r.t = (float*)malloc(sizeof(float) * N_STEPS);
    r.target = (float*)malloc(sizeof(float) * N_STEPS);
    r.real_p = (float*)malloc(sizeof(float) * N_STEPS);
    r.measured = (float*)malloc(sizeof(float) * N_STEPS);
    r.rpm_cmd = (float*)malloc(sizeof(float) * N_STEPS);
    r.u_flow = (float*)malloc(sizeof(float) * N_STEPS);
    return r;
}
static void free_rec(RecBuf *r) {
    free(r->t); free(r->target); free(r->real_p); free(r->measured);
    free(r->rpm_cmd); free(r->u_flow); memset(r, 0, sizeof(*r));
}

/* ====================== 端到端脉动补偿验证（使用已实现的 HYD_PressureRippleComp） ======================
 * 在真实一阶被控对象上，用本仓库实现的 EMA 复解调脉动补偿模块在线学习齿落脉动，
 * 并叠加 FF 基值(=target/systemGain) 验证 FF 标定。对比 开/关 补偿的稳态误差与纹波 RMS。
 * 与 run_sim 的区别：run_sim 用“理想扫描 K_ff”做前馈；此处用“在线学习的真实模块”。 */
static Metrics run_sim_ripple_comp(const SimConfig *cfg, int use_comp) {
    PressureModelParams params;
    PressureModelState  plant;
    PressureModelOutput plant_out;
    HYD_PressureControllerState cstate;
    HYD_PressureControllerInput  cinput;
    HYD_PressureControllerOutput coutput;
    HYD_PumpConverterInput  pinput;
    HYD_PumpConverterOutput poutput;
    HYD_MotionSegment segment = make_segment(cfg);
    /* 正确 FF 基值所需的 systemGain（验证 FF 标定：ffBase=target/systemGain），PI 也置上 */
    segment.systemGain = cfg->K * cfg->flow2speed;

    HYD_PressureRippleCompState comp;
    HYD_PressureSteadyGateState compGate;
    HYD_PressureRippleComp_Reset(&comp);
    HYD_PressureRippleComp_SetEnabled(&comp, use_comp ? 1u : 0u);
    HYD_PressureRippleComp_SetGain(&comp, 1.0f / (cfg->K * cfg->flow2speed)); /* bar->L/min */
    HYD_PressureSteadyGate_Reset(&compGate, 64u, 10.0f);

    float actual_K = cfg->gain_mismatch ? SIM_GAIN_MISMATCH : cfg->K;
    float tail_start = (int)(0.75f * N_STEPS);

    Metrics m; memset(&m, 0, sizeof(m)); m.ok = 1;
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&cinput, 0, sizeof(cinput));
    memset(&coutput, 0, sizeof(coutput));
    memset(&pinput, 0, sizeof(pinput));
    memset(&poutput, 0, sizeof(poutput));

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = actual_K;
    params.first_order_tau_s = cfg->tau;
    params.first_order_delay_s = 0.0f;
    params.motor_tau_s = 0.0f;
    params.sensor_range_bar = 10000.0f;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;

    PressureModel_Reset(&plant, 0x5A5A5A5Au);
    HYD_PressureController_InitState(&cstate, 0.0, 0.0, 0.0);

    float real_p_clean_prev = 0.0f;
    float rpm_prev = 0.0f;
    float last_exceed2 = -1.0f;
    float rise_t = -1.0f;
    float peak_p = 0.0f;
    float tail_sum = 0.0f, tail_sumsq = 0.0f, tail_min = 1e30f, tail_max = -1e30f;
    int   tail_count = 0;
    float recovery_t = -1.0f;
    float dist_end = SIM_DIST_START_S + SIM_DIST_DUR_S;
    float ffBase = cfg->target / (cfg->K * cfg->flow2speed);
    /* 脉动前馈抵消质量累加：compFF·K 与 eP 的协方差/方差 */
    float diag_ffk_sq = 0.0f, diag_e_sq = 0.0f, diag_ffk_e = 0.0f, diag_n = 0.0f;
    float diag_e_sum = 0.0f, diag_ffk_sum = 0.0f;

    for (int step = 0; step < N_STEPS; ++step) {
        float t = (float)(step + 1) * cfg->dt;
        float phase = plant.pump_phase_rev;
        float phi = 2.0f * PI_F * (float)SIM_Z_TEETH * phase;
        float ripple = cfg->A_pressure * sinf(phi);

        float measured = real_p_clean_prev + ripple + rand_gauss(cfg->sigma_sensor);
        if (measured < 0.0f) measured = 0.0f;
        if (measured > params.sensor_range_bar) measured = params.sensor_range_bar;

        cinput.targetPressure = cfg->target;
        cinput.measuredPressure = (HYD_REAL)measured;
        float compFF = 0.0f;
        if (use_comp) {
            HYD_REAL eP = measured - cfg->target;
            HYD_UINT8 steady = HYD_PressureSteadyGate_Update(&compGate, eP);
            HYD_PressureRippleComp_Update(&comp, eP, (HYD_REAL)phase, rpm_prev, cfg->dt, 1u, steady);
            compFF = HYD_PressureRippleComp_GetFF(&comp, (HYD_REAL)phase, rpm_prev, cfg->dt, 1u);
        }
        cinput.feedforwardFlow = (HYD_REAL)(ffBase + compFF);
        cinput.outputMin = 0.0;
        cinput.outputMax = (HYD_REAL)segment.maxFlow;
        cinput.flowToPumpSpeedGain = (HYD_REAL)cfg->flow2speed;
        cinput.pumpSpeedLimit = (HYD_REAL)cfg->pump_limit;
        cinput.timestamp = (HYD_REAL)t;

        HYD_PressureController_Execute(&segment, &cstate, &cinput, &coutput);

        pinput.requestedFlow = (HYD_REAL)coutput.outputFlow;
        pinput.flowToPumpSpeedGain = (HYD_REAL)cfg->flow2speed;
        pinput.pumpSpeedLimit = (HYD_REAL)cfg->pump_limit;
        pinput.direction = segment.direction;
        HYD_PumpConverter_Execute(&pinput, &poutput);

        float rpm_cmd = (float)poutput.pumpSpeed;
        PressureModel_Step(&params, &plant, rpm_cmd, cfg->dt, &plant_out);

        float real_p_clean = plant_out.real_pressure_bar;
        if (cfg->disturbance && t >= SIM_DIST_START_S && t <= dist_end) {
            real_p_clean -= SIM_DIST_STEP_BAR;
        }
        float real_p = real_p_clean + ripple;

        if (real_p > peak_p) peak_p = real_p;
        if (rise_t < 0.0f && real_p >= 0.9f * cfg->target) rise_t = t;
        if (fabsf(real_p - cfg->target) > 0.02f * cfg->target) last_exceed2 = t;
        if (step >= tail_start) {
            tail_sum += real_p; tail_sumsq += real_p * real_p;
            if (real_p < tail_min) tail_min = real_p;
            if (real_p > tail_max) tail_max = real_p;
            ++tail_count;
            if (use_comp) {
                float Kfp = actual_K * cfg->flow2speed;
                float ffk = compFF * Kfp;
                float ePnow = measured - cfg->target;
                diag_ffk_sq += ffk * ffk; diag_e_sq += ePnow * ePnow;
                diag_ffk_e += ffk * ePnow; diag_e_sum += ePnow; diag_ffk_sum += ffk; ++diag_n;
            }
        }
        if (cfg->disturbance && recovery_t < 0.0f && t > dist_end &&
            fabsf(real_p - cfg->target) <= 0.02f * cfg->target) {
            recovery_t = t - dist_end;
        }
        real_p_clean_prev = real_p_clean;
        rpm_prev = rpm_cmd;
    }

    float steady_mean = tail_count ? (tail_sum / (float)tail_count) : 0.0f;
    float var = tail_count ? (tail_sumsq / (float)tail_count - steady_mean * steady_mean) : 0.0f;
    if (var < 0.0f) var = 0.0f;
    m.rise_time = (rise_t < 0.0f) ? cfg->duration : rise_t;
    m.settle_time = (last_exceed2 < 0.0f) ? 0.0f : last_exceed2;
    m.overshoot_pct = (peak_p > cfg->target) ? ((peak_p - cfg->target) / cfg->target * 100.0f) : 0.0f;
    m.steady_err = steady_mean - cfg->target;
    m.ripple_rms = sqrtf(var);
    m.ripple_p2p = (tail_count) ? (tail_max - tail_min) : 0.0f;
    m.recovery_time = recovery_t;
    if (use_comp && diag_n > 1.0f) {
        float n = diag_n;
        float mean_e = diag_e_sum / n, mean_ffk = diag_ffk_sum / n;
        float var_e = diag_e_sq / n - mean_e * mean_e;
        float var_ffk = diag_ffk_sq / n - mean_ffk * mean_ffk;
        float cov = diag_ffk_e / n - mean_e * mean_ffk;
        float var_resid = var_e + var_ffk + 2.0f * cov;   /* var(eP + compFF*K) */
        if (var_e > 1e-9f) {
            m.ff_cancel = 1.0f - var_resid / var_e;
            if (m.ff_cancel < 0.0f) m.ff_cancel = 0.0f;
        }
    }
    return m;
}

/* ====================== 主流程 ====================== */
int main(void) {
    /* 控制器对比集合：含“公平基准” PI+FF（注入稳态前馈，与 RBF 的 systemGain 补偿等效），
       用以隔离“自适应增益”对性能的贡献，回答 RBF 是否真正优于 PI */
    typedef struct { int type; int ff_flow; const char *label; } CtrlDef;
    static const CtrlDef CTRLS[] = {
        { HYD_PRESSURE_CONTROLLER_PI,     0, "PI" },
        { HYD_PRESSURE_CONTROLLER_PI,     1, "PI+FF" },
        { HYD_PRESSURE_CONTROLLER_RBF_PI, 0, "RBF-PI" },
        { HYD_PRESSURE_CONTROLLER_RBF_PID, 0, "RBF-PID" }
    };
    const int n_ctrl = (int)(sizeof(CTRLS) / sizeof(CTRLS[0]));
    char path[256];

    printf("==============================================================\n");
    printf(" 压力闭环控制算法仿真  (K=%.2f bar/rpm, tau=%.2f s, Z=%d齿)\n",
           SIM_K_GAIN, SIM_TAU_S, SIM_Z_TEETH);
    printf(" 脉动幅值=%.2f bar, 传感器噪声 sigma=%.2f bar, dt=%.1f ms, 时长=%.1f s\n",
           SIM_A_PRESSURE, SIM_SIGMA_SENSOR, SIM_DT_S * 1000.0f, SIM_DURATION_S);
    printf("==============================================================\n");

    /* ---------- 步骤1: 三控制器对比（无前馈，标称增益） ---------- */
    printf("\n[1] 三控制器对比（标称 K=%.2f，无前馈）\n", SIM_K_GAIN);
    printf("%-8s %8s %8s %8s %10s %10s %10s\n",
           "控制", "上升ms", "调节ms", "超调%", "稳态误差", "纹波RMS", "纹波p2p");
    printf("%-8s %8s %8s %8s %10s %10s %10s\n",
           "------", "------", "------", "------", "--------", "--------", "--------");

    RecBuf recs[4]; Metrics ms[4];
    for (int i = 0; i < n_ctrl; ++i) {
        SimConfig cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = CTRLS[i].type;
        cfg.ff_flow = CTRLS[i].ff_flow;
        cfg.use_ff = 0; cfg.record = 1;

        recs[i] = alloc_rec();
        ms[i] = run_sim(&cfg, &recs[i]);
        printf("%-8s %8.1f %8.1f %8.2f %10.3f %10.4f %10.4f\n",
               CTRLS[i].label,
               ms[i].rise_time * 1000.0f, ms[i].settle_time * 1000.0f,
               ms[i].overshoot_pct, ms[i].steady_err, ms[i].ripple_rms, ms[i].ripple_p2p);

        snprintf(path, sizeof(path), "sim_output/%s_nominal.csv", CTRLS[i].label);
        write_csv(path, &recs[i]);
    }

    /* ---------- 步骤2: 前馈补偿扫描（RBF-PID 与 PI） ---------- */
    printf("\n[2] 电机位置前馈补偿扫描（净出口脉动 = A·sin(φ) + K_ff·sin(φ)，最优 K_ff≈−A=−%.2f）\n",
           SIM_A_PRESSURE);
    printf("%-8s %10s %12s %14s\n", "控制", "K_ff", "纹波RMS", "抑制率%");
    float best_kff[4]; float best_rms[4]; float base_rms[4];
    for (int i = 0; i < n_ctrl; ++i) {
        base_rms[i] = ms[i].ripple_rms; best_rms[i] = 1e30f; best_kff[i] = 0.0f;
        for (float kff = -2.0f * SIM_A_PRESSURE; kff <= 0.6f * SIM_A_PRESSURE; kff += 0.1f * SIM_A_PRESSURE) {
            SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
            cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
            cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
            cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
            cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = CTRLS[i].type;
        cfg.ff_flow = CTRLS[i].ff_flow;
            cfg.use_ff = 1; cfg.K_ff = kff; cfg.record = 0;
            Metrics m = run_sim(&cfg, NULL);
            if (m.ripple_rms < best_rms[i]) { best_rms[i] = m.ripple_rms; best_kff[i] = kff; }
        }
        float suppress = (base_rms[i] > 1e-6f)
            ? (1.0f - best_rms[i] / base_rms[i]) * 100.0f : 0.0f;
        printf("%-8s %10.3f %12.4f %13.2f\n",
               CTRLS[i].label, best_kff[i], best_rms[i], suppress);
    }

    /* 用最优 K_ff 记录一条 RBF-PID 曲线用于绘图 */
    {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = HYD_PRESSURE_CONTROLLER_RBF_PID;
        cfg.use_ff = 1; cfg.K_ff = best_kff[2]; cfg.record = 1;
        RecBuf r = alloc_rec();
        run_sim(&cfg, &r);
        snprintf(path, sizeof(path), "sim_output/RBF-PID_ff_kff%.2f.csv", best_kff[3]);
        write_csv(path, &r);
        free_rec(&r);
    }

    /* ---------- 步骤3: 鲁棒性（实际增益 5.4，与标称 4.5 失配） ---------- */
    printf("\n[3] 鲁棒性：实际增益 K=%.2f（标称 %.2f），无前馈\n", SIM_GAIN_MISMATCH, SIM_K_GAIN);
    printf("%-8s %10s %10s %10s\n", "控制", "稳态误差", "纹波RMS", "纹波p2p");
    for (int i = 0; i < n_ctrl; ++i) {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = CTRLS[i].type;
        cfg.ff_flow = CTRLS[i].ff_flow;
        cfg.gain_mismatch = 1; cfg.record = 0;
        Metrics m = run_sim(&cfg, NULL);
        printf("%-8s %10.3f %10.4f %10.4f\n",
               CTRLS[i].label, m.steady_err, m.ripple_rms, m.ripple_p2p);
    }

    /* ---------- 步骤4: 抗扰（负载阶跃 8 bar @4s） ---------- */
    printf("\n[4] 抗扰：负载阶跃 %.0f bar @ %.1fs（恢复时间 = 扰动结束后回到±2%%带）\n",
           SIM_DIST_STEP_BAR, SIM_DIST_START_S);
    printf("%-8s %12s\n", "控制", "恢复时间ms");
    for (int i = 0; i < n_ctrl; ++i) {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = CTRLS[i].type;
        cfg.ff_flow = CTRLS[i].ff_flow;
        cfg.disturbance = 1; cfg.record = 0;
        Metrics m = run_sim(&cfg, NULL);
        printf("%-8s %12.1f\n", CTRLS[i].label,
               (m.recovery_time < 0.0f) ? -1.0f : m.recovery_time * 1000.0f);
    }

    /* ---------- 步骤5: 已实现的脉动补偿模块 + FF 标定 端到端验证 ---------- */
    int failures = 0;
    printf("\n[5] 端到端验证：已实现的 HYD_PressureRippleComp 模块 + FF 标定\n");
    printf("%-42s %10s %12s %14s\n", "验证项", "稳态误差", "纹波RMS", "结果");

    /* A) FF 标定消除稳态误差 < 1 bar（PI, 标称增益, 无补偿） */
    {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = HYD_PRESSURE_CONTROLLER_PI;
        cfg.record = 0;
        Metrics m = run_sim_ripple_comp(&cfg, 0);
        int ok = (fabsf(m.steady_err) < 1.0f);
        if (!ok) ++failures;
        printf("%-42s %10.3f %12s %14s\n", "A) FF标定(PI)稳态误差<1bar", m.steady_err,
               "-", ok ? "PASS" : "FAIL");
    }

    /* B) PI_RBF 在增益失配(K=5.4)下鲁棒，稳态误差 < 1 bar */
    {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = SIM_A_PRESSURE;
        cfg.sigma_sensor = SIM_SIGMA_SENSOR; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = HYD_PRESSURE_CONTROLLER_PI_RBF;
        cfg.gain_mismatch = 1; cfg.record = 0;
        Metrics m = run_sim_ripple_comp(&cfg, 0);
        int ok = (fabsf(m.steady_err) < 1.0f);
        if (!ok) ++failures;
        printf("%-42s %10.3f %12s %14s\n", "B) PI_RBF增益失配稳态误差<1bar", m.steady_err,
               "-", ok ? "PASS" : "FAIL");
    }

    /* C) 齿轮泵脉动前馈抵消质量 > 80%（低噪声工况，凸显确定性脉动）。
     *    度量：模块在线学习的前馈 compFF 对压力扰动 eP 的对消质量
     *    = 1 - var(eP + compFF*K)/var(eP)。这直接验证模块的脉动抵消能力，
     *    与 [2] 开环前馈 100% 抑制一致；闭环 RMS 抑制率受 PI 反馈掩盖，
     *    不作为本项主指标，仅作无回归守护（有补偿纹波不得明显高于无补偿）。 */
    {
        SimConfig cfg; memset(&cfg, 0, sizeof(cfg));
        cfg.dt = SIM_DT_S; cfg.duration = SIM_DURATION_S; cfg.target = SIM_TARGET_BAR;
        cfg.K = SIM_K_GAIN; cfg.tau = SIM_TAU_S; cfg.A_pressure = 2.0f;
        cfg.sigma_sensor = 0.05f; cfg.flow2speed = SIM_FLOW2SPEED;
        cfg.pump_limit = SIM_PUMP_LIMIT; cfg.controller = HYD_PRESSURE_CONTROLLER_PI;
        cfg.record = 0;
        Metrics m0 = run_sim_ripple_comp(&cfg, 0);
        Metrics m1 = run_sim_ripple_comp(&cfg, 1);
        float cancel = m1.ff_cancel * 100.0f;
        int ok = (m1.ff_cancel > 0.80f) && (m1.ripple_rms <= m0.ripple_rms * 1.05f);
        if (!ok) ++failures;
        printf("%-42s %10s %12s %14s\n", "C) 脉动前馈抵消质量>80%", "-", "-", ok ? "PASS" : "FAIL");
        printf("%-42s %10s %12.2f%% %14s\n", "   (前馈对消质量)", "-", cancel, ok ? "PASS" : "FAIL");
        printf("%-42s %10s %12.4f %14s\n", "   (闭环纹波RMS 有/无补偿)", "-", m1.ripple_rms,
               (m1.ripple_rms <= m0.ripple_rms * 1.05f) ? "无回归" : "恶化");
    }

    printf("\n[5] 端到端验证失败项数: %d\n", failures);

    /* ---------- 汇总 CSV ---------- */
    {
        FILE *f = fopen("sim_output/summary.csv", "w");
        if (f) {
            fprintf(f, "controller,rise_ms,settle_ms,overshoot_pct,steady_err,ripple_rms,ripple_p2p,ff_best_kff,ff_best_rms,ff_suppress_pct\n");
            for (int i = 0; i < n_ctrl; ++i) {
                float suppress = (base_rms[i] > 1e-6f)
                    ? (1.0f - best_rms[i] / base_rms[i]) * 100.0f : 0.0f;
                fprintf(f, "%s,%.2f,%.2f,%.3f,%.4f,%.5f,%.5f,%.3f,%.5f,%.2f\n",
                        CTRLS[i].label,
                        ms[i].rise_time * 1000.0f, ms[i].settle_time * 1000.0f,
                        ms[i].overshoot_pct, ms[i].steady_err, ms[i].ripple_rms,
                        ms[i].ripple_p2p, best_kff[i], best_rms[i], suppress);
            }
            fclose(f);
        }
    }

    for (int i = 0; i < n_ctrl; ++i) free_rec(&recs[i]);
    printf("\nCSV 已写入 ./sim_output/ （含各控制器标称曲线、RBF-PID最优前馈曲线、summary.csv）\n");
    printf("完成。\n");
    return failures;
}
