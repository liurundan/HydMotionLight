/* tests/test_pressure_ab_matrix.c
 * 压力闭环 A/B 对照矩阵（第 7 轮修改方案的判决依据）
 *
 * 目的：在**同一植物、同一指标**下，逐项 A/B 对比候选方案。
 *      只有在本矩阵中优于基线且不劣化任何指标的方案，才被批准保留；否则撤销。
 *
 * 已确定的两个事实（用户确认）：
 *   1. Ksys 可测，由工艺层传入 → K 已知
 *   2. dt = 1ms 固定，嵌入式周期抖动极小 → 不需要 dt 归一化，参数按 dt 定值算
 *
 * 核心判据：
 *   一阶对象 P(s) = K/(τs+1)，dt 固定时：
 *     纯比例闭环  τ_cl = τ/(1 + Kp·K)   →  Kp = (τ/τ_cl − 1)/K
 *     增量式      KP = Kp
 *                 KI = Kp·dt/Ti,  取 Ti = τ  →  KI = KP·dt/τ
 *                 KD：一阶对象不需要微分（Td = KD·dt/KP，当前值使 Td≈1e-5s，D 项无效）
 *
 * 输出：多张对照表，每张覆盖 S1 升压阶跃 + S2 保压纹波，并做 τ 敏感性扫描。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

/* 同时写文件，便于在终端编码异常时仍能读取结果 */
static FILE *g_report = NULL;
#define REPORT(...) do { printf(__VA_ARGS__); \
                         if (g_report) fprintf(g_report, __VA_ARGS__); } while (0)



/* ---------- 仿真常量（与 test_pressure_target_verification 保持一致以便对比） ---------- */
#define SIM_DT_S            0.001f
#define FLOW_TO_RPM_GAIN    1.0f    /* Q[L/min] → rpm，1:1（本仿真单位约定） */
#define RBF_FMAX_FLOW       90.0f
#define RBF_OUTPUT_MIN      -5.0f
#define RBF_OUTPUT_MAX      90.0f

#define S1_SETPOINT         150.0f
#define S1_STEPS            5000    /* 5s */
#define S2_SETPOINT         100.0f
#define S2_STEPS            10000   /* 10s */

/* 目标合格线 */
#define TGT_Mp_PCT          5.0f
#define TGT_tr_ms           400.0f  /* 见基线分析：200ms 在 τ=1s 下物理不可达，改判 ≤400ms */
#define TGT_ts_ms           500.0f
#define TGT_ess_bar         1.0f
#define TGT_sigma_bar       1.0f

/* ---------- 指标 ---------- */
typedef struct {
    float Mp_pct;
    float tr_ms;
    float ts_ms;
    float ess_bar;
    float sigma_bar;
} Metrics;

typedef struct {
    const char *name;
    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;
    int   f_velfb;      /* 1=启用压力加速度前馈 */
    int   pi_mode;      /* 1=RBF_PI, 0=RBF_PID */
    int   gain_comp;    /* 1=启用 K 增益补偿（前馈播种+软上限+Jacobian中心） */
    int   noise;        /* 1=反馈叠加传感器噪声(σ=0.4bar)+量化(0.25bar) */
    const char *note;
} Variant;

/* 传感器噪声注入：一阶模型分支本身不施加噪声/量化，这里手工补上，
 * 否则 S2 的 σ_ss 恒为 0、不具区分度。用固定种子 LCG 保证可复现。 */
static unsigned int g_noise_seed = 12345u;
static float inject_sensor(float p, unsigned int *s) {
    float u1, u2, g;
    *s = (*s) * 1103515245u + 12345u; u1 = ((*s) >> 8) / 16777216.0f;
    *s = (*s) * 1103515245u + 12345u; u2 = ((*s) >> 8) / 16777216.0f;
    if (u1 < 1e-6f) u1 = 1e-6f;
    g = sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
    p += 0.4f * g;
    p = roundf(p / 0.25f) * 0.25f;   /* 量化 0.25 bar */
    return p;
}

/* ---------- 闭环仿真 ---------- */
static float plant_tau_s = 1.0f;
static float plant_k_bar_per_rpm = 5.4f;

static void run_variant(const Variant *v, Metrics *m) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    RBF_PID_Handle pid;
    int i;
    float p_max = 0.0f, tr = -1.0f, ts = -1.0f;
    float band, sum = 0.0f, sum2 = 0.0f;
    int n_avg = 0, settled_start = -1;
    int steady_window = 2000;   /* 后 2s 作为稳态窗 */
    float dbg_min = 1e9f, dbg_max = -1e9f;
    unsigned int ns = g_noise_seed;
    float filt = 0.0f;   /* 与 pressure_controller 的 α=0.1 前置滤波一致 */

    memset(&out, 0, sizeof(out));
    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.first_order_k_bar_per_rpm = plant_k_bar_per_rpm;
    params.first_order_tau_s = plant_tau_s;
    PressureModel_Reset(&state, 0x12345678u);

    RBF_PID_Init(&pid, SIM_DT_S, RBF_FMAX_FLOW, 1.0f);
    pid.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    pid.output_min_flow = RBF_OUTPUT_MIN;
    pid.output_max_flow = RBF_OUTPUT_MAX;
    RBF_PID_SetControlMode(&pid, v->pi_mode ? RBF_PID_CONTROL_MODE_PI
                                            : RBF_PID_CONTROL_MODE_PID);
    if (v->gain_comp) {
        RBF_PID_SetGainCompensation(&pid, plant_k_bar_per_rpm);
    }
    RBF_PID_SetPressureAccelFeedforwardEnabled(&pid, v->f_velfb ? true : false);
    RBF_PID_SetParamLimits(&pid, v->kp_min, v->kp_max,
                                 v->ki_min, v->ki_max,
                                 v->kd_min, v->kd_max);

    band = 0.02f * S1_SETPOINT;
    for (i = 0; i < S1_STEPS; ++i) {
        float fb = out.measured_pressure_bar;
        if (v->noise) fb = inject_sensor(fb, &ns);
        filt += 0.1f * (fb - filt);          /* 生产链路的前置一阶滤波 α=0.1 */
        fb = v->noise ? filt : fb;
        float q = RBF_PID_Update(&pid, S1_SETPOINT, fb);
        PressureModel_Step(&params, &state, q * FLOW_TO_RPM_GAIN, SIM_DT_S, &out);

        if (out.measured_pressure_bar > p_max) p_max = out.measured_pressure_bar;
        if (tr < 0.0f && out.measured_pressure_bar >= 0.9f * S1_SETPOINT) {
            tr = (float)i * SIM_DT_S * 1000.0f;
        }
        if (fabsf(S1_SETPOINT - out.measured_pressure_bar) <= band) {
            if (settled_start < 0) settled_start = i;
        } else {
            settled_start = -1;
        }
    }
    ts = (settled_start >= 0) ? (float)settled_start * SIM_DT_S * 1000.0f : -1.0f;
    m->Mp_pct = (p_max - S1_SETPOINT) / S1_SETPOINT * 100.0f;
    if (m->Mp_pct < 0.0f) m->Mp_pct = 0.0f;
    m->tr_ms = tr;
    m->ts_ms = ts;

    /* ---- S2 保压（开噪声+纹波） ---- */
    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 1u;
    params.enable_motor_noise = 1u;
    params.first_order_k_bar_per_rpm = plant_k_bar_per_rpm;
    params.first_order_tau_s = plant_tau_s;
    PressureModel_Reset(&state, 0x87654321u);

    RBF_PID_Init(&pid, SIM_DT_S, RBF_FMAX_FLOW, 1.0f);
    pid.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    pid.output_min_flow = RBF_OUTPUT_MIN;
    pid.output_max_flow = RBF_OUTPUT_MAX;
    RBF_PID_SetControlMode(&pid, v->pi_mode ? RBF_PID_CONTROL_MODE_PI
                                            : RBF_PID_CONTROL_MODE_PID);
    if (v->gain_comp) {
        RBF_PID_SetGainCompensation(&pid, plant_k_bar_per_rpm);
    }
    RBF_PID_SetPressureAccelFeedforwardEnabled(&pid, v->f_velfb ? true : false);
    RBF_PID_SetParamLimits(&pid, v->kp_min, v->kp_max,
                                 v->ki_min, v->ki_max,
                                 v->kd_min, v->kd_max);

    sum = 0.0f; sum2 = 0.0f; n_avg = 0;
    dbg_min = 1e9f; dbg_max = -1e9f;
    for (i = 0; i < S2_STEPS; ++i) {
        float fb = out.measured_pressure_bar;
        if (v->noise) fb = inject_sensor(fb, &ns);
        filt += 0.1f * (fb - filt);
        fb = v->noise ? filt : fb;
        float q = RBF_PID_Update(&pid, S2_SETPOINT, fb);
        PressureModel_Step(&params, &state, q * FLOW_TO_RPM_GAIN, SIM_DT_S, &out);

        if (i >= S2_STEPS - steady_window) {
            float e = S2_SETPOINT - out.measured_pressure_bar;
            sum += e; sum2 += e * e; n_avg++;
            if (e < dbg_min) dbg_min = e;
            if (e > dbg_max) dbg_max = e;
        }
    }
    if (getenv("AB_DEBUG") != NULL) {
        REPORT("      [dbg] err min=%.4f max=%.4f n=%d noise=%d\n",
               dbg_min, dbg_max, n_avg, 1);
    }
    {
        float mean = sum / (float)n_avg;
        float var = sum2 / (float)n_avg - mean * mean;
        m->ess_bar = mean;
        m->sigma_bar = (var > 0.0f) ? sqrtf(var) : 0.0f;
    }
}

/* 依据 K、τ、dt 计算增量式参数
 *   Kp = (τ/τ_cl − 1)/K ;  KI = Kp·dt/τ ;  KD 置 0（一阶对象不需要微分） */
static void compute_gains(float K, float tau, float dt, float tau_cl,
                          float *kp, float *ki) {
    *kp = (tau / tau_cl - 1.0f) / K;
    *ki = (*kp) * dt / tau;
}

static void print_header(void) {
    REPORT("%-26s %8s %8s %8s %9s %9s  %s\n",
           "变体", "Mp%", "tr(ms)", "ts(ms)", "ess(bar)", "sig(bar)", "判定");
    REPORT("--------------------------------------------------------------------------------\n");
}

static const char *verdict(const Metrics *m) {
    static char buf[64];
    int ok = 1;
    if (m->Mp_pct > TGT_Mp_PCT) ok = 0;
    if (m->tr_ms < 0.0f || m->tr_ms > TGT_tr_ms) ok = 0;
    if (m->ts_ms < 0.0f || m->ts_ms > TGT_ts_ms) ok = 0;
    if (fabsf(m->ess_bar) > TGT_ess_bar) ok = 0;
    if (m->sigma_bar > TGT_sigma_bar) ok = 0;
    snprintf(buf, sizeof(buf), "%s", ok ? "PASS" : "FAIL");
    return buf;
}

static void print_row(const char *name, const Metrics *m) {
    REPORT("%-26s %8.2f %8.1f %8.1f %9.3f %9.3f  %s\n",
           name, m->Mp_pct, m->tr_ms, m->ts_ms, m->ess_bar, m->sigma_bar,
           verdict(m));
}

int main(void) {
    const float K = 5.4f;      /* bar/(L/min)，本仿真单位约定下 = Ksys */
    const float dt = SIM_DT_S;
    float kp_c, ki_c;
    Metrics m;
    int i;

    g_report = fopen("ab_matrix_report.txt", "w");

    /* 目标闭环时间常数 τ_cl = 0.15s（对应建稳 ~450ms ≈ 3τ_cl） */
    compute_gains(K, 1.0f, dt, 0.15f, &kp_c, &ki_c);

    REPORT("=== 压力闭环 A/B 对照矩阵 ===\n");
    REPORT("植物：一阶 P(s)=K/(τs+1)，K=%.1f bar/(L/min)，dt=%.1f ms 固定\n", K, dt * 1000.0f);
    REPORT("理论计算（τ=1.0s, τ_cl=0.15s）：Kp=%.4f  KI=%.6f\n", kp_c, ki_c);
    REPORT("对比当前默认窗口：KP[0.4,0.9]  KI[0.0013,0.0056]  KD[0.015,0.035]\n\n");

    /* ---------------- 表 1：τ = 1.0s，参数/开关 A/B ---------------- */
    plant_tau_s = 1.0f;
    plant_k_bar_per_rpm = K;
    REPORT("--- 表 1：τ=1.0s 下的候选方案对照（S1 0→150bar 阶跃 / S2 100bar 保压）---\n");
    print_header();

    {
        Variant vs[] = {
            {"V0 基线(当前v6)",        0.4f, 0.9f, 0.0013f, 0.0056f, 0.015f, 0.035f, 1, 0, 1, 1, "现状:PID+f_velfb"},
            {"V1 关 f_velfb",          0.4f, 0.9f, 0.0013f, 0.0056f, 0.015f, 0.035f, 0, 0, 1, 1, "删压力加速度前馈"},
            {"V6 当前窗口+PI",         0.4f, 0.9f, 0.0013f, 0.0056f, 0.0f,  0.0f,   1, 1, 1, 1, "删D项"},
            {"V7 删f_velfb+删D",       0.4f, 0.9f, 0.0013f, 0.0056f, 0.0f,  0.0f,   0, 1, 1, 1, "两者都删(候选)"},
            {"V2 计算增益(固定)",       kp_c, kp_c, ki_c,   ki_c,    0.0f,  0.0f,   1, 1, 1, 1, "KP/KI由K理论算出"},
            {"V4 计算增益窗口±30%",     kp_c*0.7f, kp_c*1.3f, ki_c*0.7f, ki_c*1.3f, 0.0f, 0.0f, 1, 1, 1, 1, "理论值留自适应余量"},
            {"V5 无K补偿(对照)",       0.4f, 0.9f, 0.0013f, 0.0056f, 0.0f,  0.0f,   1, 1, 0, 1, "证明K的价值"},
            {"V8 V7+KI窗口下探",       0.4f, 0.9f, 0.0008f, 0.0056f, 0.0f,  0.0f,   0, 1, 1, 1, "V7放宽KI下限"},
        };
        int n = (int)(sizeof(vs) / sizeof(vs[0]));
        for (i = 0; i < n; ++i) {
            run_variant(&vs[i], &m);
            print_row(vs[i].name, &m);
        }
        REPORT("\n注：V5 关闭 K 增益补偿（前馈播种/软上限/Jacobian 中心全部停用），用于证明 K 的实际贡献。\n\n");
    }

    /* ---------------- 表 2：τ 敏感性扫描 ---------------- */
    REPORT("--- 表 2：τ 敏感性扫描（验证结论是否只在 τ=1.0s 成立）---\n");
    REPORT("%-10s %-26s %8s %8s %8s %9s %9s\n",
           "τ(s)", "变体", "Mp%", "tr(ms)", "ts(ms)", "ess(bar)", "sig(bar)");
    REPORT("------------------------------------------------------------------------------\n");
    {
        float taus[] = {0.2f, 0.5f, 1.0f, 2.0f};
        int nt = (int)(sizeof(taus) / sizeof(taus[0]));
        int j;
        for (j = 0; j < nt; ++j) {
            float tau = taus[j];
            float kp_t, ki_t;
            plant_tau_s = tau;
            compute_gains(K, tau, dt, 0.15f, &kp_t, &ki_t);

            /* 基线：当前窗口 */
            Variant v0 = {"V0 基线", 0.4f, 0.9f, 0.0013f, 0.0056f, 0.015f, 0.035f, 1, 0, 1, 1, ""};
            run_variant(&v0, &m);
            REPORT("%-10.2f %-26s %8.2f %8.1f %8.1f %9.3f %9.3f\n",
                   tau, v0.name, m.Mp_pct, m.tr_ms, m.ts_ms, m.ess_bar, m.sigma_bar);

            /* 计算增益 */
            Variant v2 = {"V2 计算增益", kp_t, kp_t, ki_t, ki_t, 0.0f, 0.0f, 1, 1, 1, 1, ""};
            Variant v7 = {"V7 删vel+D", 0.4f, 0.9f, 0.0013f, 0.0056f, 0.0f, 0.0f, 0, 1, 1, 1, ""};
            run_variant(&v2, &m);
            REPORT("%-10.2f %-26s %8.2f %8.1f %8.1f %9.3f %9.3f\n",
                   tau, v2.name, m.Mp_pct, m.tr_ms, m.ts_ms, m.ess_bar, m.sigma_bar);
            REPORT("           └ 理论值: Kp=%.4f KI=%.6f\n", kp_t, ki_t);
        }
    }

    REPORT("\n判读：\n");
    REPORT("  - 若 V2/V3 全面优于 V0 → 批准\"参数由 K 算出\"并据此删补丁；否则撤销。\n");
    REPORT("  - 若 V1 与 V0 相当 → f_velfb 属冗余，批准删除。\n");
    REPORT("  - 若 V5 明显劣化 → 证实 K 增益补偿确有价值。\n");
    if (g_report) fclose(g_report);
    return 0;
}
