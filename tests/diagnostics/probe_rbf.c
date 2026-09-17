/* out/tmp/probe_rbf.c — RBF-PID 内部机理探针（诊断用，非回归用例）
 *
 * 目的：验证一个从代码阅读中提出的具体怀疑 ——
 *   RBF 辨识出的 Jacobian 是 ∂P(k)/∂Δu(k-1)（**一步灵敏度**），
 *   而代码用**稳态增益 K**[bar/(L/min)] 给它限幅（[0.2K, 5K]）。
 *   对一阶对象：∂P(k)/∂Δu(k-1) = K·(1-exp(-dt/τ)) ≈ K·dt/τ，
 *   当 dt=1ms、τ≈1s 时 ≈ 0.001·K，比限幅下界 0.2K 小 200 倍。
 *   若成立 → 梯度被放大数百倍 → KP/KI/KD 恒撞限幅 → "自适应"退化成固定增益。
 *
 * 步骤：
 *   1) 开环测植物的 K_q[bar/(L/min)] 与 τ
 *   2) 由 K_q、τ 计算理论一步灵敏度 b = K_q·(1-exp(-dt/τ))
 *   3) 闭环阶跃，记录 Jacobian / KP / KI / KD 轨迹与"撞限幅"占比
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

#define SIM_DT   0.001f
#define D_ML     25.0f
#define RPM_MAX  1700.0f
#define ETA      0.95f
#define GAIN     (1000.0f / (D_ML * ETA))      /* 42.105 rpm/(L/min) */
#define K_NOM    (5.0f / (D_ML / 1000.0f))     /* 200 bar/(L/min) */
#define OMAX     20.0f
#define OMIN     (-5.0f)

static unsigned int ns = 12345u;
static float gauss(unsigned int *s) {
    float u1, u2;
    *s = (*s) * 1103515245u + 12345u; u1 = ((*s) >> 8) / 16777216.0f;
    *s = (*s) * 1103515245u + 12345u; u2 = ((*s) >> 8) / 16777216.0f;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static void plant_init(PressureModelParams *p, PressureModelState *s) {
    memset(p, 0, sizeof(*p));
    memset(s, 0, sizeof(*s));
    PressureModel_InitParams(p);
    PressureModel_Reset(s, 0x12345678u);
}

/* ---- 1) 开环：测 K_q 与 τ ---- */
static void measure_plant(void) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    const float rpm_test = 20.0f;          /* → Q ≈ 0.475 L/min */
    const int n = 20000;                   /* 20 s */
    float p_ss = 0.0f, t63 = -1.0f, t90 = -1.0f;
    int i;

    plant_init(&p, &s);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) {
        PressureModel_Step(&p, &s, rpm_test, SIM_DT, &o);
        if (t63 < 0 && o.measured_pressure_bar >= 0.632f * 0.0f) { }
    }
    p_ss = o.measured_pressure_bar;

    /* 重跑一次取 63.2% / 90% 时刻 */
    plant_init(&p, &s);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) {
        PressureModel_Step(&p, &s, rpm_test, SIM_DT, &o);
        if (t63 < 0 && o.measured_pressure_bar >= 0.632f * p_ss)
            t63 = (float)i * SIM_DT;
        if (t90 < 0 && o.measured_pressure_bar >= 0.90f * p_ss)
            t90 = (float)i * SIM_DT;
    }

    {
        float q_lmin = rpm_test / GAIN;
        float k_q = p_ss / q_lmin;                 /* bar/(L/min) */
        float tau  = t63;                          /* 一阶：t63 = τ */
        float b    = k_q * (1.0f - expf(-SIM_DT / tau));
        printf("=== 开环植物辨识 ===\n");
        printf("  阶跃转速        : %.1f rpm  (Q = %.4f L/min)\n", rpm_test, q_lmin);
        printf("  稳态压力 P_ss   : %.3f bar\n", p_ss);
        printf("  K_q = P_ss/Q    : %.2f bar/(L/min)   (名义 K=%.1f)\n", k_q, K_NOM);
        printf("  t63 = %.3f s  → τ ≈ %.3f s   (t90=%.3f s)\n", t63, tau, t90);
        printf("  理论一步灵敏度 b = K_q·(1-exp(-dt/τ)) = %.5f bar/(L/min)\n", b);
        printf("  代码限幅下界 0.2K = %.2f  →  倍差 = %.1f x\n",
               0.2f * K_NOM, (0.2f * K_NOM) / (b > 0 ? b : 1e-9f));
        printf("  代码限幅上界 5K   = %.1f\n\n", 5.0f * K_NOM);
    }
}

/* ---- 2) 闭环：记录内部量 ---- */
static void run_closed_loop(int seed_noise) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    RBF_PID_Handle pid;
    const float sp = 150.0f;
    const int n = 3000;
    int i;
    float filt = 0.0f;
    int kp_at_min = 0, kp_at_max = 0, ki_at_min = 0, ki_at_max = 0, kd_at_min = 0, kd_at_max = 0;
    float jac_min = 1e30f, jac_max = -1e30f, jac_sum = 0.0f;
    int jac_at_lo = 0, jac_at_hi = 0;

    plant_init(&p, &s);
    memset(&o, 0, sizeof(o));
    ns = 12345u;

    RBF_PID_Init(&pid, SIM_DT, OMAX, 1.0f);
    pid.flowToPumpSpeedGain = GAIN;
    pid.output_min_flow = OMIN;
    pid.output_max_flow = OMAX;
    RBF_PID_SetGainCompensation(&pid, K_NOM);

    printf("=== 闭环阶跃 0→150 bar（噪声=%d）内部量轨迹 ===\n", seed_noise);
    printf("  %6s %9s %9s %8s %8s %8s %11s\n", "t/ms", "P/bar", "u/Lmin", "KP", "KI", "KD", "Jacobian");
    for (i = 0; i < n; ++i) {
        float fb = o.measured_pressure_bar;
        if (seed_noise) fb += 0.4f * gauss(&ns);
        filt += 0.1f * (fb - filt);
        fb = seed_noise ? filt : fb;

        float q = RBF_PID_Update(&pid, sp, fb);
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);

        if (pid.KP <= pid.min_KP + 1e-6f) kp_at_min++;
        if (pid.KP >= pid.max_KP - 1e-6f) kp_at_max++;
        if (pid.KI <= pid.min_KI + 1e-6f) ki_at_min++;
        if (pid.KI >= pid.max_KI - 1e-6f) ki_at_max++;
        if (pid.KD <= pid.min_KD + 1e-6f) kd_at_min++;
        if (pid.KD >= pid.max_KD - 1e-6f) kd_at_max++;

        if (pid.Jacobian < jac_min) jac_min = pid.Jacobian;
        if (pid.Jacobian > jac_max) jac_max = pid.Jacobian;
        jac_sum += pid.Jacobian;
        if (fabsf(pid.Jacobian) <= 0.2f * K_NOM + 1e-3f) jac_at_lo++;
        if (fabsf(pid.Jacobian) >= 5.0f * K_NOM - 1e-3f) jac_at_hi++;

        if (i % 250 == 0 || i < 6) {
            printf("  %6d %9.3f %9.4f %8.4f %8.5f %8.5f %11.3f\n",
                   i, o.measured_pressure_bar, q, pid.KP, pid.KI, pid.KD, pid.Jacobian);
        }
    }
    printf("\n  -- 统计 --\n");
    printf("  KP 撞下限 %d/%d (%.1f%%), 撞上限 %d (%.1f%%)  [窗 %.3f..%.3f]\n",
           kp_at_min, n, 100.0f * kp_at_min / n, kp_at_max, 100.0f * kp_at_max / n,
           pid.min_KP, pid.max_KP);
    printf("  KI 撞下限 %d (%.1f%%), 撞上限 %d (%.1f%%)  [窗 %.5f..%.5f]\n",
           ki_at_min, 100.0f * ki_at_min / n, ki_at_max, 100.0f * ki_at_max / n,
           pid.min_KI, pid.max_KI);
    printf("  KD 撞下限 %d (%.1f%%), 撞上限 %d (%.1f%%)  [窗 %.5f..%.5f]\n",
           kd_at_min, 100.0f * kd_at_min / n, kd_at_max, 100.0f * kd_at_max / n,
           pid.min_KD, pid.max_KD);
    printf("  Jacobian: min %.4f  max %.4f  mean %.4f | 触下界(0.2K) %d 次, 触上界(5K) %d 次\n",
           jac_min, jac_max, jac_sum / n, jac_at_lo, jac_at_hi);
    printf("  自适应有效活动窗 = KP/KI/KD 均不在限幅上的采样占比: %.1f%%\n\n",
           100.0f * (n - kp_at_min - kp_at_max) / n);
}

int main(void) {
    measure_plant();
    run_closed_loop(0);
    run_closed_loop(1);
    return 0;
}
