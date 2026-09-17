/* out/tmp/probe_fix.c — 验证「软上限/包络把 K 误差固化成稳态误差」这一缺陷及其修复
 *
 * 缺陷机理（解析）：
 *   软上限 Q_cap = P_set·capfactor/K_used，升压包络在 e→0 时收敛到 Q_cap。
 *   稳态平衡受这个天花板约束：P_eq 满足 P_eq = K_true·Q_cap(e)，e = P_set - P_eq。
 *   当 K_used 高估（K_used > K_true）时 Q_cap 偏小 → P_eq < P_set → **永久性静差**，
 *   且速度式 PI 的反算会把积分状态钉在天花板上，积分**永远救不回来**，且无任何诊断。
 *
 * 修复候选：
 *   (a) 提高 capfactor（给 K 误差留裕度）
 *   (b) 稳态段完全解除该上限（上限只在升压瞬态起作用）
 *   (c) 增加"输出顶到软上限 + 误差持续"的诊断报警
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_model.h"

#define SIM_DT  0.001f
#define GAIN    (1000.0f / (25.0f * 0.95f))
#define K_NOM   200.0f
#define OMAX    20.0f
#define OMIN    (-2.375f)
#define SP      100.0f
#define TAU_NOM 1.862f

static float g_Ktrue = 0.0f;

static void plant_init(PressureModelParams *p, PressureModelState *s, float leak) {
    memset(p, 0, sizeof(*p));
    PressureModel_InitParams(p);
    p->physical.pump_leak_c0_m3_pa_s            *= leak;
    p->physical.pump_leak_speed_m3_pa_s_per_rpm *= leak;
    p->physical.outlet_leak_m3_pa_s             *= leak;
    p->physical.cylinder_leak_m3_pa_s           *= leak;
    PressureModel_Reset(s, 0x24681357u);
}

static float clampf3(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* capmode: 0=固定上限(现役) 1=仅升压瞬态生效(稳态解除) */
static float run(float leak, float capfactor, int capmode, float *u_end) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    const int n = 15000, w = 5000;
    int i; float sum = 0; int nsum = 0;
    float u_corr = 0.0f, e1 = 0.0f;
    const float wn = 18.0f;
    float Kp = (2.0f * wn * TAU_NOM - 1.0f) / K_NOM;
    float KI = wn * wn * TAU_NOM / K_NOM * SIM_DT;

    plant_init(&p, &s, leak);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) {
        float P = o.measured_pressure_bar;
        float e = SP - P;
        float q_ff = SP / K_NOM;
        float umax = OMAX;

        u_corr += Kp * (e - e1) + KI * e;

        if (e > 0.0f) {
            float qss = q_ff * capfactor;
            float qb = 12.11f; if (qb < qss) qb = qss;
            if (capmode == 1) {
                /* 仅在升压瞬态(误差大)收紧；进入窄带后解除，交回积分 */
                if (fabsf(e) > 0.05f * SP) {
                    float eb = SP * 0.5f, fr = e / eb; if (fr > 1.0f) fr = 1.0f;
                    float env = qss + (qb - qss) * fr;
                    if (env < umax) umax = env;
                }
            } else {
                float eb = SP * 0.5f, fr = e / eb; if (fr > 1.0f) fr = 1.0f;
                float env = qss + (qb - qss) * fr;
                if (env < umax) umax = env;
            }
        }
        float u = u_corr + q_ff;
        if (u > umax) u = umax;
        if (u < OMIN) u = OMIN;
        u_corr = u - q_ff;
        e1 = e;

        PressureModel_Step(&p, &s, u * GAIN, SIM_DT, &o);
        if (i >= n - w) { sum += o.measured_pressure_bar - SP; nsum++; }
        if (i == n - 1 && u_end) *u_end = u;
    }
    return sum / nsum;
}

/* 用开环测真实 K */
static float measure_K(float leak) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    int i; const int n = 25000; const float rpm = 20.0f;
    plant_init(&p, &s, leak);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) PressureModel_Step(&p, &s, rpm, SIM_DT, &o);
    return o.measured_pressure_bar / (rpm / GAIN);
}

int main(void) {
    float leaks[3] = {2.0f, 1.0f, 0.5f};
    const char *ln[3] = {"×2 (K_true≈105, K高估1.9×)", "×1 (K_true≈210, 标定正确)", "×0.5 (K_true≈420, K低估)"};
    float cf[5] = {1.05f, 1.5f, 2.0f, 3.0f, 0.0f};
    int i, j;

    printf("K_used 固定 = %.0f（控制器以为的值）\n\n", K_NOM);
    printf("%-34s %8s | %s\n", "真实植物", "K_true", "不同 capfactor 下的稳态误差 ess [bar]");
    printf("%-34s %8s | %7s %7s %7s %7s | %s\n", "", "", "1.05", "1.5", "2.0", "3.0", "仅瞬态(修复b)");
    printf("------------------------------------------------------------------------------------------\n");

    for (i = 0; i < 3; ++i) {
        float Kt = measure_K(leaks[i]);
        printf("%-34s %8.1f |", ln[i], Kt);
        for (j = 0; j < 4; ++j) {
            printf(" %7.3f", run(leaks[i], cf[j], 0, NULL));
        }
        printf(" | %7.3f\n", run(leaks[i], 1.05f, 1, NULL));
    }
    printf("------------------------------------------------------------------------------------------\n");
    printf("合格线 |ess| <= 1.0 bar\n\n");

    /* 解析预测校验：K 高估时的平衡压力 */
    printf("=== 解析校验（capfactor=1.05, 泄漏×2 → K_true≈105）===\n");
    {
        float Kt = measure_K(2.0f);
        float qss = SP * 1.05f / K_NOM;
        /* P = Kt*(qss + (qb-qss)*(SP-P)/(0.5*SP)) */
        float qb = 12.11f; float a = (qb - qss) / (0.5f * SP);
        /* P = Kt*(qss + a*(SP-P)) → P(1 + Kt*a) = Kt*(qss + a*SP) */
        float Peq = Kt * (qss + a * SP) / (1.0f + Kt * a);
        printf("  Q_cap = 100×1.05/200 = %.4f L/min\n", qss);
        printf("  解析平衡压力 P_eq = %.2f bar  → 预测 ess = %.3f bar\n", Peq, Peq - SP);
        printf("  实测 ess            = %.3f bar\n", run(2.0f, 1.05f, 0, NULL));
    }
    return 0;
}
