/* out/tmp/probe_gain.c — 检验「RBF 增益限幅窗口是否落在稳定区内」
 *
 * rbf_pid.h 硬编码窗口：KP∈[0.4,0.9]  KI∈[0.0013,0.0056]  KD∈[0.015,0.035]
 * 这些常数来自参考实现（ST 代码），从未针对本植物整定。
 * 本探针在同一植物上做 KP×KI 网格扫描，看窗口落在哪里。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_model.h"

#define SIM_DT 0.001f
#define GAIN   (1000.0f / (25.0f * 0.95f))
#define K_NOM  200.0f
#define OMAX   20.0f
#define OMIN   (-2.375f)
#define SP     100.0f

static unsigned int ns = 12345u;
static float gauss(unsigned int *s) {
    float u1, u2;
    *s = (*s) * 1103515245u + 12345u; u1 = ((*s) >> 8) / 16777216.0f;
    *s = (*s) * 1103515245u + 12345u; u2 = ((*s) >> 8) / 16777216.0f;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}
static float sensor(float p, unsigned int *s) {
    p += 0.4f * gauss(s);
    return roundf(p / 0.25f) * 0.25f;
}

/* 无包络、无软上限的"裸"速度式 PI + 稳态前馈，只看增益本身的稳定性 */
static void metrics(float kp, float ki, float *ess, float *sig) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    int i; const int n = 10000, w = 2000;
    float u = 0.0f, e1 = 0.0f, filt = 0.0f, sum = 0, sum2 = 0; int nsum = 0;

    memset(&p, 0, sizeof(p)); PressureModel_InitParams(&p);
    p.enable_sensor_noise = 1u; p.enable_motor_noise = 1u;
    PressureModel_Reset(&s, 0xABCDEF01u);
    memset(&o, 0, sizeof(o));
    ns = 999u;
    for (i = 0; i < n; ++i) {
        float fb = sensor(o.measured_pressure_bar, &ns);
        filt += 0.1f * (fb - filt); fb = filt;
        float e = SP - fb;
        float q_ff = SP / K_NOM;
        u += kp * (e - e1) + ki * e;
        e1 = e;
        float q = u + q_ff;
        if (q > OMAX) q = OMAX;
        if (q < OMIN) q = OMIN;
        u = q - q_ff;                     /* 反算抗饱和 */
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);
        if (i >= n - w) { float d = o.measured_pressure_bar - SP; sum += d; sum2 += d * d; nsum++; }
    }
    *ess = sum / nsum;
    *sig = sqrtf(sum2 / nsum - (*ess) * (*ess));
}

int main(void) {
    float kps[6] = {0.20f, 0.3136f, 0.40f, 0.60f, 0.80f, 0.90f};
    float kis[5] = {0.00127f, 0.00287f, 0.0056f, 0.010f, 0.020f};
    int i, j;

    printf("裸速度式 PI + 稳态前馈（无包络/无软上限），保压 100 bar 带噪声+纹波\n");
    printf("KP 窗 [0.4,0.9]，KI 窗 [0.0013,0.0056]（rbf_pid.h 硬编码）\n\n");
    printf("%8s | %s\n", "KP\\KI", "");
    printf("%8s |", "KP");
    for (j = 0; j < 5; ++j) printf(" %14.5f", kis[j]);
    printf("\n---------|");
    for (j = 0; j < 5; ++j) printf("---------------");
    printf("\n");

    for (i = 0; i < 6; ++i) {
        printf("%8.4f |", kps[i]);
        for (j = 0; j < 5; ++j) {
            float ess, sig;
            metrics(kps[i], kis[j], &ess, &sig);
            printf(" %6.2f/%6.2f", ess, sig);
        }
        printf("\n");
    }
    printf("\n单元格格式: ess / sigma  (bar)。合格: |ess|<=1.0 且 sigma<=1.0\n");
    printf("KP=0.3136 为 ζ=1/wn=18 的临界阻尼整定值（本植物的最快稳定整定）\n");
    return 0;
}
