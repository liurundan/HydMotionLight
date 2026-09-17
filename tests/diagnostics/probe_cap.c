/* out/tmp/probe_cap.c — 决定性对照：同样的增益，只差"软上限"
 *
 * 假设：RBF 之所以"稳定"，不是因为增益整定得当，而是因为软上限
 *       Q_cap = P_set·1.05/K ≈ 0.525 L/min 把有效环路增益压缩了约 40 倍
 *       （可用输出范围 20 L/min → 0.525 L/min），从而**掩盖**了增益本身的不稳定。
 *
 * 做法：固定使用 RBF 实际被钉住的增益（KP=0.9 / KI=0.0056，取自探针实测），
 *       唯一变量是"有没有软上限"。
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

static void run(float kp, float ki, int use_cap, float capfactor,
                float *ess, float *sig, float *urange) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    int i; const int n = 10000, w = 2000;
    float u = 0.0f, e1 = 0.0f, filt = 0.0f, sum = 0, sum2 = 0; int nsum = 0;
    float umin_seen = 1e9f, umax_seen = -1e9f;

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
        float umax = OMAX;
        if (use_cap) {
            float cap = capfactor * SP / K_NOM;
            if (cap < umax) umax = cap;
        }
        if (q > umax) q = umax;
        if (q < OMIN) q = OMIN;
        u = q - q_ff;
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);
        if (i >= n - w) {
            float d = o.measured_pressure_bar - SP;
            sum += d; sum2 += d * d; nsum++;
            if (q < umin_seen) umin_seen = q;
            if (q > umax_seen) umax_seen = q;
        }
    }
    *ess = sum / nsum;
    *sig = sqrtf(sum2 / nsum - (*ess) * (*ess));
    *urange = umax_seen - umin_seen;
}

int main(void) {
    float ess, sig, ur;
    printf("固定增益 = RBF 实测被钉住的值：KP=0.9, KI=0.0056（KP 窗上限 / KI 窗上限）\n");
    printf("保压 100 bar，传感器噪声 σ=0.4bar + 量化 0.25bar + 电机噪声/纹波\n\n");
    printf("%-28s %10s %10s %14s %s\n", "配置", "ess/bar", "sigma/bar", "输出波动幅度", "判定");
    printf("----------------------------------------------------------------------------\n");

    run(0.9f, 0.0056f, 0, 0.0f, &ess, &sig, &ur);
    printf("%-28s %10.3f %10.3f %14.3f %s\n", "无软上限（裸增益）", ess, sig, ur,
           (fabsf(ess) <= 1.0f && sig <= 1.0f) ? "达标" : "★失稳");

    run(0.9f, 0.0056f, 1, 1.05f, &ess, &sig, &ur);
    printf("%-28s %10.3f %10.3f %14.3f %s\n", "有软上限 1.05·P/K (=0.525)", ess, sig, ur,
           (fabsf(ess) <= 1.0f && sig <= 1.0f) ? "达标" : "★失稳");

    run(0.3136f, 0.00287f, 0, 0.0f, &ess, &sig, &ur);
    printf("%-28s %10.3f %10.3f %14.3f %s\n", "临界阻尼整定(无上限)", ess, sig, ur,
           (fabsf(ess) <= 1.0f && sig <= 1.0f) ? "达标" : "★失稳");

    printf("----------------------------------------------------------------------------\n");
    printf("结论：同一组 KP/KI，加上软上限后从失稳变为达标 →\n");
    printf("      \"稳定性\"来自**输出被钳位**，而非增益整定。\n");
    return 0;
}
