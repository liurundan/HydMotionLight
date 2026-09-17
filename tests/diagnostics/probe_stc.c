/* out/tmp/probe_stc.c — 自整定(STC) vs RBF vs 固定PI：K 漂移场景
 *
 * 关键问题：RBF 神经网络的**立身之本**就是在线适应对象变化。
 * 那就直接测：运行中让植物 K 突变为 2 倍（模拟油温升高→泄漏减小），
 * 看 RBF 是否比"固定增益 PI"表现得更好。若否 → RBF 的自适应是虚假宣传。
 *
 * STC 方案：2 参数在线辨识 P(k+1)=a·P(k)+b·Q(k) → K=b/(1-a), τ=-dt/ln(a)
 *          → 极点配置(ζ=1, 目标 wn) 自整定 PI + 稳态前馈 + 升压包络
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

#define SIM_DT  0.001f
#define GAIN    (1000.0f / (25.0f * 0.95f))
#define K_NOM   200.0f
#define OMAX    20.0f
#define OMIN    (-2.375f)
#define SP      100.0f

static void plant_init(PressureModelParams *p, PressureModelState *s,
                       int noise, unsigned seed, float leak) {
    memset(p, 0, sizeof(*p));
    PressureModel_InitParams(p);
    p->enable_sensor_noise = noise ? 1u : 0u;
    p->enable_motor_noise  = noise ? 1u : 0u;
    p->physical.pump_leak_c0_m3_pa_s           *= leak;
    p->physical.pump_leak_speed_m3_pa_s_per_rpm*= leak;
    p->physical.outlet_leak_m3_pa_s            *= leak;
    p->physical.cylinder_leak_m3_pa_s          *= leak;
    PressureModel_Reset(s, seed);
}
static void plant_set_leak(PressureModelParams *p, PressureModelState *s, float leak) {
    PressureModelParams base;
    memset(&base, 0, sizeof(base));
    PressureModel_InitParams(&base);
    p->physical.pump_leak_c0_m3_pa_s            = base.physical.pump_leak_c0_m3_pa_s * leak;
    p->physical.pump_leak_speed_m3_pa_s_per_rpm = base.physical.pump_leak_speed_m3_pa_s_per_rpm * leak;
    p->physical.outlet_leak_m3_pa_s             = base.physical.outlet_leak_m3_pa_s * leak;
    p->physical.cylinder_leak_m3_pa_s           = base.physical.cylinder_leak_m3_pa_s * leak;
    (void)s;
}

/* ---------- STC ---------- */
typedef struct {
    float a, b;
    float P_prev, Q_prev;
    float K_est, tau_est;
    float u_corr, e1;      /* 速度式 PI 修正量 */
    float wn;
    float mu;
    int   inited;
} Stc;

static void stc_init(Stc *s, float wn, float mu) {
    memset(s, 0, sizeof(*s));
    s->a = 0.99946f;  s->b = 0.113f;   /* 初值 ≈ 名义植物 */
    s->K_est = 210.0f; s->tau_est = 1.86f;
    s->wn = wn; s->mu = mu;
}

/* 第二版辨识：DC 工作点估计 K = P/Q（低通 + 稳态门控）
 * 理由：2 参数 LMS 在闭环稳态下**缺乏持续激励**而发散（见第一版结果）。
 *       但稳态工作点本身是良置的：P_ss = K·Q_ss → K_est = P/Q，
 *       只要 (a) 确实处于准稳态 (b) Q 可测 (c) 无负载流量分流。
 *       用植物实测泵流量 out.pump_flow_m3_s，避免电机动态/容积效率误差。 */
static float stc_step(Stc *s, float sp, float P, float pump_flow_lmin) {
    /* --- 1. DC 工作点辨识 --- */
    float dP = P - s->P_prev;
    if (s->inited && pump_flow_lmin > 0.02f && fabsf(dP) < 0.02f) {
        float k_inst = P / pump_flow_lmin;
        if (k_inst > 5.0f && k_inst < 5000.0f) {
            s->K_est += s->mu * (k_inst - s->K_est);   /* mu 即低通系数 */
        }
    }
    s->P_prev = P;
    s->Q_prev = pump_flow_lmin;
    s->inited = 1;
    if (s->K_est < 5.0f) s->K_est = 5.0f;
    if (s->K_est > 5000.0f) s->K_est = 5000.0f;

    /* --- 2. 极点配置自整定 (ζ=1) --- */
    float Kp = (2.0f * s->wn * s->tau_est - 1.0f) / (s->K_est > 1.0f ? s->K_est : 1.0f);
    float KI = s->wn * s->wn * s->tau_est / (s->K_est > 1.0f ? s->K_est : 1.0f) * SIM_DT;
    if (Kp < 0.0f) Kp = 0.0f;
    if (Kp > 2.0f) Kp = 2.0f;
    if (KI < 0.0f) KI = 0.0f;
    if (KI > 0.05f) KI = 0.05f;

    /* --- 3. 前馈 + 增量 PI + 包络 --- */
    float e = sp - P;
    float q_ff = sp / s->K_est;
    float du = Kp * (e - s->e1) + KI * e;
    s->u_corr += du;
    float umax = OMAX;
    if (e > 0.0f) {
        float qss = q_ff, qb = 12.11f;
        if (qb < qss) qb = qss;
        float eb = sp * 0.5f; float fr = e / eb; if (fr > 1.0f) fr = 1.0f;
        float env = qss + (qb - qss) * fr;
        if (env < umax) umax = env;
    }
    float u = s->u_corr + q_ff;
    if (u > umax) u = umax;
    if (u < OMIN) u = OMIN;
    s->u_corr = u - q_ff;
    s->e1 = e;
    return u;
}

/* ---------- 固定增益 PI+FF+ENV ---------- */
typedef struct { float u_corr, e1; } Fpi;
static float fpi_step(Fpi *s, float sp, float P, float K, float wn, float tau) {
    float Kp = (2.0f * wn * tau - 1.0f) / K;
    float KI = wn * wn * tau / K * SIM_DT;
    float e = sp - P;
    float q_ff = sp / K;
    s->u_corr += Kp * (e - s->e1) + KI * e;
    float umax = OMAX;
    if (e > 0.0f) {
        float qss = q_ff, qb = 12.11f; if (qb < qss) qb = qss;
        float eb = sp * 0.5f; float fr = e / eb; if (fr > 1.0f) fr = 1.0f;
        float env = qss + (qb - qss) * fr;
        if (env < umax) umax = env;
    }
    float u = s->u_corr + q_ff;
    if (u > umax) u = umax;
    if (u < OMIN) u = OMIN;
    s->u_corr = u - q_ff;
    s->e1 = e;
    return u;
}

typedef struct { float ess0, dev_max, ess1, K_end; } Dm;

static void scenario(const char *name, int kind, float wn, float mu,
                     float leak0, float leak1, Dm *r) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    RBF_PID_Handle pid; Stc stc; Fpi fpi;
    const int n = 12000, tdrift = 5000;
    int i; float sum = 0; int nsum = 0; float sum1 = 0; int n1 = 0;
    float dev = 0;

    plant_init(&p, &s, 0, 0x24681357u, leak0);
    memset(&o, 0, sizeof(o));
    memset(&fpi, 0, sizeof(fpi));
    stc_init(&stc, wn, mu);
    if (kind == 0) {
        RBF_PID_Init(&pid, SIM_DT, OMAX, 1.0f);
        pid.flowToPumpSpeedGain = GAIN;
        pid.output_min_flow = OMIN; pid.output_max_flow = OMAX;
        RBF_PID_SetGainCompensation(&pid, K_NOM);
        RBF_PID_SetBoostFlowLimit(&pid, 12.11f);
        RBF_PID_SetBoostBrakeFrac(&pid, 0.5f);
    }
    for (i = 0; i < n; ++i) {
        if (i == tdrift) plant_set_leak(&p, &s, leak1);
        float P = o.measured_pressure_bar, q;
        float pump_lmin = o.pump_flow_m3_s * 60000.0f;   /* m3/s → L/min */
        if (kind == 0)      q = RBF_PID_Update(&pid, SP, P);
        else if (kind == 1) q = fpi_step(&fpi, SP, P, K_NOM, wn, 1.862f);
        else                q = stc_step(&stc, SP, P, pump_lmin);
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);

        if (i >= 3000 && i < tdrift) { sum += o.measured_pressure_bar - SP; nsum++; }
        if (i >= tdrift) {
            float d = fabsf(o.measured_pressure_bar - SP);
            if (d > dev) dev = d;
            if (i >= tdrift + 4000) { sum1 += o.measured_pressure_bar - SP; n1++; }
        }
    }
    r->ess0 = sum / nsum;
    r->dev_max = dev;
    r->ess1 = sum1 / n1;
    r->K_end = (kind == 2) ? stc.K_est : K_NOM;
    printf("%-18s 漂移前ess %7.3f | 漂移后峰值偏差 %7.2f bar | 漂移后ess %7.3f | 末端辨识K %7.1f\n",
           name, r->ess0, r->dev_max, r->ess1, r->K_end);
}

int main(void) {
    Dm r;
    printf("场景: 保压 100 bar，t=5s 时泄漏系数 %s → 真实 K_q %s\n",
           "×0.5", "翻倍(210→~420)");
    printf("（模拟油温升高 / 阀磨损 → 泄漏减小 → 同样流量建更高压力）\n\n");

    printf("--- wn=18 (稳定区)，STC 用 DC 工作点估计（mu = 低通系数）---\n");
    scenario("RBF",        0, 18.0f, 0.0f,  1.0f, 0.5f, &r);
    scenario("PI+FF+ENV固定", 1, 18.0f, 0.0f,  1.0f, 0.5f, &r);
    scenario("STC mu=0.001", 2, 18.0f, 0.001f, 1.0f, 0.5f, &r);
    scenario("STC mu=0.005", 2, 18.0f, 0.005f, 1.0f, 0.5f, &r);
    scenario("STC mu=0.020", 2, 18.0f, 0.020f, 1.0f, 0.5f, &r);

    printf("\n--- 反向漂移：泄漏 ×2 → K_q 减半(210→~105) ---\n");
    scenario("RBF",        0, 18.0f, 0.0f,  1.0f, 2.0f, &r);
    scenario("PI+FF+ENV固定", 1, 18.0f, 0.0f,  1.0f, 2.0f, &r);
    scenario("STC mu=0.005", 2, 18.0f, 0.005f, 1.0f, 2.0f, &r);

    printf("\n--- STC 辨识正确性（无漂移，应收敛到真实 K≈210）---\n");
    scenario("STC mu=0.005", 2, 18.0f, 0.005f, 1.0f, 1.0f, &r);
    printf("\n真实 K_q ≈ 210 (leak=1.0), ≈ 420 (leak=0.5), ≈ 105 (leak=2.0)\n");
    return 0;
}
