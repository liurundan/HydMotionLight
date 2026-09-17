/* out/tmp/probe_cmp2.c — RBF-PID vs 工程替代方案（干净版）
 *
 * 架构澄清：所有方案都允许使用「稳态前馈 Q_ff = P_set/K」+「升压制动包络」，
 * 因为这两者是**开环成型**手段、与"用不用神经网络"正交。
 * 真正要回答的是：在同样的前馈/包络条件下，**RBF 自适应到底贡献了什么**。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

#define SIM_DT   0.001f
#define D_ML     25.0f
#define ETA_V    0.95f
#define GAIN     (1000.0f / (D_ML * ETA_V))
#define K_NOM    200.0f
#define OMAX     20.0f
#define OMIN     (-2.375f)

#define SP1     150.0f
#define SP2     100.0f

static float g_Kq  = 0.0f;
static float g_tau = 0.0f;

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

static void plant_init(PressureModelParams *p, PressureModelState *s,
                       int noise, unsigned seed, float leak_scale) {
    memset(p, 0, sizeof(*p));
    PressureModel_InitParams(p);
    p->enable_sensor_noise = noise ? 1u : 0u;
    p->enable_motor_noise  = noise ? 1u : 0u;
    if (leak_scale != 1.0f) {
        p->physical.pump_leak_c0_m3_pa_s      *= leak_scale;
        p->physical.pump_leak_speed_m3_pa_s_per_rpm *= leak_scale;
        p->physical.outlet_leak_m3_pa_s       *= leak_scale;
        p->physical.cylinder_leak_m3_pa_s     *= leak_scale;
    }
    PressureModel_Reset(s, seed);
}

static void identify(void) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    const float rpm = 20.0f; const int n = 25000;
    float pss = 0, t63 = -1; int i;
    plant_init(&p, &s, 0, 0x12345678u, 1.0f);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) PressureModel_Step(&p, &s, rpm, SIM_DT, &o);
    pss = o.measured_pressure_bar;
    plant_init(&p, &s, 0, 0x12345678u, 1.0f);
    memset(&o, 0, sizeof(o));
    for (i = 0; i < n; ++i) {
        PressureModel_Step(&p, &s, rpm, SIM_DT, &o);
        if (t63 < 0 && o.measured_pressure_bar >= 0.632f * pss) t63 = (float)i * SIM_DT;
    }
    g_Kq = pss / (rpm / GAIN);
    g_tau = t63;
}

/* ---------- 控制器配置 ---------- */
typedef struct {
    int   use_rbf;
    float kp, ki;          /* 速度式 PI 增益 */
    int   use_ff;          /* 稳态前馈 Q_ff = P_set/K */
    int   use_env;         /* 升压制动包络 */
    float boost_lim;       /* L/min, <=0 关闭 */
    float k_used;          /* 控制器以为的 K（可制造失配） */
    int   scaled_eta;
} Cfg;

typedef struct {
    /* 速度式 PI 的"修正量"状态；总输出 = u + q_ff */
    float u, e1;
} PiState;

static float clampf2(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 返回总流量指令 */
static float ctl_step(void *stv, const Cfg *c, float sp, float fb) {
    PiState *st = (PiState *)stv;
    float e = sp - fb;
    float q_ff = 0.0f;
    float du, u_total, umax = OMAX;

    if (c->use_ff && c->k_used > 0.0f) q_ff = sp / c->k_used;

    du = c->kp * (e - st->e1) + c->ki * e;
    st->u += du;

    /* 升压制动包络（仅收紧上限） */
    if (c->use_env && c->boost_lim > 0.0f && c->k_used > 0.0f && e > 0.0f) {
        float qss = sp / c->k_used;
        float qb  = c->boost_lim;
        if (qb < qss) qb = qss;
        float eb = sp * 0.5f; if (eb < 1.0f) eb = 1.0f;
        float fr = e / eb; if (fr > 1.0f) fr = 1.0f;
        float env = qss + (qb - qss) * fr;
        if (env < umax) umax = env;
    }

    u_total = st->u + q_ff;
    if (u_total > umax) u_total = umax;
    if (u_total < OMIN) u_total = OMIN;
    /* 反算：把限幅结果回写到积分状态（速度式抗饱和） */
    st->u = u_total - q_ff;
    st->e1 = e;
    return u_total;
}

/* ---------- 指标 ---------- */
typedef struct {
    float Mp, tr, ts, ess, sigma, ddev, drec;
} Metrics;

static void run(const Cfg *c, Metrics *m, float leak_scale, int dump) {
    PressureModelParams p; PressureModelState s; PressureModelOutput o;
    RBF_PID_Handle pid; PiState pi;
    int i, n, settled = -1;
    float pmax = 0, sum = 0, sum2 = 0; int nsum = 0;
    float filt = 0;

    memset(m, 0, sizeof(*m));
    m->tr = -1; m->ts = -1; m->drec = -1;

    /* ---- S1 升压 0→150（无噪声） ---- */
    plant_init(&p, &s, 0, 0x12345678u, leak_scale);
    memset(&o, 0, sizeof(o));
    if (c->use_rbf) {
        RBF_PID_Init(&pid, SIM_DT, OMAX, 1.0f);
        pid.flowToPumpSpeedGain = GAIN;
        pid.output_min_flow = OMIN; pid.output_max_flow = OMAX;
        RBF_PID_SetGainCompensation(&pid, c->k_used);
        if (c->boost_lim > 0.0f) {
            RBF_PID_SetBoostFlowLimit(&pid, c->boost_lim);
            RBF_PID_SetBoostBrakeFrac(&pid, 0.5f);
        }
        if (c->scaled_eta) {
            float k = 1.0f / 353.0f;
            RBF_PID_SetLearningRates(&pid, 0.002f, 0.002f, 0.002f,
                                     0.01f * k, 0.00025f * k, 0.00025f * k);
        }
    }
    memset(&pi, 0, sizeof(pi));
    n = 6000;
    for (i = 0; i < n; ++i) {
        float fb = o.measured_pressure_bar, q;
        if (c->use_rbf) q = RBF_PID_Update(&pid, SP1, fb);
        else            q = ctl_step(&pi, c, SP1, fb);
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);
        if (o.measured_pressure_bar > pmax) pmax = o.measured_pressure_bar;
        if (m->tr < 0 && o.measured_pressure_bar >= 0.9f * SP1) m->tr = (float)i * SIM_DT * 1000.0f;
        if (fabsf(SP1 - o.measured_pressure_bar) <= 0.02f * SP1) { if (settled < 0) settled = i; }
        else settled = -1;
        if (dump && i % 200 == 0)
            printf("      t=%4dms P=%8.3f  u=%8.4f\n", i, o.measured_pressure_bar, q);
    }
    m->Mp = (pmax - SP1) / SP1 * 100.0f; if (m->Mp < 0) m->Mp = 0;
    m->ts = (settled >= 0) ? (float)settled * SIM_DT * 1000.0f : -1.0f;

    /* ---- S2 保压 100bar（噪声+纹波） ---- */
    plant_init(&p, &s, 1, 0xABCDEF01u, leak_scale);
    memset(&o, 0, sizeof(o));
    if (c->use_rbf) {
        RBF_PID_Init(&pid, SIM_DT, OMAX, 1.0f);
        pid.flowToPumpSpeedGain = GAIN;
        pid.output_min_flow = OMIN; pid.output_max_flow = OMAX;
        RBF_PID_SetGainCompensation(&pid, c->k_used);
        if (c->boost_lim > 0.0f) {
            RBF_PID_SetBoostFlowLimit(&pid, c->boost_lim);
            RBF_PID_SetBoostBrakeFrac(&pid, 0.5f);
        }
        if (c->scaled_eta) {
            float k = 1.0f / 353.0f;
            RBF_PID_SetLearningRates(&pid, 0.002f, 0.002f, 0.002f,
                                     0.01f * k, 0.00025f * k, 0.00025f * k);
        }
    }
    memset(&pi, 0, sizeof(pi));
    ns = 999u; filt = 0; n = 10000;
    for (i = 0; i < n; ++i) {
        float fb = sensor(o.measured_pressure_bar, &ns);
        filt += 0.1f * (fb - filt); fb = filt;
        float q;
        if (c->use_rbf) q = RBF_PID_Update(&pid, SP2, fb);
        else            q = ctl_step(&pi, c, SP2, fb);
        PressureModel_Step(&p, &s, q * GAIN, SIM_DT, &o);
        if (i >= n - 2000) { float d = o.measured_pressure_bar - SP2; sum += d; sum2 += d * d; nsum++; }
    }
    m->ess = sum / nsum;
    m->sigma = sqrtf(sum2 / nsum - m->ess * m->ess);
    if (m->sigma < 0) m->sigma = 0;

    /* ---- S3 保压 100bar + 负载流量阶跃 0.15 L/min (t=3~5s) ---- */
    plant_init(&p, &s, 0, 0x13572468u, leak_scale);
    memset(&o, 0, sizeof(o));
    if (c->use_rbf) {
        RBF_PID_Init(&pid, SIM_DT, OMAX, 1.0f);
        pid.flowToPumpSpeedGain = GAIN;
        pid.output_min_flow = OMIN; pid.output_max_flow = OMAX;
        RBF_PID_SetGainCompensation(&pid, c->k_used);
        if (c->boost_lim > 0.0f) {
            RBF_PID_SetBoostFlowLimit(&pid, c->boost_lim);
            RBF_PID_SetBoostBrakeFrac(&pid, 0.5f);
        }
    }
    memset(&pi, 0, sizeof(pi));
    n = 9000;
    {
        float dev = 0, rec = -1;
        for (i = 0; i < n; ++i) {
            PressureModelInput in; float fb, q;
            fb = o.measured_pressure_bar;
            if (c->use_rbf) q = RBF_PID_Update(&pid, SP2, fb);
            else            q = ctl_step(&pi, c, SP2, fb);
            in.target_rpm = q * GAIN;
            in.dt_s = SIM_DT;
            in.load_flow_m3_s = (i >= 4000 && i < 6000) ? (0.15f / 60000.0f) : 0.0f;
            PressureModel_StepInput(&p, &s, &in, &o);
            if (i >= 4000) {
                float d = fabsf(o.measured_pressure_bar - SP2);
                if (d > dev) dev = d;
                if (i >= 6000 && d < 1.0f && rec < 0) rec = (float)(i - 6000) * SIM_DT * 1000.0f;
            }
        }
        m->ddev = dev; m->drec = rec;
    }
}

static void row(const char *name, const Cfg *c, float leak_scale, int dump) {
    Metrics m; run(c, &m, leak_scale, dump);
    int ok_mp = m.Mp <= 5.0f, ok_ess = fabsf(m.ess) <= 1.0f, ok_sig = m.sigma <= 1.0f;
    printf("%-14s %7.2f %7.0f %7.0f %9.3f %8.3f | %8.2f %8.0f | %s%s%s\n",
           name, m.Mp, m.tr, m.ts, m.ess, m.sigma, m.ddev, m.drec,
           ok_mp ? "Mp✓" : "Mp✗", ok_ess ? " ess✓" : " ess✗", ok_sig ? " σ✓" : " σ✗");
}

int main(int argc, char **argv) {
    Cfg c; Metrics m;
    int dump = (argc > 1) ? atoi(argv[1]) : -1;
    float wn;

    identify();
    printf("植物辨识: K_q = %.2f bar/(L/min),  tau = %.3f s\n", g_Kq, g_tau);
    printf("真实一步灵敏度 b = K_q(1-exp(-dt/tau)) = %.5f  |  代码限幅下界 0.2K = %.1f  → 膨胀 %.0fx\n\n",
           g_Kq * (1.0f - expf(-SIM_DT / g_tau)), 0.2f * K_NOM,
           (0.2f * K_NOM) / (g_Kq * (1.0f - expf(-SIM_DT / g_tau))));

    /* ---- PI 整定扫描：临界阻尼 ζ=1 ---- */
    printf("=== PI 整定扫描 (ζ=1, Kp=(2·wn·τ-1)/K_q, KI=wn²·τ/K_q·dt) ===\n");
    printf("%-8s %8s %8s %7s %7s %7s %8s %7s\n", "wn", "Kp", "KI", "Mp%", "tr/ms", "ts/ms", "ess", "sigma");
    {
        float best_ts = 1e9f; float best_wn = 12.0f;
        for (wn = 4.0f; wn <= 24.0f; wn += 2.0f) {
            memset(&c, 0, sizeof(c));
            c.kp = (2.0f * wn * g_tau - 1.0f) / g_Kq;
            c.ki = wn * wn * g_tau / g_Kq * SIM_DT;
            c.use_ff = 1; c.k_used = g_Kq; c.use_env = 0; c.boost_lim = 0;
            run(&c, &m, 1.0f, 0);
            printf("%-8.1f %8.4f %8.5f %7.2f %7.0f %7.0f %8.3f %7.3f%s\n",
                   wn, c.kp, c.ki, m.Mp, m.tr, m.ts, m.ess, m.sigma,
                   (m.sigma > 1.0f) ? "  ← σ 超标(纹波放大)" : "");
            /* 合格判据必须包含 σ/ess，否则会选中"升压好看但保压振荡"的整定 */
            if (m.Mp <= 5.0f && m.ts >= 0 && m.sigma <= 1.0f &&
                fabsf(m.ess) <= 1.0f && m.ts < best_ts) {
                best_ts = m.ts; best_wn = wn;
            }
        }
        printf("→ 满足 Mp<=5%% 且 σ<=1bar 且 |ess|<=1bar 的最快整定: wn = %.1f (ts=%.0fms)\n\n",
               best_wn, best_ts);
        wn = best_wn;
    }

    printf("=== 方案对照（S1 升压0→150 / S2 保压100±噪声 / S3 负载扰动0.15L/min）===\n");
    printf("%-14s %7s %7s %7s %9s %8s | %8s %8s | %s\n",
           "方案", "Mp%", "tr/ms", "ts/ms", "ess/bar", "sigma", "扰动Δbar", "恢复/ms", "达标");
    printf("------------------------------------------------------------------------------------------\n");

    /* 1. 现役 RBF（生产默认：K=200 + 推导 boost=12.11） */
    memset(&c, 0, sizeof(c));
    c.use_rbf = 1; c.k_used = K_NOM; c.boost_lim = 12.11f;
    row("RBF(生产)", &c, 1.0f, (dump == 1));

    /* 2. RBF 不带包络 */
    memset(&c, 0, sizeof(c));
    c.use_rbf = 1; c.k_used = K_NOM; c.boost_lim = 0;
    row("RBF(无包络)", &c, 1.0f, (dump == 2));

    /* 3. RBF 学习率按膨胀倍数缩放（验证"自适应已死"） */
    memset(&c, 0, sizeof(c));
    c.use_rbf = 1; c.k_used = K_NOM; c.boost_lim = 12.11f; c.scaled_eta = 1;
    row("RBF(η/353)", &c, 1.0f, (dump == 3));

    /* 4~6. 工程方案 */
    memset(&c, 0, sizeof(c));
    c.kp = (2.0f * wn * g_tau - 1.0f) / g_Kq;
    c.ki = wn * wn * g_tau / g_Kq * SIM_DT;
    c.k_used = g_Kq; c.use_ff = 0; c.use_env = 0; c.boost_lim = 0;
    row("PI(裸)", &c, 1.0f, (dump == 4));

    c.use_ff = 1;
    row("PI+FF", &c, 1.0f, (dump == 5));

    c.use_env = 1; c.boost_lim = 12.11f;
    row("PI+FF+ENV", &c, 1.0f, (dump == 6));
    printf("------------------------------------------------------------------------------------------\n");
    printf("（PI 整定 wn=%.1f, Kp=%.4f, KI=%.5f；K_used=实测 %.1f）\n\n",
           wn, c.kp, c.ki, g_Kq);

    /* ---- 鲁棒性：控制器 K 与真实 K 失配 ---- */
    printf("=== 鲁棒性：K 失配（控制器按 K=200 设计，真实 K_q 被缩放）===\n");
    printf("%-14s %7s %7s %7s %9s %8s | %8s %8s\n", "方案/真实K", "Mp%", "tr/ms", "ts/ms", "ess", "sigma", "扰动Δ", "恢复");
    {
        float scales[3] = {0.5f, 1.0f, 2.0f};
        int i;
        for (i = 0; i < 3; ++i) {
            char buf[64];
            memset(&c, 0, sizeof(c));
            c.use_rbf = 1; c.k_used = K_NOM; c.boost_lim = 12.11f;
            run(&c, &m, scales[i], 0);
            snprintf(buf, sizeof(buf), "RBF   Kx%.1f", scales[i]);
            printf("%-14s %7.2f %7.0f %7.0f %9.3f %8.3f | %8.2f %8.0f\n",
                   buf, m.Mp, m.tr, m.ts, m.ess, m.sigma, m.ddev, m.drec);

            memset(&c, 0, sizeof(c));
            c.kp = (2.0f * wn * g_tau - 1.0f) / g_Kq;
            c.ki = wn * wn * g_tau / g_Kq * SIM_DT;
            c.k_used = K_NOM; c.use_ff = 1; c.use_env = 1; c.boost_lim = 12.11f;
            run(&c, &m, scales[i], 0);
            snprintf(buf, sizeof(buf), "PI+FF+ENV Kx%.1f", scales[i]);
            printf("%-14s %7.2f %7.0f %7.0f %9.3f %8.3f | %8.2f %8.0f\n",
                   buf, m.Mp, m.tr, m.ts, m.ess, m.sigma, m.ddev, m.drec);
        }
    }
    printf("\n注：Kx0.5 = 泄漏加倍（油温升高/磨损）→ 真实 K_q 减半；Kx2.0 = 相反。\n");
    return 0;
}
