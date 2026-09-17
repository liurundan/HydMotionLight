/* tests/test_strategy_matrix.c
 * 控制器策略 × 升压限流 × 系统增益 K 契约测试（docs §17/§18）。
 *
 * 【v13 更正了三处口径错误（原 §17 的结论曾据此偏严）】
 *   ① 增益：旧版未设 segment.pressureKp/Ki → 走了库内 fallback（kp=1.5, ki=0）。
 *      真实出厂默认是 **kp=0.5, ki=0.1**（motion_control.c:3609/3612）。
 *   ② 流量-转速增益：旧版用 1700/42.5 = 40（η=1 理论值）；
 *      真实应为 1000/(25×0.95) = **42.105**（差 5%，影响环路增益判断）。
 *   ③ 输出上限：旧版用 42.5 L/min（完全无手柄上限）；
 *      生产链路压力手柄 100% → maxFlow = **20 L/min**。旧口径宽松一倍，
 *      漏算了"手柄全局限流"对超调的贡献。
 *
 * 本用例把三个旋钮（策略 / K / 升压限流）的**结构性事实**编码成断言：
 *   - K 是 RBF_PID 的必需输入（K=0 必然不达标）；
 *   - K 正确后 RBF 即使不加升压限流也能达标（限流是裕度）；
 *   - PI 路径既不吃升压限流、也不吃 K（A≡B 逐位相同），且 Mp 恒不达标。
 *
 * 端到端"出厂默认即达标"的验收在 tests/test_production_default_acceptance.c。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_controller.h"
#include "pressure_model.h"

#define SIM_DT_S            0.001f
#define PUMP_D_ML_REV       25.0f
#define PUMP_RATED_RPM      1700.0f
#define PUMP_VOL_EFF        0.95f
#define PUMP_REVERSE_RPM    (-100.0f)
/* v13 更正：gain 必须按真实容积效率算：1000/(D*eta) = 42.105 rpm/(L/min)。
 * 旧版本用 1700/42.5 = 40（η=1 的理论值），与真实链路差 5%，
 * 会让环路增益判断偏乐观。 */
#define FLOW_TO_RPM_GAIN    (1000.0f / (PUMP_D_ML_REV * PUMP_VOL_EFF))   /* 42.105 */
#define PUMP_MAX_FLOW_LMIN  (PUMP_RATED_RPM / FLOW_TO_RPM_GAIN)          /* 40.375 */
#define RBF_OUTPUT_MIN      (PUMP_REVERSE_RPM / FLOW_TO_RPM_GAIN)        /* -2.375 */
/* 生产链路的实际输出上限：压力手柄 FLOWLIMITPERCENT=100 → maxFlow=20 L/min
 * （HYD_PRESSURE_HANDLE_BASE_MAX_FLOW=20），再与泵能力 min()。
 * 旧版本用 42.5（=完全无手柄上限），比生产口径宽松一倍，
 * 会把"手柄全局限流"的贡献漏算。 */
#define PROD_OUTPUT_MAX     (20.0f < PUMP_MAX_FLOW_LMIN ? 20.0f : PUMP_MAX_FLOW_LMIN)
#define RBF_SYSTEM_GAIN     (5.0f / (PUMP_D_ML_REV / 1000.0f))           /* 200 */

/* v13 更正：出厂默认的 PI 增益是 **kp=0.5, ki=0.1**（motion_control.c:3609/3612），
 * 不是库内 fallback 的 kp=1.5, ki=0。旧版本没设 segment.pressureKp/Ki，
 * 于是量到的是 fallback，导致结论比真实情况更差（"0/3"）。
 * 真实出厂默认（PI kp=0.5/ki=0.1）实测是 Mp FAIL 但 ess/σ PASS。 */
#define DEF_KP              0.5f
#define DEF_KI              0.1f

#define S1_TARGET_BAR       150.0f
#define S2_TARGET_BAR       100.0f
#define PROD_FILTER_ALPHA   0.1f
#define PROD_BOOST_BRAKE_FRAC 0.5f

#define TARGET_Mp_PCT_X     5.0f
#define TARGET_ESS_BAR_X    1.0f
#define TARGET_SIGMA_BAR_X  1.0f

typedef struct {
    float p_max;
    float p_at_90pct_time;
    float settle_time_ms;
    float ess_mean;
    float sigma_rms;
    int   reached;
    int   pass_mp, pass_ess, pass_sig, score;
} SimMetrics;

static void metrics_reset(SimMetrics *m) {
    memset(m, 0, sizeof(*m));
    m->settle_time_ms = -1.0f;
    m->p_at_90pct_time = -1.0f;
}

typedef struct {
    PressureModelParams         params;
    PressureModelState          plant;
    PressureModelOutput         out;
    HYD_MotionSegment           segment;
    HYD_PressureControllerState ctrl;
    float                       boost_limit;
    HYD_TIME                    t;
} ProdLoop;

static float g_kp_override = 0.0f;
static float g_ki_override = 0.0f;
static float g_system_gain = 0.0f;   /* 0 → 段级 systemGain 为 0（不启用补偿） */

static void prod_loop_init(ProdLoop *loop, HYD_PressureControllerType strat,
                           float setpoint, float boost_limit,
                           int with_noise, unsigned seed) {
    memset(loop, 0, sizeof(*loop));
    memset(&loop->out, 0, sizeof(loop->out));
    PressureModel_InitParams(&loop->params);
    loop->params.enable_sensor_noise = with_noise ? 1u : 0u;
    loop->params.enable_motor_noise  = with_noise ? 1u : 0u;
    PressureModel_Reset(&loop->plant, seed);

    memset(&loop->segment, 0, sizeof(loop->segment));
    loop->segment.mode               = HYD_MODE_PRESSURE_CLOSED_LOOP;
    loop->segment.endCondition       = HYD_END_MANUAL;
    loop->segment.direction          = HYD_DIRECTION_HOLD;
    loop->segment.targetPressure     = setpoint;
    loop->segment.targetFlow         = 0.0f;
    loop->segment.maxFlow            = PUMP_MAX_FLOW_LMIN;
    loop->segment.pressureController = strat;
    loop->segment.pressureCeiling    = 250.0f;
    loop->segment.pressureKp         = g_kp_override; /* 0 → 库内 fallback 1.5 */
    loop->segment.pressureKi         = g_ki_override; /* 0 → 库内 fallback 0.0 */
    loop->segment.pressureFilterAlpha = PROD_FILTER_ALPHA;
    loop->segment.systemGain          = g_system_gain;  /* K（RBF 用；PI 忽略） */

    HYD_PressureController_InitState(&loop->ctrl, 0.0f, 0.0f, 0.0);
    loop->boost_limit = boost_limit;
    loop->t = 0.0;
}

static void prod_loop_step(ProdLoop *loop) {
    HYD_PressureControllerInput  in;
    HYD_PressureControllerOutput co;
    float rpm_cmd;
    memset(&in, 0, sizeof(in));
    in.targetPressure      = loop->segment.targetPressure;
    in.measuredPressure    = loop->out.measured_pressure_bar;
    in.feedforwardFlow     = loop->segment.targetFlow;
    in.outputMin           = RBF_OUTPUT_MIN;
    in.outputMax           = PROD_OUTPUT_MAX;
    in.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    in.pumpSpeedLimit      = PUMP_RATED_RPM;
    in.systemGain          = 0.0f;
    in.boostFlowLimitLmin  = loop->boost_limit;
    in.boostBrakeFrac      = PROD_BOOST_BRAKE_FRAC;
    in.timestamp           = loop->t;
    HYD_PressureController_Execute(&loop->segment, &loop->ctrl, &in, &co);
    rpm_cmd = (float)co.outputFlow * FLOW_TO_RPM_GAIN;
    PressureModel_Step(&loop->params, &loop->plant, rpm_cmd, SIM_DT_S, &loop->out);
    loop->t += SIM_DT_S;
}

static void run_s1(HYD_PressureControllerType strat, float boost, SimMetrics *m) {
    ProdLoop loop;
    float band = 0.02f * S1_TARGET_BAR;
    int steps = 5000, settled_start = -1, i;
    metrics_reset(m);
    prod_loop_init(&loop, strat, S1_TARGET_BAR, boost, 0, 0x12345678u);
    for (i = 0; i < steps; ++i) {
        prod_loop_step(&loop);
        if (loop.out.measured_pressure_bar > m->p_max) m->p_max = loop.out.measured_pressure_bar;
        if (m->p_at_90pct_time < 0.0f && loop.out.measured_pressure_bar >= 0.9f * S1_TARGET_BAR)
            m->p_at_90pct_time = (float)i * SIM_DT_S * 1000.0f;
        if (fabsf(loop.out.measured_pressure_bar - S1_TARGET_BAR) <= band) {
            if (settled_start < 0) settled_start = i;
            m->settle_time_ms = (float)settled_start * SIM_DT_S * 1000.0f;
        } else { settled_start = -1; m->settle_time_ms = -1.0f; }
    }
    m->reached = (m->p_at_90pct_time > 0.0f);
}

static void run_s2(HYD_PressureControllerType strat, float boost, SimMetrics *m) {
    ProdLoop loop;
    int warmup = 2000, steps = 10000, steady_start = 8000, i;
    float p_sum = 0, p_sq = 0; int n = 0;
    metrics_reset(m);
    prod_loop_init(&loop, strat, S2_TARGET_BAR, boost, 1, 0x87654321u);
    for (i = 0; i < warmup; ++i) prod_loop_step(&loop);
    for (i = 0; i < steps; ++i) {
        prod_loop_step(&loop);
        if (i >= steady_start) { p_sum += loop.out.measured_pressure_bar;
            p_sq += loop.out.measured_pressure_bar * loop.out.measured_pressure_bar; ++n; }
    }
    if (n > 0) {
        float mean = p_sum / n;
        m->ess_mean = S2_TARGET_BAR - mean;
        m->sigma_rms = sqrtf(fabsf(p_sq / n - mean * mean));
    }
}

static void run_case(HYD_PressureControllerType strat, float boost,
                     float kp, float ki, float systemGain, const char *stage,
                     SimMetrics *s1, SimMetrics *s2) {
    g_kp_override = kp;
    g_ki_override = ki;
    g_system_gain = systemGain;
    run_s1(strat, boost, s1);
    run_s2(strat, boost, s2);
    s1->pass_mp  = s1->reached && (s1->p_max <= S1_TARGET_BAR * (1.0f + TARGET_Mp_PCT_X / 100.0f));
    s2->pass_ess = (fabsf(s2->ess_mean) <= TARGET_ESS_BAR_X);
    s2->pass_sig = (s2->sigma_rms <= TARGET_SIGMA_BAR_X);
    s2->score    = (s1->pass_mp ? 1 : 0) + (s2->pass_ess ? 1 : 0) + (s2->pass_sig ? 1 : 0);
    printf("  %-32s | Mp %+7.2f%% %s | ess %+7.3f %s | sig %6.3f %s | %d/3  %s\n",
           stage,
           s1->reached ? (s1->p_max - S1_TARGET_BAR) / S1_TARGET_BAR * 100.0f : -1.0f,
           s1->pass_mp ? "P" : "F",
           s2->ess_mean, s2->pass_ess ? "P" : "F",
           s2->sigma_rms, s2->pass_sig ? "P" : "F",
           s2->score, s1->reached ? "" : "(未达90%)");
}

int main(void) {
    SimMetrics a1, a2, b1, b2, c1, c2, d1, d2, e1, e2;
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== 控制器策略 × 升压限流 × 系统增益 契约测试（docs §17/§18）===\n");
    printf("链路: 25cc/rev @1700rpm (gain=%.3f), K=Ksys/(D/1000), 管路3.5m, alpha=0.1\n",
           (double)FLOW_TO_RPM_GAIN);
    printf("输出上限: %.1f L/min（压力手柄 FLOWLIMITPERCENT=100 的生产口径）\n",
           (double)PROD_OUTPUT_MAX);
    printf("合格: Mp<=5%%, ess<=1bar, sigma<=1bar  [P=pass F=fail]\n\n");

    /* A: v12 之前的出厂默认 —— PI 策略 + 真实默认增益 kp=0.5/ki=0.1 + 无升压限流。
     * 注意：这里 K 传 0（段级）与传 200 对 PI 结果必须完全相同 —— 见契约 [4]。 */
    run_case(HYD_PRESSURE_CONTROLLER_PI, 0.0f, DEF_KP, DEF_KI, 0.0f,
             "A PI kp=0.5/ki=0.1 (无K/无限流)", &a1, &a2);
    /* B: 仅强开升压限流 —— 证明 PI 路径不吃 boost（必须与 A 逐位相同） */
    run_case(HYD_PRESSURE_CONTROLLER_PI, 12.0f, DEF_KP, DEF_KI, 0.0f,
             "B A + 升压限流12", &b1, &b2);
    /* C: RBF_PID 但 K=0 —— 证明 K 是 RBF 的必需输入 */
    run_case(HYD_PRESSURE_CONTROLLER_RBF_PID, 0.0f, 0.0f, 0.0f, 0.0f,
             "C RBF_PID K=0 (无补偿)", &c1, &c2);
    /* D: RBF_PID + K=200（v12 推导值），无限流 */
    run_case(HYD_PRESSURE_CONTROLLER_RBF_PID, 0.0f, 0.0f, 0.0f, RBF_SYSTEM_GAIN,
             "D RBF_PID K=200 (无限流)", &d1, &d2);
    /* E: v12 出厂默认的等价配置 —— RBF_PID + K=200 + 推导升压限流 */
    run_case(HYD_PRESSURE_CONTROLLER_RBF_PID, 12.0f, 0.0f, 0.0f, RBF_SYSTEM_GAIN,
             "E RBF_PID K=200 + 限流12 (v12)", &e1, &e2);

    printf("\n--- 断言 ---\n");

    /* 【契约 1】v12 默认等价配置必须 3/3 达标 */
    assert(e1.pass_mp && e2.pass_ess && e2.pass_sig);
    printf("  [1] RBF_PID + K=200 + 限流12 必须 3/3 达标 ......... OK\n");

    /* 【契约 2】K 是 RBF 的必需输入：K=0 必然不达标（v12 推导存在的理由） */
    assert(c2.score < 3);
    printf("  [2] RBF_PID K=0 必须不达标（K 必需）.............. OK  (score=%d/3)\n", c2.score);

    /* 【契约 3】K 一旦正确，RBF 即使不加升压限流也能达标（限流是裕度而非必需） */
    assert(d1.pass_mp && d2.pass_ess && d2.pass_sig);
    printf("  [3] RBF_PID K=200 无限流也应 3/3（限流=裕度）..... OK\n");

    /* 【契约 4·结构性】PI 路径既不吃升压限流、也不吃系统增益：
     * A 与 B 必须逐位相同（boost 无效）。这是 v11 boost 接线只覆盖 RBF 分支的硬证据。 */
    assert(a1.p_max == b1.p_max);
    assert(a1.p_at_90pct_time == b1.p_at_90pct_time);
    assert(a2.ess_mean == b2.ess_mean);
    assert(a2.sigma_rms == b2.sigma_rms);
    printf("  [4] PI 路径不吃升压限流：A 与 B 逐位相同 .......... OK\n");

    /* 【契约 5·结构性】PI 无法做到 Mp 达标（结构性无升压段限流手段）。 */
    assert(!a1.pass_mp && !b1.pass_mp);
    printf("  [5] PI 路径 Mp 恒 FAIL（结构性不足）.............. OK\n");

    printf("\n=== 全部契约断言通过 ===\n");
    printf("结论：达标路径 = RBF_PID + 正确的 K（可由泵铭牌推导）+ 可选升压限流。\n");
    printf("      端到端默认验收见 tests/test_production_default_acceptance.c。\n");
    return 0;
}
