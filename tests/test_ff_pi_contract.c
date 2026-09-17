/* tests/test_ff_pi_contract.c
 *
 * FF_PI（v13 中期重构产物）的**结构性契约 + 鲁棒性**测试。
 *
 * 【为什么单独一个文件，而不是并进 test_strategy_matrix】
 * strategy_matrix 负责"策略 × K × 限流"的横向对比（说明为什么换掉 RBF）；
 * 本文件负责 FF_PI 自身的内部机制是否真的按设计工作，以及最危险的那个缺陷
 * （D4：K 高估 → 静默稳态欠压）是否真的被修掉。两者性质不同，分开更好定位。
 *
 * 【被测的四件事】
 *   1. 解析整定公式：KP=(2ζωn·τ−1)/K、KI=ωn²·τ/K 必须被τ/ωn/K 正确驱动；
 *   2. 升压制动包络真的在做功（brake 窗口收紧 → Mp 达标；放松 → Mp 恶化）；
 *   3. **D4 回归**：K 高估 1.9× 时 FF_PI 的 ess 仍达标，而 RBF_PID 明显欠压
 *      —— 这是"窄带解除"存在的唯一理由，也是本重构最重要的安全收益；
 *   4. legacy 名义保压流量（defaultTargetFlow=5 L/min）不得与 Q_ff 叠加。
 *
 * 物理对象：PressureModel（25cc/rev @1700rpm，含压力相关内漏），
 * 与生产链路同源；输出上限取压力手柄 100% 的生产口径 20 L/min。
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
#define FLOW_TO_RPM_GAIN    (1000.0f / (PUMP_D_ML_REV * PUMP_VOL_EFF))   /* 42.105 */
#define PUMP_MAX_FLOW_LMIN  (PUMP_RATED_RPM / FLOW_TO_RPM_GAIN)          /* 40.375 */
#define PROD_OUTPUT_MAX     20.0f   /* 压力手柄 FLOWLIMITPERCENT=100 */
#define PROD_OUTPUT_MIN     (-2.375f)
#define PROD_FILTER_ALPHA   0.1f

#define K_NOMINAL           (5.0f / (PUMP_D_ML_REV / 1000.0f))           /* 200 */
/* 实测对象稳态增益（含内漏标定），见评估报告 §1 */
#define K_TRUE              210.55f
/* D4 复现：K 高估 1.9× —— RBF 路径在此 ess = -1.943 bar 且无报警 */
#define K_OVERESTATE        (1.9f * K_NOMINAL)                           /* 380 */

#define TAU_DEFAULT         1.0f
#define OMEGA_DEFAULT       12.0f

#define S1_TARGET_BAR       150.0f
#define S2_TARGET_BAR       100.0f
#define TARGET_MP_PCT       5.0f
#define TARGET_ESS_BAR      1.0f
#define TARGET_SIGMA_BAR    1.0f

typedef struct {
    PressureModelParams         params;
    PressureModelState          plant;
    PressureModelOutput         out;
    HYD_MotionSegment           segment;
    HYD_PressureControllerState ctrl;
    HYD_PressureControllerInput in;
    HYD_TIME                    t;
} Loop;

/* 用τ/ωn/K/前馈 装配一个 FF_PI 回路（其余按生产口径） */
static void loop_init(Loop* L, HYD_PressureControllerType strat,
                      float setpoint, float tau, float omega, float K,
                      float boostLimit, float brakeFrac, float legacyFF,
                      int withNoise, unsigned seed) {
    memset(L, 0, sizeof(*L));
    PressureModel_InitParams(&L->params);
    L->params.enable_sensor_noise = withNoise ? 1u : 0u;
    L->params.enable_motor_noise  = withNoise ? 1u : 0u;
    PressureModel_Reset(&L->plant, seed);

    memset(&L->segment, 0, sizeof(L->segment));
    L->segment.mode                = HYD_MODE_PRESSURE_CLOSED_LOOP;
    L->segment.endCondition        = HYD_END_MANUAL;
    L->segment.direction           = HYD_DIRECTION_HOLD;
    L->segment.targetPressure      = setpoint;
    L->segment.targetFlow          = legacyFF;
    L->segment.maxFlow             = PUMP_MAX_FLOW_LMIN;
    L->segment.pressureController  = strat;
    L->segment.pressureCeiling     = 250.0f;
    L->segment.pressureFilterAlpha = PROD_FILTER_ALPHA;
    L->segment.pressureKp          = 0.5f;   /* 出厂默认，FF_PI 不使用（解析整定覆盖） */
    L->segment.pressureKi          = 0.1f;

    memset(&L->in, 0, sizeof(L->in));
    L->in.targetPressure      = setpoint;
    L->in.outputMin           = PROD_OUTPUT_MIN;
    L->in.outputMax           = PROD_OUTPUT_MAX;
    L->in.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    L->in.pumpSpeedLimit      = PUMP_RATED_RPM;
    L->in.systemGain          = K;
    L->in.boostFlowLimitLmin  = boostLimit;
    L->in.boostBrakeFrac      = brakeFrac;
    L->in.plantTauS           = tau;
    L->in.loopOmega           = omega;
    L->in.feedforwardFlow     = legacyFF;

    HYD_PressureController_InitState(&L->ctrl, 0.0f, 0.0f, 0.0);
    L->t = 0.0;
}

static void loop_step(Loop* L) {
    HYD_PressureControllerOutput co;
    float rpm;
    L->in.measuredPressure = L->out.measured_pressure_bar;
    L->in.timestamp        = L->t;
    HYD_PressureController_Execute(&L->segment, &L->ctrl, &L->in, &co);
    rpm = (float)co.outputFlow * FLOW_TO_RPM_GAIN;
    PressureModel_Step(&L->params, &L->plant, rpm, SIM_DT_S, &L->out);
    L->t += SIM_DT_S;
}

/* 升压段：0 → S1_TARGET，无噪声，量 Mp / tr */
static void run_boost(Loop* L, float* pMax, float* trMs, int* reached, int steps) {
    int i;
    *pMax = 0.0f; *trMs = -1.0f; *reached = 0;
    for (i = 0; i < steps; ++i) {
        loop_step(L);
        if (L->out.measured_pressure_bar > *pMax) *pMax = L->out.measured_pressure_bar;
        if (*trMs < 0.0f && L->out.measured_pressure_bar >= 0.9f * L->in.targetPressure) {
            *trMs = (float)i * SIM_DT_S * 1000.0f;
        }
    }
    *reached = (*trMs > 0.0f);
}

/* 保压段：带噪声，最后 2000 拍统计 ess 与 σ */
static void run_hold(Loop* L, float* ess, float* sigma) {
    int i, n = 0;
    float sum = 0.0f, sq = 0.0f;
    for (i = 0; i < 12000; ++i) {
        loop_step(L);
        if (i >= 10000) {
            sum += L->out.measured_pressure_bar;
            sq  += L->out.measured_pressure_bar * L->out.measured_pressure_bar;
            ++n;
        }
    }
    if (n > 0) {
        float mean = sum / (float)n;
        *ess   = L->in.targetPressure - mean;
        *sigma = sqrtf(fabsf(sq / (float)n - mean * mean));
    } else {
        *ess = 1e9f; *sigma = 1e9f;
    }
}

int main(void) {
    Loop L;
    float pMax, tr, ess, sigma, mp;
    int reached;
    float kp1, ki1, kp2, ki2, kpK, kiK;

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("=== FF_PI 结构性契约 + 鲁棒性测试（v13 中期重构）===\n");
    printf("对象：25cc/rev @1700rpm（含内漏），K_true≈%.2f bar/(L/min)，输出上限 %.1f L/min\n",
           (double)K_TRUE, (double)PROD_OUTPUT_MAX);
    printf("合格：Mp<=%.0f%%，ess<=%.0f bar，σ<=%.0f bar\n\n",
           TARGET_MP_PCT, TARGET_ESS_BAR, TARGET_SIGMA_BAR);

    /* ---------- [1] 解析整定公式必须被 τ / ωn / K 正确驱动 ---------- */
    printf("--- [1] 解析整定 KP=(2ζωn·τ−1)/K, KI=ωn²·τ/K ---\n");
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S2_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, K_NOMINAL,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 0, 0x1u);
    loop_step(&L);
    kp1 = (float)L.ctrl.resolvedKp; ki1 = (float)L.ctrl.resolvedKi;
    printf("      τ=1.0 ωn=12 K=200  → KP=%.4f KI=%.4f (期望 0.1150 / 0.7200)\n",
           (double)kp1, (double)ki1);

    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S2_TARGET_BAR,
              2.0f, OMEGA_DEFAULT, K_NOMINAL,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 0, 0x1u);
    loop_step(&L);
    kp2 = (float)L.ctrl.resolvedKp; ki2 = (float)L.ctrl.resolvedKi;
    printf("      τ=2.0 ωn=12 K=200  → KP=%.4f KI=%.4f (期望 0.2350 / 1.4400)\n",
           (double)kp2, (double)ki2);

    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S2_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, K_NOMINAL * 2.0f,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 0, 0x1u);
    loop_step(&L);
    kpK = (float)L.ctrl.resolvedKp; kiK = (float)L.ctrl.resolvedKi;
    printf("      τ=1.0 ωn=12 K=400  → KP=%.4f KI=%.4f (期望 0.0575 / 0.3600)\n",
           (double)kpK, (double)kiK);

    assert(fabsf(kp1 - 0.1150f) < 1e-3f && fabsf(ki1 - 0.7200f) < 1e-3f);
    assert(fabsf(kp2 - 0.2350f) < 1e-3f && fabsf(ki2 - 1.4400f) < 1e-3f);
    assert(fabsf(kpK - 0.0575f) < 1e-3f && fabsf(kiK - 0.3600f) < 1e-3f);
    printf("  [1] 增益随 τ 线性、随 K 反比，且非写死常量 ............ OK\n\n");

    /* ---------- [2] 升压制动包络真的在做功 ---------- */
    printf("--- [2] 升压制动包络（brake 窗口）对 Mp 的因果性 ---\n");
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S1_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, K_NOMINAL,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 0, 0x1234u);
    run_boost(&L, &pMax, &tr, &reached, 5000);
    mp = (pMax - S1_TARGET_BAR) / S1_TARGET_BAR * 100.0f;
    printf("      brake=2.0（收口）→ Mp=%+.2f%%  峰值 %.2f bar  tr %.0f ms\n",
           (double)mp, (double)pMax, (double)tr);
    assert(reached && pMax <= S1_TARGET_BAR * (1.0f + TARGET_MP_PCT / 100.0f));

    {
        float pMaxLoose, trLoose, mpLoose; int reachedLoose;
        loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S1_TARGET_BAR,
                  TAU_DEFAULT, OMEGA_DEFAULT, K_NOMINAL,
                  0.30f * PUMP_MAX_FLOW_LMIN, 0.05f, 0.0f, 0, 0x1234u);
        run_boost(&L, &pMaxLoose, &trLoose, &reachedLoose, 5000);
        mpLoose = (pMaxLoose - S1_TARGET_BAR) / S1_TARGET_BAR * 100.0f;
        printf("      brake=0.05（放开）→ Mp=%+.2f%%  峰值 %.2f bar\n",
               (double)mpLoose, (double)pMaxLoose);
        /* 放开包络后超调必须显著恶化 —— 否则说明包络根本没接进回路 */
        assert(mpLoose > TARGET_MP_PCT && mpLoose > mp + 5.0f);
    }
    printf("  [2] 包络收紧→Mp达标 / 放开→Mp恶化（因果成立）....... OK\n\n");

    /* ---------- [3] D4 回归：K 高估 1.9× 不得静默欠压 ---------- */
    printf("--- [3] D4 回归：K 高估 1.9× 时的稳态欠压 ---\n");
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S2_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, K_OVERESTATE,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 1, 0x8765u);
    run_hold(&L, &ess, &sigma);
    printf("      FF_PI   K=380 → ess=%+.3f bar  σ=%.3f bar\n", (double)ess, (double)sigma);
    assert(fabsf(ess) <= TARGET_ESS_BAR);
    assert(sigma <= TARGET_SIGMA_BAR);
    {
        float essRbf, sigmaRbf;
        loop_init(&L, HYD_PRESSURE_CONTROLLER_RBF_PID, S2_TARGET_BAR,
                  TAU_DEFAULT, OMEGA_DEFAULT, K_OVERESTATE,
                  0.30f * PUMP_MAX_FLOW_LMIN, 0.5f, 0.0f, 1, 0x8765u);
        run_hold(&L, &essRbf, &sigmaRbf);
        printf("      RBF_PID K=380 → ess=%+.3f bar  σ=%.3f bar（对照：静默欠压）\n",
               (double)essRbf, (double)sigmaRbf);
        /* RBF 必须仍然欠压 —— 这条断言同时是"对照组成立"的证明：
         * 若哪天 RBF 也变好了，说明本用例的植物或链路变了，需要重新审视结论。 */
        assert(essRbf > TARGET_ESS_BAR);
        assert(essRbf - fabsf(ess) > 0.5f);
    }
    printf("  [3] FF_PI 窄带解除生效，K 高估不再产生静默欠压 ..... OK\n\n");

    /* ---------- [4] legacy 名义保压流量不得与 Q_ff 叠加 ---------- */
    printf("--- [4] legacy 名义保压流量（defaultTargetFlow=5 L/min）---\n");
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S1_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, K_NOMINAL,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 5.0f, 0, 0x1234u);
    run_boost(&L, &pMax, &tr, &reached, 5000);
    mp = (pMax - S1_TARGET_BAR) / S1_TARGET_BAR * 100.0f;
    printf("      FF_PI + legacy FF=5.0 → Mp=%+.2f%%  峰值 %.2f bar\n",
           (double)mp, (double)pMax);
    assert(reached && pMax <= S1_TARGET_BAR * (1.0f + TARGET_MP_PCT / 100.0f));
    printf("  [4] Q_ff 取代（而非叠加）legacy 前馈，无双重前馈 .... OK\n\n");

    /* ---------- [5] 窄带宽度回归保护：K 高估 2.5× 仍须够得着目标 ---------- */
    printf("--- [5] 窄带解除宽度（%.2f×P_set）的 K 失配可达性 ---\n", 0.07);
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S1_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, 2.5f * K_NOMINAL,
              0.30f * PUMP_MAX_FLOW_LMIN, 2.0f, 0.0f, 0, 0x1234u);
    run_boost(&L, &pMax, &tr, &reached, 5000);
    mp = (pMax - S1_TARGET_BAR) / S1_TARGET_BAR * 100.0f;
    printf("      K=500（高估2.5×）→ 峰值 %.2f bar  Mp=%+.2f%%  tr %.0f ms\n",
           (double)pMax, (double)mp, (double)tr);
    /* 这是 HYD_FF_PI_NARROW_BAND_FRAC = 0.07 的存在理由。
     * 窄带若被调回 0.05/0.06，包络会把压力永久卡在比例项静平衡点（≈141.9 bar），
     * 积分被抗饱和锁死 → 退回到 D4 那种"静默欠压"。此断言即该常数的回归保护。 */
    assert(reached);
    assert(pMax <= S1_TARGET_BAR * (1.0f + TARGET_MP_PCT / 100.0f));
    printf("  [5] K 高估 2.5× 仍能到达目标且 Mp 达标 ............ OK\n\n");

    /* ---------- [6] 未配 K 时不产生幽灵前馈（保守回退） ---------- */
    printf("--- [6] K 缺失（systemGain=0）时的行为 ---\n");
    loop_init(&L, HYD_PRESSURE_CONTROLLER_FF_PI, S2_TARGET_BAR,
              TAU_DEFAULT, OMEGA_DEFAULT, 0.0f,
              0.0f, 2.0f, 0.0f, 0, 0x2u);
    loop_step(&L);
    printf("      K=0 → KP=%.4f KI=%.4f Q_ff=%.4f\n",
           (double)L.ctrl.resolvedKp, (double)L.ctrl.resolvedKi,
           (double)L.ctrl.resolvedSteadyStateFF);
    /* K 缺失 → 无法解析整定，也无法算 Q_ff。此时的行为必须是"保守回退"：
     *   - Q_ff 必须为 0（**绝不**凭空造一个稳态前馈）；
     *   - 增益回退到段级配置（本例 0.5/0.1），即退化成普通 PI，而不是造一组
     *     自己都算不出来的"解析增益"。
     * 换言之：没有对象模型时，FF_PI 退回"和 PI 一样"，绝不会比 PI 更激进。 */
    assert(L.ctrl.resolvedSteadyStateFF == 0.0f);
    assert(L.ctrl.resolvedKp == L.segment.pressureKp);
    assert(L.ctrl.resolvedKi == L.segment.pressureKi);
    printf("  [6] K 缺失时 Q_ff=0、增益回退段级（退化成 PI）...... OK\n\n");

    printf("=== FF_PI 契约断言全部通过 ===\n");
    printf("注：[3] 的 RBF 对照组保留为“负向断言” —— 它证明本用例的植物确实能复现\n");
    printf("    D4 缺陷，从而证明 FF_PI 的达标不是因为植物被改宽松了。\n");
    return 0;
}
