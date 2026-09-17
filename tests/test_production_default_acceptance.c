/* tests/test_production_default_acceptance.c
 *
 * 量产默认验收（v13）：**只配泵铭牌，其余全部保持出厂默认**，
 * 走真实 IEC 入口（__mcl_cmd_PressureHandle）+ 真实控制器（Execute）+ 物理植物，
 * 验证"现场不额外调参也能达标"。
 *
 * 【为什么需要这个用例】
 * v12 之前，出厂默认是 PI 策略 + systemGain=0，实测在真实机上 **0/3**——
 * 即"现场照默认跑 = 不可用"。v12 把默认改成 RBF_PID 并从泵铭牌推导
 *   K = Ksys/(D/1000)              （Ksys 默认 5.0 bar/rpm）
 *   q_boost = clamp(0.30*Q_max, 3*P_ceiling/K, Q_max)
 * 本用例把"默认即达标"这一结论钉进 CI：任何让默认退化的改动都会立刻红。
 *
 * 【v13 变更：默认策略 RBF_PID → FF_PI】
 * 2026-09-17 的实测评估（docs/RBF-PID压力闭环算法工程实用性评估-2026-09-17.md）
 * 证明 RBF-PID 的神经网络对输出**零贡献**，稳定性实际来自输出软上限钳位而非整定，
 * 且全面劣于正确整定的前馈+PI。因此默认策略换成 FF_PI：
 *   Q_ff = P_set/K,  KP = (2ζωn·τ − 1)/K,  KI = ωn²·τ/K,  ζ = 1
 * 推导 K 与升压限流的规则不变（仍是 v12 那套），只是消费方换了。
 * 本用例同步改为断言 **FF_PI 句柄里的实际生效值**（resolvedKp/Ki/SteadyStateFF），
 * 这样"K 有没有真的到达算法"仍是可断言的硬事实，而不是"配置写了就算数"。
 *
 * 本用例与 tests/test_strategy_matrix.c 的分工：
 *   - strategy_matrix：控制器策略/增益空间的**对比**（说明"为什么必须 RBF+K"）
 *   - 本用例：**端到端**验证出厂默认本身达标（走 IEC 适配层 + FB 周期）
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "pressure_controller.h"
#include "rbf_pid.h"
#include "pressure_model.h"
#include "common_types.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL [line %d]: %s\n", __LINE__, msg); } \
} while (0)

/* ---- 用户机实际泵参数（25 cc/rev @ 1700 rpm） ---- */
#define PUMP_DISP_ML_REV     25.0f
#define PUMP_VOL_EFF         0.95f
#define PUMP_MAX_RPM         1700.0f
#define PUMP_FLOW_SPEED_GAIN (1000.0f / (PUMP_DISP_ML_REV * PUMP_VOL_EFF)) /* 42.1053 */
#define PUMP_MAX_FLOW_LMIN   (PUMP_MAX_RPM / PUMP_FLOW_SPEED_GAIN)         /* 40.375 */

/* v12 推导期望值 */
#define EXPECT_K             (5.0f / (PUMP_DISP_ML_REV / 1000.0f))         /* 200 */
#define EXPECT_BOOST         (0.30f * PUMP_MAX_FLOW_LMIN)                  /* 12.1125 */

/* v13 库默认对象参数（未现场标定时的保守值），用于算出期望的解析整定结果 */
#define DEFAULT_TAU          1.0f    /* HYD_DEFAULT_PLANT_TAU_S */
#define DEFAULT_OMEGA        12.0f   /* HYD_DEFAULT_LOOP_OMEGA */
#define DEFAULT_ZETA         1.0f    /* HYD_FF_PI_DAMPING */

#define SIM_DT               0.001f
#define TARGET_MP_PCT        5.0f
#define TARGET_ESS_BAR       1.0f
#define TARGET_SIGMA_BAR     1.0f

static int create_axis(void) {
    HYD_CREATEMOTION cm;
    __HydMotion_framework_Init();
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = PUMP_FLOW_SPEED_GAIN;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = PUMP_MAX_RPM;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static void iec_write(int axisId, int param, HYD_REAL value) {
    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wp.EXECUTE) = true;
    IEC_VAL(wp.PARAMETERNUMBER) = (IEC_DINT)param;
    IEC_VAL(wp.VALUE) = (IEC_LREAL)value;
    __mcl_cmd_WriteParameter(&wp);
    ASSERT_TRUE(IEC_VAL(wp.DONE) == true && IEC_VAL(wp.ERROR) == false,
                "IEC WriteParameter should complete");
}

/* 一次闭环：IEC 手柄 → 框架发布（内部 Scan=Sample+Cycle，并按 dfCycleTime=1ms
 * 自增时间戳）→ 指令流量 → 植物 → 反馈。与 tests/test_parameter_iec.c 同一手法。 */
static void loop_step(HYD_MotionControlFB* fb, HYD_PRESSUREHANDLE* ph,
                      PressureModelParams* pp, PressureModelState* ps,
                      PressureModelOutput* po) {
    float rpm;
    __mcl_cmd_PressureHandle(ph);              /* 生产入口：活更新目标/限流 */
    fb->AXIS_REF.pressure = po->measured_pressure_bar;
    __HydMotion_framework_Publish();
    rpm = (float)fb->_lastCommandedFlow * PUMP_FLOW_SPEED_GAIN;
    PressureModel_Step(pp, ps, rpm, SIM_DT, po);
}

static void init_handle(HYD_PRESSUREHANDLE* ph, int axisId, float target) {
    memset(ph, 0, sizeof(*ph));
    IEC_VAL(ph->EN) = true;
    IEC_VAL(ph->EXECUTE) = true;
    IEC_VAL(ph->AXISID) = (IEC_SINT)axisId;
    IEC_VAL(ph->PRESSURE) = target;
    IEC_VAL(ph->DURATION) = 1000.0f;           /* 足够长，避免段结束 */
    IEC_VAL(ph->FLOWLIMITPERCENT) = 100.0f;    /* 手柄上限 = 20 L/min */
    IEC_VAL(ph->PRESSURERAMPRATE) = 100000.0f; /* 不额外限速 */
    IEC_VAL(ph->CONTINUOUSUPDATE) = true;
}

/* ---------------- 主验收：出厂默认（仅配铭牌）升压 + 保压 ---------------- */
static void test_default_config_meets_all_three(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    PressureModelParams pp;
    PressureModelState ps;
    PressureModelOutput po;
    HYD_PRESSUREHANDLE ph;
    float pmax = 0.0f, p90 = -1.0f;
    float sum = 0.0f, sq = 0.0f;
    int i, n = 0, settled = 0;
    float ess, sigma, mp;

    printf("  [1] 出厂默认（仅配泵铭牌）→ 是否 3/3 达标\n");

    /* 只写泵铭牌。策略/增益/滤波/升限流一律不写 —— 全部走出厂默认与 v12 推导。 */
    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV);
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF);
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM);

    /* ---- 升压段：0 → 150 bar，无噪声看超调 ---- */
    PressureModel_InitParams(&pp);
    pp.enable_sensor_noise = 0u;
    pp.enable_motor_noise = 0u;
    PressureModel_Reset(&ps, 0x1234u);
    memset(&po, 0, sizeof(po));
    fb->AXIS_REF.pressure = 0.0f;
    fb->AXIS_REF.timestamp = 0.0;
    init_handle(&ph, axisId, 150.0f);

    for (i = 0; i < 5000; ++i) {
        loop_step(fb, &ph, &pp, &ps, &po);
        if (po.measured_pressure_bar > pmax) pmax = po.measured_pressure_bar;
        if (p90 < 0.0f && po.measured_pressure_bar >= 0.9f * 150.0f) p90 = (float)i * SIM_DT * 1000.0f;
    }
    mp = (p90 > 0.0f) ? (pmax - 150.0f) / 150.0f * 100.0f : -1.0f;
    printf("      S1 Mp=%.2f%%  (峰值 %.2f bar, tr %.1f ms)\n", mp, pmax, p90);

    /* ---- 保压段：100 bar，带噪声与纹波 ---- */
    PressureModel_Reset(&ps, 0x8765u);
    memset(&po, 0, sizeof(po));
    pp.enable_sensor_noise = 1u;
    pp.enable_motor_noise = 1u;
    fb->_activeSegmentValid = false;   /* 重开段，让控制器重置 */
    fb->AXIS_REF.timestamp = 0.0;
    init_handle(&ph, axisId, 100.0f);
    for (i = 0; i < 2000; ++i) {
        loop_step(fb, &ph, &pp, &ps, &po);
    }
    for (i = 0; i < 10000; ++i) {
        loop_step(fb, &ph, &pp, &ps, &po);
        if (i >= 8000) {
            sum += po.measured_pressure_bar;
            sq += po.measured_pressure_bar * po.measured_pressure_bar;
            ++n;
        }
    }
    if (n > 0) {
        float mean = sum / (float)n;
        ess = 100.0f - mean;
        sigma = sqrtf(fabsf(sq / (float)n - mean * mean));
    } else {
        ess = 1e9f; sigma = 1e9f;
    }
    (void)settled;
    printf("      S2 ess=%.3f bar  sigma=%.3f bar RMS\n", ess, sigma);

    ASSERT_TRUE(p90 > 0.0f, "默认配置必须真的升到 90% 目标压力");
    ASSERT_TRUE(pmax <= 150.0f * (1.0f + TARGET_MP_PCT / 100.0f),
                "默认配置：升压超调 Mp 必须 <= 5%");
    ASSERT_TRUE(fabsf(ess) <= TARGET_ESS_BAR, "默认配置：保压 ess 必须 <= 1 bar");
    ASSERT_TRUE(sigma <= TARGET_SIGMA_BAR, "默认配置：保压 sigma 必须 <= 1 bar RMS");
}

/* ---------------- 推导值是否真的到达算法（硬证据） ---------------- */
static void test_derived_k_and_boost_reach_the_algorithm(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    PressureModelParams pp;
    PressureModelState ps;
    PressureModelOutput po;
    HYD_PRESSUREHANDLE ph;
    float expectKp, expectKi, expectFF;
    int i;

    printf("  [2] 推导的 K 与升压限流是否真的到达 FF_PI\n");

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV);
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF);
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM);

    PressureModel_InitParams(&pp);
    pp.enable_sensor_noise = 0u;
    pp.enable_motor_noise = 0u;
    PressureModel_Reset(&ps, 0x1u);
    memset(&po, 0, sizeof(po));
    fb->AXIS_REF.pressure = 0.0f;
    fb->AXIS_REF.timestamp = 0.0;
    init_handle(&ph, axisId, 150.0f);

    for (i = 0; i < 50; ++i) {
        loop_step(fb, &ph, &pp, &ps, &po);
    }

    /* 期望的解析整定结果（τ=1.0, ωn=12, ζ=1, K=200, P_set=150）：
     *   Q_ff = 150/200            = 0.75
     *   KP   = (2*1*12*1 − 1)/200 = 0.115
     *   KI   = 144*1/200          = 0.72   （位置式：∫ += KI·e·dt） */
    expectKp = (2.0f * DEFAULT_ZETA * DEFAULT_OMEGA * DEFAULT_TAU - 1.0f) / EXPECT_K;
    expectKi = (DEFAULT_OMEGA * DEFAULT_OMEGA * DEFAULT_TAU) / EXPECT_K;
    expectFF = 150.0f / EXPECT_K;

    ASSERT_TRUE(fb->_pressureController.activeStrategy == HYD_PRESSURE_CONTROLLER_FF_PI,
                "出厂默认策略应为 FF_PI（v13）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedSteadyStateFF -
                      (double)expectFF) <= 0.01,
                "推导的 K 必须到达 FF_PI：Q_ff == P_set/K（期望 0.75）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedKp - (double)expectKp) <= 0.005,
                "FF_PI 的 KP 必须按 (2ζωn·τ−1)/K 解析整定（期望 0.115）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedKi - (double)expectKi) <= 0.005,
                "FF_PI 的 KI 必须按 ωn²·τ/K 解析整定（期望 0.72）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedBoostFlowLimitLmin -
                      (double)EXPECT_BOOST) <= 0.2,
                "推导的升压限流必须到达控制器（期望 ~12.11）");
}

/* ---------------- 显式配置必须仍然优先 ---------------- */
static void test_explicit_config_still_overrides(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    PressureModelParams pp;
    PressureModelState ps;
    PressureModelOutput po;
    HYD_PRESSUREHANDLE ph;
    float expectKp, expectKi, expectFF;
    int i;

    printf("  [3] 显式配置优先于推导（不破坏现场标定）\n");

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV);
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF);
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM);
    iec_write(axisId, HYD_PARAM_PRESSURE_SYSTEM_GAIN, 120.0f);      /* 显式 K */
    iec_write(axisId, HYD_PARAM_PRESSURE_BOOST_FLOW_LIMIT, 8.0f);   /* 显式限流 */

    PressureModel_InitParams(&pp);
    pp.enable_sensor_noise = 0u;
    pp.enable_motor_noise = 0u;
    PressureModel_Reset(&ps, 0x2u);
    memset(&po, 0, sizeof(po));
    fb->AXIS_REF.pressure = 0.0f;
    fb->AXIS_REF.timestamp = 0.0;
    init_handle(&ph, axisId, 150.0f);

    for (i = 0; i < 50; ++i) {
        loop_step(fb, &ph, &pp, &ps, &po);
    }

    /* 显式 K=120 → 解析整定与前馈都必须跟着变（证明不是写死常量）：
     *   Q_ff = 150/120 = 1.25,  KP = 23/120 = 0.19167,  KI = 144/120 = 1.2 */
    expectKp = (2.0f * DEFAULT_ZETA * DEFAULT_OMEGA * DEFAULT_TAU - 1.0f) / 120.0f;
    expectKi = (DEFAULT_OMEGA * DEFAULT_OMEGA * DEFAULT_TAU) / 120.0f;
    expectFF = 150.0f / 120.0f;

    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedSteadyStateFF -
                      (double)expectFF) <= 0.01,
                "显式写入的 K 必须覆盖推导值（Q_ff 应为 1.25）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedKp - (double)expectKp) <= 0.005,
                "显式 K 必须改变 FF_PI 的解析整定 KP（期望 0.19167）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedKi - (double)expectKi) <= 0.005,
                "显式 K 必须改变 FF_PI 的解析整定 KI（期望 1.2）");
    ASSERT_TRUE(fabsf((double)fb->_pressureController.resolvedBoostFlowLimitLmin - 8.0) <= 0.2,
                "显式写入的升压限流必须覆盖推导值");
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("=== 量产默认验收（v13：出厂默认即达标 · 默认策略 FF_PI）===\n");
    printf("链路：IEC 压力手柄 -> FB 周期 -> Execute -> 植物（真实泵 25cc/1700rpm）\n");
    printf("配置：仅写泵铭牌；策略/增益/滤波/限流全部走出厂默认 + v12 推导\n\n");

    test_default_config_meets_all_three();
    test_derived_k_and_boost_reach_the_algorithm();
    test_explicit_config_still_overrides();

    printf("\n=== 量产默认验收：%d/%d 通过", tests_passed, tests_run);
    if (tests_failed > 0) {
        printf("（%d 失败）===\n", tests_failed);
        return 1;
    }
    printf(" ===\n");
    return 0;
}
