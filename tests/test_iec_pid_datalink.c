/* tests/test_iec_pid_datalink.c
 *
 * IEC → PID 数据链贯通验证（v10）。
 *
 * 背景：泵电机反馈（转速/转矩/角度）、系统增益 K、泵铭牌参数（排量/容积效率/
 * 最高转速）此前虽然都能通过 IEC 写进 FB，但**链路在中间断了两处**：
 *   1) pumpConfig 与 PID 实际使用的最大流量之间没有显式推导闭环；
 *   2) HYD_PumpFeedback 只被存储，从未被算法消费；
 *   3) systemGain 只有段级字段，没有 IEC 初始化入口。
 *
 * 本用例按"链路自上而下逐段验证"的方式组织，任何一段断开都会红：
 *
 *   [A] IEC 写泵铭牌       → HYD_PumpConfig
 *   [B] HYD_PumpConfig     → 推导最大流量（25cc/0.95/1700 → 40.375 L/min）
 *   [C] 推导最大流量       → IEC 可读回（MAX_FLOW_DERIVED，只读）
 *   [D] IEC 写系统增益     → HYD_PressureControllerInput.systemGain → RBF_PID.K
 *   [E] IEC 写泵反馈       → HYD_PumpFeedback → pressureInput → RBF_PID
 *   [F] 实测转速饱和       → 软上限收缩（anti-windup），可开关
 *   [G] 无反馈 / 未使能    → 逐位退回 v9 行为（不改变既有基线）
 *
 * 用户机实际参数：油泵 25 cc/rev，额定 1700 rpm（见对话记录 2026-09-16）。
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "pressure_controller.h"
#include "rbf_pid.h"
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

#define ASSERT_NEAR(actual, expected, tol, msg) do { \
    tests_run++; \
    if (fabs((double)(actual) - (double)(expected)) <= (double)(tol)) { tests_passed++; } \
    else { \
        tests_failed++; \
        printf("  FAIL [line %d]: %s (got %.6f, want %.6f)\n", \
               __LINE__, msg, (double)(actual), (double)(expected)); \
    } \
} while (0)

/* ---- 用户机实际泵参数 ---- */
#define PUMP_DISP_ML_REV   25.0f
#define PUMP_VOL_EFF       0.95f
#define PUMP_MAX_RPM       1700.0f
/* gain = 1000 / (25 * 0.95) = 42.1053 rpm/(L/min) */
#define PUMP_FLOW_SPEED_GAIN (1000.0f / (PUMP_DISP_ML_REV * PUMP_VOL_EFF))
/* Q_max = 1700 / 42.1053 = 40.375 L/min */
#define PUMP_MAX_FLOW_LMIN  (PUMP_MAX_RPM / PUMP_FLOW_SPEED_GAIN)

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

static void iec_write(int axisId, int param, HYD_REAL value, const char* label) {
    HYD_WRITEPARAMETER wp;

    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wp.EXECUTE) = true;
    IEC_VAL(wp.PARAMETERNUMBER) = (IEC_DINT)param;
    IEC_VAL(wp.VALUE) = (IEC_LREAL)value;
    __mcl_cmd_WriteParameter(&wp);

    tests_run++;
    if (IEC_VAL(wp.DONE) == true && IEC_VAL(wp.ERROR) == false) {
        tests_passed++;
    } else {
        tests_failed++;
        printf("  FAIL [line %d]: WriteParameter(%s) did not complete\n", __LINE__, label);
    }
    (void)label;
}

static HYD_REAL iec_read(int axisId, int param, HYD_BOOL* validOut) {
    HYD_READPARAMETER rp;

    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = (IEC_DINT)param;
    __mcl_cmd_ReadParameter(&rp);
    if (validOut != NULL) {
        *validOut = (IEC_VAL(rp.VALID) == true);
    }
    return (HYD_REAL)IEC_VAL(rp.VALUE);
}

/* ============================ [A][B][C] ============================ */

/* 核心链路：IEC 写铭牌 → 自动算出最大流量 → IEC 可读回 */
static void test_iec_pump_nameplate_to_max_flow(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_BOOL valid = false;
    HYD_REAL derived;

    printf("  [A/B/C] IEC 泵铭牌 -> 最大流量推导闭环\n");

    /* 写之前：未配置 → 推导不出（0），且回退到 gain/limit 路径 */
    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");

    /* [C] 读回推导结果 */
    derived = iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, &valid);
    ASSERT_TRUE(valid == true, "MAX_FLOW_DERIVED must be readable");
    ASSERT_NEAR(derived, PUMP_MAX_FLOW_LMIN, 0.01f,
                "Q_max = 1700 / (1000/(25*0.95)) = 40.375 L/min");

    /* [B] 直接查 FB 解析结果，应与 IEC 读回一致 */
    ASSERT_NEAR(HYD_MotionControlFB_ResolveMaxFlowLmin(fb), PUMP_MAX_FLOW_LMIN, 0.01f,
                "ResolveMaxFlowLmin agrees with the IEC read-back");

    /* gain 也应对得上：1700/40.375 = 42.105 */
    ASSERT_NEAR(HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig),
                PUMP_FLOW_SPEED_GAIN, 0.001f, "derived flow->speed gain");
}

/* 只读语义：MAX_FLOW_DERIVED 不接受写入 */
static void test_max_flow_derived_is_read_only(void) {
    int axisId = create_axis();
    HYD_WRITEPARAMETER wp;
    HYD_BOOL valid = false;
    HYD_REAL before;
    HYD_REAL after;

    printf("  [C] MAX_FLOW_DERIVED 只读性\n");

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");
    before = iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, &valid);

    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = (IEC_SINT)axisId;
    IEC_VAL(wp.EXECUTE) = true;
    IEC_VAL(wp.PARAMETERNUMBER) = (IEC_DINT)HYD_PARAM_MAX_FLOW_DERIVED;
    IEC_VAL(wp.VALUE) = (IEC_LREAL)9999.0;
    __mcl_cmd_WriteParameter(&wp);

    ASSERT_TRUE(IEC_VAL(wp.ERROR) == true,
                "writing the derived value must be rejected");
    after = iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, &valid);
    ASSERT_NEAR(after, before, 1e-6f, "derived value unchanged after rejected write");
}

/* 改铭牌后推导结果必须跟着变（防止"配置改了限幅没变"的静默失配） */
static void test_derived_max_flow_tracks_nameplate_change(void) {
    int axisId = create_axis();
    HYD_REAL derived;

    printf("  [B] 铭牌变更 -> 推导值随动\n");

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");
    ASSERT_NEAR(iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, NULL),
                PUMP_MAX_FLOW_LMIN, 0.01f, "baseline derived flow");

    /* 现场把泵换成 32 cc/rev */
    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, 32.0f, "displacement");
    derived = iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, NULL);
    /* 1000/(32*0.95) = 32.8947; 1700/32.8947 = 51.68 L/min */
    ASSERT_NEAR(derived, 1700.0f / (1000.0f / (32.0f * 0.95f)), 0.01f,
                "derived flow follows a displacement change");
    ASSERT_TRUE(derived > PUMP_MAX_FLOW_LMIN,
                "larger displacement must raise the derived max flow");
}

/* ============================== [D] ============================== */

/* IEC 写系统增益 → 段未配置时作为全机默认 K 进入 RBF-PID */
static void test_iec_system_gain_reaches_rbf_pid(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_MotionSegment segment;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;
    HYD_BOOL valid = false;

    printf("  [D] IEC 系统增益 -> RBF_PID.K\n");

    iec_write(axisId, HYD_PARAM_PRESSURE_SYSTEM_GAIN, 5.0f, "system_gain");
    ASSERT_NEAR(iec_read(axisId, HYD_PARAM_PRESSURE_SYSTEM_GAIN, &valid), 5.0f, 1e-6f,
                "system gain round-trips through IEC");
    ASSERT_TRUE(valid == true, "system gain is readable");

    /* 段级留空 → 应回退到 IEC 下发的值 */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.systemGain = 0.0;   /* 明确不设段级值 */

    input.targetPressure = 100.0f;
    input.measuredPressure = 0.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = iec_read(axisId, HYD_PARAM_PRESSURE_SYSTEM_GAIN, NULL);
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &fb->_pressureController, &input, &output);
    ASSERT_NEAR(fb->_pressureController.rbfPid.K, 5.0f, 1e-5f,
                "IEC system gain becomes RBF-PID K when the segment leaves it unset");

    /* 段级显式配置必须覆盖 IEC 默认值 */
    segment.systemGain = 200.0f;
    input.timestamp = 0.002f;
    HYD_PressureController_Execute(&segment, &fb->_pressureController, &input, &output);
    ASSERT_NEAR(fb->_pressureController.rbfPid.K, 200.0f, 1e-4f,
                "segment-level system gain overrides the IEC default");
}

/* ============================ [E][F][G] ============================ */

/* 泵反馈 → 算法观测输出（actualFlow / trackingError） */
static void test_pump_feedback_becomes_control_observation(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [E] 泵转速反馈 -> 实际流量观测\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 90.0f, 0.5f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 90.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpSpeedFeedbackRpm = 1000.0f;   /* 1000 / 42.105 = 23.75 L/min */
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    ASSERT_TRUE(output.pumpFeedbackApplied == true, "rpm feedback is consumed");
    ASSERT_NEAR(output.actualFlow, 1000.0f / PUMP_FLOW_SPEED_GAIN, 0.01f,
                "actualFlow = rpm / flowToPumpSpeedGain");
    ASSERT_TRUE(output.pumpSpeedSaturated == false, "1000 rpm is not at the 1700 limit");
    ASSERT_NEAR(output.flowTrackingError, output.outputFlow - output.actualFlow, 1e-4f,
                "flowTrackingError = commanded - actual");
}

/* 反转（快速泄压）反馈必须保留符号，且不得触发防饱和 */
static void test_reversing_pump_feedback_is_not_anti_windup(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [E/F] 泵反转反馈（快速泄压）不被误判为饱和\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 50.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 120.0f, 0.0f, 0.0f);

    input.targetPressure = 50.0f;
    input.measuredPressure = 120.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpSpeedFeedbackRpm = -PUMP_MAX_RPM;   /* 满速反转泄压 */
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    input.pumpFeedbackAntiWindup = true;          /* 即使使能也不能干预 */
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    ASSERT_NEAR(output.actualFlow, -PUMP_MAX_FLOW_LMIN, 0.02f,
                "negative rpm yields negative actual flow (relief direction)");
    ASSERT_TRUE(output.pumpSpeedSaturated == false,
                "reversing at full speed is relief, not saturation");
    ASSERT_TRUE(state.rbfPid.external_flow_cap_valid == false,
                "anti-windup cap must stay disarmed while relieving");
}

/* 正向贴限 + 使能 → 软上限收缩到实测可达流量 */
static void test_saturated_rpm_arms_anti_windup_cap(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [F] 正向贴限 -> anti-windup 上限\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 10.0f, 30.0f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 10.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpSpeedFeedbackRpm = PUMP_MAX_RPM;    /* 精确贴限 */
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    input.pumpFeedbackAntiWindup = true;
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    ASSERT_TRUE(output.pumpSpeedSaturated == true, "rpm at the limit is saturation");
    ASSERT_TRUE(state.rbfPid.external_flow_cap_valid == true,
                "anti-windup cap is armed when enabled + saturated forward");
    ASSERT_NEAR(state.rbfPid.external_flow_cap, PUMP_MAX_FLOW_LMIN, 0.02f,
                "cap equals the actually reachable flow");
    ASSERT_TRUE(output.outputFlow <= PUMP_MAX_FLOW_LMIN + 1e-3f,
                "output cannot exceed the reachable flow");
}

/* 未使能时，即使贴限也不干预（默认行为 = v9） */
static void test_anti_windup_disabled_leaves_baseline_untouched(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [G] 未使能 -> 退回 v9 行为\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 10.0f, 30.0f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 10.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpSpeedFeedbackRpm = PUMP_MAX_RPM;
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    input.pumpFeedbackAntiWindup = false;         /* 默认关闭 */
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    ASSERT_TRUE(output.pumpSpeedSaturated == true, "saturation is still reported");
    ASSERT_TRUE(state.rbfPid.external_flow_cap_valid == false,
                "cap stays disarmed when the PLC has not enabled it");
}

/* 无 VALID_RPM 位 → 完全不介入，且观测置 0 */
static void test_missing_rpm_validity_flag_falls_back(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [G] 无有效 rpm 位 -> 安全回退\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 10.0f, 30.0f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 10.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpSpeedFeedbackRpm = PUMP_MAX_RPM;    /* 有数值… */
    input.pumpFeedbackValidFlags = 0u;            /* …但没有声明有效 */
    input.pumpFeedbackAntiWindup = true;
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    ASSERT_TRUE(output.pumpFeedbackApplied == false, "unflagged rpm is not applied");
    ASSERT_NEAR(output.actualFlow, 0.0f, 1e-9f, "actualFlow stays 0 without validity");
    ASSERT_TRUE(output.pumpSpeedSaturated == false, "no saturation without validity");
    ASSERT_TRUE(state.rbfPid.external_flow_cap_valid == false, "cap disarmed");
}

/* 转矩过载判定（IEC 配阈值） */
static void test_torque_overload_detection(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [E] 转矩过载判定\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 60.0f, 5.0f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 60.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpTorqueOverloadPermille = 900.0f;    /* IEC 阈值 900‰ */
    input.timestamp = 0.001f;

    /* 800‰：未过载 */
    input.pumpTorquePermille = 800.0f;
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_TORQUE;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    ASSERT_TRUE(output.pumpTorqueOverload == false, "800 permille < 900 threshold");

    /* 950‰：过载 */
    input.pumpTorquePermille = 950.0f;
    input.timestamp = 0.002f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    ASSERT_TRUE(output.pumpTorqueOverload == true, "950 permille >= 900 threshold");

    /* 未声明有效：不判定 */
    input.pumpTorquePermille = 2000.0f;
    input.pumpFeedbackValidFlags = 0u;
    input.timestamp = 0.003f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    ASSERT_TRUE(output.pumpTorqueOverload == false,
                "torque without validity bit must not raise an overload");
}

/* 阈值 0 = 关闭判定 */
static void test_torque_overload_disabled_by_default(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    printf("  [G] 过载判定默认关闭\n");

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.systemGain = 200.0f;

    HYD_PressureController_InitState(&state, 60.0f, 5.0f, 0.0f);

    input.targetPressure = 100.0f;
    input.measuredPressure = 60.0f;
    input.outputMin = -5.0;
    input.outputMax = 40.0f;
    input.flowToPumpSpeedGain = PUMP_FLOW_SPEED_GAIN;
    input.pumpSpeedLimit = PUMP_MAX_RPM;
    input.systemGain = 200.0f;
    input.pumpTorquePermille = 2000.0f;
    input.pumpFeedbackValidFlags = HYD_PUMP_FEEDBACK_VALID_TORQUE;
    input.pumpTorqueOverloadPermille = 0.0f;      /* 关闭 */
    input.timestamp = 0.001f;

    HYD_PressureController_Execute(&segment, &state, &input, &output);
    ASSERT_TRUE(output.pumpTorqueOverload == false, "threshold 0 disables the check");
}

/* ======================= 结构级：FB 是否真的转发了 ======================= */

/* 构造一个"已激活的压力闭环段"，与 tests/test_mode_switch_bumpless.c 同一手法。
 * 这条路径绕开 LoadDirectSegment 的配方校验，直接落 _activeSegment，
 * 目的是把 Cycle 推到压力闭环分支上。 */
static void seed_active_pressure_segment(HYD_MotionControlFB* fb) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.targetFlow = 2.0f;
    segment.maxFlow = 40.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0f;
    segment.pressureRampRate = 500.0f;
    segment.systemGain = 200.0f;
    segment.pressureRbfConfig.minKp = 0.5f;
    segment.pressureRbfConfig.maxKp = 1.2f;
    segment.pressureRbfConfig.minKi = 0.005f;
    segment.pressureRbfConfig.maxKi = 0.050f;
    segment.pressureRbfConfig.minKd = 0.5f;
    segment.pressureRbfConfig.maxKd = 2.0f;

    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->AXIS_REF.position = 0.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.flow = 0.0f;
    fb->AXIS_REF.pressure = 10.0f;
    fb->AXIS_REF.timestamp = 1.0f;
    fb->_activeSegment = segment;
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb->STATE.active = true;
    fb->FB_STATE = HYD_FB_STATE_RUNNING;
    fb->_directOwnerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    fb->_directOwnerTicket = 1U;
    fb->_executionId = 1U;
    fb->_plannerState.initialized = true;
}

/* 把反馈交给 FB（模拟 HAL），跑一个压力闭环周期，
 * 直接检查 RBF_PID 句柄里是否收到了 cap —— 这是链路打通的硬证据。 */
static void test_fb_forwards_pump_feedback_into_rbf_pid(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback packet = {0};

    printf("  [E/F] FB 层：IEC 泵反馈确实到达 RBF_PID\n");

    seed_active_pressure_segment(fb);

    /* IEC 配置：泵铭牌 + 使能防饱和（必须在 Init 之后，否则被清掉） */
    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");
    iec_write(axisId, HYD_PARAM_PUMP_FEEDBACK_ANTI_WINDUP, 1.0f, "anti_windup");

    /* HAL 送来"泵正向贴限"的反馈 */
    packet.rpm = PUMP_MAX_RPM;
    packet.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    ASSERT_TRUE(HYD_MotionControlFB_SetPumpFeedback(fb, &packet), "feedback accepted");

    fb->AXIS_REF.timestamp = 1.001f;
    HYD_MotionControlFB_Cycle(fb);

    ASSERT_TRUE(fb->_pressureController.rbfPid.external_flow_cap_valid == true,
                "RBF_PID received the cap -> IEC->FB->PID chain is connected");
    ASSERT_NEAR(fb->_pressureController.rbfPid.external_flow_cap,
                PUMP_MAX_FLOW_LMIN, 0.05f,
                "the cap carried the reachable flow derived from the nameplate");
}

/* 关闭使能后，同一路径不得介入 */
static void test_fb_chain_is_inert_when_disabled(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback packet = {0};

    printf("  [G] FB 层：未使能时不介入\n");

    seed_active_pressure_segment(fb);

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");
    /* 注意：不写 ANTI_WINDUP，保持默认 0 */

    packet.rpm = PUMP_MAX_RPM;
    packet.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &packet);

    fb->AXIS_REF.timestamp = 1.001f;
    HYD_MotionControlFB_Cycle(fb);

    ASSERT_TRUE(fb->_pressureController.rbfPid.external_flow_cap_valid == false,
                "cap stays disarmed by default");
}

/* IEC 反写：泵铭牌经 IEC 写入后，FB 的解析结果与推导值一致 */
static void test_iec_written_pump_config_drives_fb_resolution(void) {
    int axisId = create_axis();
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
    HYD_PumpFeedback packet = {0};
    HYD_BOOL valid = false;

    printf("  [A/B] IEC 铭牌 -> FB 解析 -> cap 数值一致\n");

    iec_write(axisId, HYD_PARAM_PUMP_DISPLACEMENT, PUMP_DISP_ML_REV, "displacement");
    iec_write(axisId, HYD_PARAM_PUMP_VOLUMETRIC_EFF, PUMP_VOL_EFF, "vol_eff");
    iec_write(axisId, HYD_PARAM_PUMP_MAX_SPEED, PUMP_MAX_RPM, "max_speed");

    ASSERT_TRUE(fb->pumpConfig.displacementMlRev == PUMP_DISP_ML_REV,
                "nameplate displacement landed on the FB");
    ASSERT_NEAR(iec_read(axisId, HYD_PARAM_MAX_FLOW_DERIVED, &valid),
                HYD_MotionControlFB_ResolveMaxFlowLmin(fb), 1e-4f,
                "IEC read-back equals the FB resolution");

    /* 反转反馈的符号必须原样穿过 IEC 与 FB */
    packet.rpm = -120.0f;
    packet.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM;
    (void)HYD_MotionControlFB_SetPumpFeedback(fb, &packet);
    ASSERT_NEAR(HYD_PumpFeedback_GetActualFlowLmin(&fb->_pumpFeedback, PUMP_FLOW_SPEED_GAIN),
                -120.0f / PUMP_FLOW_SPEED_GAIN, 1e-4f,
                "negative rpm survives into the flow conversion");
}

int main(void) {
    printf("=== IEC -> PID data-link (v10) ===\n\n");

    test_iec_pump_nameplate_to_max_flow();
    test_max_flow_derived_is_read_only();
    test_derived_max_flow_tracks_nameplate_change();

    test_iec_system_gain_reaches_rbf_pid();

    test_pump_feedback_becomes_control_observation();
    test_reversing_pump_feedback_is_not_anti_windup();
    test_saturated_rpm_arms_anti_windup_cap();
    test_anti_windup_disabled_leaves_baseline_untouched();
    test_missing_rpm_validity_flag_falls_back();
    test_torque_overload_detection();
    test_torque_overload_disabled_by_default();

    test_fb_forwards_pump_feedback_into_rbf_pid();
    test_fb_chain_is_inert_when_disabled();
    test_iec_written_pump_config_drives_fb_resolution();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_failed == 0) ? 0 : 1;
}
