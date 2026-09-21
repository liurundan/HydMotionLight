#ifndef HYD_PRESSURE_CONTROLLER_H
#define HYD_PRESSURE_CONTROLLER_H

#include "common_types.h"
#include "rbf_pid.h"

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL measuredPressure;
    HYD_REAL feedforwardFlow;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
    HYD_REAL flowToPumpSpeedGain;  /* rpm per L/min, > 0 — used by RBF PID for fMaxFlow derivation */
    HYD_REAL pumpSpeedLimit;       /* rpm, >= 0 — pump speed upper bound */
    HYD_TIME timestamp;

    /* --- 泵电机反馈（IEC HYD_SetPumpFeedback → HAL → 此处） ---
     * 均为原始反馈量，本层负责换算为算法可用量：
     *   actualFlow = rpm / flowToPumpSpeedGain
     * validFlags 未置位对应位时不参与控制决策（保守回退到原行为）。 */
    HYD_REAL pumpSpeedFeedbackRpm;     /* 实测转速 [rpm]，负值 = 泵反转泄压 */
    HYD_REAL pumpTorquePermille;       /* 实测转矩 [‰ 额定] */
    HYD_REAL pumpAngleDeg;             /* 实测机械相位 [deg]，仅诊断 */
    uint32_t pumpFeedbackValidFlags;   /* HYD_PUMP_FEEDBACK_VALID_* 位掩码 */
    HYD_BOOL pumpFeedbackAntiWindup;   /* 实测转速饱和时冻结积分 */
    HYD_REAL pumpTorqueOverloadPermille; /* 0 = 关闭过载判定 */

    /* --- 系统稳态过程增益 K [bar/(L/min)] ---
     * 【K 是 RBF-PID 的核心过程增益，不是可选补偿】rbf_pid.c 四处依赖：
     * Jacobian 限幅中心 [0.2K,5K]、稳态前馈 P_set/K、过驱动软上限、可达性下界。
     * K=0 → 四处全失效 → RBF "盲学" → 实测真实机 0/3 达标。
     * 来源优先级：段级 HYD_MotionSegment.systemGain > 本字段 > 泵铭牌推导（v12）。
     * 由 motion_control.c 在段解析后填入。 */
    HYD_REAL systemGain;

    /* --- 升压段限流 ---
     * 【v12】<= 0 不再表示"关闭"：上游 HYD_ProduceControlOutputs 已按
     * q_boost = clamp(0.30·Q_max, 3·P_ceiling/K, Q_max) 推导后再下发。
     * 现场经验："升压段必须限流" —— 前置滤波滞后会让满流量建压冲过目标
     * 30% 以上（实测 0→150bar Mp = 32%），限流是 Mp 达标的必要条件。 */
    HYD_REAL boostFlowLimitLmin;   /* L/min，<= 0 = 上游未提供可用限流值 */
    HYD_REAL boostBrakeFrac;       /* 制动窗口 = frac×P_set，<= 0 = 用库默认 */

    /* --- v13：FF_PI 解析整定所需的对象参数（由 motion_control.c 从 FB 参数填入） ---
     * <= 0 表示"未配置 → 用库默认"。 */
    HYD_REAL plantTauS;            /* τ [s]，对象一阶时间常数 */
    HYD_REAL loopOmega;            /* ωn [rad/s]，目标闭环带宽 */
    HYD_REAL systemGainKsys;       /* Ksys [bar/rpm]，机型标定兼容字段 */
} HYD_PressureControllerInput;

typedef struct {
    HYD_BOOL initialized;
    HYD_BOOL trackingRequested;
    HYD_BOOL rbfInitialized;
    HYD_BOOL softResetPending;  /* 目标工况跳变 → 软复位（不清网络，只清工况特定状态）*/
    HYD_REAL integralOutput;
    HYD_REAL previousError;
    HYD_REAL previousFilteredPressure;
    HYD_REAL previousFilteredPressureRate;
    HYD_REAL previousOutput;
    HYD_TIME previousTimestamp;
    HYD_PressureControllerType activeStrategy;
    RBF_PID_Handle rbfPid;

    /* --- v13：本拍**实际生效**的整定结果快照（纯观测，不参与控制） ---
     *
     * 存在的理由来自 RBF 路径的血泪教训：算法内部"以为"自己在做什么对外完全
     * 不可见，出问题只能黑箱猜（Jacobian 被钉死在边界、增益恒撞限幅，现场只看到
     * "压力不对"，没有任何量能把矛头指向它）。FF_PI 把这三个数暴露出来之后，
     * "K / τ / ωn 有没有真的到达算法"从推断变成了可断言的事实：
     *   resolvedSteadyStateFF = P_set / K        → 反推 K
     *   resolvedKp = (2ζωn·τ − 1)/K, resolvedKi = ωn²·τ/K
     * 非 FF_PI 策略同样填充（kp/ki 取 config 值，steadyStateFF 为 0），便于横向对比。 */
    HYD_REAL resolvedKp;
    HYD_REAL resolvedKi;
    HYD_REAL resolvedSteadyStateFF;
    HYD_REAL resolvedBoostFlowLimitLmin;  /* 0 = 上游未给可用限流值 */

    /* Gate 0 contract: requested/applied strategy and adaptation observability. */
    HYD_PressureControllerType requestedStrategy;
    HYD_PressureCalibrationStatus calibrationStatus;
    HYD_PressureLimitStatus limitStatus;
    HYD_BOOL dtValid;
    HYD_REAL gDu;                    /* bar/(L/min) per one-sample flow increment */
    HYD_REAL effectiveUpperCap;      /* effective output upper cap [L/min] */
    uint32_t promotionValidSamples;
    uint32_t adaptationFreezeCount;
    HYD_PressureAdaptationFreezeReason adaptationFreezeReason;
} HYD_PressureControllerState;

typedef struct {
    HYD_PressureControllerType appliedStrategy;
    HYD_REAL targetPressure;
    HYD_REAL filteredPressure;
    HYD_REAL filteredPressureRate;
    HYD_REAL controlError;
    HYD_REAL proportionalTerm;
    HYD_REAL integralTerm;
    HYD_REAL derivativeTerm;
    HYD_REAL trackingTerm;
    HYD_REAL feedforwardFlow;
    HYD_REAL feedbackFlow;
    HYD_REAL unsaturatedOutputFlow;
    HYD_REAL outputFlow;
    HYD_REAL samplingPeriod;
    HYD_REAL adaptiveKp;
    HYD_REAL adaptiveKi;
    HYD_REAL adaptiveKd;
    HYD_REAL adaptiveJacobian;
    HYD_BOOL trackingApplied;
    HYD_BOOL saturated;
    HYD_BOOL adaptiveActive;

    /* --- 泵反馈观测（数据链贯通后的对外结果） --- */
    HYD_REAL actualFlow;            /* 由实测转速换算的实际流量 [L/min] */
    HYD_REAL flowTrackingError;     /* 指令流量 − 实际流量 [L/min] */
    HYD_BOOL pumpSpeedSaturated;    /* 实测转速贴限 且 指令流量仍在增大 */
    HYD_BOOL pumpTorqueOverload;    /* 实测转矩超过 IEC 配置阈值 */
    HYD_BOOL pumpFeedbackApplied;   /* 本次是否真的用上了 rpm 反馈 */

    /* --- v13：FF_PI 输出观测 ---
     * steadyStateFF：稳态前馈流量 Q_ff = P_set/K [L/min]
     * capBoundDemand：控制器想要的流量超过当前有效上限（上限成为瓶颈）。
     *   该位持续置位 = "压力打不到目标，且原因是输出被上限卡住"
     *   —— 正是旧 RBF 路径"K 高估 → 静默稳态欠压"那个缺陷的可观测化。
     *   上层诊断应对其做防抖（例如持续 200 ms）后再报警。 */
    HYD_REAL steadyStateFF;
    HYD_BOOL capBoundDemand;

    /* Gate 0 contract: same-cycle diagnostic snapshot as state reporter input. */
    HYD_PressureControllerType requestedStrategy;
    HYD_PressureCalibrationStatus calibrationStatus;
    HYD_PressureLimitStatus limitStatus;
    HYD_BOOL dtValid;
    HYD_REAL gDu;
    HYD_REAL effectiveUpperCap;
    uint32_t promotionValidSamples;
    uint32_t adaptationFreezeCount;
    HYD_PressureAdaptationFreezeReason adaptationFreezeReason;
} HYD_PressureControllerOutput;

void HYD_PressureController_ClearState(HYD_PressureControllerState* state);
void HYD_PressureController_InitState(HYD_PressureControllerState* state,
                                      HYD_REAL initialPressure,
                                      HYD_REAL initialOutputFlow,
                                      HYD_TIME timestamp);
void HYD_PressureController_RequestTracking(HYD_PressureControllerState* state,
                                           HYD_REAL trackedOutputFlow);
void HYD_PressureController_Execute(const HYD_MotionSegment* segment,
                                    HYD_PressureControllerState* state,
                                    const HYD_PressureControllerInput* input,
                                    HYD_PressureControllerOutput* output);

#endif /* HYD_PRESSURE_CONTROLLER_H */
