#include "pressure_controller.h"
#include <math.h>
#include <string.h>

#define HYD_LEGACY_PRESSURE_FLOW_KP 1.5
#define HYD_DEFAULT_PRESSURE_FILTER_ALPHA 0.1
#define HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA 0.05

/* --- v13：FF_PI（前馈 + 解析整定 PI）默认对象参数 ---
 *
 * 整定式（对象 P(s) = K/(τs+1)，位置式 PI，闭环匹配 s² + 2ζωn·s + ωn²）：
 *     KP = (2·ζ·ωn·τ − 1) / K        [L/min per bar]
 *     KI = ωn²·τ / K                 [L/min per bar per s]
 *
 * 【τ 默认 1.0 s 是刻意保守的】KP 与 KI 都 ∝ τ：
 *   低估 τ（默认 1.0 < 实测 1.862）→ 增益偏小 → 响应偏慢但**绝不会失稳**；
 *   高估 τ                          → 增益偏大 → 可能越过稳定边界（实测边界 wn≈18~20）。
 * 因此默认值选在"安全侧"，现场按实测 t63 写入 HYD_PARAM_PRESSURE_PLANT_TAU 即可提速。
 *
 * 【ωn 默认 12 rad/s】实测稳定边界 wn≈18~20（超过后 σ 从 0.5 恶化到 6.5 bar）。
 * 取边界的 ~65%，留出 τ 标定误差与机型差异的裕度。 */
#define HYD_DEFAULT_PLANT_TAU_S     1.0f
#define HYD_DEFAULT_LOOP_OMEGA      12.0f
#define HYD_FF_PI_DAMPING           1.0f   /* ζ = 1：压力闭环不希望有超调 */
/* 升压制动包络的"窄带解除"阈值：|e| <= 该比例 × P_set 时完全不收紧上限。
 * 这是修复"K 高估 → 软上限变硬天花板 → 静默稳态欠压"的关键，
 * 实测（评估报告 §4.1）：K 高估 1.9× 时 ess 由 -1.943 bar 归零。
 *
 * 【0.07 是实测拐点，勿凭直觉改 —— 它同时决定"超调裕度"与"K 失配可达性"】
 * 解除太晚：K 高估时，比例项的静平衡点 P_eq = (K·Q_ff + K·kp·P_set)/(1 + K·kp)
 *   落在解除区**之外** → 包络把压力永久卡在 P_eq（实测 K=500：卡在 141.9/150 bar，
 *   Mp 记为负 = 根本没到目标），积分被抗饱和锁死 → 又变成"静默欠压"。
 * 解除太早：升压末期提前放开上限 → 建压速率突增 → 10 ms 滤波滞后 → 超调恶化。
 *
 * 生产链路实测（25cc/1700rpm、τ=1.0、ωn=12、目标 150 bar，boost=12.11）：
 *   窄带   K=100   K=200    K=300   K=380   K=500
 *   0.05   7.79F   2.42P    0.86P   0.44P   -6.07F(卡死)
 *   0.06   7.79F   2.91P    0.86P   0.44P   -6.07F(卡死)
 *   0.07   8.45F   3.20P    0.99P   0.46P    0.66P   ← 采用
 *   0.08   9.36F   3.67P    1.10P   0.73P    0.79P
 *   0.10  10.37F   4.52P    1.65P   1.05P    0.95P
 *   0.15  14.29F   6.75F    3.35P   1.85P    1.69P
 * （数字为 Mp%，P=3/3 达标 F=未达标；K 真值 ≈210.55）
 * 0.07 是"能救回 K 高估 2.5×"的最小值，名义点仍留 1.8 pp 超调裕度。 */
#define HYD_FF_PI_NARROW_BAND_FRAC  0.07f
/* FF_PI 的升压制动窗口默认比例 e_b = brake_frac × P_set。
 *
 * 【为什么是 2.0 而不是 RBF 沿用的 0.5 —— 实测，勿凭直觉改】
 * 生产链路前置滤波 α=0.1（时间常数 dt/α = 10 ms）。升压若过快，滤波滞后会让控制器
 * "看不见"已经上升的压力 → 冲过目标。实测 25cc/1700rpm、K=200、目标 150 bar：
 *     brake=0.5, boost=12.11 → Mp 26.27%（滤波滞后只看到 ~112 bar 时实际已到 150）
 *     brake=1.0, boost=12.11 → Mp 10.43%
 *     brake=2.0, boost=12.11 → Mp  2.42%   tr 265ms  ts 435ms   ← 采用
 * 物理含义：e_b = 2·P_set 意味着从**开始升压**就在收口（frac = e/(2·P_set) ≤ 0.5），
 * 等效把建压速率限制在 τ_filter 跟得上的水平（≈0.05·P_set/τ_filter ≈ 750 bar/s）。
 * 窗口再大只是更慢，不再改善超调；窗口小于 1.0 则超调急剧恶化。 */
#define HYD_FF_PI_BRAKE_FRAC_DEFAULT 2.0f

typedef struct {
    HYD_PressureControllerType strategy;
    HYD_BOOL supportsIntegral;
    HYD_BOOL supportsDerivative;
    HYD_BOOL adaptive;
} HYD_PressureStrategySpec;

typedef struct {
    HYD_REAL minKp;
    HYD_REAL maxKp;
    HYD_REAL minKi;
    HYD_REAL maxKi;
    HYD_REAL minKd;
    HYD_REAL maxKd;
    HYD_REAL etaW;
    HYD_REAL etaC;
    HYD_REAL etaB;
    HYD_REAL etaP;
    HYD_REAL etaI;
    HYD_REAL etaD;
    HYD_BOOL disablePressureAccelFeedforward;
} HYD_RbfPidResolvedConfig;

typedef struct {
    const HYD_PressureStrategySpec* strategySpec;
    HYD_PressureControllerType requestedStrategy;
    HYD_PressureControllerType strategy;
    HYD_REAL kp;
    HYD_REAL kpHigh;
    HYD_REAL gainBand;
    HYD_REAL ki;
    HYD_REAL kd;
    HYD_REAL integralLimit;
    HYD_REAL deadband;
    HYD_REAL filterAlpha;
    HYD_REAL derivativeFilterAlpha;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
    HYD_REAL dt;
    HYD_REAL samplingPeriod;
    HYD_REAL systemGain;      /* v10: 段级优先，回退到 IEC 下发的系统增益 */
    HYD_REAL boostFlowLimitLmin; /* v11: 升压段限流（<=0 = 关闭） */
    HYD_REAL boostBrakeFrac;     /* v11: 制动窗口（<=0 = 用库默认） */
    HYD_REAL steadyStateFF;      /* v13: FF_PI 稳态前馈 Q_ff = P_set/K [L/min] */
    HYD_REAL plantTauS;          /* v13: FF_PI 整定用的对象时间常数 */
    HYD_REAL loopOmega;          /* v13: FF_PI 整定用的目标带宽 */
    HYD_RbfPidResolvedConfig rbf;
} HYD_PressureResolvedConfig;

static const HYD_PressureStrategySpec HYD_PRESSURE_STRATEGY_SPECS[] = {
    {HYD_PRESSURE_CONTROLLER_P, false, false, false},
    {HYD_PRESSURE_CONTROLLER_PI, true, false, false},
    {HYD_PRESSURE_CONTROLLER_PID, true, true, false},
    {HYD_PRESSURE_CONTROLLER_RBF_PID, true, true, true},
    {HYD_PRESSURE_CONTROLLER_RBF_PI, true, false, true},
    /* v13：前馈 + 解析整定 PI。supportsIntegral=true，不要微分（一阶对象 + 噪声下
     * 微分只会放大纹波，实测 KD 对 σ 无益），不要"自适应"标志（本策略是确定的）。 */
    {HYD_PRESSURE_CONTROLLER_FF_PI, true, false, false}
};

static const HYD_PressureStrategySpec* HYD_FindPressureStrategySpec(HYD_PressureControllerType strategy) {
    size_t index;

    for (index = 0U; index < sizeof(HYD_PRESSURE_STRATEGY_SPECS) / sizeof(HYD_PRESSURE_STRATEGY_SPECS[0]); ++index) {
        if (HYD_PRESSURE_STRATEGY_SPECS[index].strategy == strategy) {
            return &HYD_PRESSURE_STRATEGY_SPECS[index];
        }
    }

    return NULL;
}

static const HYD_PressureStrategySpec* HYD_ResolvePressureStrategySpec(const HYD_MotionSegment* segment) {
    const HYD_PressureStrategySpec* spec;

    if (segment != NULL) {
        spec = HYD_FindPressureStrategySpec(segment->pressureController);
        if (spec != NULL) {
            return spec;
        }
    }

    return HYD_FindPressureStrategySpec(HYD_PRESSURE_CONTROLLER_P);
}

static HYD_REAL HYD_ResolvePositiveOrDefault(HYD_REAL configuredValue, HYD_REAL defaultValue) {
    if (configuredValue > 0.0) {
        return configuredValue;
    }
    return defaultValue;
}

static HYD_REAL HYD_ResolveGain(HYD_REAL configuredGain, HYD_REAL fallbackGain) {
    return HYD_ResolvePositiveOrDefault(configuredGain, fallbackGain);
}

static HYD_REAL HYD_ResolveIntegralLimit(const HYD_MotionSegment* segment,
                                         const HYD_PressureControllerInput* input) {
    HYD_REAL limit;

    if (segment == NULL) {
        return 0.0;
    }

    if (segment->pressureIntegralLimit > 0.0) {
        limit = segment->pressureIntegralLimit;
    } else if (segment->maxFlow > 0.0) {
        limit = segment->maxFlow;
    } else {
        limit = 0.0;
    }

    if (input != NULL && input->outputMax > 0.0 &&
        (limit <= 0.0 || input->outputMax < limit)) {
        limit = input->outputMax;
    }

    return limit;
}

static HYD_REAL HYD_ResolveFilterAlpha(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_PRESSURE_FILTER_ALPHA;
    }

    if (segment->pressureFilterAlpha > 0.0) {
        return HYD_ClampReal(segment->pressureFilterAlpha, 0.0, 1.0);
    }

    return HYD_DEFAULT_PRESSURE_FILTER_ALPHA;
}

static HYD_REAL HYD_ResolveDerivativeFilterAlpha(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
    }

    if (segment->pressureDerivativeFilterAlpha > 0.0) {
        return HYD_ClampReal(segment->pressureDerivativeFilterAlpha, 0.0, 1.0);
    }

    return HYD_DEFAULT_PRESSURE_DERIVATIVE_FILTER_ALPHA;
}

static HYD_REAL HYD_ResolveDeadband(const HYD_MotionSegment* segment) {
    if (segment == NULL || segment->pressureDeadband <= 0.0) {
        return 0.0;
    }
    return segment->pressureDeadband;
}

static HYD_REAL HYD_ApplyPressureDeadband(HYD_REAL error, HYD_REAL deadband) {
    if (deadband <= 0.0 || fabs(error) <= deadband) {
        return deadband > 0.0 ? 0.0 : error;
    }

    return error > 0.0 ? error - deadband : error + deadband;
}

static HYD_REAL HYD_ResolveTrackedIntegralOutput(const HYD_MotionSegment* segment,
                                                 const HYD_PressureControllerInput* input,
                                                 HYD_REAL proportionalTerm,
                                                 HYD_REAL derivativeTerm,
                                                 HYD_REAL trackedOutputFlow) {
    HYD_REAL trackedIntegral;
    HYD_REAL integralLimit;

    if (input == NULL) {
        return 0.0;
    }

    trackedIntegral = trackedOutputFlow - input->feedforwardFlow - proportionalTerm - derivativeTerm;
    integralLimit = HYD_ResolveIntegralLimit(segment, input);
    if (integralLimit > 0.0) {
        trackedIntegral = HYD_ClampReal(trackedIntegral, -integralLimit, integralLimit);
    }
    return trackedIntegral;
}

static HYD_REAL HYD_ResolveAdaptiveSamplingPeriod(const HYD_PressureControllerState* state,
                                                  HYD_REAL dt) {
    if (dt > 0.0) {
        return dt;
    }

    if (state != NULL && state->rbfInitialized && state->rbfPid.sampling_period > 0.0f) {
        return (HYD_REAL)state->rbfPid.sampling_period;
    }

    return HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD;
}

static void HYD_ResolveRbfPidConfig(const HYD_MotionSegment* segment,
                                    HYD_RbfPidResolvedConfig* config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->minKp = (HYD_REAL)PID_MIN_KP;
    config->maxKp = (HYD_REAL)PID_MAX_KP;
    config->minKi = (HYD_REAL)PID_MIN_KI;
    config->maxKi = (HYD_REAL)PID_MAX_KI;
    config->minKd = (HYD_REAL)PID_MIN_KD;
    config->maxKd = (HYD_REAL)PID_MAX_KD;
    config->etaW = (HYD_REAL)HYD_DEFAULT_RBF_W_LEARNING_RATE;
    config->etaC = (HYD_REAL)HYD_DEFAULT_RBF_C_LEARNING_RATE;
    config->etaB = (HYD_REAL)HYD_DEFAULT_RBF_B_LEARNING_RATE;
    config->etaP = (HYD_REAL)HYD_DEFAULT_PID_P_LEARNING_RATE;
    config->etaI = (HYD_REAL)HYD_DEFAULT_PID_I_LEARNING_RATE;
    config->etaD = (HYD_REAL)HYD_DEFAULT_PID_D_LEARNING_RATE;
    config->disablePressureAccelFeedforward = false;

    if (segment == NULL) {
        return;
    }

    config->minKp = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKp, config->minKp);
    config->maxKp = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKp, config->maxKp);
    config->minKi = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKi, config->minKi);
    config->maxKi = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKi, config->maxKi);
    config->minKd = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKd, config->minKd);
    config->maxKd = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKd, config->maxKd);
    config->etaW = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaW, config->etaW);
    config->etaC = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaC, config->etaC);
    config->etaB = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaB, config->etaB);
    config->etaP = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaP, config->etaP);
    config->etaI = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaI, config->etaI);
    config->etaD = HYD_ResolvePositiveOrDefault(segment->pressureRbfConfig.etaD, config->etaD);
    config->disablePressureAccelFeedforward =
        segment->pressureRbfConfig.disablePressureAccelFeedforward > 0.0 ? true : false;
}

static void HYD_ResolvePressureControllerConfig(const HYD_MotionSegment* segment,
                                                const HYD_PressureControllerState* state,
                                                const HYD_PressureControllerInput* input,
                                                HYD_PressureResolvedConfig* config) {
    const HYD_PressureStrategySpec* strategySpec;

    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    strategySpec = HYD_ResolvePressureStrategySpec(segment);
    config->requestedStrategy = strategySpec->strategy;
    config->strategySpec = strategySpec;
    config->strategy = strategySpec->strategy;
    config->kp = HYD_ResolveGain((segment != NULL) ? segment->pressureKp : 0.0,
                                 HYD_LEGACY_PRESSURE_FLOW_KP);
    config->kpHigh = (segment != NULL && segment->pressureKpHigh > 0.0)
        ? segment->pressureKpHigh : 0.0;
    config->gainBand = (segment != NULL && segment->pressureGainBand > 0.0)
        ? segment->pressureGainBand : 0.2;
    config->ki = strategySpec->supportsIntegral
        ? HYD_ResolveGain((segment != NULL) ? segment->pressureKi : 0.0, 0.0)
        : 0.0;
    config->kd = strategySpec->supportsDerivative
        ? HYD_ResolveGain((segment != NULL) ? segment->pressureKd : 0.0, 0.0)
        : 0.0;
    config->integralLimit = HYD_ResolveIntegralLimit(segment, input);
    config->deadband = HYD_ResolveDeadband(segment);
    config->filterAlpha = HYD_ResolveFilterAlpha(segment);
    config->derivativeFilterAlpha = HYD_ResolveDerivativeFilterAlpha(segment);
    config->outputMin = (input != NULL) ? input->outputMin : 0.0;
    /* M4: 泄压目标下限钳位：仅在目标压和反馈压都进入低压区时禁止负流量 */
    if (input != NULL &&
        input->targetPressure < 5.0 &&
        input->measuredPressure < 5.0) {
        config->outputMin = 0.0; /* 目标压力 < 5 bar 时禁止负流量 */
    }

    config->outputMax = (input != NULL) ? input->outputMax : 0.0;
    if (config->outputMax < config->outputMin) {
        config->outputMax = config->outputMin;
    }

    if (input != NULL && state != NULL) {
        config->dt = input->timestamp - state->previousTimestamp;
        if (config->dt < 0.0) {
            config->dt = 0.0;
        }
    }
    config->samplingPeriod = HYD_ResolveAdaptiveSamplingPeriod(state, config->dt);

    /* v10: 系统增益来源优先级 —— 段级显式配置 > IEC 下发的 FB 级参数。
     * 段级用于"同一台机器不同动作段需要不同 K"的场合（如射胶段刚性高于合模段）；
     * FB 级由 IEC 在初始化时一次性下发，作为全机默认值。 */
    if (segment != NULL && segment->systemGain > 0.0) {
        config->systemGain = segment->systemGain;
    } else if (input != NULL && input->systemGain > 0.0) {
        config->systemGain = input->systemGain;
    } else {
        config->systemGain = 0.0;
    }

    /* v11: 升压段限流 —— IEC 下发。
     * v12: <=0 不再等同于"关闭"：上游 HYD_ProduceControlOutputs 已按泵铭牌
     *      推导出 q_boost 后再下发；此处 <=0 只代表"上游确实没有可用的限流值"。 */
    config->boostFlowLimitLmin = (input != NULL && input->boostFlowLimitLmin > 0.0)
        ? input->boostFlowLimitLmin : 0.0;
    config->boostBrakeFrac = (input != NULL && input->boostBrakeFrac > 0.0)
        ? input->boostBrakeFrac : 0.0;

    /* v13: FF_PI 解析整定 —— 用对象参数直接算出 PI 增益，取代"继承来的经验常数"。
     *
     * 为什么不再用 segment->pressureKp/Ki：那组值（出厂 0.5/0.1，库内 fallback 1.5/0）
     * 从未针对本对象整定过。实测稳定区是 KP<=0.31 且 KI<=0.0029（报告 §2.2），
     * 而 0.5/0.1 都在区外 —— 这正是旧默认"必须靠软上限钳位才不发散"的根因。 */
    config->plantTauS = HYD_DEFAULT_PLANT_TAU_S;
    config->loopOmega = HYD_DEFAULT_LOOP_OMEGA;
    config->steadyStateFF = 0.0;
    if (config->strategy == HYD_PRESSURE_CONTROLLER_FF_PI) {
        if (input != NULL && input->plantTauS > 0.0) {
            config->plantTauS = input->plantTauS;
        }
        if (input != NULL && input->loopOmega > 0.0) {
            config->loopOmega = input->loopOmega;
        }
        if (config->systemGain > 0.0) {
            HYD_REAL tau = config->plantTauS;
            HYD_REAL wn = config->loopOmega;
            HYD_REAL kp = (2.0f * HYD_FF_PI_DAMPING * wn * tau - 1.0f) / config->systemGain;
            /* 分子为负说明 wn·τ 太小（对象极慢 / 带宽要求极低），此时比例项无意义，取 0。 */
            config->kp = (kp > 0.0f) ? kp : 0.0f;
            config->ki = (wn * wn * tau) / config->systemGain;
            /* 前馈：吃掉维持目标压力所需的绝大部分稳态流量，PI 只处理残差与扰动。
             * 这是"快速性"与"鲁棒性"解耦的关键 —— K 有 ±10% 误差也不会产生静差，
             * 因为积分项会补上（前提是上限没有把输出卡死，见 Execute 里的窄带解除）。 */
            if (input != NULL && input->targetPressure > 0.0f) {
                config->steadyStateFF = input->targetPressure / config->systemGain;
            }
        }
    }

    /* An uncalibrated physical gain cannot safely drive FF or online RBF
     * tuning.  Keep the requested strategy observable, but apply a fixed
     * conservative PI until the motion layer provides a calibrated status.
     * Direct controller callers retain compatibility when they provide the
     * legacy K_process and a positive tau explicitly. */
    {
        HYD_BOOL explicitCalibrated = (config->systemGain > 0.0 &&
                                       config->plantTauS > 0.0 &&
                                       input != NULL && input->plantTauS > 0.0);
        HYD_BOOL stateCalibrated = (state != NULL &&
                                    state->calibrationStatus >=
                                    HYD_PRESSURE_CALIBRATION_CALIBRATED);
        HYD_BOOL adaptiveRequested =
            (config->requestedStrategy == HYD_PRESSURE_CONTROLLER_FF_PI) ||
            (config->requestedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PID) ||
            (config->requestedStrategy == HYD_PRESSURE_CONTROLLER_RBF_PI);
        if (adaptiveRequested && !explicitCalibrated && !stateCalibrated) {
            config->strategy = HYD_PRESSURE_CONTROLLER_PI;
            config->strategySpec = HYD_FindPressureStrategySpec(
                HYD_PRESSURE_CONTROLLER_PI);
            config->kp = 0.10;
            config->ki = 0.05;
            config->kd = 0.0;
            config->integralLimit = 0.10 * config->outputMax;
            config->steadyStateFF = 0.0;
            config->boostFlowLimitLmin = 0.0;
        }
    }

    HYD_ResolveRbfPidConfig(segment, &config->rbf);

}

static void HYD_EnsureRbfPidInitialized(HYD_PressureControllerState* state,
                                        HYD_REAL samplingPeriod,
                                        HYD_REAL outputMax,
                                        HYD_REAL flowToPumpSpeedGain,
                                        HYD_REAL pumpSpeedLimit) {
    HYD_REAL resolvedOutputMax;
    float fMaxFlow, fFlowRateLimit;

    if (state == NULL) {
        return;
    }

    resolvedOutputMax = (outputMax > 0.0) ? outputMax : 0.0;

    /* Compute max pump flow [L/min] = pumpSpeedLimit [rpm] / flowToPumpSpeedGain [rpm/(L/min)] */
    if (flowToPumpSpeedGain > 0.0 && pumpSpeedLimit >= 0.0) {
        fMaxFlow = (float)(pumpSpeedLimit / flowToPumpSpeedGain);
    } else {
        fMaxFlow = 0.0f;
    }
    if (fMaxFlow <= 0.0f) fMaxFlow = 90.0f;  /* safe fallback: 1800 rpm / 20 rpm per L/min */

    /* Compute flow rate limit as fraction of max flow, capped at 1.0 */
    fFlowRateLimit = (resolvedOutputMax > 0.0 && fMaxFlow > 0.0f)
        ? (float)(resolvedOutputMax / (HYD_REAL)fMaxFlow) : 1.0f;
    if (fFlowRateLimit > 1.0f) fFlowRateLimit = 1.0f;

    if (!state->rbfInitialized) {
        RBF_PID_Init(&state->rbfPid,
                     (float)samplingPeriod,
                     fMaxFlow,
                     fFlowRateLimit);
        state->rbfInitialized = true;
    }

    state->rbfPid.sampling_period = (float)samplingPeriod;
    state->rbfPid.fMaxFlow = fMaxFlow;
    state->rbfPid.fFlowRateLimit = fFlowRateLimit;
    state->rbfPid.output_min_flow = (float)MIN_OUTPUT;
    state->rbfPid.output_max_flow = (float)resolvedOutputMax;
}

static void HYD_ApplyRbfPidConfig(HYD_PressureControllerState* state,
                                  const HYD_PressureResolvedConfig* config,
                                  const HYD_MotionSegment* segment,
                                  HYD_REAL flowToPumpSpeedGain,
                                  HYD_REAL pumpSpeedLimit) {
    if (state == NULL || config == NULL) {
        return;
    }

    HYD_EnsureRbfPidInitialized(state, config->samplingPeriod, config->outputMax,
                                flowToPumpSpeedGain, pumpSpeedLimit);
    state->rbfPid.output_min_flow = (float)config->outputMin;
    state->rbfPid.output_max_flow = (float)config->outputMax;
    RBF_PID_SetParamLimits(&state->rbfPid,
                           (float)config->rbf.minKp,
                           (float)config->rbf.maxKp,
                           (float)config->rbf.minKi,
                           (float)config->rbf.maxKi,
                           (float)config->rbf.minKd,
                           (float)config->rbf.maxKd);
    RBF_PID_SetLearningRates(&state->rbfPid,
                             (float)config->rbf.etaW,
                             (float)config->rbf.etaC,
                             (float)config->rbf.etaB,
                             (float)config->rbf.etaP,
                             (float)config->rbf.etaI,
                             (float)config->rbf.etaD);
    RBF_PID_SetControlMode(
        &state->rbfPid,
        config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI
            ? RBF_PID_CONTROL_MODE_PI
            : RBF_PID_CONTROL_MODE_PID);
    RBF_PID_SetPressureAccelFeedforwardEnabled(
        &state->rbfPid,
        config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI ||
            config->rbf.disablePressureAccelFeedforward ? false : true);
    RBF_PID_SetFlowNormalization(
        &state->rbfPid,
        (float)HYD_ResolvePositiveOrDefault(config->outputMax,
                                            (HYD_REAL)state->rbfPid.fMaxFlow));
    state->rbfPid.KP = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KP,
                                            config->rbf.minKp,
                                            config->rbf.maxKp);
    state->rbfPid.KI = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KI,
                                            config->rbf.minKi,
                                            config->rbf.maxKi);
    state->rbfPid.KD = (float)HYD_ClampReal((HYD_REAL)state->rbfPid.KD,
                                            config->rbf.minKd,
                                            config->rbf.maxKd);
    state->rbfPid.pid_mode_kd = state->rbfPid.KD;
    if (config->strategy == HYD_PRESSURE_CONTROLLER_RBF_PI) {
        state->rbfPid.KD = 0.0f;
    }

    {
        HYD_REAL pressureScale = 0.0;
        if (segment != NULL && segment->pressureCeiling > 0.0) {
            pressureScale = segment->pressureCeiling;
        } else if (segment != NULL && segment->targetPressure > 0.0) {
            HYD_REAL candidate = segment->targetPressure * 3.0;
            pressureScale = (candidate > (HYD_REAL)MAX_PRESSURE) ?
                candidate : (HYD_REAL)MAX_PRESSURE;
        }
        RBF_PID_SetPressureNormalization(&state->rbfPid, (float)pressureScale);
    }

    if (config->systemGain > 0.0) {
        RBF_PID_SetGainCompensation(&state->rbfPid, (float)config->systemGain);
    } else {
        RBF_PID_SetGainCompensation(&state->rbfPid, 0.0f);
    }

    /* v11: 升压段限流接线。
     *
     * 【为什么必须在这里接】RBF_PID_SetBoostFlowLimit() 自 v8 就存在，
     * 但在 v11 之前 src/ 里**没有任何调用者** —— 于是这条"现场经验"（升压段
     * 必须限流）在库侧完全无法启用，PLC 也够不着。实测后果：α=0.1（生产滤波）
     * 且限流关闭时 0→150bar 的 Mp = 32.03%（合格线 5%）。
     *
     * 语义：<= 0 = 关闭限流（库默认，保持既有回归基线逐位不变）。
     *       每次每段都会重设，因此换段/换配方不需额外接线。 */
    RBF_PID_SetBoostFlowLimit(&state->rbfPid, (float)config->boostFlowLimitLmin);
    if (config->boostFlowLimitLmin > 0.0 && config->boostBrakeFrac > 0.0) {
        RBF_PID_SetBoostBrakeFrac(&state->rbfPid, (float)config->boostBrakeFrac);
    }


}

/* v10: 把泵电机实测反馈接入 RBF-PID —— 数据链 IEC → FB → PID 的最后一环。
 *
 * 链路：IEC __mcl_cmd_SetPumpFeedback → HYD_MotionControlFB_SetPumpFeedback
 *       → fb->_pumpFeedback → pressureInput.* → 此处 → RBF_PID
 *
 * 换算：actual_flow = rpm / flowToPumpSpeedGain
 * 用途：
 *   1) 观测：actualFlow / flowTrackingError / pumpSpeedSaturated 对外输出，
 *      供 HMI 判断"泵是否跟得上指令"（此前这些反馈只被存储、从未被消费）。
 *   2) 抗饱和：实测转速贴上 IEC 配置的转速上限 → 把 PID 软上限收缩到实测
 *      可达流量，闭合 windup 回路（需 IEC 使能 HYD_PARAM_PUMP_FEEDBACK_ANTI_WINDUP）。
 *
 * 保守回退：validFlags 无 VALID_RPM 位时，cap 关闭 + 观测置 0，
 *           行为与 v9 逐位一致（无反馈硬件也不影响控制）。 */
static void HYD_ApplyPumpFeedbackToRbfPid(HYD_PressureControllerState* state,
                                          const HYD_PressureControllerInput* input,
                                          HYD_PressureControllerOutput* output) {
    HYD_PumpFeedback packet;
    HYD_REAL limit;
    HYD_REAL actualFlow = 0.0;
    HYD_BOOL rpmValid;
    HYD_BOOL speedSaturated = false;
    HYD_BOOL torqueOverload = false;
    HYD_BOOL forward = true;

    if (state == NULL || input == NULL) {
        return;
    }

    limit = input->pumpSpeedLimit;
    rpmValid = HYD_PumpFeedback_HasValid(input->pumpFeedbackValidFlags,
                                         HYD_PUMP_FEEDBACK_VALID_RPM);

    memset(&packet, 0, sizeof(packet));
    packet.rpm = (HYD_REAL)input->pumpSpeedFeedbackRpm;
    packet.validFlags = input->pumpFeedbackValidFlags;
    actualFlow = HYD_PumpFeedback_GetActualFlowLmin(&packet,
                                                    input->flowToPumpSpeedGain);

    if (rpmValid && limit > 0.0) {
        /* 1% 容差：编码器/驱动器上报的"到限"很少精确等于配置值。 */
        speedSaturated = (fabs(input->pumpSpeedFeedbackRpm) >= 0.99 * limit);
        forward = (input->pumpSpeedFeedbackRpm >= 0.0);
    }

    if (input->pumpTorqueOverloadPermille > 0.0 &&
        HYD_PumpFeedback_HasValid(input->pumpFeedbackValidFlags,
                                  HYD_PUMP_FEEDBACK_VALID_TORQUE)) {
        torqueOverload = (fabs(input->pumpTorquePermille) >=
                          input->pumpTorqueOverloadPermille);
    }

    /* 对外报告的"转速饱和"只针对正向贴限：泵反转满速是**泄压能力充足**，
     * 不是"加不了流量"，把它报成饱和会让 HMI/上层误判为限幅故障。 */
    if (output != NULL) {
        output->pumpSpeedSaturated = (speedSaturated && forward);
    }

    /* 抗饱和只在"正向贴限"时收紧上限。
     * 泵反转（rpm < 0，快速泄压）并非流量不足，干预会阻碍泄压；
     * 且 RBF_PID 的 cap 只作用于上限，下限（负流量）本就不受影响。 */
    if (input->pumpFeedbackAntiWindup && speedSaturated && forward) {
        RBF_PID_SetExternalFlowCap(&state->rbfPid,
                                   (float)((actualFlow > 0.0) ? actualFlow : 0.0),
                                   true);
    } else {
        RBF_PID_SetExternalFlowCap(&state->rbfPid, 0.0f, false);
    }

    if (output != NULL) {
        output->actualFlow = actualFlow;
        output->pumpTorqueOverload = torqueOverload;
        output->pumpFeedbackApplied = rpmValid;
    }
}

static void HYD_SynchronizeRbfPidState(HYD_PressureControllerState* state,
                                       HYD_REAL trackedOutputFlow,
                                       HYD_REAL targetPressure,
                                       HYD_REAL measuredPressure,
                                       const HYD_PressureResolvedConfig* config,
                                       const HYD_MotionSegment* segment,
                                       HYD_REAL flowToPumpSpeedGain,
                                       HYD_REAL pumpSpeedLimit) {
    HYD_REAL seededFlow;
    HYD_REAL error;

    if (state == NULL || config == NULL) {
        return;
    }

    RBF_PID_Reset(&state->rbfPid);
    HYD_ApplyRbfPidConfig(state, config, segment,
                          flowToPumpSpeedGain, pumpSpeedLimit);
    seededFlow = HYD_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);

    /* v6: 稳态前馈播种 — K>0 时用 P_set/K 作为 u_prev 初值
     * 消除积分从 0 爬坡到稳态流量的过程(v5 基线需 588 步)→tr 降低。
     * 依据: 段切换后新工况稳态流量 = P_set/K, 以此为积分基准,
     *       PID 只需处理 ΔQ(瞬态), 不必重建整个 u_prev。
     * 泄压保护: 实测压力已高于目标(需负流量泄压)时禁用前馈,
     *         否则 P_set/K 恒正会把输出顶住阻碍泄压。 */
    if (state->rbfPid.K > 0.0f && targetPressure > 0.0f &&
        measuredPressure <= targetPressure + 1.0f) {
        HYD_REAL ffFlow = (HYD_REAL)RBF_PID_OverdriveFlowCap(
            &state->rbfPid, (float)(targetPressure - measuredPressure));
        seededFlow = HYD_ClampReal(ffFlow, config->outputMin, config->outputMax);
    }

    error = targetPressure - measuredPressure;

    state->rbfPid.Output = (float)seededFlow;
    state->rbfPid.u_prev = (float)seededFlow;

    state->rbfPid.P_set = (float)targetPressure;
    state->rbfPid.P_actual = (float)measuredPressure;
    state->rbfPid.Error = (float)error;
    state->rbfPid.du = 0.0f;
    state->rbfPid.du_prev = 0.0f;
    state->rbfPid.e_prev1 = (float)error;
    state->rbfPid.e_prev2 = (float)error;
    state->rbfPid.y_prev1 = (float)measuredPressure;
    state->rbfPid.y_prev2 = (float)measuredPressure;
    state->rbfPid.fLastActPress = (float)measuredPressure;
    state->rbfPid.fLastActPress2 = (float)measuredPressure;
    state->rbfPid.last_ref = (float)targetPressure;
    state->rbfPid.press_stuck_time_s = 0.0f;
    state->rbfPid.overdrive_peak_press = (float)measuredPressure;
    state->rbfPid.output_saturated = false;
    state->rbfPid.Status = 1;
    state->rbfPid.TuneResult = 0;
}

/* 判定目标是否发生"工况级"跳变（非 ramp 渐进）。
 * 相对阈值 5%·|old|（最小 2 bar），避免 ramp 每拍误触发。
 * 双向：大幅下调或大幅上调(>3倍阈值)都算工况跳变。 */
static HYD_BOOL HYD_TargetOperatingPointChanged(HYD_REAL newTarget,
                                                  HYD_REAL oldTarget) {
    HYD_REAL threshold;
    if (oldTarget <= 0.0f) {
        return false;
    }
    threshold = 0.05f * (HYD_REAL)fabsf((float)oldTarget);
    if (threshold < 2.0f) {
        threshold = 2.0f;
    }
    return (newTarget < oldTarget - threshold) ||
           (newTarget > oldTarget + threshold * 3.0f);
}

/* 软复位：保留网络通用知识（不 memset），只清工况特定状态 + 增益向窗口中心收敛。
 * 与 HYD_SynchronizeRbfPidState（全量复位）的区别：
 * - 不调 RBF_PID_Reset → 网络权重 w/c/b 保留
 * - 增益收敛到窗口中心（非归 MIN）→ 新工况段升速不被拖慢
 * - 清 f_velfb 的 P₀ 参考点 → 消除跨段累积偏置残留 */
static void HYD_SoftResetRbfPidState(HYD_PressureControllerState* state,
                                      HYD_REAL trackedOutputFlow,
                                      HYD_REAL targetPressure,
                                      HYD_REAL measuredPressure,
                                      const HYD_PressureResolvedConfig* config) {
    RBF_PID_Handle *pid = &state->rbfPid;
    HYD_REAL seededFlow;

    if (state == NULL || config == NULL) {
        return;
    }

    /* 1. 增益向窗口中心收敛（非归 MIN，非保留旧工况值） */
    pid->KP = 0.5f * (pid->min_KP + pid->max_KP);
    pid->KI = 0.5f * (pid->min_KI + pid->max_KI);
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PID) {
        pid->KD = 0.5f * (pid->min_KD + pid->max_KD);
    }

    /* 2. bumpless 播种：u_prev = 当前输出（不归零）
     * v6: K>0 时改用 P_set/K 稳态前馈——工况跳变后新稳态流量才是正确基准，
     *    避免沿用旧工况流量(偏高→超调/偏低→爬坡慢)。 */
    seededFlow = HYD_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);
    /* 泄压保护同 HYD_SynchronizeRbfPidState：P_set/K 恒正，实测压力已高于目标
     * (需负流量泄压)时禁用前馈，否则会把输出顶在正流量上阻碍泄压。 */
    if (pid->K > 0.0f && targetPressure > 0.0f &&
        measuredPressure <= targetPressure + 1.0f) {
        HYD_REAL ffFlow = (HYD_REAL)RBF_PID_OverdriveFlowCap(
            pid, (float)(targetPressure - measuredPressure));
        seededFlow = HYD_ClampReal(ffFlow, config->outputMin, config->outputMax);
    }
    pid->Output = (float)seededFlow;
    pid->u_prev = (float)seededFlow;

    /* 3. 清除工况特定历史（这些是旧工况点的，过期） */
    pid->e_prev1 = 0.0f;
    pid->e_prev2 = 0.0f;
    pid->du_prev = 0.0f;
    pid->prev_d_term = 0.0f;
    pid->steady_count = 0;
    pid->steady_state = false;

    /* 4. 重置 f_velfb 的 P₀ 参考点（消除跨段累积偏置残留） */
    pid->fLastActPress = (float)measuredPressure;
    pid->fLastActPress2 = (float)measuredPressure;
    pid->last_ref = (float)targetPressure;
    pid->v_ref_k1 = 0.0f;
    pid->press_stuck_time_s = 0.0f;
    pid->overdrive_peak_press = (float)measuredPressure;

    /* 5. 同步 P_set / P_actual / Error */
    pid->P_set = (float)targetPressure;
    pid->P_actual = (float)measuredPressure;
    pid->Error = (float)(targetPressure - measuredPressure);
    pid->du = 0.0f;
    pid->output_saturated = false;
    pid->Status = 1;
}

void HYD_PressureController_ClearState(HYD_PressureControllerState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->activeStrategy = HYD_PRESSURE_CONTROLLER_NONE;
}

void HYD_PressureController_InitState(HYD_PressureControllerState* state,
                                      HYD_REAL initialPressure,
                                      HYD_REAL initialOutputFlow,
                                      HYD_TIME timestamp) {
    if (state == NULL) {
        return;
    }

    HYD_PressureController_ClearState(state);
    state->initialized = true;
    state->trackingRequested = false;
    state->previousFilteredPressure = initialPressure;
    state->previousFilteredPressureRate = 0.0;
    state->previousOutput = initialOutputFlow;
    state->previousTimestamp = timestamp;
}

void HYD_PressureController_RequestTracking(HYD_PressureControllerState* state,
                                           HYD_REAL trackedOutputFlow) {
    if (state == NULL) {
        return;
    }

    state->previousOutput = trackedOutputFlow;
    state->trackingRequested = true;
}

/* v13: FF_PI 的升压制动包络 —— 只收紧、不放宽，且**窄带内完全解除**。
 *
 * 数学形式（位置规划器 sqrt(2·a·s) 制动律在压力域的对应形式）：
 *     Q_allow(e) = Q_ss + (Q_boost − Q_ss)·min(1, e / (P_set·brake_frac)),  e > 0
 *     Q_ss = P_set/K
 * 代入对象 dP/dt = (K·Q − P)/τ 可得闭环时间常数
 *     τ_eff = τ / [1 + (K·Q_boost − P_set)/e_b]  <<  τ
 * 且 e → 0 时 Q → Q_ss，等效平衡压力恰好 = P_set ⇒ 理论上无稳态超调。
 *
 * 【窄带解除是本次重构的关键修复】
 * 旧 RBF 路径把这个包络一直用到 e → 0（软上限 1.05·P_set/K 稳态也生效），
 * 于是 Q_cap 变成一个由 K 决定的**硬天花板**。当 K 高估时 Q_cap 偏小，
 * 压力永远打不到目标，而速度式反算又把积分钉在天花板上 → 积分永远救不回来。
 * 实测（评估报告 §2.4）：K 高估 1.9× → ess = -1.943 bar，且无任何报警。
 *
 * 这里在 |e| <= HYD_FF_PI_NARROW_BAND_FRAC·P_set 时直接返回"不限"，
 * 把稳态决策权交还给积分器 —— 包络回归它本来的职责：只管升压瞬态。 */
static HYD_REAL HYD_FfPiOutputMax(const HYD_PressureResolvedConfig* config,
                                  HYD_REAL targetPressure,
                                  HYD_REAL error,
                                  HYD_REAL hardMax) {
    HYD_REAL qss;
    HYD_REAL qBoost;
    HYD_REAL eBrake;
    HYD_REAL frac;

    if (config->boostFlowLimitLmin <= 0.0 || config->systemGain <= 0.0) {
        return hardMax;
    }
    if (targetPressure <= 0.0) {
        return hardMax;
    }
    /* 超压方向：升压包络不收紧，泄压由独立的下限路径处理。 */
    if (error <= 0.0) {
        return hardMax;
    }

    qss = targetPressure / config->systemGain;
    qBoost = config->boostFlowLimitLmin;
    if (qBoost < qss) {
        /* 可达性下界：限流值低于维持流量时目标压力永不可达 → 抬到 qss。
         * 与 rbf_pid.c 的 v9 修正同款，避免"静默打不到"。 */
        qBoost = qss;
    }

    eBrake = targetPressure * ((config->boostBrakeFrac > 0.0)
        ? config->boostBrakeFrac : HYD_FF_PI_BRAKE_FRAC_DEFAULT);
    if (eBrake < 1.0f) {
        eBrake = 1.0f;
    }
    frac = error / eBrake;
    if (frac > 1.0f) {
        frac = 1.0f;
    }

    {
        HYD_REAL envelope = qss + (qBoost - qss) * frac;
        HYD_REAL release = HYD_FF_PI_NARROW_BAND_FRAC * targetPressure;
        HYD_REAL hysteresis = 0.01f * targetPressure;
        HYD_REAL transitionStart = release - hysteresis;
        HYD_REAL transitionEnd = release + hysteresis;
        HYD_REAL releaseFrac;

        if (hysteresis < 0.5f) {
            hysteresis = 0.5f;
            transitionStart = release - hysteresis;
            transitionEnd = release + hysteresis;
        }
        if (error >= transitionEnd) {
            return (envelope < hardMax) ? envelope : hardMax;
        }
        if (error <= transitionStart) {
            return hardMax;
        }

        /* Cubic release avoids the old discontinuity at 0.07*Pset. */
        releaseFrac = (error - transitionStart) / (transitionEnd - transitionStart);
        releaseFrac = releaseFrac * releaseFrac * (3.0f - 2.0f * releaseFrac);
        return envelope + (hardMax - envelope) * (1.0f - releaseFrac);
    }
}

static HYD_REAL HYD_ResolvePumpFlowCap(const HYD_PressureControllerInput* input,
                                       HYD_REAL hardMax,
                                       HYD_BOOL* valid) {
    HYD_PumpFeedback packet;
    HYD_REAL actualFlow;
    HYD_BOOL rpmValid;

    if (valid != NULL) {
        *valid = false;
    }
    if (input == NULL || !input->pumpFeedbackAntiWindup ||
        input->pumpSpeedLimit <= 0.0 ||
        input->flowToPumpSpeedGain <= 0.0) {
        return hardMax;
    }

    rpmValid = HYD_PumpFeedback_HasValid(input->pumpFeedbackValidFlags,
                                         HYD_PUMP_FEEDBACK_VALID_RPM);
    if (!rpmValid || fabs(input->pumpSpeedFeedbackRpm) < 0.99 * input->pumpSpeedLimit ||
        input->pumpSpeedFeedbackRpm < 0.0) {
        return hardMax;
    }

    memset(&packet, 0, sizeof(packet));
    packet.rpm = input->pumpSpeedFeedbackRpm;
    packet.validFlags = input->pumpFeedbackValidFlags;
    actualFlow = HYD_PumpFeedback_GetActualFlowLmin(
        &packet, input->flowToPumpSpeedGain);
    if (!isfinite(actualFlow) || actualFlow < 0.0) {
        return hardMax;
    }
    if (valid != NULL) {
        *valid = true;
    }
    return (actualFlow < hardMax) ? actualFlow : hardMax;
}

static HYD_REAL HYD_ResolveEffectiveUpperCap(
    const HYD_PressureResolvedConfig* config,
    const HYD_PressureControllerInput* input,
    HYD_PressureControllerState* state,
    HYD_REAL targetPressure,
    HYD_REAL error,
    HYD_REAL hardMax) {
    HYD_REAL externalCap;
    HYD_REAL boostCap;
    HYD_REAL qss;
    HYD_BOOL externalValid;
    HYD_PressureLimitStatus limitStatus = HYD_PRESSURE_LIMIT_NONE;

    externalCap = HYD_ResolvePumpFlowCap(input, hardMax, &externalValid);
    boostCap = hardMax;
    qss = (config->systemGain > 0.0 && targetPressure > 0.0)
        ? targetPressure / config->systemGain : 0.0;

    if (config->boostFlowLimitLmin > 0.0 && qss > 0.0 &&
        config->boostFlowLimitLmin < qss) {
        limitStatus = HYD_PRESSURE_LIMIT_CAP_BOUND_UNREACHABLE;
    }
    if (qss > hardMax || (externalValid && externalCap < qss)) {
        limitStatus = HYD_PRESSURE_LIMIT_CAPACITY_INSUFFICIENT;
    }

    if (limitStatus == HYD_PRESSURE_LIMIT_NONE &&
        config->boostFlowLimitLmin > 0.0) {
        boostCap = HYD_FfPiOutputMax(config, targetPressure, error, hardMax);
    }
    if (limitStatus != HYD_PRESSURE_LIMIT_NONE) {
        boostCap = hardMax;
    }
    if (externalValid && externalCap < boostCap) {
        boostCap = externalCap;
    }

    if (state != NULL) {
        state->limitStatus = limitStatus;
        state->effectiveUpperCap = boostCap;
    }
    return boostCap;
}

void HYD_PressureController_Execute(const HYD_MotionSegment* segment,
                                    HYD_PressureControllerState* state,
                                    const HYD_PressureControllerInput* input,
                                    HYD_PressureControllerOutput* output) {
    HYD_PressureResolvedConfig config;
    HYD_REAL filteredPressure;
    HYD_REAL rawPressureRate;
    HYD_REAL filteredPressureRate;
    HYD_REAL error;
    HYD_REAL derivativeTerm;
    HYD_REAL proportionalTerm;
    HYD_REAL integralCandidate;
    HYD_REAL integralTerm;
    HYD_REAL trackingTerm;
    HYD_REAL unsaturatedOutput;
    HYD_REAL outputFlow;
    HYD_REAL trackedOutputFlow;
    HYD_REAL effectiveMax;   /* v13: FF_PI 升压包络后的有效上限（其余策略 == config.outputMax） */
    HYD_REAL ffFlow;         /* v13: 本拍实际采用的前馈流量（FF_PI 下被 Q_ff 取代，见下） */
    HYD_BOOL trackingRequested;

    if (output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->appliedStrategy = HYD_PRESSURE_CONTROLLER_NONE;

    if (segment == NULL || state == NULL || input == NULL) {
        return;
    }

    if (!state->initialized) {
        HYD_PressureController_InitState(state,
                                         input->measuredPressure,
                                         HYD_ClampReal(input->feedforwardFlow,
                                                       input->outputMin,
                                                       input->outputMax),
                                         input->timestamp);
    }

    HYD_ResolvePressureControllerConfig(segment, state, input, &config);

    /* v13：把本拍实际生效的整定结果留在 state 上，供诊断与验收断言读取。
     * 见 HYD_PressureControllerState 中三个字段的注释（RBF 黑箱的教训）。 */
    state->resolvedKp = config.kp;
    state->resolvedKi = config.ki;
    state->resolvedSteadyStateFF = config.steadyStateFF;
    state->resolvedBoostFlowLimitLmin = config.boostFlowLimitLmin;

    filteredPressure = state->previousFilteredPressure +
        config.filterAlpha * (input->measuredPressure - state->previousFilteredPressure);
    filteredPressureRate = state->previousFilteredPressureRate;
    if (config.dt > 0.0) {
        rawPressureRate = (filteredPressure - state->previousFilteredPressure) / config.dt;
        filteredPressureRate = state->previousFilteredPressureRate +
            config.derivativeFilterAlpha * (rawPressureRate - state->previousFilteredPressureRate);
    }

    error = HYD_ApplyPressureDeadband(input->targetPressure - filteredPressure,
                                      config.deadband);

    ffFlow = input->feedforwardFlow;

    /* Step 3: one resolved upper cap is shared by PI, FF_PI and RBF. */
    effectiveMax = HYD_ResolveEffectiveUpperCap(&config, input, state,
                                                input->targetPressure, error,
                                                config.outputMax);
    if (config.strategy == HYD_PRESSURE_CONTROLLER_FF_PI) {
        output->steadyStateFF = config.steadyStateFF;
        /* 【实测踩坑，勿删】FF_PI **取代**而不是叠加 legacy 名义保压流量。
         *
         * input->feedforwardFlow 来自 segment->targetFlow（出厂 defaultTargetFlow = 5.0 L/min），
         * 那是"没有对象模型时代"的占位常量，物理上根本不成立：
         *   5.0 L/min × K(200 bar/(L/min)) = 1000 bar —— 比任何工艺压力都高一个量级。
         * PI 路径靠积分慢慢把它抵消掉（并因此付出超调代价）；FF_PI 已经有物理正确的
         * Q_ff = P_set/K，再叠 5.0 就是双重前馈。
         *
         * 叠加后的实测后果（生产链路，25cc/1700rpm，目标 150 bar）：
         *   升压包络把压力卡在内漏平衡点 137 bar → 进入窄带解除区（|e|<=7.5 bar）→
         *   上限突然放开 → 5.0 的幽灵前馈把压力从 137 直接顶到 200 bar（Mp 33%）。
         * 取代后 Mp 由 33.49% 降到见 CI 实测值。 */
        ffFlow = 0.0;
    }

    trackingRequested = state->trackingRequested ||
        ((state->activeStrategy != HYD_PRESSURE_CONTROLLER_NONE) &&
         (state->activeStrategy != config.strategy));
    trackedOutputFlow = HYD_ClampReal(state->previousOutput, config.outputMin, config.outputMax);

    output->appliedStrategy = config.strategy;
    output->targetPressure = input->targetPressure;
    output->filteredPressure = filteredPressure;
    output->filteredPressureRate = filteredPressureRate;
    output->controlError = error;
    output->feedforwardFlow = ffFlow;   /* v13: FF_PI 下为 0（Q_ff 已取代 legacy 名义流量） */
    output->samplingPeriod = config.dt;
    output->adaptiveActive = config.strategySpec->adaptive;
    output->effectiveUpperCap = effectiveMax;
    output->limitStatus = state->limitStatus;
    output->requestedStrategy = config.requestedStrategy;
    output->calibrationStatus = state->calibrationStatus;
    output->dtValid = state->dtValid;

    if (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PID ||
        config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI) {
        HYD_REAL effectiveTargetPressure;
        HYD_REAL rawOutputFlow;
        HYD_BOOL needsAdaptiveReset;
        HYD_BOOL internalSaturated;

        needsAdaptiveReset =  !state->rbfInitialized || trackingRequested;

        /* 目标工况跳变 → 软复位（非全量，保留网络通用知识）。
         * 仅当已初始化且目标发生 5% 以上跳变时触发，ramp 渐进不误触发。 */
        if (state->rbfInitialized &&
            HYD_TargetOperatingPointChanged(input->targetPressure,
                                             (HYD_REAL)state->rbfPid.P_set)) {
            needsAdaptiveReset = true;
            state->softResetPending = true;
        }

        HYD_ApplyRbfPidConfig(state, &config, segment,
                              input->flowToPumpSpeedGain, input->pumpSpeedLimit);

        if (needsAdaptiveReset) {
            output->trackingApplied = true;
            if (state->softResetPending) {
                /* 软复位：保留网络，只清工况特定状态 + 增益收敛到窗口中心 */
                HYD_SoftResetRbfPidState(state, trackedOutputFlow,
                                          input->targetPressure, filteredPressure,
                                          &config);
                state->softResetPending = false;
            } else {
                /* 首次初始化 / 策略切换 → 全量复位（保留原行为） */
                HYD_SynchronizeRbfPidState(state,
                                           trackedOutputFlow,
                                           input->targetPressure,
                                           filteredPressure,
                                           &config,
                                           segment,
                                           input->flowToPumpSpeedGain,
                                           input->pumpSpeedLimit);
            }
        }

        /* v10: 泵电机反馈 → 算法（数据链最后一环）。
         * 必须放在复位之后：RBF_PID_Reset() 会 memset 句柄，
         * 把 external_flow_cap 一并清零，因此每拍都在复位后重新注入。 */
        HYD_ApplyPumpFeedbackToRbfPid(state, input, output);
        RBF_PID_SetEffectiveUpperCap(&state->rbfPid, (float)effectiveMax, true);

        effectiveTargetPressure = input->targetPressure -
            ((input->targetPressure - filteredPressure) - error);
        rawOutputFlow = (HYD_REAL)RBF_PID_Update(&state->rbfPid,
                                                 (float)effectiveTargetPressure,
                                                 (float)filteredPressure);
        internalSaturated = state->rbfPid.output_saturated;

        outputFlow = HYD_ClampReal(rawOutputFlow, config.outputMin, effectiveMax);

        /* 负流量死区：仅当压力偏差 <= -2.0 bar（超压 >= 2 bar）时才允许负流量 */
        if (config.outputMin < 0.0 && outputFlow < 0.0 && fabs(error) < 5.0) {
            outputFlow = 0.0; /* 小偏差时不使用负流量，防止0附近震荡 */
        }

        output->targetPressure = effectiveTargetPressure;
        output->feedbackFlow = rawOutputFlow - input->feedforwardFlow;
        output->unsaturatedOutputFlow = rawOutputFlow;
        output->outputFlow = outputFlow;
        output->samplingPeriod = config.samplingPeriod;
        output->adaptiveKp = (HYD_REAL)state->rbfPid.KP;
        output->adaptiveKi = (HYD_REAL)state->rbfPid.KI;
        output->adaptiveKd = (HYD_REAL)state->rbfPid.KD;
        output->adaptiveJacobian = (HYD_REAL)state->rbfPid.Jacobian;
        output->saturated =
            (config.strategy == HYD_PRESSURE_CONTROLLER_RBF_PI && internalSaturated) ||
            (outputFlow != rawOutputFlow);
        output->capBoundDemand = (rawOutputFlow > effectiveMax + 1.0e-9) ? 1 : 0;

        /* v10: 指令 vs 实测的流量跟踪误差（负值 = 泵给多了）。 */
        if (output->pumpFeedbackApplied) {
            output->flowTrackingError = outputFlow - output->actualFlow;
        }

        state->rbfPid.output_saturated = output->saturated ? true : false;
        state->rbfPid.Output = (float)outputFlow;
        state->rbfPid.u_prev = (float)outputFlow;

        state->initialized = true;
        state->trackingRequested = false;
        state->integralOutput = 0.0;
        state->previousError = error;
        state->previousFilteredPressure = filteredPressure;
        state->previousFilteredPressureRate = filteredPressureRate;
        state->previousOutput = outputFlow;
        state->previousTimestamp = input->timestamp;
        state->activeStrategy = config.strategy;
        return;
    }

    proportionalTerm = config.kp * error;

    if (config.kpHigh > 0.0 && fabs(input->targetPressure) > 0.0) {
        HYD_REAL errorRatio = fabs(error) / input->targetPressure;
        HYD_REAL fraction = HYD_ClampReal(errorRatio / config.gainBand, 0.0, 1.0);
        HYD_REAL kpEff = config.kp + fraction * (config.kpHigh - config.kp);
        proportionalTerm = kpEff * error;
    }

    derivativeTerm = 0.0;
    if (config.strategySpec->supportsDerivative && config.dt > 0.0) {
        derivativeTerm = -config.kd * filteredPressureRate;
    }

    trackingTerm = 0.0;
    integralTerm = state->integralOutput;

    if (trackingRequested) {
        output->trackingApplied = true;
        if (config.strategySpec->supportsIntegral) {
            integralTerm = HYD_ResolveTrackedIntegralOutput(segment,
                                                            input,
                                                            proportionalTerm,
                                                            derivativeTerm,
                                                            trackedOutputFlow);
        } else {
            trackingTerm = trackedOutputFlow - ffFlow - proportionalTerm - derivativeTerm;
        }
    }

    if (config.strategySpec->supportsIntegral && config.dt > 0.0 && config.ki > 0.0) {
        integralCandidate = integralTerm + config.ki * error * config.dt;
        if (config.integralLimit > 0.0) {
            integralCandidate = HYD_ClampReal(integralCandidate,
                                              -config.integralLimit,
                                              config.integralLimit);
        }

        unsaturatedOutput = config.steadyStateFF + ffFlow + proportionalTerm + integralCandidate + derivativeTerm + trackingTerm;
        if ((unsaturatedOutput >= config.outputMin && unsaturatedOutput <= effectiveMax) ||
            (unsaturatedOutput > effectiveMax && error < 0.0) ||
            (unsaturatedOutput < config.outputMin && error > 0.0)) {
            integralTerm = integralCandidate;
        }
    }

    unsaturatedOutput = config.steadyStateFF + ffFlow + proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    outputFlow = HYD_ClampReal(unsaturatedOutput, config.outputMin, effectiveMax);

    /* v13: 上限成为瓶颈的可观测化（旧 RBF 路径"K 高估 → 静默稳态欠压"的止血）。
     * 语义：控制器想要的流量超过当前有效上限。持续置位即代表"压力打不到目标的原因是
     * 输出被卡住"，上层诊断防抖后应报警（注塑机上这是欠压/缺料/短射方向）。 */
    output->capBoundDemand = (unsaturatedOutput > effectiveMax + 1.0e-9f) ? 1 : 0;

    /* 负流量死区：仅当压力偏差 <= -2.0 bar（超压 >= 2 bar）时才允许负流量 */
    if (config.outputMin < 0.0 && outputFlow < 0.0 && error > -2.0) {
        outputFlow = HYD_ClampReal(unsaturatedOutput, 0.0, effectiveMax);
    }

    output->proportionalTerm = proportionalTerm;
    output->integralTerm = integralTerm;
    output->derivativeTerm = derivativeTerm;
    output->trackingTerm = trackingTerm;
    output->feedbackFlow = proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    output->unsaturatedOutputFlow = unsaturatedOutput;
    output->outputFlow = outputFlow;
    output->saturated = (outputFlow != unsaturatedOutput);

    state->initialized = true;
    state->trackingRequested = false;
    state->integralOutput = integralTerm;
    state->previousError = error;
    state->previousFilteredPressure = filteredPressure;
    state->previousFilteredPressureRate = filteredPressureRate;
    state->previousOutput = outputFlow;
    state->previousTimestamp = input->timestamp;
    state->activeStrategy = config.strategy;
}
