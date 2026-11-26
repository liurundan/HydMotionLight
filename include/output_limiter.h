#ifndef HYD_OUTPUT_LIMITER_H
#define HYD_OUTPUT_LIMITER_H

#include "common_types.h"

typedef struct {
    /* --- 原有字段 --- */
    HYD_REAL requestedFlow;
    HYD_REAL requestedPumpSpeed;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_ProtectionAction protectionAction;
    HYD_REAL derateRatio;

    /* Resolved strategy lower bound [L/min].  The effective lower bound is
     * max(minimumFlow, hardware reverse-flow capability). */
    HYD_REAL minimumFlow;

    /* Legacy compatibility flag.  New callers must provide minimumFlow; this
     * flag is used only when minimumFlow is zero and no canonical negative
     * policy was supplied by an older caller. */
    HYD_BOOL allowNegativeFlow;

    /* --- 压力限制（新增） --- */
    HYD_REAL actualPressure;         /* 当前压力反馈 [bar] */
    HYD_REAL effectiveMaxPressure;   /* 生效的最大压力限制 [bar], 0 = 不启用 */
    HYD_REAL pressureLimitKp;        /* 限压 PI 比例增益 [L/min/bar]，<= 0 用默认值 */
    HYD_REAL pressureLimitKi;        /* 限压 PI 积分增益 [L/min/(bar·s)]，<= 0 用默认值 */
    HYD_REAL previousCommandFlow;    /* 上一拍实际执行流量 [L/min]，PI 未选中时的积分跟踪源 */
    HYD_REAL deltaTime;              /* 本拍周期 [s]，非正或非有限时按 1 ms */

    /* --- 软限位（新增） --- */
    HYD_REAL actualPosition;         /* 当前位置反馈 [mm]（电子尺） */
    HYD_REAL strokeMm;               /* 正向极限 [mm], 0 = 不启用 */
    HYD_REAL softLimitRetractMm;     /* 负向极限 [mm] */
    HYD_REAL softLimitBandMm;        /* 减速带宽度 [mm], 0 = 不启用 */
    HYD_MotionDirection direction;   /* 当前运动方向 */

    /* --- 时间（新增，用于 debounce/升级计时） --- */
    HYD_TIME currentTime;            /* 当前时间戳 [s] */
} HYD_OutputLimiterInput;

typedef struct {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_BOOL derated;

    /* --- 保护状态输出（新增） --- */
    HYD_BOOL pressureLimitActive;    /* 限压 PI 本拍被选中（Q_p < Q_v） */
    HYD_BOOL softLimitActive;        /* 软限位减速正在生效 */
    HYD_DiagnosticCode diagnosticCode; /* 最高优先级的保护诊断码, NONE = 无 */
} HYD_OutputLimiterOutput;

/* output_limiter 内部持久状态（由调用方持有，每段开始时 reset） */
typedef struct {
    /* 限压 PI：上一拍是否被选中、积分 [L/min]。位置/速度段不升级为库级故障。 */
    HYD_BOOL pressureLimitActive;
    HYD_REAL pressureLimitIntegral;

    /* 软限位计时 */
    HYD_BOOL softLimitBreachActive;    /* 当前是否处于越界状态 */
    HYD_TIME softLimitBreachStartTime; /* 越界开始时间 */
    HYD_BOOL softLimitFaultEscalated;  /* 已升级为 FAULT */
} HYD_OutputLimiterState;

/* 原有接口保留（向后兼容，不含保护逻辑） */
void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output);

/* 带状态的扩展版本（限压 PI 取小 + 软限位诊断） */
void HYD_OutputLimiter_ExecuteWithProtection(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_OutputLimiterOutput* output);

/* 重置保护状态（每段开始时调用） */
void HYD_OutputLimiter_ResetState(HYD_OutputLimiterState* state);

#endif /* HYD_OUTPUT_LIMITER_H */
