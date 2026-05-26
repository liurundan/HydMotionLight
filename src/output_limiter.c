#include "output_limiter.h"
#include <math.h>
#include "hyd_config.h"

#define HYD_DEFAULT_DERATE_RATIO 0.5

static HYD_BOOL HYD_OutputLimiter_IsFinite(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

static HYD_REAL HYD_OutputLimiter_AbsReal(HYD_REAL value) {
    return (value < 0.0) ? -value : value;
}

static HYD_REAL HYD_OutputLimiter_ResolveDerateRatio(HYD_REAL configuredRatio) {
    if (HYD_OutputLimiter_IsFinite(configuredRatio) &&
        configuredRatio > 0.0 &&
        configuredRatio < 1.0) {
        return configuredRatio;
    }

    return HYD_DEFAULT_DERATE_RATIO;
}

void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output) {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_REAL ratio;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->derated = false;

    if (input == NULL) {
        return;
    }

    if (input->protectionAction == HYD_PROTECTION_ACTION_STOP) {
        return;
    }

    if (!HYD_OutputLimiter_IsFinite(input->requestedFlow) ||
        !HYD_OutputLimiter_IsFinite(input->requestedPumpSpeed) ||
        !HYD_OutputLimiter_IsFinite(input->flowToPumpSpeedGain) ||
        !HYD_OutputLimiter_IsFinite(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    commandFlow = HYD_OutputLimiter_AbsReal(input->requestedFlow);
    pumpSpeed = HYD_OutputLimiter_AbsReal(input->requestedPumpSpeed);

    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    if (pumpSpeed > input->pumpSpeedLimit) {
        pumpSpeed = input->pumpSpeedLimit;
        commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}

void HYD_OutputLimiter_ResetState(HYD_OutputLimiterState* state) {
    if (state == NULL) return;
    state->pressureBreachActive = false;
    state->pressureBreachStartTime = 0.0;
    state->pressureFaultEscalated = false;
    state->softLimitBreachActive = false;
    state->softLimitBreachStartTime = 0.0;
    state->softLimitFaultEscalated = false;
}

/* ============================================================================
 * 压力限制比例限速算法
 * ----------------------------------------------------------------------------
 * 当实际压力超过 effectiveMaxPressure 时，按超压比例线性缩减输出流量。
 *
 * 算法：
 *   overRatio = (actualPressure - maxPressure) / maxPressure
 *   scale = 1.0 - Kp * overRatio
 *   scale = clamp(scale, minScale, 1.0)
 *
 * 设计意图：
 * - Kp=3.0（保守值）：超压10%时输出降至70%，避免振荡
 * - minScale=0.1：比例限速不完全停泵，完全停泵由 FAULT/STOP 层负责
 * - 只在 actualPressure > maxPressure 时生效，否则 scale = 1.0
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_CalcPressureScale(
    HYD_REAL actualPressure,
    HYD_REAL effectiveMaxPressure)
{
    HYD_REAL overRatio;
    HYD_REAL scale;

    /* 未启用或未超压：不限制 */
    if (effectiveMaxPressure <= 0.0 || actualPressure <= effectiveMaxPressure) {
        return 1.0;
    }

    /* 计算超压比例：(实际 - 限制) / 限制 */
    overRatio = (actualPressure - effectiveMaxPressure) / effectiveMaxPressure;

    /* 比例缩减：Kp 越大响应越快，但振荡风险越高 */
    scale = 1.0 - HYD_THRESH_PRESSURE_LIMIT_KP * overRatio;

    /* 钳位到 [minScale, 1.0]，minScale > 0 保证不完全停泵 */
    if (scale < HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE) {
        scale = HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE;
    }
    if (scale > 1.0) {
        scale = 1.0;
    }

    return scale;
}

/* ============================================================================
 * 软限位减速算法
 * ----------------------------------------------------------------------------
 * 当位置进入减速带（距极限 < softLimitBandMm）时，按剩余距离比例缩减输出。
 *
 * 算法：
 *   remaining = 极限位置 - 当前位置（正向）或 当前位置 - 负向极限（负向）
 *   scale = remaining / softLimitBandMm
 *   scale = clamp(scale, 0.0, 1.0)
 *
 * 关键约束：
 * - 只限制【向极限方向】的运动，不阻止【远离极限方向】的运动（允许退回）
 * - 使用电子尺位置反馈（AXIS_REF.position）
 * - strokeMm > 0 且 softLimitBandMm > 0 时才启用
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_CalcSoftLimitScale(
    HYD_REAL actualPosition,
    HYD_REAL strokeMm,
    HYD_REAL softLimitRetractMm,
    HYD_REAL softLimitBandMm,
    HYD_MotionDirection direction)
{
    HYD_REAL remaining;
    HYD_REAL scale;

    /* 未启用软限位：不限制 */
    if (strokeMm <= 0.0 || softLimitBandMm <= 0.0) {
        return 1.0;
    }

    /* 正向运动（EXTEND）：检查是否接近 strokeMm */
    if (direction == HYD_DIRECTION_EXTEND) {
        remaining = strokeMm - actualPosition;
        if (remaining < softLimitBandMm) {
            /* 进入减速带：剩余距离越小，scale 越小 */
            scale = remaining / softLimitBandMm;
            if (scale < 0.0) scale = 0.0;
            if (scale > 1.0) scale = 1.0;
            return scale;
        }
    }

    /* 负向运动（RETRACT）：检查是否接近 softLimitRetractMm */
    if (direction == HYD_DIRECTION_RETRACT) {
        remaining = actualPosition - softLimitRetractMm;
        if (remaining < softLimitBandMm) {
            /* 进入减速带：剩余距离越小，scale 越小 */
            scale = remaining / softLimitBandMm;
            if (scale < 0.0) scale = 0.0;
            if (scale > 1.0) scale = 1.0;
            return scale;
        }
    }

    /* 未进入减速带或方向不匹配：不限制 */
    return 1.0;
}

/* 压力限制诊断升级：debounce → WARNING → FAULT
 * 返回当前应报告的诊断码（NONE 表示无需报告） */
static HYD_DiagnosticCode HYD_OutputLimiter_UpdatePressureDiag(
    HYD_REAL pressureScale,
    HYD_TIME currentTime,
    HYD_OutputLimiterState* state)
{
    HYD_TIME elapsed;

    /* 已升级为 FAULT：锁定不回退 */
    if (state->pressureFaultEscalated) {
        return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
    }

    if (pressureScale < 1.0) {
        /* 超压中 */
        if (!state->pressureBreachActive) {
            /* 首次进入超压：记录起始时间 */
            state->pressureBreachActive = true;
            state->pressureBreachStartTime = currentTime;
        }
        elapsed = currentTime - state->pressureBreachStartTime;

        /* debounce 通过后报 WARNING */
        if (elapsed >= HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S) {
            /* 检查是否升级为 FAULT */
            if (elapsed >= (HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S +
                           HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S)) {
                state->pressureFaultEscalated = true;
                return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
            }
            return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT;
        }
    } else {
        /* 压力恢复正常：重置计时（但不重置 FAULT 锁定） */
        state->pressureBreachActive = false;
    }

    return HYD_DIAG_CODE_NONE;
}

/* 软限位诊断升级：到达极限 → WARNING → FAULT
 * 返回当前应报告的诊断码 */
static HYD_DiagnosticCode HYD_OutputLimiter_UpdateSoftLimitDiag(
    HYD_REAL softLimitScale,
    HYD_TIME currentTime,
    HYD_OutputLimiterState* state)
{
    HYD_TIME elapsed;

    /* 已升级为 FAULT：锁定 */
    if (state->softLimitFaultEscalated) {
        return HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
    }

    /* scale <= 0 表示已到达或超出极限 */
    if (softLimitScale <= 0.0) {
        if (!state->softLimitBreachActive) {
            state->softLimitBreachActive = true;
            state->softLimitBreachStartTime = currentTime;
        }
        elapsed = currentTime - state->softLimitBreachStartTime;

        if (elapsed >= HYD_THRESH_SOFT_LIMIT_FAULT_DEBOUNCE_S) {
            state->softLimitFaultEscalated = true;
            return HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        }
        return HYD_DIAG_CODE_SOFT_LIMIT_REACHED;
    } else if (softLimitScale < 1.0) {
        /* 在减速带内但未到极限：报 WARNING，不计时升级 */
        state->softLimitBreachActive = false;
        return HYD_DIAG_CODE_SOFT_LIMIT_REACHED;
    } else {
        /* 正常区域：重置 */
        state->softLimitBreachActive = false;
        return HYD_DIAG_CODE_NONE;
    }
}

void HYD_OutputLimiter_ExecuteWithProtection(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_OutputLimiterOutput* output)
{
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_REAL ratio;
    HYD_REAL pressureScale;
    HYD_REAL softLimitScale;
    HYD_REAL finalScale;
    HYD_DiagnosticCode pressureDiag;
    HYD_DiagnosticCode softLimitDiag;

    if (output == NULL) return;

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->derated = false;
    output->pressureLimitActive = false;
    output->softLimitActive = false;
    output->diagnosticCode = HYD_DIAG_CODE_NONE;

    if (input == NULL) return;

    /* --- STOP 优先：立即归零 --- */
    if (input->protectionAction == HYD_PROTECTION_ACTION_STOP) {
        return;
    }

    /* --- 输入有效性检查 --- */
    if (!HYD_OutputLimiter_IsFinite(input->requestedFlow) ||
        !HYD_OutputLimiter_IsFinite(input->requestedPumpSpeed) ||
        !HYD_OutputLimiter_IsFinite(input->flowToPumpSpeedGain) ||
        !HYD_OutputLimiter_IsFinite(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    commandFlow = HYD_OutputLimiter_AbsReal(input->requestedFlow);
    pumpSpeed = HYD_OutputLimiter_AbsReal(input->requestedPumpSpeed);

    /* --- 1. 现有 DERATE 逻辑（pressureCeiling 等触发） --- */
    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    /* --- 2. 计算压力限制 scale（独立于软限位） --- */
    pressureScale = HYD_OutputLimiter_CalcPressureScale(
        input->actualPressure, input->effectiveMaxPressure);

    /* --- 3. 计算软限位 scale（独立于压力限制） --- */
    softLimitScale = HYD_OutputLimiter_CalcSoftLimitScale(
        input->actualPosition, input->strokeMm,
        input->softLimitRetractMm, input->softLimitBandMm,
        input->direction);

    /* --- 4. 取两者较小值作为最终 scale（不叠加相乘） ---
     * 设计决策：两种保护独立评估，取更严格的那个生效。
     * 例：pressureScale=0.7, softLimitScale=0.5 → finalScale=0.5 */
    finalScale = (pressureScale < softLimitScale) ? pressureScale : softLimitScale;

    /* --- 5. 应用 finalScale --- */
    if (finalScale < 1.0) {
        commandFlow *= finalScale;
        pumpSpeed *= finalScale;

        if (pressureScale <= softLimitScale) {
            output->pressureLimitActive = true;
        }
        if (softLimitScale <= pressureScale) {
            output->softLimitActive = true;
        }
        output->derated = true;
    }

    /* --- 6. 诊断升级（需要 state） --- */
    if (state != NULL) {
        pressureDiag = HYD_OutputLimiter_UpdatePressureDiag(
            pressureScale, input->currentTime, state);
        softLimitDiag = HYD_OutputLimiter_UpdateSoftLimitDiag(
            softLimitScale, input->currentTime, state);

        /* FAULT 优先于 WARNING */
        if (pressureDiag == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT) {
            output->diagnosticCode = HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
        } else if (softLimitDiag == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED) {
            output->diagnosticCode = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        } else if (pressureDiag != HYD_DIAG_CODE_NONE) {
            output->diagnosticCode = pressureDiag;
        } else {
            output->diagnosticCode = softLimitDiag;
        }
    }

    /* --- 7. pumpSpeedLimit 硬裁剪（最终兜底） --- */
    if (pumpSpeed > input->pumpSpeedLimit) {
        pumpSpeed = input->pumpSpeedLimit;
        commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}
