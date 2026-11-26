#include "output_limiter.h"
#include <math.h>
#include "hyd_config.h"

#define HYD_DEFAULT_DERATE_RATIO 0.5

static HYD_BOOL HYD_OutputLimiter_IsFinite(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

/* Resolve one lower bound for both flow and speed.  The strategy bound is
 * authoritative when supplied; the pump reverse capability remains the hard
 * safety boundary.  The legacy flag is retained only for old callers that
 * leave minimumFlow at zero. */
static void HYD_OutputLimiter_GetLimits(
    const HYD_OutputLimiterInput* input,
    HYD_REAL* minFlow,
    HYD_REAL* minSpeed)
{
    HYD_REAL hardwareMinFlow = 0.0;
    HYD_REAL strategyMinFlow = input->minimumFlow;

    if (input->flowToPumpSpeedGain > 0.0 && input->pumpSpeedLimit > 0.0) {
        hardwareMinFlow = -input->pumpSpeedLimit *
                          HYD_PUMP_NEGATIVE_SPEED_RATIO /
                          input->flowToPumpSpeedGain;
    }

    if (!HYD_OutputLimiter_IsFinite(strategyMinFlow)) {
        strategyMinFlow = 0.0;
    } else if (strategyMinFlow == 0.0 && input->allowNegativeFlow) {
        /* Preserve the pre-minimumFlow ABI for existing callers. */
        strategyMinFlow = hardwareMinFlow;
    }

    *minFlow = (strategyMinFlow > hardwareMinFlow)
        ? strategyMinFlow : hardwareMinFlow;
    *minSpeed = *minFlow * input->flowToPumpSpeedGain;
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

    HYD_REAL minFlow, minSpeed;
    HYD_OutputLimiter_GetLimits(input, &minFlow, &minSpeed);
    commandFlow = (input->requestedFlow > minFlow) ? input->requestedFlow : minFlow;
    pumpSpeed   = (input->requestedPumpSpeed > minSpeed) ? input->requestedPumpSpeed : minSpeed;

    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    /* --- pumpSpeedLimit 硬裁剪（最终兜底） --- */
    {
        HYD_REAL lo = minSpeed;
        HYD_REAL hi = input->pumpSpeedLimit;
        if (pumpSpeed > hi) {
            pumpSpeed = hi;
            commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
        } else if (pumpSpeed < lo) {
            pumpSpeed = lo;
            commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
        }
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}

void HYD_OutputLimiter_ResetState(HYD_OutputLimiterState* state) {
    if (state == NULL) return;
    state->pressureLimitActive = false;
    state->pressureLimitIntegral = 0.0f;
    state->softLimitBreachActive = false;
    state->softLimitBreachStartTime = 0.0;
    state->softLimitFaultEscalated = false;
}

/* ============================================================================
 * 速度段限压 PI（spec 2026-11-26 §4.1）
 * ----------------------------------------------------------------------------
 * 速度链路流量 Q_v 与限压 PI 输出 Q_p 取小：
 *   e   = Plim − P（P 不滤波）
 *   Q_p = Kp·e + I
 *   选中（Q_p < Q_v）：输出 Q_p，然后 I ← clamp(I + Ki·e·dt, 0, Qmax)
 *   未选中：输出 Q_v；下一拍 I 取上一拍实际执行流量（外部复位反馈）
 * 压力越接近 Plim，Q_p 比上一拍流量高出 Kp·e 越少，到 Plim 时接管；
 * 负载下降后 Q_p 连续升过 Q_v，自动回到速度控制。
 * actualPressure 与 effectiveMaxPressure 同为 bar。
 * 返回本拍流量上限；*selected 为 true 表示压力环被选中。
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_ApplyPressureLimit(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_REAL velocityFlow,
    HYD_REAL maxFlow,
    HYD_BOOL* selected)
{
    HYD_REAL kp;
    HYD_REAL ki;
    HYD_REAL dt;
    HYD_REAL error;
    HYD_REAL integral;
    HYD_REAL pressureFlow;

    *selected = false;
    if (!(input->effectiveMaxPressure > 0.0f) ||
        !HYD_OutputLimiter_IsFinite(input->effectiveMaxPressure) ||
        !HYD_OutputLimiter_IsFinite(input->actualPressure)) {
        state->pressureLimitActive = false;
        return velocityFlow;
    }

    kp = (input->pressureLimitKp > 0.0f &&
          HYD_OutputLimiter_IsFinite(input->pressureLimitKp))
        ? input->pressureLimitKp : HYD_THRESH_PRESSURE_LIMIT_PI_KP;
    ki = (input->pressureLimitKi > 0.0f &&
          HYD_OutputLimiter_IsFinite(input->pressureLimitKi))
        ? input->pressureLimitKi : HYD_THRESH_PRESSURE_LIMIT_PI_KI;
    dt = input->deltaTime;
    if (!HYD_OutputLimiter_IsFinite(dt) || dt <= 0.0f) {
        dt = 0.001f;
    } else if (dt > 0.1f) {
        dt = 0.1f;
    }

    if (state->pressureLimitActive) {
        integral = state->pressureLimitIntegral;
    } else {
        integral = HYD_OutputLimiter_IsFinite(input->previousCommandFlow)
            ? input->previousCommandFlow : 0.0f;
        if (integral < 0.0f) {
            integral = 0.0f;
        } else if (integral > maxFlow) {
            integral = maxFlow;
        }
    }

    error = input->effectiveMaxPressure - input->actualPressure;
    pressureFlow = kp * error + integral;
    if (pressureFlow < velocityFlow) {
        *selected = true;
        integral += ki * error * dt;
        if (integral < 0.0f) {
            integral = 0.0f;
        } else if (integral > maxFlow) {
            integral = maxFlow;
        }
    }

    state->pressureLimitActive = *selected;
    state->pressureLimitIntegral = integral;
    return *selected ? pressureFlow : velocityFlow;
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
    HYD_REAL softLimitScale;
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

    HYD_REAL minFlow, minSpeed;
    HYD_OutputLimiter_GetLimits(input, &minFlow, &minSpeed);
    commandFlow = (input->requestedFlow > minFlow) ? input->requestedFlow : minFlow;
    pumpSpeed   = (input->requestedPumpSpeed > minSpeed) ? input->requestedPumpSpeed : minSpeed;
    /* --- 1. 现有 DERATE 逻辑（pressureCeiling 等触发） --- */
    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    /* --- 2. 软限位减速 --- */
    softLimitScale = HYD_OutputLimiter_CalcSoftLimitScale(
        input->actualPosition, input->strokeMm,
        input->softLimitRetractMm, input->softLimitBandMm,
        input->direction);
    if (softLimitScale < 1.0f) {
        commandFlow *= softLimitScale;
        pumpSpeed *= softLimitScale;
        output->softLimitActive = true;
        output->derated = true;
    }

    /* --- 3. 限压 PI 与速度链路流量取小（spec 2026-11-26 §4.1） --- */
    {
        HYD_OutputLimiterState scratch;
        HYD_OutputLimiterState* piState = state;
        HYD_BOOL pressureSelected;
        HYD_REAL limitedFlow;

        if (piState == NULL) {
            HYD_OutputLimiter_ResetState(&scratch);
            piState = &scratch;
        }
        limitedFlow = HYD_OutputLimiter_ApplyPressureLimit(
            input, piState, commandFlow,
            input->pumpSpeedLimit / input->flowToPumpSpeedGain,
            &pressureSelected);
        if (pressureSelected) {
            commandFlow = (limitedFlow > minFlow) ? limitedFlow : minFlow;
            pumpSpeed = commandFlow * input->flowToPumpSpeedGain;
            output->pressureLimitActive = true;
            output->softLimitActive = false;
            output->derated = true;
        }
    }

    /* --- 4. 软限位诊断；段压力限幅本身不是库级故障 --- */
    if (state != NULL) {
        softLimitDiag = HYD_OutputLimiter_UpdateSoftLimitDiag(
            softLimitScale, input->currentTime, state);
        output->diagnosticCode = softLimitDiag;
    }

    /* --- 5. pumpSpeedLimit 硬裁剪（最终兜底） --- */
    {
        HYD_REAL lo = minSpeed;
        HYD_REAL hi = input->pumpSpeedLimit;
        if (pumpSpeed > hi) {
            pumpSpeed = hi;
            commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
        } else if (pumpSpeed < lo) {
            pumpSpeed = lo;
            commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
        }
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}
