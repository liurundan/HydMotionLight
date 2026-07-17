#include "diagnostics_criteria.h"
#include <math.h>
#include <string.h>

void HYD_DiagnosticCriteria_InitState(HYD_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));

    /* 初始化告警到故障升级状态 */
    state->warningActive = false;
    state->warningStartTime = 0.0;
    state->faultEscalated = false;
}

void HYD_DiagnosticCriteria_ResetState(HYD_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));

    /* 重置告警到故障升级状态 */
    HYD_DiagnosticCriteria_ResetFaultEscalation(state);
}

HYD_BOOL HYD_IsStartupSuppressActive(HYD_TIME segmentElapsedTime, HYD_TIME suppressTime) {
    if (suppressTime <= 0.0) {
        return false;
    }

    return (segmentElapsedTime < suppressTime) ? true : false;
}

HYD_BOOL HYD_IsSwitchSuppressActive(HYD_BOOL isSwitchPhase, HYD_TIME segmentElapsedTime, HYD_TIME suppressTime) {
    if (!isSwitchPhase || suppressTime <= 0.0) {
        return false;
    }

    return (segmentElapsedTime < suppressTime) ? true : false;
}

HYD_REAL HYD_CalculateLoopBuildFactor(HYD_TIME loopBuildTime, HYD_TIME suppressTime) {
    if (suppressTime <= 0.0) {
        return 1.0;
    }

    if (loopBuildTime >= suppressTime) {
        return 1.0;
    }

    /* 线性递增：0.0 ~ 1.0 */
    return loopBuildTime / suppressTime;
}

static HYD_BOOL HYD_DiagnosticCriteria_IsFiniteReal(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

static HYD_BOOL HYD_DiagnosticCriteria_CheckSuppressCondition(const HYD_DiagnosticCriteria* criteria,
                                                               HYD_TIME segmentElapsedTime,
                                                               HYD_BOOL isStartupPhase,
                                                               HYD_BOOL isSwitchPhase,
                                                               HYD_SuppressType* suppressType,
                                                               HYD_TIME* suppressTime) {
    *suppressType = HYD_SUPPRESS_NONE;
    *suppressTime = 0.0;

    if (criteria == NULL) {
        return false;
    }

    /* 检查启动阶段抑制 */
    if (criteria->enableStartupSuppress && isStartupPhase) {
        if (HYD_IsStartupSuppressActive(segmentElapsedTime, criteria->startupSuppressTime)) {
            *suppressType = HYD_SUPPRESS_STARTUP;
            *suppressTime = criteria->startupSuppressTime;
            return true;
        }
    }

    /* 检查切段阶段抑制 */
    if (criteria->enableSwitchSuppress && isSwitchPhase) {
        if (HYD_IsSwitchSuppressActive(isSwitchPhase, segmentElapsedTime, criteria->switchSuppressTime)) {
            *suppressType = HYD_SUPPRESS_SWITCH;
            *suppressTime = criteria->switchSuppressTime;
            return true;
        }
    }

    /* 闭环建立抑制不是简单的抑制，而是降低敏感度，在调用方处理 */
    if (criteria->enableLoopBuildSuppress) {
        /* 这里不返回true，由调用方计算降低的阈值 */
    }

    return false;
}

static HYD_REAL HYD_DiagnosticCriteria_CalculateEffectiveThreshold(const HYD_DiagnosticCriteria* criteria,
                                                                    HYD_TIME loopBuildTime,
                                                                    HYD_BOOL errorActive,
                                                                    const HYD_DiagnosticCriteriaState* state) {
    HYD_REAL effectiveThreshold = criteria->baseThreshold;
    HYD_REAL loopBuildFactor = 1.0;

    if (criteria == NULL) {
        return 0.0;
    }

    /* 应用闭环建立因子（降低敏感度）
     * 仅当误差实际存在时才降低阈值；若误差不存在（errorActive=false），
     * 不应将阈值降为0，否则微小数值噪声就会触发诊断。 */
    if (criteria->enableLoopBuildSuppress && errorActive) {
        loopBuildFactor = HYD_CalculateLoopBuildFactor(loopBuildTime, criteria->loopBuildSuppressTime);
        effectiveThreshold = effectiveThreshold * loopBuildFactor;
    }

    /* 应用滞回逻辑 */
    if (state->hysteresisActive && criteria->hysteresisRatio > 0.0) {
        /* 滞回激活时，降低触发阈值，防止诊断抖动 */
        effectiveThreshold = effectiveThreshold * (1.0 - criteria->hysteresisRatio);
    }

    return effectiveThreshold;
}

static HYD_BOOL HYD_DiagnosticCriteria_CheckDebounce(const HYD_DiagnosticCriteria* criteria,
                                                      HYD_DiagnosticCriteriaState* state,
                                                      HYD_BOOL errorExceedsThreshold,
                                                      HYD_TIME currentTime,
                                                      HYD_TIME* triggerTime) {
    if (criteria == NULL || state == NULL) {
        return false;
    }

    /* 无去抖动要求，直接返回 */
    if (criteria->debounceTime <= 0.0) {
        *triggerTime = currentTime;
        return errorExceedsThreshold;
    }

    /* 误差超过阈值 */
    if (errorExceedsThreshold) {
        if (!state->lastTriggered) {
            /* 第一次超过阈值，记录开始时间 */
            state->triggerStartTime = currentTime;
            state->lastTriggered = true;
        }

        /* 检查是否满足持续时间 */
        if ((currentTime - state->triggerStartTime) >= criteria->debounceTime) {
            *triggerTime = currentTime;
            return true;
        }

        return false;
    } else {
        /* 误差未超过阈值，重置状态 */
        state->lastTriggered = false;
        state->triggerStartTime = 0.0;

        /* 如果之前有滞回激活，现在误差恢复，可以清除滞回 */
        if (state->hysteresisActive) {
            state->hysteresisActive = false;
        }

        return false;
    }
}

HYD_BOOL HYD_DiagnosticCriteria_CheckPressure(HYD_DiagnosticResult* result,
                                              const HYD_ErrorMonitor* monitor,
                                              const HYD_DiagnosticCriteria* criteria,
                                              HYD_DiagnosticCriteriaState* state,
                                              HYD_TIME currentTime,
                                              HYD_TIME segmentElapsedTime,
                                              HYD_BOOL isStartupPhase,
                                              HYD_BOOL isSwitchPhase) {
    HYD_REAL absoluteError;
    HYD_REAL effectiveThreshold;
    HYD_BOOL errorExceedsThreshold;
    HYD_SuppressType suppressType;
    HYD_TIME suppressTime;
    HYD_BOOL triggered;
    HYD_TIME loopBuildTime = 0.0;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->pressureError);
    if (!HYD_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件 */
    if (HYD_DiagnosticCriteria_CheckSuppressCondition(criteria,
                                                       segmentElapsedTime,
                                                       isStartupPhase,
                                                       isSwitchPhase,
                                                       &suppressType,
                                                       &suppressTime)) {
        result->suppressType = suppressType;
        result->suppressTime = suppressTime;
        result->triggered = false;
        return false;
    }

    /* 计算闭环建立时间 */
    if (monitor->pressureErrorActive) {
        loopBuildTime = monitor->pressureErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HYD_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime, monitor->pressureErrorActive, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HYD_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        /* 激活滞回 */
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

HYD_BOOL HYD_DiagnosticCriteria_CheckFlow(HYD_DiagnosticResult* result,
                                           const HYD_ErrorMonitor* monitor,
                                           const HYD_DiagnosticCriteria* criteria,
                                           HYD_DiagnosticCriteriaState* state,
                                           HYD_TIME currentTime,
                                           HYD_TIME segmentElapsedTime,
                                           HYD_BOOL isStartupPhase,
                                           HYD_BOOL isSwitchPhase) {
    HYD_REAL absoluteError;
    HYD_REAL effectiveThreshold;
    HYD_BOOL errorExceedsThreshold;
    HYD_SuppressType suppressType;
    HYD_TIME suppressTime;
    HYD_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->flowError);
    if (!HYD_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件 */
    if (HYD_DiagnosticCriteria_CheckSuppressCondition(criteria,
                                                       segmentElapsedTime,
                                                       isStartupPhase,
                                                       isSwitchPhase,
                                                       &suppressType,
                                                       &suppressTime)) {
        result->suppressType = suppressType;
        result->suppressTime = suppressTime;
        result->triggered = false;
        return false;
    }

    /* 计算闭环建立时间 */
    HYD_TIME loopBuildTime_flow = 0.0;
    if (monitor->flowErrorActive) {
        loopBuildTime_flow = monitor->flowErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HYD_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_flow, monitor->flowErrorActive, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HYD_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

HYD_BOOL HYD_DiagnosticCriteria_CheckVelocity(HYD_DiagnosticResult* result,
                                                const HYD_ErrorMonitor* monitor,
                                                const HYD_DiagnosticCriteria* criteria,
                                                HYD_DiagnosticCriteriaState* state,
                                                HYD_TIME currentTime,
                                                HYD_TIME segmentElapsedTime,
                                                HYD_BOOL isStartupPhase,
                                                HYD_BOOL isSwitchPhase) {
    HYD_REAL absoluteError;
    HYD_REAL effectiveThreshold;
    HYD_BOOL errorExceedsThreshold;
    HYD_SuppressType suppressType;
    HYD_TIME suppressTime;
    HYD_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->velocityError);
    if (!HYD_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件（速度检查启动和切换阶段抑制） */
    if (HYD_DiagnosticCriteria_CheckSuppressCondition(criteria,
                                                       segmentElapsedTime,
                                                       isStartupPhase,
                                                       isSwitchPhase,
                                                       &suppressType,
                                                       &suppressTime)) {
        result->suppressType = suppressType;
        result->suppressTime = suppressTime;
        result->triggered = false;
        return false;
    }

    /* 计算闭环建立时间 */
    HYD_TIME loopBuildTime_velocity = 0.0;
    if (monitor->velocityErrorActive) {
        loopBuildTime_velocity = monitor->velocityErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HYD_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_velocity, monitor->velocityErrorActive, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HYD_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

HYD_BOOL HYD_DiagnosticCriteria_CheckPosition(HYD_DiagnosticResult* result,
                                               const HYD_ErrorMonitor* monitor,
                                               const HYD_DiagnosticCriteria* criteria,
                                               HYD_DiagnosticCriteriaState* state,
                                               HYD_TIME currentTime,
                                               HYD_TIME segmentElapsedTime,
                                               HYD_BOOL isStartupPhase,
                                               HYD_BOOL isSwitchPhase) {
    HYD_REAL absoluteError;
    HYD_REAL effectiveThreshold;
    HYD_BOOL errorExceedsThreshold;
    HYD_SuppressType suppressType;
    HYD_TIME suppressTime;
    HYD_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->positionError);
    if (!HYD_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件（位置检查启动和切换阶段抑制） */
    if (HYD_DiagnosticCriteria_CheckSuppressCondition(criteria,
                                                       segmentElapsedTime,
                                                       isStartupPhase,
                                                       isSwitchPhase,
                                                       &suppressType,
                                                       &suppressTime)) {
        result->suppressType = suppressType;
        result->suppressTime = suppressTime;
        result->triggered = false;
        return false;
    }

    /* 计算闭环建立时间 */
    HYD_TIME loopBuildTime_position = 0.0;
    if (monitor->positionErrorActive) {
        loopBuildTime_position = monitor->positionErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HYD_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_position, monitor->positionErrorActive, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HYD_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

void HYD_DiagnosticCriteria_CreateDefaultPressureCriteria(HYD_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 0.5;        /* 默认0.5 bar */
    criteria->debounceTime = 0.1;         /* 默认100ms去抖动 */
    criteria->hysteresisRatio = 0.2;      /* 默认20%滞回 */

    /* 启用启动抑制（默认500ms） */
    criteria->enableStartupSuppress = true;
    criteria->startupSuppressTime = 0.5;

    /* 启用切段抑制（默认300ms） */
    criteria->enableSwitchSuppress = true;
    criteria->switchSuppressTime = 0.3;

    /* 启用闭环建立抑制（默认200ms） */
    criteria->enableLoopBuildSuppress = true;
    criteria->loopBuildSuppressTime = 0.2;

    /* 启用告警到故障升级（默认2秒升级） */
    criteria->enableFaultEscalation = true;
    criteria->faultEscalationTime = 2.0;
    criteria->faultCode = HYD_DIAG_CODE_OVER_PRESSURE;

    criteria->diagnosticCode = HYD_DIAG_CODE_OVER_PRESSURE;
    criteria->severity = HYD_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HYD_PROTECTION_ACTION_WARNING;
}

void HYD_DiagnosticCriteria_CreateDefaultFlowCriteria(HYD_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 2.0;        /* 默认2.0 L/min */
    criteria->debounceTime = 0.1;         /* 默认100ms去抖动 */
    criteria->hysteresisRatio = 0.2;      /* 默认20%滞回 */

    /* 启用启动抑制（默认500ms） */
    criteria->enableStartupSuppress = true;
    criteria->startupSuppressTime = 0.5;

    /* 启用切段抑制（默认300ms） */
    criteria->enableSwitchSuppress = true;
    criteria->switchSuppressTime = 0.3;

    /* 启用闭环建立抑制（默认300ms） */
    criteria->enableLoopBuildSuppress = true;
    criteria->loopBuildSuppressTime = 0.3;

    /* 启用告警到故障升级（默认3秒升级） */
    criteria->enableFaultEscalation = true;
    criteria->faultEscalationTime = 3.0;
    criteria->faultCode = HYD_DIAG_CODE_FLOW_DEVIATION;

    criteria->diagnosticCode = HYD_DIAG_CODE_FLOW_DEVIATION;
    criteria->severity = HYD_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HYD_PROTECTION_ACTION_DERATE;
}

void HYD_DiagnosticCriteria_CreateDefaultVelocityCriteria(HYD_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 10.0;        /* 默认10.0 mm/s */
    criteria->debounceTime = 0.1;         /* 默认100ms去抖动 */
    criteria->hysteresisRatio = 0.2;      /* 默认20%滞回 */

    /* 启用启动抑制（默认300ms） */
    criteria->enableStartupSuppress = true;
    criteria->startupSuppressTime = 0.3;

    /* 启用切段抑制（默认300ms） */
    criteria->enableSwitchSuppress = true;
    criteria->switchSuppressTime = 0.3;

    /* 启用闭环建立抑制（默认200ms） */
    criteria->enableLoopBuildSuppress = true;
    criteria->loopBuildSuppressTime = 0.2;

    /* 启用告警到故障升级（默认2秒升级） */
    criteria->enableFaultEscalation = true;
    criteria->faultEscalationTime = 2.0;
    criteria->faultCode = HYD_DIAG_CODE_VELOCITY_DEVIATION;

    criteria->diagnosticCode = HYD_DIAG_CODE_VELOCITY_DEVIATION;
    criteria->severity = HYD_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HYD_PROTECTION_ACTION_WARNING;
}

void HYD_DiagnosticCriteria_CreateDefaultPositionCriteria(HYD_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 1.0;        /* 默认1.0 mm */
    criteria->debounceTime = 0.2;         /* 默认200ms去抖动 */
    criteria->hysteresisRatio = 0.2;      /* 默认20%滞回 */

    /* 启用启动抑制（默认500ms） */
    criteria->enableStartupSuppress = true;
    criteria->startupSuppressTime = 0.5;

    /* 启用切段抑制（默认300ms） */
    criteria->enableSwitchSuppress = true;
    criteria->switchSuppressTime = 0.3;

    /* 启用闭环建立抑制（默认300ms） */
    criteria->enableLoopBuildSuppress = true;
    criteria->loopBuildSuppressTime = 0.3;

    /* 启用告警到故障升级（默认2秒升级） */
    criteria->enableFaultEscalation = true;
    criteria->faultEscalationTime = 2.0;
    criteria->faultCode = HYD_DIAG_CODE_POSITION_DEVIATION;

    criteria->diagnosticCode = HYD_DIAG_CODE_POSITION_DEVIATION;
    criteria->severity = HYD_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HYD_PROTECTION_ACTION_WARNING;
}

HYD_BOOL HYD_DiagnosticCriteria_CheckFaultEscalation(HYD_DiagnosticResult* result,
                                                      const HYD_DiagnosticCriteria* criteria,
                                                      HYD_DiagnosticCriteriaState* state,
                                                      HYD_TIME currentTime) {
    if (result == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    /* 未启用升级，直接返回 */
    if (!criteria->enableFaultEscalation) {
        return false;
    }

    /* 无升级时间配置，直接返回 */
    if (criteria->faultEscalationTime <= 0.0) {
        return false;
    }

    /* 如果 WARNING 不再触发，清除升级状态 */
    if (!result->triggered && state->warningActive) {
        state->warningActive = false;
        state->warningStartTime = 0.0;
        return false;
    }

    /* 只有 WARNING 级别才能升级 */
    if (result->severity != HYD_DIAG_SEVERITY_WARNING) {
        return false;
    }

    /* 已经是 FAULT 级别，无需升级 */
    if (state->faultEscalated) {
        return false;
    }

    /* 触发 WARNING */
    if (result->triggered && !state->warningActive) {
        state->warningActive = true;
        state->warningStartTime = currentTime;
        return false;
    }

    /* WARNING 持续存在，检查是否升级 */
    if (state->warningActive) {
        HYD_TIME warningDuration = currentTime - state->warningStartTime;

        /* 超过升级时间，升级为 FAULT */
        if (warningDuration >= criteria->faultEscalationTime) {
            state->faultEscalated = true;

            /* 升级诊断结果 */
            result->severity = HYD_DIAG_SEVERITY_FAULT;
            result->action = HYD_PROTECTION_ACTION_STOP;

            /* 如果配置了故障码，更新故障码 */
            if (criteria->faultCode != HYD_DIAG_CODE_NONE) {
                result->code = criteria->faultCode;
            }

            return true;
        }
    }

    return false;
}

void HYD_DiagnosticCriteria_ResetFaultEscalation(HYD_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    state->warningActive = false;
    state->warningStartTime = 0.0;
    state->faultEscalated = false;
}

HYD_BOOL HYD_DiagnosticCriteria_CheckTimeout(HYD_DiagnosticResult* result,
                                               const HYD_DiagnosticCriteria* criteria,
                                               HYD_DiagnosticCriteriaState* state,
                                               HYD_TIME currentTime,
                                               HYD_TIME segmentElapsedTime,
                                               HYD_BOOL isStartupPhase,
                                               HYD_BOOL isSwitchPhase) {
    HYD_SuppressType suppressType;
    HYD_TIME suppressTime;
    HYD_BOOL timeoutExceeded;
    HYD_BOOL triggered;

    if (result == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* baseThreshold stores the timeout limit for this criteria */
    if (criteria->baseThreshold <= 0.0) {
        return false;
    }

    /* Check suppression conditions — timeout should still be suppressed
     * during startup/switch phases to avoid false alarms during transition. */
    if (HYD_DiagnosticCriteria_CheckSuppressCondition(criteria,
                                                       segmentElapsedTime,
                                                       isStartupPhase,
                                                       isSwitchPhase,
                                                       &suppressType,
                                                       &suppressTime)) {
        result->suppressType = suppressType;
        result->suppressTime = suppressTime;
        result->triggered = false;
        return false;
    }

    /* Timeout is a direct time comparison; no error monitor needed */
    timeoutExceeded = (segmentElapsedTime > criteria->baseThreshold);

    /* Apply debounce (though timeout typically uses debounceTime=0) */
    triggered = HYD_DiagnosticCriteria_CheckDebounce(criteria, state, timeoutExceeded, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

void HYD_DiagnosticCriteria_CreateDefaultTimeoutCriteria(HYD_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 0.0;         /* 0 = timeout disabled; set per segment */
    criteria->debounceTime = 0.0;          /* No debounce for timeout */
    criteria->hysteresisRatio = 0.0;       /* No hysteresis for timeout */

    /* 启用启动抑制（默认500ms） */
    criteria->enableStartupSuppress = true;
    criteria->startupSuppressTime = 0.5;

    /* 启用切段抑制（默认300ms） */
    criteria->enableSwitchSuppress = true;
    criteria->switchSuppressTime = 0.3;

    /* No loop build suppression for timeout */
    criteria->enableLoopBuildSuppress = false;
    criteria->loopBuildSuppressTime = 0.0;

    /* No fault escalation — timeout is directly FAULT */
    criteria->enableFaultEscalation = false;
    criteria->faultEscalationTime = 0.0;

    criteria->diagnosticCode = HYD_DIAG_CODE_TIMEOUT;
    criteria->severity = HYD_DIAG_SEVERITY_FAULT;
    criteria->protectionAction = HYD_PROTECTION_ACTION_STOP;
}
