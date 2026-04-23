#include "diagnostics_criteria.h"
#include <math.h>
#include <string.h>

void HDY_DiagnosticCriteria_InitState(HDY_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));

    /* 初始化告警到故障升级状态 */
    state->warningActive = false;
    state->warningStartTime = 0.0;
    state->faultEscalated = false;
}

void HDY_DiagnosticCriteria_ResetState(HDY_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));

    /* 重置告警到故障升级状态 */
    HDY_DiagnosticCriteria_ResetFaultEscalation(state);
}

HDY_BOOL HDY_IsStartupSuppressActive(HDY_TIME segmentElapsedTime, HDY_TIME suppressTime) {
    if (suppressTime <= 0.0) {
        return false;
    }

    return (segmentElapsedTime < suppressTime) ? true : false;
}

HDY_BOOL HDY_IsSwitchSuppressActive(HDY_BOOL isSwitchPhase, HDY_TIME segmentElapsedTime, HDY_TIME suppressTime) {
    if (!isSwitchPhase || suppressTime <= 0.0) {
        return false;
    }

    return (segmentElapsedTime < suppressTime) ? true : false;
}

HDY_REAL HDY_CalculateLoopBuildFactor(HDY_TIME loopBuildTime, HDY_TIME suppressTime) {
    if (suppressTime <= 0.0) {
        return 1.0;
    }

    if (loopBuildTime >= suppressTime) {
        return 1.0;
    }

    /* 线性递增：0.0 ~ 1.0 */
    return loopBuildTime / suppressTime;
}

static HDY_BOOL HDY_DiagnosticCriteria_IsFiniteReal(HDY_REAL value) {
    return isfinite(value) ? true : false;
}

static HDY_BOOL HDY_DiagnosticCriteria_CheckSuppressCondition(const HDY_DiagnosticCriteria* criteria,
                                                               HDY_TIME segmentElapsedTime,
                                                               HDY_BOOL isStartupPhase,
                                                               HDY_BOOL isSwitchPhase,
                                                               HDY_SuppressType* suppressType,
                                                               HDY_TIME* suppressTime) {
    *suppressType = HDY_SUPPRESS_NONE;
    *suppressTime = 0.0;

    if (criteria == NULL) {
        return false;
    }

    /* 检查启动阶段抑制 */
    if (criteria->enableStartupSuppress && isStartupPhase) {
        if (HDY_IsStartupSuppressActive(segmentElapsedTime, criteria->startupSuppressTime)) {
            *suppressType = HDY_SUPPRESS_STARTUP;
            *suppressTime = criteria->startupSuppressTime;
            return true;
        }
    }

    /* 检查切段阶段抑制 */
    if (criteria->enableSwitchSuppress && isSwitchPhase) {
        if (HDY_IsSwitchSuppressActive(isSwitchPhase, segmentElapsedTime, criteria->switchSuppressTime)) {
            *suppressType = HDY_SUPPRESS_SWITCH;
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

static HDY_REAL HDY_DiagnosticCriteria_CalculateEffectiveThreshold(const HDY_DiagnosticCriteria* criteria,
                                                                    HDY_TIME loopBuildTime,
                                                                    const HDY_DiagnosticCriteriaState* state) {
    HDY_REAL effectiveThreshold = criteria->baseThreshold;
    HDY_REAL loopBuildFactor = 1.0;

    if (criteria == NULL) {
        return 0.0;
    }

    /* 应用闭环建立因子（降低敏感度） */
    if (criteria->enableLoopBuildSuppress) {
        loopBuildFactor = HDY_CalculateLoopBuildFactor(loopBuildTime, criteria->loopBuildSuppressTime);
        effectiveThreshold = effectiveThreshold * loopBuildFactor;
    }

    /* 应用滞回逻辑 */
    if (state->hysteresisActive && criteria->hysteresisRatio > 0.0) {
        /* 滞回激活时，降低触发阈值，防止诊断抖动 */
        effectiveThreshold = effectiveThreshold * (1.0 - criteria->hysteresisRatio);
    }

    return effectiveThreshold;
}

static HDY_BOOL HDY_DiagnosticCriteria_CheckDebounce(const HDY_DiagnosticCriteria* criteria,
                                                      HDY_DiagnosticCriteriaState* state,
                                                      HDY_BOOL errorExceedsThreshold,
                                                      HDY_TIME currentTime,
                                                      HDY_TIME* triggerTime) {
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

HDY_BOOL HDY_DiagnosticCriteria_CheckPressure(HDY_DiagnosticResult* result,
                                              const HDY_ErrorMonitor* monitor,
                                              const HDY_DiagnosticCriteria* criteria,
                                              HDY_DiagnosticCriteriaState* state,
                                              HDY_TIME currentTime,
                                              HDY_TIME segmentElapsedTime,
                                              HDY_BOOL isStartupPhase,
                                              HDY_BOOL isSwitchPhase) {
    HDY_REAL absoluteError;
    HDY_REAL effectiveThreshold;
    HDY_BOOL errorExceedsThreshold;
    HDY_SuppressType suppressType;
    HDY_TIME suppressTime;
    HDY_BOOL triggered;
    HDY_TIME loopBuildTime = 0.0;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->pressureError);
    if (!HDY_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件 */
    if (HDY_DiagnosticCriteria_CheckSuppressCondition(criteria,
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
    effectiveThreshold = HDY_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HDY_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

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

HDY_BOOL HDY_DiagnosticCriteria_CheckFlow(HDY_DiagnosticResult* result,
                                           const HDY_ErrorMonitor* monitor,
                                           const HDY_DiagnosticCriteria* criteria,
                                           HDY_DiagnosticCriteriaState* state,
                                           HDY_TIME currentTime,
                                           HDY_TIME segmentElapsedTime,
                                           HDY_BOOL isStartupPhase,
                                           HDY_BOOL isSwitchPhase) {
    HDY_REAL absoluteError;
    HDY_REAL effectiveThreshold;
    HDY_BOOL errorExceedsThreshold;
    HDY_SuppressType suppressType;
    HDY_TIME suppressTime;
    HDY_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->flowError);
    if (!HDY_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件 */
    if (HDY_DiagnosticCriteria_CheckSuppressCondition(criteria,
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
    HDY_TIME loopBuildTime_flow = 0.0;
    if (monitor->flowErrorActive) {
        loopBuildTime_flow = monitor->flowErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HDY_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_flow, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HDY_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

HDY_BOOL HDY_DiagnosticCriteria_CheckVelocity(HDY_DiagnosticResult* result,
                                                const HDY_ErrorMonitor* monitor,
                                                const HDY_DiagnosticCriteria* criteria,
                                                HDY_DiagnosticCriteriaState* state,
                                                HDY_TIME currentTime,
                                                HDY_TIME segmentElapsedTime,
                                                HDY_BOOL isStartupPhase,
                                                HDY_BOOL isSwitchPhase) {
    HDY_REAL absoluteError;
    HDY_REAL effectiveThreshold;
    HDY_BOOL errorExceedsThreshold;
    HDY_SuppressType suppressType;
    HDY_TIME suppressTime;
    HDY_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->velocityError);
    if (!HDY_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件（速度检查启动和切换阶段抑制） */
    if (HDY_DiagnosticCriteria_CheckSuppressCondition(criteria,
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
    HDY_TIME loopBuildTime_velocity = 0.0;
    if (monitor->velocityErrorActive) {
        loopBuildTime_velocity = monitor->velocityErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HDY_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_velocity, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HDY_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

HDY_BOOL HDY_DiagnosticCriteria_CheckPosition(HDY_DiagnosticResult* result,
                                               const HDY_ErrorMonitor* monitor,
                                               const HDY_DiagnosticCriteria* criteria,
                                               HDY_DiagnosticCriteriaState* state,
                                               HDY_TIME currentTime,
                                               HDY_TIME segmentElapsedTime,
                                               HDY_BOOL isStartupPhase,
                                               HDY_BOOL isSwitchPhase) {
    HDY_REAL absoluteError;
    HDY_REAL effectiveThreshold;
    HDY_BOOL errorExceedsThreshold;
    HDY_SuppressType suppressType;
    HDY_TIME suppressTime;
    HDY_BOOL triggered;

    if (result == NULL || monitor == NULL || criteria == NULL || state == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    /* 计算绝对误差 */
    absoluteError = fabs(monitor->positionError);
    if (!HDY_DiagnosticCriteria_IsFiniteReal(absoluteError)) {
        return false;
    }

    /* 检查抑制条件（位置检查启动和切换阶段抑制） */
    if (HDY_DiagnosticCriteria_CheckSuppressCondition(criteria,
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
    HDY_TIME loopBuildTime_position = 0.0;
    if (monitor->positionErrorActive) {
        loopBuildTime_position = monitor->positionErrorDuration;
    }

    /* 计算有效阈值（考虑滞回和闭环建立） */
    effectiveThreshold = HDY_DiagnosticCriteria_CalculateEffectiveThreshold(criteria, loopBuildTime_position, state);
    result->effectiveThreshold = effectiveThreshold;

    /* 判断误差是否超过阈值 */
    errorExceedsThreshold = (absoluteError > effectiveThreshold);

    /* 应用去抖动逻辑 */
    triggered = HDY_DiagnosticCriteria_CheckDebounce(criteria, state, errorExceedsThreshold, currentTime, &result->triggerTime);

    if (triggered) {
        state->hysteresisActive = true;

        result->triggered = true;
        result->code = criteria->diagnosticCode;
        result->severity = criteria->severity;
        result->action = criteria->protectionAction;
    }

    return triggered;
}

void HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(HDY_DiagnosticCriteria* criteria) {
    if (criteria == NULL) {
        return;
    }

    memset(criteria, 0, sizeof(*criteria));

    criteria->baseThreshold = 0.5;        /* 默认0.5 MPa */
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
    criteria->faultCode = HDY_DIAG_CODE_OVER_PRESSURE;

    criteria->diagnosticCode = HDY_DIAG_CODE_OVER_PRESSURE;
    criteria->severity = HDY_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HDY_PROTECTION_ACTION_WARNING;
}

void HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(HDY_DiagnosticCriteria* criteria) {
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
    criteria->faultCode = HDY_DIAG_CODE_FLOW_DEVIATION;

    criteria->diagnosticCode = HDY_DIAG_CODE_FLOW_DEVIATION;
    criteria->severity = HDY_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HDY_PROTECTION_ACTION_DERATE;
}

void HDY_DiagnosticCriteria_CreateDefaultVelocityCriteria(HDY_DiagnosticCriteria* criteria) {
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
    criteria->faultCode = HDY_DIAG_CODE_VELOCITY_DEVIATION;

    criteria->diagnosticCode = HDY_DIAG_CODE_VELOCITY_DEVIATION;
    criteria->severity = HDY_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HDY_PROTECTION_ACTION_WARNING;
}

void HDY_DiagnosticCriteria_CreateDefaultPositionCriteria(HDY_DiagnosticCriteria* criteria) {
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
    criteria->faultCode = HDY_DIAG_CODE_POSITION_DEVIATION;

    criteria->diagnosticCode = HDY_DIAG_CODE_POSITION_DEVIATION;
    criteria->severity = HDY_DIAG_SEVERITY_WARNING;
    criteria->protectionAction = HDY_PROTECTION_ACTION_WARNING;
}

HDY_BOOL HDY_DiagnosticCriteria_CheckFaultEscalation(HDY_DiagnosticResult* result,
                                                      const HDY_DiagnosticCriteria* criteria,
                                                      HDY_DiagnosticCriteriaState* state,
                                                      HDY_TIME currentTime) {
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
    if (result->severity != HDY_DIAG_SEVERITY_WARNING) {
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
        HDY_TIME warningDuration = currentTime - state->warningStartTime;

        /* 超过升级时间，升级为 FAULT */
        if (warningDuration >= criteria->faultEscalationTime) {
            state->faultEscalated = true;

            /* 升级诊断结果 */
            result->severity = HDY_DIAG_SEVERITY_FAULT;
            result->action = HDY_PROTECTION_ACTION_STOP;

            /* 如果配置了故障码，更新故障码 */
            if (criteria->faultCode != HDY_DIAG_CODE_NONE) {
                result->code = criteria->faultCode;
            }

            return true;
        }
    }

    return false;
}

void HDY_DiagnosticCriteria_ResetFaultEscalation(HDY_DiagnosticCriteriaState* state) {
    if (state == NULL) {
        return;
    }

    state->warningActive = false;
    state->warningStartTime = 0.0;
    state->faultEscalated = false;
}
