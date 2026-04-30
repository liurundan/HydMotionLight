#include "diagnostics_monitor.h"
#include <math.h>
#include <string.h>

void HYD_ErrorMonitor_Init(HYD_ErrorMonitor* monitor) {
    if (monitor == NULL) {
        return;
    }

    HYD_ErrorMonitor_Reset(monitor);
}

static HYD_BOOL HYD_ErrorMonitor_IsFiniteReal(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

void HYD_ErrorMonitor_Update(HYD_ErrorMonitor* monitor,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_TIME currentTime) {
    HYD_REAL newPositionError;
    HYD_REAL newVelocityError;
    HYD_REAL newFlowError;
    HYD_REAL newPressureError;

    if (monitor == NULL || axisRef == NULL || references == NULL) {
        return;
    }

    /* 计算实时误差 */
    newPositionError = 0.0;
    newVelocityError = 0.0;
    newFlowError = 0.0;
    newPressureError = 0.0;

    /* 压力误差 = 参考压力 - 实测压力 */
    if (HYD_ErrorMonitor_IsFiniteReal(references->pressureReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->pressure)) {
        newPressureError = references->pressureReference - axisRef->pressure;
        monitor->pressureError = newPressureError;
    }

    /* 流量误差 = 参考流量 - |实测流量|（流量为幅值） */
    if (HYD_ErrorMonitor_IsFiniteReal(references->flowReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->flow)) {
        newFlowError = references->flowReference - fabs(axisRef->flow);
        monitor->flowError = newFlowError;
    }

    /* 速度误差 = 参考速度 - 实测速度（速度有符号） */
    if (HYD_ErrorMonitor_IsFiniteReal(references->velocityReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->velocity)) {
        newVelocityError = references->velocityReference - axisRef->velocity;
        monitor->velocityError = newVelocityError;
    }

    /* 更新压力误差激活状态和持续时间 */
    if (monitor->pressureError != 0.0) {
        if (!monitor->pressureErrorActive) {
            monitor->pressureErrorActive = true;
            monitor->pressureErrorStartTime = currentTime;
            monitor->pressureErrorDuration = 0.0;
        } else {
            monitor->pressureErrorDuration = currentTime - monitor->pressureErrorStartTime;
        }
    } else {
        monitor->pressureErrorActive = false;
        monitor->pressureErrorDuration = 0.0;
    }

    /* 更新流量误差激活状态和持续时间 */
    if (monitor->flowError != 0.0) {
        if (!monitor->flowErrorActive) {
            monitor->flowErrorActive = true;
            monitor->flowErrorStartTime = currentTime;
            monitor->flowErrorDuration = 0.0;
        } else {
            monitor->flowErrorDuration = currentTime - monitor->flowErrorStartTime;
        }
    } else {
        monitor->flowErrorActive = false;
        monitor->flowErrorDuration = 0.0;
    }

    /* 更新速度误差激活状态和持续时间 */
    if (monitor->velocityError != 0.0) {
        if (!monitor->velocityErrorActive) {
            monitor->velocityErrorActive = true;
            monitor->velocityErrorStartTime = currentTime;
            monitor->velocityErrorDuration = 0.0;
        } else {
            monitor->velocityErrorDuration = currentTime - monitor->velocityErrorStartTime;
        }
    } else {
        monitor->velocityErrorActive = false;
        monitor->velocityErrorDuration = 0.0;
    }

    /* 更新位置误差激活状态和持续时间 */
    if (monitor->positionError != 0.0) {
        if (!monitor->positionErrorActive) {
            monitor->positionErrorActive = true;
            monitor->positionErrorStartTime = currentTime;
            monitor->positionErrorDuration = 0.0;
        } else {
            monitor->positionErrorDuration = currentTime - monitor->positionErrorStartTime;
        }
    } else {
        monitor->positionErrorActive = false;
        monitor->positionErrorDuration = 0.0;
    }

    /* 更新统计信息（仅在有效值时更新） */
    if (HYD_ErrorMonitor_IsFiniteReal(newPressureError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxPressureError = newPressureError;
            monitor->minPressureError = newPressureError;
            monitor->avgPressureError = newPressureError;
        } else {
            if (newPressureError > monitor->maxPressureError) {
                monitor->maxPressureError = newPressureError;
            }
            if (newPressureError < monitor->minPressureError) {
                monitor->minPressureError = newPressureError;
            }
            /* 累积平均值：avg = old_avg + (new - old_avg) / (n + 1) */
            monitor->avgPressureError = monitor->avgPressureError +
                (newPressureError - monitor->avgPressureError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }

    if (HYD_ErrorMonitor_IsFiniteReal(newFlowError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxFlowError = newFlowError;
            monitor->minFlowError = newFlowError;
            monitor->avgFlowError = newFlowError;
        } else {
            if (newFlowError > monitor->maxFlowError) {
                monitor->maxFlowError = newFlowError;
            }
            if (newFlowError < monitor->minFlowError) {
                monitor->minFlowError = newFlowError;
            }
            monitor->avgFlowError = monitor->avgFlowError +
                (newFlowError - monitor->avgFlowError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }

    if (HYD_ErrorMonitor_IsFiniteReal(newVelocityError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxVelocityError = newVelocityError;
            monitor->minVelocityError = newVelocityError;
            monitor->avgVelocityError = newVelocityError;
        } else {
            if (newVelocityError > monitor->maxVelocityError) {
                monitor->maxVelocityError = newVelocityError;
            }
            if (newVelocityError < monitor->minVelocityError) {
                monitor->minVelocityError = newVelocityError;
            }
            monitor->avgVelocityError = monitor->avgVelocityError +
                (newVelocityError - monitor->avgVelocityError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }

    if (HYD_ErrorMonitor_IsFiniteReal(newPositionError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxPositionError = newPositionError;
            monitor->minPositionError = newPositionError;
            monitor->avgPositionError = newPositionError;
        } else {
            if (newPositionError > monitor->maxPositionError) {
                monitor->maxPositionError = newPositionError;
            }
            if (newPositionError < monitor->minPositionError) {
                monitor->minPositionError = newPositionError;
            }
            monitor->avgPositionError = monitor->avgPositionError +
                (newPositionError - monitor->avgPositionError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }

    monitor->sampleCount++;
}

void HYD_ErrorMonitor_ResetStatistics(HYD_ErrorMonitor* monitor) {
    if (monitor == NULL) {
        return;
    }

    /* 重置统计信息，但保留当前误差状态 */
    monitor->maxPositionError = 0.0;
    monitor->minPositionError = 0.0;
    monitor->avgPositionError = 0.0;
    monitor->maxVelocityError = 0.0;
    monitor->minVelocityError = 0.0;
    monitor->avgVelocityError = 0.0;
    monitor->maxFlowError = 0.0;
    monitor->minFlowError = 0.0;
    monitor->avgFlowError = 0.0;
    monitor->maxPressureError = 0.0;
    monitor->minPressureError = 0.0;
    monitor->avgPressureError = 0.0;
    monitor->sampleCount = 0;
}

void HYD_ErrorMonitor_Reset(HYD_ErrorMonitor* monitor) {
    if (monitor == NULL) {
        return;
    }

    memset(monitor, 0, sizeof(*monitor));
}
