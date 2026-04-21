#ifndef HDY_DIAGNOSTICS_MONITOR_H
#define HDY_DIAGNOSTICS_MONITOR_H

#include "common_types.h"

/*
 * Error Monitor - 监视层
 *
 * 职责：
 * - 实时采样位置/速度/流量/压力误差
 * - 计算偏差统计（最大值、最小值、平均值、标准差）
 * - 跟踪误差持续时间
 * - 记录误差变化趋势
 *
 * 设计原则：
 * - 纯数据采样，不做判据判断
 * - 无状态切换逻辑，仅记录实时数据
 * - 为判据层提供完整的误差统计信息
 */

typedef struct {
    /* 实时误差值 */
    HDY_REAL positionError;
    HDY_REAL velocityError;
    HDY_REAL flowError;
    HDY_REAL pressureError;

    /* 误差激活状态 */
    HDY_BOOL positionErrorActive;
    HDY_BOOL velocityErrorActive;
    HDY_BOOL flowErrorActive;
    HDY_BOOL pressureErrorActive;

    /* 误差持续时间 */
    HDY_TIME positionErrorStartTime;
    HDY_TIME velocityErrorStartTime;
    HDY_TIME flowErrorStartTime;
    HDY_TIME pressureErrorStartTime;

    HDY_TIME positionErrorDuration;
    HDY_TIME velocityErrorDuration;
    HDY_TIME flowErrorDuration;
    HDY_TIME pressureErrorDuration;

    /* 误差统计（自上次清零以来的统计） */
    HDY_REAL maxPositionError;
    HDY_REAL minPositionError;
    HDY_REAL avgPositionError;
    HDY_REAL maxVelocityError;
    HDY_REAL minVelocityError;
    HDY_REAL avgVelocityError;
    HDY_REAL maxFlowError;
    HDY_REAL minFlowError;
    HDY_REAL avgFlowError;
    HDY_REAL maxPressureError;
    HDY_REAL minPressureError;
    HDY_REAL avgPressureError;

    /* 采样计数 */
    HDY_UINT sampleCount;
} HDY_ErrorMonitor;

/*
 * 初始化误差监视器
 */
void HDY_ErrorMonitor_Init(HDY_ErrorMonitor* monitor);

/*
 * 更新误差监视器
 *
 * 参数：
 * - monitor: 误差监视器实例
 * - axisRef: 轴反馈数据
 * - references: 执行参考值
 * - currentTime: 当前时间戳
 *
 * 说明：
 * - 计算各项误差并更新统计信息
 * - 跟踪误差持续时间
 * - 更新最大值/最小值/平均值
 */
void HDY_ErrorMonitor_Update(HDY_ErrorMonitor* monitor,
                             const HDY_AxisRef* axisRef,
                             const HDY_ExecutionReference* references,
                             HDY_TIME currentTime);

/*
 * 重置误差统计
 *
 * 说明：
 * - 清除历史统计信息（最大值/最小值/平均值/采样计数）
 * - 不影响当前误差状态和持续时间跟踪
 */
void HDY_ErrorMonitor_ResetStatistics(HDY_ErrorMonitor* monitor);

/*
 * 重置误差监视器
 *
 * 说明：
 * - 完全重置监视器状态
 * - 清除所有误差信息、统计数据、持续时间
 */
void HDY_ErrorMonitor_Reset(HDY_ErrorMonitor* monitor);

#endif /* HDY_DIAGNOSTICS_MONITOR_H */
