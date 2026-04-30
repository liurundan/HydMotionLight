#ifndef HYD_DIAGNOSTICS_MONITOR_H
#define HYD_DIAGNOSTICS_MONITOR_H

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
    HYD_REAL positionError;
    HYD_REAL velocityError;
    HYD_REAL flowError;
    HYD_REAL pressureError;

    /* 误差激活状态 */
    HYD_BOOL positionErrorActive;
    HYD_BOOL velocityErrorActive;
    HYD_BOOL flowErrorActive;
    HYD_BOOL pressureErrorActive;

    /* 误差持续时间 */
    HYD_TIME positionErrorStartTime;
    HYD_TIME velocityErrorStartTime;
    HYD_TIME flowErrorStartTime;
    HYD_TIME pressureErrorStartTime;

    HYD_TIME positionErrorDuration;
    HYD_TIME velocityErrorDuration;
    HYD_TIME flowErrorDuration;
    HYD_TIME pressureErrorDuration;

    /* 误差统计（自上次清零以来的统计） */
    HYD_REAL maxPositionError;
    HYD_REAL minPositionError;
    HYD_REAL avgPositionError;
    HYD_REAL maxVelocityError;
    HYD_REAL minVelocityError;
    HYD_REAL avgVelocityError;
    HYD_REAL maxFlowError;
    HYD_REAL minFlowError;
    HYD_REAL avgFlowError;
    HYD_REAL maxPressureError;
    HYD_REAL minPressureError;
    HYD_REAL avgPressureError;

    /* 采样计数 */
    HYD_UINT sampleCount;
} HYD_ErrorMonitor;

/*
 * 初始化误差监视器
 */
void HYD_ErrorMonitor_Init(HYD_ErrorMonitor* monitor);

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
void HYD_ErrorMonitor_Update(HYD_ErrorMonitor* monitor,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_TIME currentTime);

/*
 * 重置误差统计
 *
 * 说明：
 * - 清除历史统计信息（最大值/最小值/平均值/采样计数）
 * - 不影响当前误差状态和持续时间跟踪
 */
void HYD_ErrorMonitor_ResetStatistics(HYD_ErrorMonitor* monitor);

/*
 * 重置误差监视器
 *
 * 说明：
 * - 完全重置监视器状态
 * - 清除所有误差信息、统计数据、持续时间
 */
void HYD_ErrorMonitor_Reset(HYD_ErrorMonitor* monitor);

#endif /* HYD_DIAGNOSTICS_MONITOR_H */
