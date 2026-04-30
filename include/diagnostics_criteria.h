#ifndef HYD_DIAGNOSTICS_CRITERIA_H
#define HYD_DIAGNOSTICS_CRITERIA_H

#include "diagnostics_monitor.h"
#include "diagnostics.h"

/*
 * Diagnostic Criteria - 判据层
 *
 * 职责：
 * - 判断误差是否超过阈值
 * - 检查持续时间是否满足
 * - 应用滞回逻辑
 * - 应用抑制条件
 * - 确定诊断严重级别
 * - 决定保护动作
 *
 * 设计原则：
 * - 基于监视层的统计数据进行判断
 * - 不直接采样反馈数据
 * - 支持误报抑制逻辑
 * - 支持告警/故障分级
 */

/* 误报抑制类型 */
typedef enum {
    HYD_SUPPRESS_NONE = 0,
    HYD_SUPPRESS_STARTUP,      /* 启动阶段抑制 */
    HYD_SUPPRESS_SWITCH,       /* 切段阶段抑制 */
    HYD_SUPPRESS_LOOP_BUILD    /* 闭环建立抑制 */
} HYD_SuppressType;

/* 诊断判据配置 */
typedef struct {
    /* 基础阈值 */
    HYD_REAL baseThreshold;         /* 基础误差阈值 */

    /* 判据参数 */
    HYD_TIME debounceTime;         /* 去抖动持续时间阈值（秒） */
    HYD_REAL hysteresisRatio;      /* 滞回比例（0~1），用于防止诊断抖动 */

    /* 误报抑制参数 */
    HYD_BOOL enableStartupSuppress;      /* 是否启用启动阶段抑制 */
    HYD_TIME startupSuppressTime;        /* 启动抑制时长（秒） */

    HYD_BOOL enableSwitchSuppress;       /* 是否启用切段阶段抑制 */
    HYD_TIME switchSuppressTime;         /* 切段抑制时长（秒） */

    HYD_BOOL enableLoopBuildSuppress;    /* 是否启用闭环建立抑制 */
    HYD_TIME loopBuildSuppressTime;      /* 闭环建立抑制时长（秒） */

    /* 告警到故障升级参数 */
    HYD_BOOL enableFaultEscalation;      /* 是否启用告警到故障升级 */
    HYD_TIME faultEscalationTime;        /* 告警升级为故障的持续时间（秒） */

    /* 诊断配置 */
    HYD_DiagnosticCode diagnosticCode;
    HYD_DiagnosticSeverity severity;      /* 初始严重级别（WARNING） */
    HYD_DiagnosticCode faultCode;        /* 升级后的故障码（可选） */
    HYD_ProtectionAction protectionAction;
} HYD_DiagnosticCriteria;

/* 诊断判据结果 */
typedef struct {
    HYD_BOOL triggered;                /* 诊断是否触发 */
    HYD_DiagnosticCode code;            /* 诊断码 */
    HYD_DiagnosticSeverity severity;    /* 严重级别 */
    HYD_ProtectionAction action;        /* 保护动作 */
    HYD_TIME triggerTime;               /* 触发时间 */
    HYD_TIME suppressTime;              /* 抑制时间（如果被抑制） */
    HYD_SuppressType suppressType;      /* 抑制类型 */
    HYD_REAL effectiveThreshold;        /* 有效阈值（考虑滞回后） */
} HYD_DiagnosticResult;

/* 诊断判据状态 */
typedef struct {
    HYD_BOOL lastTriggered;             /* 上次是否触发 */
    HYD_TIME triggerStartTime;          /* 触发开始时间 */
    HYD_BOOL hysteresisActive;          /* 滞回是否激活 */
    HYD_UINT8 debounceCount;            /* 去抖动计数器 */

    /* 告警到故障升级状态 */
    HYD_BOOL warningActive;             /* WARNING 级别是否激活 */
    HYD_TIME warningStartTime;         /* WARNING 激活开始时间 */
    HYD_BOOL faultEscalated;           /* 是否已升级为故障 */
} HYD_DiagnosticCriteriaState;

/*
 * 初始化诊断判据
 */
void HYD_DiagnosticCriteria_InitState(HYD_DiagnosticCriteriaState* state);

/*
 * 检查压力诊断判据
 *
 * 参数：
 * - monitor: 误差监视器
 * - criteria: 诊断判据配置
 * - state: 诊断判据状态
 * - currentTime: 当前时间戳
 * - segmentElapsedTime: 当前段已执行时间
 * - isStartupPhase: 是否处于启动阶段
 * - isSwitchPhase: 是否处于切段阶段
 * - result: 输出诊断结果
 *
 * 返回：
 * - true: 诊断触发
 * - false: 诊断未触发
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckPressure(HYD_DiagnosticResult* result,
                                              const HYD_ErrorMonitor* monitor,
                                              const HYD_DiagnosticCriteria* criteria,
                                              HYD_DiagnosticCriteriaState* state,
                                              HYD_TIME currentTime,
                                              HYD_TIME segmentElapsedTime,
                                              HYD_BOOL isStartupPhase,
                                              HYD_BOOL isSwitchPhase);

/*
 * 检查流量诊断判据
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckFlow(HYD_DiagnosticResult* result,
                                           const HYD_ErrorMonitor* monitor,
                                           const HYD_DiagnosticCriteria* criteria,
                                           HYD_DiagnosticCriteriaState* state,
                                           HYD_TIME currentTime,
                                           HYD_TIME segmentElapsedTime,
                                           HYD_BOOL isStartupPhase,
                                           HYD_BOOL isSwitchPhase);

/*
 * 检查速度诊断判据
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckVelocity(HYD_DiagnosticResult* result,
                                                const HYD_ErrorMonitor* monitor,
                                                const HYD_DiagnosticCriteria* criteria,
                                                HYD_DiagnosticCriteriaState* state,
                                                HYD_TIME currentTime,
                                                HYD_TIME segmentElapsedTime,
                                                HYD_BOOL isStartupPhase,
                                                HYD_BOOL isSwitchPhase);

/*
 * 检查位置诊断判据
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckPosition(HYD_DiagnosticResult* result,
                                               const HYD_ErrorMonitor* monitor,
                                               const HYD_DiagnosticCriteria* criteria,
                                               HYD_DiagnosticCriteriaState* state,
                                               HYD_TIME currentTime,
                                               HYD_TIME segmentElapsedTime,
                                               HYD_BOOL isStartupPhase,
                                               HYD_BOOL isSwitchPhase);

/*
 * 检查超时诊断判据
 *
 * 超时诊断不基于 ErrorMonitor，而是直接比较段运行时间与超时限制。
 * 仍通过判据层的抑制机制（启动/切段抑制）统一处理，确保与其他
 * 诊断通道语义一致。
 *
 * 参数：
 * - result: 输出诊断结果
 * - criteria: 诊断判据配置（baseThreshold 用作 timeoutLimit）
 * - state: 诊断判据状态
 * - currentTime: 当前时间戳
 * - segmentElapsedTime: 当前段已执行时间
 * - isStartupPhase: 是否处于启动阶段
 * - isSwitchPhase: 是否处于切段阶段
 *
 * 返回：
 * - true: 超时诊断触发
 * - false: 超时诊断未触发
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckTimeout(HYD_DiagnosticResult* result,
                                               const HYD_DiagnosticCriteria* criteria,
                                               HYD_DiagnosticCriteriaState* state,
                                               HYD_TIME currentTime,
                                               HYD_TIME segmentElapsedTime,
                                               HYD_BOOL isStartupPhase,
                                               HYD_BOOL isSwitchPhase);

/*
 * 创建默认超时诊断判据
 */
void HYD_DiagnosticCriteria_CreateDefaultTimeoutCriteria(HYD_DiagnosticCriteria* criteria);

/*
 * 重置诊断判据状态
 */
void HYD_DiagnosticCriteria_ResetState(HYD_DiagnosticCriteriaState* state);

/*
 * 判断启动阶段抑制是否激活
 */
HYD_BOOL HYD_IsStartupSuppressActive(HYD_TIME segmentElapsedTime, HYD_TIME suppressTime);

/*
 * 判断切段阶段抑制是否激活
 */
HYD_BOOL HYD_IsSwitchSuppressActive(HYD_BOOL isSwitchPhase, HYD_TIME segmentElapsedTime, HYD_TIME suppressTime);

/*
 * 计算闭环建立因子
 *
 * 参数：
 * - loopBuildTime: 闭环已建立时间
 * - suppressTime: 抑制时长
 *
 * 返回：
 * - 0.0 ~ 1.0 的因子值，用于降低诊断敏感度
 */
HYD_REAL HYD_CalculateLoopBuildFactor(HYD_TIME loopBuildTime, HYD_TIME suppressTime);

/*
 * 创建默认压力诊断判据
 */
void HYD_DiagnosticCriteria_CreateDefaultPressureCriteria(HYD_DiagnosticCriteria* criteria);

/*
 * 创建默认流量诊断判据
 */
void HYD_DiagnosticCriteria_CreateDefaultFlowCriteria(HYD_DiagnosticCriteria* criteria);

/*
 * 创建默认速度诊断判据
 */
void HYD_DiagnosticCriteria_CreateDefaultVelocityCriteria(HYD_DiagnosticCriteria* criteria);

/*
 * 创建默认位置诊断判据
 */
void HYD_DiagnosticCriteria_CreateDefaultPositionCriteria(HYD_DiagnosticCriteria* criteria);

/*
 * 检查告警到故障升级
 *
 * 参数：
 * - criteria: 诊断判据配置
 * - state: 诊断判据状态
 * - currentTime: 当前时间戳
 * - result: 诊断结果（可能升级为FAULT）
 *
 * 返回：
 * - true: 升级为故障
 * - false: 未升级或保持原级别
 */
HYD_BOOL HYD_DiagnosticCriteria_CheckFaultEscalation(HYD_DiagnosticResult* result,
                                                      const HYD_DiagnosticCriteria* criteria,
                                                      HYD_DiagnosticCriteriaState* state,
                                                      HYD_TIME currentTime);

/*
 * 重置告警到故障升级状态
 */
void HYD_DiagnosticCriteria_ResetFaultEscalation(HYD_DiagnosticCriteriaState* state);

#endif /* HYD_DIAGNOSTICS_CRITERIA_H */
