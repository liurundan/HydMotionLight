#ifndef HDY_DIAGNOSTICS_CRITERIA_H
#define HDY_DIAGNOSTICS_CRITERIA_H

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
    HDY_SUPPRESS_NONE = 0,
    HDY_SUPPRESS_STARTUP,      /* 启动阶段抑制 */
    HDY_SUPPRESS_SWITCH,       /* 切段阶段抑制 */
    HDY_SUPPRESS_LOOP_BUILD    /* 闭环建立抑制 */
} HDY_SuppressType;

/* 诊断判据配置 */
typedef struct {
    /* 基础阈值 */
    HDY_REAL baseThreshold;         /* 基础误差阈值 */

    /* 判据参数 */
    HDY_TIME debounceTime;         /* 去抖动持续时间阈值（秒） */
    HDY_REAL hysteresisRatio;      /* 滞回比例（0~1），用于防止诊断抖动 */

    /* 误报抑制参数 */
    HDY_BOOL enableStartupSuppress;      /* 是否启用启动阶段抑制 */
    HDY_TIME startupSuppressTime;        /* 启动抑制时长（秒） */

    HDY_BOOL enableSwitchSuppress;       /* 是否启用切段阶段抑制 */
    HDY_TIME switchSuppressTime;         /* 切段抑制时长（秒） */

    HDY_BOOL enableLoopBuildSuppress;    /* 是否启用闭环建立抑制 */
    HDY_TIME loopBuildSuppressTime;      /* 闭环建立抑制时长（秒） */

    /* 告警到故障升级参数 */
    HDY_BOOL enableFaultEscalation;      /* 是否启用告警到故障升级 */
    HDY_TIME faultEscalationTime;        /* 告警升级为故障的持续时间（秒） */

    /* 诊断配置 */
    HDY_DiagnosticCode diagnosticCode;
    HDY_DiagnosticSeverity severity;      /* 初始严重级别（WARNING） */
    HDY_DiagnosticCode faultCode;        /* 升级后的故障码（可选） */
    HDY_ProtectionAction protectionAction;
} HDY_DiagnosticCriteria;

/* 诊断判据结果 */
typedef struct {
    HDY_BOOL triggered;                /* 诊断是否触发 */
    HDY_DiagnosticCode code;            /* 诊断码 */
    HDY_DiagnosticSeverity severity;    /* 严重级别 */
    HDY_ProtectionAction action;        /* 保护动作 */
    HDY_TIME triggerTime;               /* 触发时间 */
    HDY_TIME suppressTime;              /* 抑制时间（如果被抑制） */
    HDY_SuppressType suppressType;      /* 抑制类型 */
    HDY_REAL effectiveThreshold;        /* 有效阈值（考虑滞回后） */
} HDY_DiagnosticResult;

/* 诊断判据状态 */
typedef struct {
    HDY_BOOL lastTriggered;             /* 上次是否触发 */
    HDY_TIME triggerStartTime;          /* 触发开始时间 */
    HDY_BOOL hysteresisActive;          /* 滞回是否激活 */
    HDY_UINT8 debounceCount;            /* 去抖动计数器 */

    /* 告警到故障升级状态 */
    HDY_BOOL warningActive;             /* WARNING 级别是否激活 */
    HDY_TIME warningStartTime;         /* WARNING 激活开始时间 */
    HDY_BOOL faultEscalated;           /* 是否已升级为故障 */
} HDY_DiagnosticCriteriaState;

/*
 * 初始化诊断判据
 */
void HDY_DiagnosticCriteria_InitState(HDY_DiagnosticCriteriaState* state);

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
HDY_BOOL HDY_DiagnosticCriteria_CheckPressure(HDY_DiagnosticResult* result,
                                              const HDY_ErrorMonitor* monitor,
                                              const HDY_DiagnosticCriteria* criteria,
                                              HDY_DiagnosticCriteriaState* state,
                                              HDY_TIME currentTime,
                                              HDY_TIME segmentElapsedTime,
                                              HDY_BOOL isStartupPhase,
                                              HDY_BOOL isSwitchPhase);

/*
 * 检查流量诊断判据
 */
HDY_BOOL HDY_DiagnosticCriteria_CheckFlow(HDY_DiagnosticResult* result,
                                           const HDY_ErrorMonitor* monitor,
                                           const HDY_DiagnosticCriteria* criteria,
                                           HDY_DiagnosticCriteriaState* state,
                                           HDY_TIME currentTime,
                                           HDY_TIME segmentElapsedTime,
                                           HDY_BOOL isStartupPhase,
                                           HDY_BOOL isSwitchPhase);

/*
 * 检查速度诊断判据
 */
HDY_BOOL HDY_DiagnosticCriteria_CheckVelocity(HDY_DiagnosticResult* result,
                                                const HDY_ErrorMonitor* monitor,
                                                const HDY_DiagnosticCriteria* criteria,
                                                HDY_DiagnosticCriteriaState* state,
                                                HDY_TIME currentTime,
                                                HDY_TIME segmentElapsedTime,
                                                HDY_BOOL isStartupPhase,
                                                HDY_BOOL isSwitchPhase);

/*
 * 检查位置诊断判据
 */
HDY_BOOL HDY_DiagnosticCriteria_CheckPosition(HDY_DiagnosticResult* result,
                                               const HDY_ErrorMonitor* monitor,
                                               const HDY_DiagnosticCriteria* criteria,
                                               HDY_DiagnosticCriteriaState* state,
                                               HDY_TIME currentTime,
                                               HDY_TIME segmentElapsedTime,
                                               HDY_BOOL isStartupPhase,
                                               HDY_BOOL isSwitchPhase);

/*
 * 重置诊断判据状态
 */
void HDY_DiagnosticCriteria_ResetState(HDY_DiagnosticCriteriaState* state);

/*
 * 判断启动阶段抑制是否激活
 */
HDY_BOOL HDY_IsStartupSuppressActive(HDY_TIME segmentElapsedTime, HDY_TIME suppressTime);

/*
 * 判断切段阶段抑制是否激活
 */
HDY_BOOL HDY_IsSwitchSuppressActive(HDY_BOOL isSwitchPhase, HDY_TIME segmentElapsedTime, HDY_TIME suppressTime);

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
HDY_REAL HDY_CalculateLoopBuildFactor(HDY_TIME loopBuildTime, HDY_TIME suppressTime);

/*
 * 创建默认压力诊断判据
 */
void HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(HDY_DiagnosticCriteria* criteria);

/*
 * 创建默认流量诊断判据
 */
void HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(HDY_DiagnosticCriteria* criteria);

/*
 * 创建默认速度诊断判据
 */
void HDY_DiagnosticCriteria_CreateDefaultVelocityCriteria(HDY_DiagnosticCriteria* criteria);

/*
 * 创建默认位置诊断判据
 */
void HDY_DiagnosticCriteria_CreateDefaultPositionCriteria(HDY_DiagnosticCriteria* criteria);

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
HDY_BOOL HDY_DiagnosticCriteria_CheckFaultEscalation(HDY_DiagnosticResult* result,
                                                      const HDY_DiagnosticCriteria* criteria,
                                                      HDY_DiagnosticCriteriaState* state,
                                                      HDY_TIME currentTime);

/*
 * 重置告警到故障升级状态
 */
void HDY_DiagnosticCriteria_ResetFaultEscalation(HDY_DiagnosticCriteriaState* state);

#endif /* HDY_DIAGNOSTICS_CRITERIA_H */
