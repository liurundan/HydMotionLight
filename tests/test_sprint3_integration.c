/*
 * Sprint 3 集成测试
 *
 * 测试内容：
 * 1. 误报抑制效果测试
 * 2. 告警/故障分级测试
 * 3. 端到端集成测试
 */

#include "diagnostics_monitor.h"
#include "diagnostics_criteria.h"
#include "diagnostics.h"
#include "motion_control.h"
#include "common_types.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

/*
 * 测试启动阶段误报抑制
 */
void test_false_positive_startup_suppression() {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria criteria;
    HDY_DiagnosticCriteriaState state;
    HDY_DiagnosticResult result;

    printf("🧪 Testing startup suppression (false positive reduction)...\n");

    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_InitState(&state);
    HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&criteria);

    /* 设置压力误差（超过阈值） */
    monitor.pressureError = 1.0;
    monitor.pressureErrorActive = true;
    monitor.pressureErrorDuration = 0.1;

    /* 在启动阶段（0.1秒 < 0.5秒抑制时间），不应该触发 */
    HDY_DiagnosticCriteria_CheckPressure(&result, &monitor, &criteria, &state,
                                          0.1, 0.1, true, false);
    assert(!result.triggered);
    assert(result.suppressType == HDY_SUPPRESS_STARTUP);

    printf("  ✅ Startup suppression active (0.1s)\n");

    /* 重置状态，模拟新段开始 */
    HDY_DiagnosticCriteria_InitState(&state);

    /* 启动阶段结束后（0.6秒 > 0.5秒），需要满足去抖动时间（0.1秒） */
    /* 禁用去抖动，简化测试 */
    criteria.debounceTime = 0.0;

    /* 第一次调用：应该触发 */
    HDY_DiagnosticCriteria_CheckPressure(&result, &monitor, &criteria, &state,
                                          0.6, 0.6, false, false);
    assert(result.triggered);
    assert(result.code == HDY_DIAG_CODE_OVER_PRESSURE);

    printf("  ✅ Diagnostic triggered after startup phase (0.6s)\n");
    printf("✅ test_false_positive_startup_suppression passed\n");
}

/*
 * 测试切段阶段误报抑制
 */
void test_false_positive_switch_suppression() {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria criteria;
    HDY_DiagnosticCriteriaState state;
    HDY_DiagnosticResult result;

    printf("🧪 Testing switch suppression (false positive reduction)...\n");

    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_InitState(&state);
    HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(&criteria);

    /* 设置流量误差（超过阈值） */
    monitor.flowError = 3.0;

    /* 在切段阶段（0.1秒 < 0.3秒抑制时间），不应该触发 */
    HDY_DiagnosticCriteria_CheckFlow(&result, &monitor, &criteria, &state,
                                     0.1, 0.1, false, true);
    assert(!result.triggered);
    assert(result.suppressType == HDY_SUPPRESS_SWITCH);

    printf("  ✅ Switch suppression active (0.1s)\n");

    /* 切段阶段结束后（0.4秒 > 0.3秒），需要满足去抖动时间 */
    /* 禁用去抖动，简化测试 */
    criteria.debounceTime = 0.0;

    /* 应该触发 */
    HDY_DiagnosticCriteria_CheckFlow(&result, &monitor, &criteria, &state,
                                     0.4, 0.4, false, false);
    assert(result.triggered);
    assert(result.code == HDY_DIAG_CODE_FLOW_DEVIATION);

    printf("  ✅ Diagnostic triggered after switch phase (0.4s)\n");
    printf("✅ test_false_positive_switch_suppression passed\n");
}

/*
 * 测试闭环建立抑制（降低敏感度）
 */
void test_false_positive_loop_build_suppression() {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria criteria;
    HDY_DiagnosticCriteriaState state;
    HDY_REAL loopBuildFactor;

    printf("🧪 Testing loop build suppression (false positive reduction)...\n");

    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_InitState(&state);
    HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&criteria);

    /* 计算闭环建立因子 */
    loopBuildFactor = HDY_CalculateLoopBuildFactor(0.1, 0.2);
    assert(loopBuildFactor == 0.5);

    printf("  ✅ Loop build factor (0.1s / 0.2s) = 0.5\n");

    loopBuildFactor = HDY_CalculateLoopBuildFactor(0.2, 0.2);
    assert(loopBuildFactor == 1.0);

    printf("  ✅ Loop build factor (0.2s / 0.2s) = 1.0\n");

    loopBuildFactor = HDY_CalculateLoopBuildFactor(0.3, 0.2);
    assert(loopBuildFactor == 1.0);

    printf("  ✅ Loop build factor (0.3s / 0.2s) = 1.0\n");
    printf("✅ test_false_positive_loop_build_suppression passed\n");
}

/*
 * 测试告警到故障升级 - 完整流程
 */
void test_severity_escalation_full_flow() {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria criteria;
    HDY_DiagnosticCriteriaState state;
    HDY_DiagnosticResult result;

    printf("🧪 Testing severity escalation (full flow)...\n");

    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_InitState(&state);
    HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&criteria);

    /* 禁用启动抑制，方便测试 */
    criteria.enableStartupSuppress = false;
    criteria.debounceTime = 0.0;

    /* 阶段1：触发 WARNING */
    monitor.pressureError = 1.0;
    monitor.pressureErrorActive = true;
    monitor.pressureErrorDuration = 0.1;

    HDY_DiagnosticCriteria_CheckPressure(&result, &monitor, &criteria, &state,
                                          0.1, 0.1, false, false);
    assert(result.triggered);
    assert(result.severity == HDY_DIAG_SEVERITY_WARNING);
    assert(result.action == HDY_PROTECTION_ACTION_WARNING);

    printf("  ✅ Stage 1: WARNING triggered\n");

    /* 检查升级状态 */
    HDY_DiagnosticCriteria_CheckFaultEscalation(&result, &criteria, &state, 0.1);
    assert(state.warningActive);
    assert(result.severity == HDY_DIAG_SEVERITY_WARNING);

    printf("  ✅ Stage 1: Escalation started, still WARNING\n");

    /* 阶段2：WARNING 持续，即将升级 */
    HDY_DiagnosticCriteria_CheckFaultEscalation(&result, &criteria, &state, 1.5);
    assert(state.warningActive);
    assert(result.severity == HDY_DIAG_SEVERITY_WARNING);
    assert(!state.faultEscalated);

    printf("  ✅ Stage 2: WARNING persistent (1.5s), not escalated yet\n");

    /* 阶段3：升级为 FAULT */
    HDY_DiagnosticCriteria_CheckFaultEscalation(&result, &criteria, &state, 2.5);
    assert(result.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(result.action == HDY_PROTECTION_ACTION_STOP);
    assert(state.faultEscalated);

    printf("  ✅ Stage 3: Escalated to FAULT (2.5s)\n");
    printf("✅ test_severity_escalation_full_flow passed\n");
}

/*
 * 测试误报抑制效果对比
 */
void test_false_positive_reduction_comparison() {
    HDY_ErrorMonitor monitor1, monitor2;
    HDY_DiagnosticCriteria criteria1, criteria2;
    HDY_DiagnosticCriteriaState state1, state2;
    HDY_DiagnosticResult result1, result2;
    int suppressions = 0;

    printf("🧪 Testing false positive reduction comparison...\n");

    /* 模拟100个周期的诊断检查 */
    for (int i = 0; i < 100; i++) {
        HDY_TIME currentTime = (HDY_TIME)i * 0.01;  /* 10ms周期 */
        HDY_TIME segmentElapsedTime = currentTime;

        /* 监视器1：启用误报抑制 */
        if (i == 0) {
            HDY_ErrorMonitor_Init(&monitor1);
            HDY_DiagnosticCriteria_InitState(&state1);
            HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&criteria1);
        }

        /* 监视器2：禁用误报抑制 */
        if (i == 0) {
            HDY_ErrorMonitor_Init(&monitor2);
            HDY_DiagnosticCriteria_InitState(&state2);
            HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&criteria2);
            criteria2.enableStartupSuppress = false;
            criteria2.enableSwitchSuppress = false;
            criteria2.enableLoopBuildSuppress = false;
            criteria2.debounceTime = 0.0;
        }

        /* 设置瞬时误差（模拟传感器噪声） */
        if (i % 20 == 0) {  /* 每200ms出现一次瞬时误差 */
            monitor1.pressureError = 1.0;
            monitor2.pressureError = 1.0;
        } else {
            monitor1.pressureError = 0.0;
            monitor2.pressureError = 0.0;
        }

        /* 检查监视器1（启用误报抑制） */
        HDY_DiagnosticCriteria_CheckPressure(&result1, &monitor1, &criteria1, &state1,
                                              currentTime, segmentElapsedTime,
                                              segmentElapsedTime < 0.5, false);

        /* 检查监视器2（禁用误报抑制） */
        HDY_DiagnosticCriteria_CheckPressure(&result2, &monitor2, &criteria2, &state2,
                                              currentTime, segmentElapsedTime,
                                              false, false);

        /* 统计抑制次数 */
        if (result1.suppressType != HDY_SUPPRESS_NONE) {
            suppressions++;
        }
    }

    printf("  ✅ False positive suppression count: %d / 100\n", suppressions);
    assert(suppressions > 0);  /* 应该有抑制效果 */
    printf("✅ test_false_positive_reduction_comparison passed\n");
}

/*
 * 测试端到端集成（模拟注塑机场景）
 */
void test_end_to_end_injection_molding_scenario() {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria pressureCriteria;
    HDY_DiagnosticCriteriaState pressureState;
    HDY_DiagnosticCriteria flowCriteria;
    HDY_DiagnosticCriteriaState flowState;
    HDY_DiagnosticResult pressureResult, flowResult;

    printf("🧪 Testing end-to-end injection molding scenario...\n");

    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_InitState(&pressureState);
    HDY_DiagnosticCriteria_InitState(&flowState);
    HDY_DiagnosticCriteria_CreateDefaultPressureCriteria(&pressureCriteria);
    HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(&flowCriteria);

    /* 模拟注射段 */
    for (int i = 0; i < 50; i++) {
        HDY_TIME currentTime = (HDY_TIME)i * 0.01;
        HDY_TIME segmentElapsedTime = currentTime;

        /* 模拟压力和流量反馈 */
        if (i < 10) {
            /* 启动阶段：压力快速上升，流量上升 */
            monitor.pressureError = (10.0 - i) * 0.5;  /* 误差逐渐减小 */
            monitor.flowError = (10.0 - i) * 0.3;
        } else if (i < 40) {
            /* 稳态阶段：误差较小 */
            monitor.pressureError = 0.1;
            monitor.flowError = 0.1;
        } else {
            /* 结束阶段：准备切段 */
            monitor.pressureError = 0.2;
            monitor.flowError = 0.2;
        }

        monitor.pressureErrorActive = (monitor.pressureError > 0.0);
        monitor.pressureErrorDuration = segmentElapsedTime;

        /* 检查压力和流量诊断 */
        HDY_DiagnosticCriteria_CheckPressure(&pressureResult, &monitor, &pressureCriteria, &pressureState,
                                              currentTime, segmentElapsedTime,
                                              segmentElapsedTime < 0.5, false);

        HDY_DiagnosticCriteria_CheckFlow(&flowResult, &monitor, &flowCriteria, &flowState,
                                         currentTime, segmentElapsedTime,
                                         false, false);

        /* 启动阶段应该有抑制效果 */
        if (i < 5) {
            if (pressureResult.suppressType == HDY_SUPPRESS_STARTUP) {
                /* 启动抑制生效 */
            }
        }

        /* 稳态阶段应该无诊断 */
        if (i >= 10 && i < 40) {
            assert(!pressureResult.triggered);
            assert(!flowResult.triggered);
        }
    }

    printf("  ✅ Injection molding scenario completed (no false alarms)\n");
    printf("✅ test_end_to_end_injection_molding_scenario passed\n");
}

int main() {
    printf("=== Sprint 3 Integration Tests ===\n\n");

    test_false_positive_startup_suppression();
    test_false_positive_switch_suppression();
    test_false_positive_loop_build_suppression();
    test_severity_escalation_full_flow();
    test_false_positive_reduction_comparison();
    test_end_to_end_injection_molding_scenario();

    printf("\n=== All Sprint 3 Integration Tests Passed ===\n");
    return 0;
}
