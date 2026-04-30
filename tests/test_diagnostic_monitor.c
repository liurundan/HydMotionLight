#include "../include/diagnostics_monitor.h"
#include "../include/common_types.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

#define EPSILON 1e-6

void test_error_monitor_init() {
    HYD_ErrorMonitor monitor;

    HYD_ErrorMonitor_Init(&monitor);

    assert(monitor.positionError == 0.0);
    assert(monitor.velocityError == 0.0);
    assert(monitor.flowError == 0.0);
    assert(monitor.pressureError == 0.0);

    assert(!monitor.positionErrorActive);
    assert(!monitor.velocityErrorActive);
    assert(!monitor.flowErrorActive);
    assert(!monitor.pressureErrorActive);

    assert(monitor.sampleCount == 0);

    printf("✅ test_error_monitor_init passed\n");
}

void test_error_monitor_pressure_error() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;

    HYD_ErrorMonitor_Init(&monitor);

    /* 模拟压力误差 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;  /* 实测压力 */
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;  /* 参考压力 */
    references.flowReference = 6.0;
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);

    /* 验证压力误差 = 12.0 - 10.0 = 2.0 */
    assert(fabs(monitor.pressureError - 2.0) < EPSILON);
    assert(monitor.pressureErrorActive);

    printf("✅ test_error_monitor_pressure_error passed\n");
}

void test_error_monitor_flow_error() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;

    HYD_ErrorMonitor_Init(&monitor);

    /* 模拟流量误差 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = -5.0;  /* 实测流量（有符号） */
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 10.0;  /* 参考流量（幅值） */
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);

    /* 验证流量误差 = 10.0 - |-5.0| = 5.0 */
    assert(fabs(monitor.flowError - 5.0) < EPSILON);
    assert(monitor.flowErrorActive);

    printf("✅ test_error_monitor_flow_error passed\n");
}

void test_error_monitor_velocity_error() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;

    HYD_ErrorMonitor_Init(&monitor);

    /* 模拟速度误差 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;  /* 实测速度 */
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 6.0;
    references.velocityReference = 15.0;  /* 参考速度 */
    references.elapsedTime = 0.1;

    HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);

    /* 验证速度误差 = 15.0 - 10.0 = 5.0 */
    assert(fabs(monitor.velocityError - 5.0) < EPSILON);
    assert(monitor.velocityErrorActive);

    printf("✅ test_error_monitor_velocity_error passed\n");
}

void test_error_monitor_duration_tracking() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;
    int i;

    HYD_ErrorMonitor_Init(&monitor);

    /* 设置初始误差状态 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;  /* 压力误差2.0 */
    references.flowReference = 6.0;
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    /* 模拟10个周期的误差持续 */
    for (i = 0; i < 10; i++) {
        currentTime += 0.05;
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);
    }

    /* 验证持续时间 = 0.5秒 */
    assert(fabs(monitor.pressureErrorDuration - 0.45) < 0.01);

    printf("✅ test_error_monitor_duration_tracking passed\n");
}

void test_error_monitor_statistics() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;
    int i;

    HYD_ErrorMonitor_Init(&monitor);

    /* 模拟变化的压力误差 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 6.0;
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    /* 第一批采样：压力误差2.0 */
    for (i = 0; i < 5; i++) {
        currentTime += 0.01;
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);
    }

    /* 第二批采样：压力误差1.0 */
    axisRef.pressure = 11.0;
    for (i = 0; i < 5; i++) {
        currentTime += 0.01;
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);
    }

    /* 验证统计信息 */
    assert(monitor.sampleCount == 10);
    assert(fabs(monitor.maxPressureError - 2.0) < EPSILON);
    assert(fabs(monitor.minPressureError - 1.0) < EPSILON);
    assert(fabs(monitor.avgPressureError - 1.5) < 0.1);

    printf("✅ test_error_monitor_statistics passed\n");
}

void test_error_monitor_reset_statistics() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;
    int i;

    HYD_ErrorMonitor_Init(&monitor);

    /* 生成一些统计数据 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 6.0;
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    for (i = 0; i < 10; i++) {
        currentTime += 0.01;
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);
    }

    assert(monitor.sampleCount > 0);
    assert(monitor.pressureErrorActive);

    /* 重置统计 */
    HYD_ErrorMonitor_ResetStatistics(&monitor);

    /* 验证统计已清除，但误差状态保留 */
    assert(monitor.sampleCount == 0);
    assert(monitor.maxPressureError == 0.0);
    assert(monitor.minPressureError == 0.0);
    assert(monitor.avgPressureError == 0.0);
    assert(monitor.pressureErrorActive);  /* 状态仍保留 */

    printf("✅ test_error_monitor_reset_statistics passed\n");
}

void test_error_monitor_reset() {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_TIME currentTime = 0.0;
    int i;

    HYD_ErrorMonitor_Init(&monitor);

    /* 生成一些数据 */
    axisRef.position = 100.0;
    axisRef.velocity = 10.0;
    axisRef.flow = 5.0;
    axisRef.pressure = 10.0;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 6.0;
    references.velocityReference = 12.0;
    references.elapsedTime = 0.1;

    for (i = 0; i < 10; i++) {
        currentTime += 0.01;
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &references, currentTime);
    }

    assert(monitor.sampleCount > 0);
    assert(monitor.pressureErrorActive);

    /* 完全重置 */
    HYD_ErrorMonitor_Reset(&monitor);

    /* 验证所有数据已清除 */
    assert(monitor.sampleCount == 0);
    assert(monitor.pressureError == 0.0);
    assert(monitor.velocityError == 0.0);
    assert(monitor.flowError == 0.0);
    assert(!monitor.pressureErrorActive);
    assert(!monitor.velocityErrorActive);

    printf("✅ test_error_monitor_reset passed\n");
}

int main() {
    printf("=== Error Monitor Tests ===\n\n");

    test_error_monitor_init();
    test_error_monitor_pressure_error();
    test_error_monitor_flow_error();
    test_error_monitor_velocity_error();
    test_error_monitor_duration_tracking();
    test_error_monitor_statistics();
    test_error_monitor_reset_statistics();
    test_error_monitor_reset();

    printf("\n=== All Error Monitor Tests Passed ===\n");
    return 0;
}
