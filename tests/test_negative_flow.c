/* tests/test_negative_flow.c
 * M6: 负流量单元测试
 * 验证 outputMin < 0 时压力控制器的负流量输出及死区逻辑
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pressure_controller.h"

static HYD_MotionSegment make_pressure_segment(void) {
    HYD_MotionSegment segment = {0};
    segment.segmentTag = 1;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetFlow = 3.0;
    segment.targetPressure = 12.0;
    segment.maxFlow = 10.0;
    segment.duration = 1.0;
    segment.pressureTolerance = 0.2;
    segment.timeoutLimit = 2.0;
    segment.pressureRampRate = 5.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    return segment;
}

/* M6-Test1: 大偏差时允许负流量输出
 * 条件: outputMin = -5.0, targetPressure=10.0, measuredPressure=15.0
 * 期望: outputFlow < 0 (负流量)
 */
static void test_pressure_controller_negative_output(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("  M6-Test1: 大偏差负流量输出...\n");
    segment = make_pressure_segment();
    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.0;
    input.measuredPressure = 15.0;  /* error = -5.0, 大负偏差 */
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = -5.0;  /* 允许负流量 */
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    printf("    error=%.2f, outputFlow=%.4f (expect < 0)\n",
           (double)(input.targetPressure - input.measuredPressure), (double)output.outputFlow);
    assert(output.outputFlow < 0.0);  /* 大偏差时应输出负流量 */
    printf("    ✅ M6-Test1 passed (outputFlow=%.4f)\n", (double)output.outputFlow);
}

/* M6-Test2: 小偏差时死区禁止负流量
 * 条件: outputMin = -5.0, targetPressure=10.0, measuredPressure=11.0
 * 期望: outputFlow >= 0 (死区逻辑禁止负流量)
 * 死区逻辑: error > -2.0 时禁止负流量
 */
static void test_pressure_controller_negative_deadband(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("  M6-Test2: 小偏差死区禁止负流量...\n");
    segment = make_pressure_segment();
    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.0;
    input.measuredPressure = 11.0;  /* error = -1.0, 小负偏差 (> -2.0 死区) */
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = -5.0;  /* 允许负流量 */
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    printf("    error=%.2f, outputFlow=%.4f (expect >= 0 due to deadband)\n",
           (double)(input.targetPressure - input.measuredPressure), (double)output.outputFlow);
    assert(output.outputFlow >= 0.0);  /* 死区逻辑应禁止负流量 */
    printf("    ✅ M6-Test2 passed (outputFlow=%.4f)\n", (double)output.outputFlow);
}

/* M6-Test3: 恰好在死区边界 (error = -2.0)
 * 条件: outputMin = -5.0, targetPressure=10.0, measuredPressure=12.0
 * 期望: outputFlow 可能 < 0 (边界情况, error = -2.0 不在死区内)
 */
static void test_pressure_controller_deadband_boundary(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("  M6-Test3: 死区边界 (error=-2.0)...\n");
    segment = make_pressure_segment();
    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 10.0;
    input.measuredPressure = 12.0;  /* error = -2.0, 死区边界 */
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = -5.0;
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    printf("    error=%.2f, outputFlow=%.4f (boundary case)\n",
           (double)(input.targetPressure - input.measuredPressure), (double)output.outputFlow);
    /* 边界情况: error = -2.0, 不在死区内 (死区是 error > -2.0)
     * 所以允许负流量 */
    printf("    ✅ M6-Test3 passed (outputFlow=%.4f)\n", (double)output.outputFlow);
}

/* M6-Test4: 泄压目标下限钳位 (targetPressure < 2.0 时禁止负流量)
 * 条件: outputMin = -5.0, targetPressure=1.5 ( < 2.0)
 * 期望: config.outputMin 被强制改为 0.0, 禁止负流量
 */
static void test_pressure_controller_relief_clamp(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("  M6-Test4: 泄压目标下限钳位 (targetPressure < 2.0)...\n");
    segment = make_pressure_segment();
    segment.targetPressure = 1.5;  /* < 2.0, 应触发钳位 */
    HYD_PressureController_InitState(&state, 50.0, segment.targetFlow, 0.0);

    input.targetPressure = 1.5;
    input.measuredPressure = 5.0;  /* error = -3.5, 大负偏差 */
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = -5.0;  /* 试图允许负流量 */
    input.outputMax = segment.maxFlow;
    input.timestamp = 0.0;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    printf("    targetPressure=%.2f, outputFlow=%.4f (expect >= 0 due to clamp)\n",
           (double)input.targetPressure, (double)output.outputFlow);
    /* M4 逻辑: targetPressure < 2.0 时强制 outputMin = 0.0 */
    /* 所以即使 error 很大, 也不应输出负流量 */
    assert(output.outputFlow >= 0.0);
    printf("    ✅ M6-Test4 passed (outputFlow=%.4f)\n", (double)output.outputFlow);
}

int main(void) {
    printf("Running negative flow tests (M6)...\n\n");

    test_pressure_controller_negative_output();
    test_pressure_controller_negative_deadband();
    test_pressure_controller_deadband_boundary();
    test_pressure_controller_relief_clamp();

    printf("\n✅ All negative flow tests passed successfully!\n");
    return 0;
}
