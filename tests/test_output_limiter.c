#include "output_limiter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void assert_real_eq(HYD_REAL actual, HYD_REAL expected, HYD_REAL tolerance, const char* message) {
    if (fabs(actual - expected) > tolerance) {
        fprintf(stderr,
                "%s: expected %.6f, got %.6f (tol %.6f)\n",
                message,
                (double)expected,
                (double)actual,
                (double)tolerance);
        assert(0);
    }
}

static void test_derate_halves_flow_and_speed(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = 12.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, 6.0, 0.001, "Derate should halve flow");
    assert_real_eq(output.pumpSpeed, 600.0, 0.001, "Derate should halve pump speed");
    assert(output.derated);
}

static void test_no_derate_preserves_outputs(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = 8.5;
    input.requestedPumpSpeed = 850.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, 8.5, 0.001, "No derate should preserve flow");
    assert_real_eq(output.pumpSpeed, 850.0, 0.001, "No derate should preserve speed");
    assert(!output.derated);
}

static void test_stop_forces_safe_zero(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = 8.5;
    input.requestedPumpSpeed = 850.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_STOP;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, 0.0, 0.001, "STOP should force zero flow");
    assert_real_eq(output.pumpSpeed, 0.0, 0.001, "STOP should force zero speed");
    assert(!output.derated);
}

static void test_invalid_ratio_uses_default_derate(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = 20.0;
    input.requestedPumpSpeed = 2000.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    input.derateRatio = NAN;

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, 10.0, 0.001, "Invalid ratio should use default flow derate");
    assert_real_eq(output.pumpSpeed, 1000.0, 0.001, "Invalid ratio should use default speed derate");
    assert(output.derated);
}

static void test_segment_derate_ratio_overrides_default(void) {
    HYD_OutputLimiterInput in;
    HYD_OutputLimiterOutput out;

    memset(&in, 0, sizeof(in));
    in.requestedFlow = 20.0;
    in.requestedPumpSpeed = 1000.0;
    in.flowToPumpSpeedGain = 50.0;
    in.pumpSpeedLimit = 5000.0;
    in.protectionAction = HYD_PROTECTION_ACTION_DERATE;

    /* derateRatio = 0 -> default 0.5 */
    in.derateRatio = 0.0;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 10.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 500.0) < 1e-6);

    /* derateRatio = 0.3 -> 30% of requested */
    in.derateRatio = 0.3;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 6.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 300.0) < 1e-6);

    /* derateRatio = 0.9 -> 90% (gentle derate) */
    in.derateRatio = 0.9;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 18.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 900.0) < 1e-6);

    /* derateRatio = 1.5 (out of range) -> fall back to default 0.5 */
    in.derateRatio = 1.5;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 10.0) < 1e-6);

    printf("test_segment_derate_ratio_overrides_default PASSED\n");
}

static void test_pressure_limit_proportional_reduction(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.derateRatio = 0.0;

    /* 22 bar / 20 bar: overRatio=10%, scale = 1.0 - 3.0*0.1 = 0.7 */
    input.actualPressure = 22.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0; /* 软限位不启用 */
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert(output.softLimitActive == false);
    /* 100.0 * 0.7 = 70.0 */
    assert_real_eq(output.commandFlow, 70.0, 0.01, "pressure limit 10% over in bar domain");
    assert_real_eq(output.pumpSpeed, 1050.0, 0.01, "pump speed 10% over in bar domain");
    printf("test_pressure_limit_proportional_reduction PASSED\n");
}

static void test_pressure_limit_zero_scale_clamp(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 30 bar / 20 bar: overRatio=50%, scale = 1.0 - 3.0*0.5 = -0.5 -> clamp to 0.0 */
    input.actualPressure = 30.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 0.0, 0.01, "pressure limit zero scale");
    assert_real_eq(output.pumpSpeed, 0.0, 0.01, "pressure limit zero pump speed");
    printf("test_pressure_limit_zero_scale_clamp PASSED\n");
}

static void test_soft_limit_extend_deceleration(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 80.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 位置 97mm, 行程 100mm, 减速带 5mm → remaining=3, scale=3/5=0.6 */
    input.actualPosition = 97.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_EXTEND;
    input.effectiveMaxPressure = 0.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == true);
    assert(output.pressureLimitActive == false);
    /* 80.0 * 0.6 = 48.0 */
    assert_real_eq(output.commandFlow, 48.0, 0.01, "soft limit extend");
    printf("test_soft_limit_extend_deceleration PASSED\n");
}

static void test_soft_limit_does_not_block_retract(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 80.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 位置 99mm（接近正向极限），但方向是 RETRACT → 不限制 */
    input.actualPosition = 99.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_RETRACT;
    input.effectiveMaxPressure = 0.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == false);
    assert_real_eq(output.commandFlow, 80.0, 0.01, "retract not blocked");
    printf("test_soft_limit_does_not_block_retract PASSED\n");
}

static void test_protection_takes_min_scale(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 压力限制: 超压10% → scale=0.7, 软限位: remaining=2, band=5 → scale=0.4, 最终取 min=0.4 */
    input.actualPressure = 22.0;
    input.effectiveMaxPressure = 20.0;
    input.actualPosition = 98.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_EXTEND;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == true);
    /* 100.0 * 0.4 = 40.0 */
    assert_real_eq(output.commandFlow, 40.0, 0.01, "min of two scales");
    printf("test_protection_takes_min_scale PASSED\n");
}

static void test_pressure_limit_stays_non_fault_and_hysteretic(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    HYD_OutputLimiter_ResetState(&state);

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.actualPressure = 25.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0;

    /* t=0: 首次超压，保持限压状态但不产生诊断。 */
    input.currentTime = 0.0;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);

    /* t=0.25s: 持续超压仍不升级为 WARNING/FAULT。 */
    input.currentTime = 0.25;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);

    /* t=1.3s: 即使长期超压，也保持非故障限压。 */
    input.currentTime = 1.3;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);

    /* 限制点附近抖动时，回差保持限压状态，不反复恢复满流量。 */
    input.actualPressure = 19.9;
    input.currentTime = 1.31;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.commandFlow < 100.0);

    input.actualPressure = 20.1;
    input.currentTime = 1.32;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.commandFlow < 100.0);

    /* 低于限制值减回差后，输出按速率逐步恢复；恢复完成才解除限压。 */
    input.actualPressure = 19.5;
    input.currentTime = 2.0;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.commandFlow < 100.0);

    for (int step = 1; step <= 5; ++step) {
        input.currentTime = (HYD_TIME)(2.0 + step * 0.1);
        HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    }
    assert(output.pressureLimitActive == false);
    assert_real_eq(output.commandFlow, 100.0, 0.01, "pressure limit hysteresis release");

    printf("test_pressure_limit_stays_non_fault_and_hysteretic PASSED\n");
}

static void test_no_protection_when_disabled(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    HYD_OutputLimiter_ResetState(&state);

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 两者都为 0：不启用任何保护 */
    input.effectiveMaxPressure = 0.0;
    input.strokeMm = 0.0;
    input.actualPressure = 999.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == false);
    assert(output.softLimitActive == false);
    assert_real_eq(output.commandFlow, 100.0, 0.01, "backward compat");
    printf("test_no_protection_when_disabled PASSED\n");
}

static void test_allow_negative_flow_passes_negative_input(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = -1.0;
    input.requestedPumpSpeed = -100.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.allowNegativeFlow = true;

    HYD_OutputLimiter_Execute(&input, &output);

    /* -1.0 > minFlow(-30), -100.0 > minSpeed(-3000) -> 原值通过 */
    assert_real_eq(output.commandFlow, -1.0, 0.001, "negative flow should pass");
    assert_real_eq(output.pumpSpeed, -100.0, 0.001, "negative speed should pass");
    printf("test_allow_negative_flow_passes_negative_input PASSED\n");
}

static void test_allow_negative_flow_clamps_to_min(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;
    HYD_REAL minSpeed, minFlow;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = -40.0;
    input.requestedPumpSpeed = -4000.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.allowNegativeFlow = true;

    /* 计算期望下限 */
    minSpeed = -input.pumpSpeedLimit;  /* -3000.0 */
    minFlow  = minSpeed / input.flowToPumpSpeedGain; /* -30.0 */

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, minFlow, 0.001, "flow should clamp to minFlow");
    assert_real_eq(output.pumpSpeed, minSpeed, 0.001, "speed should clamp to minSpeed");
    printf("test_allow_negative_flow_clamps_to_min PASSED\n");
}

static void test_allow_negative_flow_false_clamps_to_zero(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    input.requestedFlow = -5.0;
    input.requestedPumpSpeed = -500.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.allowNegativeFlow = false;

    HYD_OutputLimiter_Execute(&input, &output);

    assert_real_eq(output.commandFlow, 0.0, 0.001, "negative flow should clamp to 0");
    assert_real_eq(output.pumpSpeed, 0.0, 0.001, "negative speed should clamp to 0");
    printf("test_allow_negative_flow_false_clamps_to_zero PASSED\n");
}

int main(void) {
    test_derate_halves_flow_and_speed();
    test_no_derate_preserves_outputs();
    test_stop_forces_safe_zero();
    test_invalid_ratio_uses_default_derate();
    test_segment_derate_ratio_overrides_default();
    test_pressure_limit_proportional_reduction();
    test_pressure_limit_zero_scale_clamp();
    test_soft_limit_extend_deceleration();
    test_soft_limit_does_not_block_retract();
    test_protection_takes_min_scale();
    test_pressure_limit_stays_non_fault_and_hysteretic();
    test_no_protection_when_disabled();
    test_allow_negative_flow_passes_negative_input();
    test_allow_negative_flow_clamps_to_min();
    test_allow_negative_flow_false_clamps_to_zero();

    printf("test_output_limiter passed\n");
    return 0;
}
