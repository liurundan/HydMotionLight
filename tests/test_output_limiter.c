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

/* 限压 PI（spec 2026-11-26 §4.1）公共输入：G = 20 rpm/(L/min)，泵 1700 rpm → Qmax 85 L/min，
 * Q_v = 50 L/min，Plim = 140 bar，参数 40/41 = 0 → 默认 Kp 1.0、Ki 16，dt = 1 ms。 */
static void init_pressure_limit_input(HYD_OutputLimiterInput* input) {
    memset(input, 0, sizeof(*input));
    input->requestedFlow = 50.0f;
    input->requestedPumpSpeed = 1000.0f;
    input->flowToPumpSpeedGain = 20.0f;
    input->pumpSpeedLimit = 1700.0f;
    input->protectionAction = HYD_PROTECTION_ACTION_NONE;
    input->effectiveMaxPressure = 140.0f;
    input->previousCommandFlow = 50.0f;
    input->deltaTime = 0.001f;
    input->currentTime = 1.0f;
}

static void test_pressure_limit_not_selected_below_limit(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 100.0f;   /* Q_p = 1.0·40 + 50 = 90 > Q_v */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == false);
    assert(output.derated == false);
    assert_real_eq(output.commandFlow, 50.0f, 1e-4f, "below limit keeps Q_v");
    assert_real_eq(output.pumpSpeed, 1000.0f, 1e-3f, "below limit keeps pump speed");
    printf("test_pressure_limit_not_selected_below_limit PASSED\n");
}

static void test_pressure_limit_takes_over_before_limit_when_flow_rises(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.previousCommandFlow = 30.0f;  /* 上一拍 30，本拍 Q_v 50 */
    input.actualPressure = 135.0f;      /* Q_p = 1.0·5 + 30 = 35 < 50 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 35.0f, 1e-4f, "rise limited to previous flow + Kp*e");
    assert_real_eq(output.pumpSpeed, 700.0f, 1e-3f, "pump speed follows limited flow");
    assert_real_eq(state.pressureLimitIntegral, 30.08f, 1e-4f, "integral = 30 + 16*5*0.001");
    printf("test_pressure_limit_takes_over_before_limit_when_flow_rises PASSED\n");
}

static void test_pressure_limit_over_limit_reduces_flow_with_integral(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 150.0f;      /* Q_p = −10 + 50 = 40 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);
    assert_real_eq(output.commandFlow, 40.0f, 1e-4f, "first over-limit scan");
    assert_real_eq(state.pressureLimitIntegral, 49.84f, 1e-4f, "integral = 50 − 16*10*0.001");

    /* 第二拍：沿用自身积分，不再取 previousCommandFlow */
    input.previousCommandFlow = 40.0f;
    input.currentTime = 1.001f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert_real_eq(output.commandFlow, 39.84f, 1e-4f, "second scan uses own integral");
    printf("test_pressure_limit_over_limit_reduces_flow_with_integral PASSED\n");
}

static void test_pressure_limit_output_never_negative(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 300.0f;      /* Q_p = −160 + 50 = −110 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 0.0f, 1e-6f, "motion segment flow never negative");
    assert_real_eq(output.pumpSpeed, 0.0f, 1e-6f, "motion segment pump speed never negative");
    assert_real_eq(state.pressureLimitIntegral, 47.44f, 1e-4f, "integral = 50 − 16*160*0.001");
    printf("test_pressure_limit_output_never_negative PASSED\n");
}

static void test_pressure_limit_uses_configured_gains(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.pressureLimitKp = 2.0f;
    input.pressureLimitKi = 32.0f;
    input.actualPressure = 150.0f;      /* Q_p = 2·(−10) + 50 = 30 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert_real_eq(output.commandFlow, 30.0f, 1e-4f, "configured Kp");
    assert_real_eq(state.pressureLimitIntegral, 49.68f, 1e-4f, "configured Ki");
    printf("test_pressure_limit_uses_configured_gains PASSED\n");
}

/* 积分对象 dP/dt = Kq_f·(Q − P/R_f)，Kq_f = 64 bar/s/(L/min)，R_f = 104 bar/(L/min)，
 * 执行流量滞后一拍。Q_v = 10 L/min 堵转（油阀全关），默认增益。 */
static void test_pressure_limit_closed_loop_settles_at_limit_and_releases(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;
    HYD_REAL pressure = 60.0f;
    HYD_REAL applied = 10.0f;
    HYD_REAL peak = 0.0f;
    HYD_REAL previous;
    int step;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.requestedFlow = 10.0f;
    input.requestedPumpSpeed = 200.0f;
    input.previousCommandFlow = 10.0f;

    for (step = 0; step < 2000; ++step) {
        input.actualPressure = pressure;
        input.currentTime = 1.0f + 0.001f * (HYD_REAL)step;
        HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
        pressure += 64.0f * (applied - pressure / 104.0f) * 0.001f;
        applied = output.commandFlow;
        input.previousCommandFlow = output.commandFlow;
        if (pressure > peak) {
            peak = pressure;
        }
    }
    assert(output.pressureLimitActive == true);
    assert(fabs(pressure - 140.0f) < 0.5f);
    assert(peak < 154.0f);

    /* 负载下降：压力以 200 bar/s 降到 100 bar 并保持，流量连续回升到 Q_v，
     * 单拍增量不超过 Kp·|ΔP| + Ki·e·dt。 */
    previous = output.commandFlow;
    for (step = 0; step < 300; ++step) {
        HYD_REAL lastPressure = pressure;
        pressure = (pressure - 0.2f > 100.0f) ? pressure - 0.2f : 100.0f;
        input.actualPressure = pressure;
        HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
        assert(output.commandFlow >= previous - 1e-5f);
        assert(output.commandFlow - previous <=
               1.0f * (lastPressure - pressure) + 16.0f * (140.0f - pressure) * 0.001f + 1e-4f);
        previous = output.commandFlow;
        input.previousCommandFlow = output.commandFlow;
    }
    assert(output.pressureLimitActive == false);
    assert_real_eq(output.commandFlow, 10.0f, 1e-4f, "released back to Q_v");
    printf("test_pressure_limit_closed_loop_settles_at_limit_and_releases PASSED (peak %.2f bar)\n",
           (double)peak);
}

static void test_pressure_limit_and_soft_limit_take_minimum(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    /* 软限位：remaining 2 / band 5 → Q_v = 50 × 0.4 = 20 */
    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPosition = 98.0f;
    input.strokeMm = 100.0f;
    input.softLimitBandMm = 5.0f;
    input.direction = HYD_DIRECTION_EXTEND;

    /* 压力 145：Q_p = −5 + 50 = 45 > 20 → 软限位决定输出 */
    input.actualPressure = 145.0f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.softLimitActive == true);
    assert(output.pressureLimitActive == false);
    assert_real_eq(output.commandFlow, 20.0f, 1e-4f, "soft limit wins");

    /* 压力 180：Q_p = −40 + 50 = 10 < 20 → 限压决定输出 */
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 180.0f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.softLimitActive == false);
    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 10.0f, 1e-4f, "pressure limit wins");
    printf("test_pressure_limit_and_soft_limit_take_minimum PASSED\n");
}

int main(void) {
    test_derate_halves_flow_and_speed();
    test_no_derate_preserves_outputs();
    test_stop_forces_safe_zero();
    test_invalid_ratio_uses_default_derate();
    test_segment_derate_ratio_overrides_default();
    test_pressure_limit_not_selected_below_limit();
    test_pressure_limit_takes_over_before_limit_when_flow_rises();
    test_pressure_limit_over_limit_reduces_flow_with_integral();
    test_pressure_limit_output_never_negative();
    test_pressure_limit_uses_configured_gains();
    test_pressure_limit_closed_loop_settles_at_limit_and_releases();
    test_pressure_limit_and_soft_limit_take_minimum();
    test_soft_limit_extend_deceleration();
    test_soft_limit_does_not_block_retract();
    test_no_protection_when_disabled();
    test_allow_negative_flow_passes_negative_input();
    test_allow_negative_flow_clamps_to_min();
    test_allow_negative_flow_false_clamps_to_zero();

    printf("test_output_limiter passed\n");
    return 0;
}
