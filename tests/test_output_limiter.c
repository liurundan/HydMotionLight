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

int main(void) {
    test_derate_halves_flow_and_speed();
    test_no_derate_preserves_outputs();
    test_stop_forces_safe_zero();
    test_invalid_ratio_uses_default_derate();

    printf("test_output_limiter passed\n");
    return 0;
}
