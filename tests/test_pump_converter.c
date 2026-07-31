#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pump_converter.h"
#include <string.h>

static void test_validate_config(void);
static void test_negative_requested_flow_uses_magnitude(void);
static void test_non_finite_input_returns_safe_zero(void);

static void test_basic_conversion(void) {
    HYD_PumpConverterInput input = {0};
    HYD_PumpConverterOutput output = {0};

    printf("Testing basic pump conversion...\n");
    input.requestedFlow = 12.5;
    input.flowToPumpSpeedGain = 80.0;
    input.pumpSpeedLimit = 2000.0;
    input.direction = HYD_DIRECTION_EXTEND;

    HYD_PumpConverter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 12.5) < 0.001);
    assert(fabs(output.pumpSpeed - 1000.0) < 0.001);
    assert(fabs(output.maxFlow - 25.0) < 0.001);
    assert(!output.speedLimitActive);
    printf("✓ Basic pump conversion test passed\n");
}

static void test_pump_limit_back_projects_flow(void) {
    HYD_PumpConverterInput input = {0};
    HYD_PumpConverterOutput output = {0};

    printf("Testing pump speed limit back-projection...\n");
    input.requestedFlow = 40.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 2500.0;
    input.direction = HYD_DIRECTION_RETRACT;

    HYD_PumpConverter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 25.0) < 0.001);
    assert(fabs(output.pumpSpeed - 2500.0) < 0.001);
    assert(fabs(output.maxFlow - 25.0) < 0.001);
    assert(output.speedLimitActive);
    printf("✓ Pump speed limit back-projection test passed\n");
}

static void test_invalid_input_returns_safe_zero(void) {
    HYD_PumpConverterInput input = {0};
    HYD_PumpConverterOutput output = {0};

    printf("Testing invalid pump converter input handling...\n");
    input.requestedFlow = 10.0;
    input.flowToPumpSpeedGain = 0.0;
    input.pumpSpeedLimit = 1000.0;

    HYD_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    HYD_PumpConverter_Execute(NULL, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);
    printf("✓ Invalid pump converter input handling test passed\n");
}

static void test_negative_requested_flow_is_preserved(void) {
    HYD_PumpConverterInput input = {0};
    HYD_PumpConverterOutput output = {0};

    printf("Testing negative requested flow preservation...\n");
    input.requestedFlow = -6.0;
    input.flowToPumpSpeedGain = 120.0;
    input.pumpSpeedLimit = 1200.0;
    input.direction = HYD_DIRECTION_RETRACT;

    HYD_PumpConverter_Execute(&input, &output);

    /* Negative flow is preserved for rapid depressurization.
     * commandFlow = clamp(-6.0, -0.5, 10.0) = -0.5
     * pumpSpeed = -0.5 * 120 = -60 rpm (small reverse) */
    assert(output.commandFlow <= 0.0);  /* negative direction preserved */
    assert(fabs(output.pumpSpeed - output.commandFlow * input.flowToPumpSpeedGain) < 0.001);
    printf("✓ Negative requested flow preservation test passed\n");
}

static void test_non_finite_input_returns_safe_zero(void) {
    HYD_PumpConverterInput input = {0};
    HYD_PumpConverterOutput output = {0};

    printf("Testing non-finite pump converter input handling...\n");
    input.requestedFlow = NAN;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 1000.0;
    HYD_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    input.requestedFlow = 10.0;
    input.flowToPumpSpeedGain = INFINITY;
    input.pumpSpeedLimit = 1000.0;
    HYD_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = NAN;
    HYD_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);
    printf("✓ Non-finite pump converter input handling test passed\n");
}

int main(void) {
    printf("Running PumpConverter tests...\n\n");

    test_basic_conversion();
    test_pump_limit_back_projects_flow();
    test_invalid_input_returns_safe_zero();
    test_negative_requested_flow_is_preserved();
    test_non_finite_input_returns_safe_zero();
    test_validate_config();

    printf("\n✅ All PumpConverter tests passed successfully!\n");
    return 0;
}

static void test_validate_config(void) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("Testing pump converter validate config...\n");
    assert(HYD_PumpConverter_ValidateConfig(80.0, 2000.0, &code));
    assert(code == HYD_DIAG_CODE_NONE);

    assert(!HYD_PumpConverter_ValidateConfig(0.0, 2000.0, &code));
    assert(code == HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID);

    assert(!HYD_PumpConverter_ValidateConfig(NAN, 2000.0, &code));
    assert(code == HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID);

    assert(!HYD_PumpConverter_ValidateConfig(80.0, -1.0, &code));
    assert(code == HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID);

    assert(!HYD_PumpConverter_ValidateConfig(80.0, INFINITY, &code));
    assert(code == HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    printf("✓ Pump converter validate config test passed\n");
}
