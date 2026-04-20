#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pump_converter.h"
#include <string.h>

static void test_validate_config(void);
static void test_negative_requested_flow_uses_magnitude(void);
static void test_non_finite_input_returns_safe_zero(void);

static void test_basic_conversion(void) {
    HDY_PumpConverterInput input = {0};
    HDY_PumpConverterOutput output = {0};

    printf("Testing basic pump conversion...\n");
    input.requestedFlow = 12.5;
    input.flowToPumpSpeedGain = 80.0;
    input.pumpSpeedLimit = 2000.0;
    input.direction = HDY_DIRECTION_EXTEND;

    HDY_PumpConverter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 12.5) < 0.001);
    assert(fabs(output.pumpSpeed - 1000.0) < 0.001);
    printf("✓ Basic pump conversion test passed\n");
}

static void test_pump_limit_back_projects_flow(void) {
    HDY_PumpConverterInput input = {0};
    HDY_PumpConverterOutput output = {0};

    printf("Testing pump speed limit back-projection...\n");
    input.requestedFlow = 40.0;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 2500.0;
    input.direction = HDY_DIRECTION_RETRACT;

    HDY_PumpConverter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 25.0) < 0.001);
    assert(fabs(output.pumpSpeed - 2500.0) < 0.001);
    printf("✓ Pump speed limit back-projection test passed\n");
}

static void test_invalid_input_returns_safe_zero(void) {
    HDY_PumpConverterInput input = {0};
    HDY_PumpConverterOutput output = {0};

    printf("Testing invalid pump converter input handling...\n");
    input.requestedFlow = 10.0;
    input.flowToPumpSpeedGain = 0.0;
    input.pumpSpeedLimit = 1000.0;

    HDY_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    HDY_PumpConverter_Execute(NULL, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);
    printf("✓ Invalid pump converter input handling test passed\n");
}

static void test_negative_requested_flow_uses_magnitude(void) {
    HDY_PumpConverterInput input = {0};
    HDY_PumpConverterOutput output = {0};

    printf("Testing negative requested flow normalization...\n");
    input.requestedFlow = -6.0;
    input.flowToPumpSpeedGain = 120.0;
    input.pumpSpeedLimit = 1200.0;
    input.direction = HDY_DIRECTION_RETRACT;

    HDY_PumpConverter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 6.0) < 0.001);
    assert(fabs(output.pumpSpeed - 720.0) < 0.001);
    printf("✓ Negative requested flow normalization test passed\n");
}

static void test_non_finite_input_returns_safe_zero(void) {
    HDY_PumpConverterInput input = {0};
    HDY_PumpConverterOutput output = {0};

    printf("Testing non-finite pump converter input handling...\n");
    input.requestedFlow = NAN;
    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = 1000.0;
    HDY_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    input.requestedFlow = 10.0;
    input.flowToPumpSpeedGain = INFINITY;
    input.pumpSpeedLimit = 1000.0;
    HDY_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);

    input.flowToPumpSpeedGain = 100.0;
    input.pumpSpeedLimit = NAN;
    HDY_PumpConverter_Execute(&input, &output);
    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);
    printf("✓ Non-finite pump converter input handling test passed\n");
}

int main(void) {
    printf("Running PumpConverter tests...\n\n");

    test_basic_conversion();
    test_pump_limit_back_projects_flow();
    test_invalid_input_returns_safe_zero();
    test_negative_requested_flow_uses_magnitude();
    test_non_finite_input_returns_safe_zero();
    test_validate_config();

    printf("\n✅ All PumpConverter tests passed successfully!\n");
    return 0;
}

static void test_validate_config(void) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    printf("Testing pump converter validate config...\n");
    assert(HDY_PumpConverter_ValidateConfig(80.0, 2000.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_NONE);
    assert(message[0] == '\0');

    assert(!HDY_PumpConverter_ValidateConfig(0.0, 2000.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL || message[0] != '\0');

    assert(!HDY_PumpConverter_ValidateConfig(NAN, 2000.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL || message[0] != '\0');

    assert(!HDY_PumpConverter_ValidateConfig(80.0, -1.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(message, "PUMP_SPEED_LIMIT") != NULL || message[0] != '\0');

    assert(!HDY_PumpConverter_ValidateConfig(80.0, INFINITY, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(message, "PUMP_SPEED_LIMIT") != NULL || message[0] != '\0');
    printf("✓ Pump converter validate config test passed\n");
}
