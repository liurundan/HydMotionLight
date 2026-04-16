#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pump_converter.h"

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

int main(void) {
    printf("Running PumpConverter tests...\n\n");

    test_basic_conversion();
    test_pump_limit_back_projects_flow();
    test_invalid_input_returns_safe_zero();

    printf("\n✅ All PumpConverter tests passed successfully!\n");
    return 0;
}
