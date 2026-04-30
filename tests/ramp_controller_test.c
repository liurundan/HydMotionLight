#include <stdio.h>
#include <assert.h>
#include "ramp_controller.h"

void test_ramp_controller_linear_ramp() {
    printf("Testing ramp controller linear ramp...\n");

    HYD_RampController controller;
    HYD_RampController_Init(&controller, 0.0, 0.0);

    HYD_RampControllerInput input = {0};
    HYD_RampControllerOutput output = {0};

    input.targetPressure = 10.0;
    input.rampRate = 2.0;
    input.currentTime = 1.0;
    HYD_RampController_Execute(&controller, &input, &output);
    assert(output.rampedPressure == 2.0);

    input.currentTime = 2.0;
    HYD_RampController_Execute(&controller, &input, &output);
    assert(output.rampedPressure == 4.0);

    input.currentTime = 6.0;
    HYD_RampController_Execute(&controller, &input, &output);
    assert(output.rampedPressure == 10.0);

    printf("✓ Ramp controller linear ramp test passed\n");
}

void test_ramp_controller_instant_target() {
    printf("Testing ramp controller instant target...\n");

    HYD_RampController controller;
    HYD_RampController_Init(&controller, 50.0, 0.0);

    HYD_RampControllerInput input = {0};
    HYD_RampControllerOutput output = {0};

    input.targetPressure = 75.0;
    input.rampRate = 0.0;
    input.currentTime = 1.0;
    HYD_RampController_Execute(&controller, &input, &output);
    assert(output.rampedPressure == 75.0);

    printf("✓ Ramp controller instant target test passed\n");
}

int main() {
    printf("Running RampController tests...\n\n");

    test_ramp_controller_linear_ramp();
    test_ramp_controller_instant_target();

    printf("\n✅ All RampController tests passed successfully!\n");
    return 0;
}
