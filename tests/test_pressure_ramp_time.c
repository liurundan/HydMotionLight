#include <math.h>
#include <stdio.h>
#include "ramp_controller.h"

static int failures;

static void check(int condition, const char *message)
{
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", message);
    }
}

int main(void)
{
    HYD_RampController controller;
    HYD_RampControllerInput input = {0};
    HYD_RampControllerOutput output = {0};
    HYD_REAL previous;

    HYD_RampController_Init(&controller, 0.0, 0.0);
    input.targetPressure = 100.0;
    input.rampRate = 20.0;
    for (int cycle = 0; cycle < 20; ++cycle) {
        input.currentTime = (cycle + 1) * 0.001;
        previous = controller.rampedPressure;
        HYD_RampController_ExecuteWithDt(&controller, &input, 0.001, &output);
        check(output.rampedPressure >= previous, "pressure ramp should be monotonic");
        check(output.rampedPressure <= 100.0, "pressure ramp should not overshoot target");
    }
    check(fabs(output.rampedPressure - 100.0) < 1e-4,
          "20 ms pressure transition should reach target in 20 cycles");

    input.targetPressure = 40.0;
    previous = controller.rampedPressure;
    HYD_RampController_ExecuteWithDt(&controller, &input, 0.001, &output);
    check(output.rampedPressure < previous && output.rampedPressure > 40.0,
          "target update should restart from current ramp value without a jump");

    HYD_RampController_Init(&controller, 0.0, 0.0);
    input.targetPressure = 10.0;
    input.rampRate = 0.0;
    HYD_RampController_ExecuteWithDt(&controller, &input, 0.001, &output);
    check(output.rampedPressure > 0.0 && output.rampedPressure < 10.0,
          "zero pressure time should use the 20 ms default");

    HYD_RampController_Init(&controller, 0.0, 0.0);
    input.targetPressure = 10.0;
    input.rampRate = 5.0;
    for (int cycle = 0; cycle < 5; ++cycle) {
        HYD_RampController_ExecuteWithDt(&controller, &input, 0.001, &output);
    }
    check(fabs(output.rampedPressure - 10.0) < 1e-6,
          "positive pressure time below 20 ms should be honored");

    printf("pressure ramp time contract: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
