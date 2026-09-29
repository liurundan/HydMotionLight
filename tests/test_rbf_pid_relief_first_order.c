#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"

static float first_order_step(float p, float flow, float gain, float tau) {
    float target = gain * flow;
    return p + 0.001f * (target - p) / tau;
}

int main(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    float pressure = 100.0f;
    int relief_samples = 0;
    int exited = 0;
    float min_pressure = pressure;
    int step;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.maxFlow = 90.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;

    HYD_PressureController_InitState(&state, pressure, 0.0, 0.0);
    for (step = 0; step < 3000; ++step) {
        memset(&input, 0, sizeof(input));
        input.targetPressure = 20.0;
        input.measuredPressure = pressure;
        input.outputMin = -5.0;
        input.outputMax = 90.0;
        input.flowToPumpSpeedGain = 20.0;
        input.pumpSpeedLimit = 1800.0;
        input.systemGainBarPerRpm = 1.5;
        input.timestamp = (HYD_TIME)(step + 1) * 0.001;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        if (output.reliefActive) relief_samples++;
        if (!output.reliefActive && relief_samples > 0) exited = 1;
        pressure = first_order_step(pressure, (float)output.outputFlow,
                                     30.0f, 1.0f);
        if (pressure < min_pressure) min_pressure = pressure;
    }

    assert(relief_samples > 0);
    assert(exited);
    assert(output.effectiveOutputMin >= -1.0e-6);
    assert(min_pressure > 0.0f);
    printf("first-order relief samples=%d min_pressure=%.3f final_pressure=%.3f\n",
           relief_samples, min_pressure, pressure);
    return 0;
}

