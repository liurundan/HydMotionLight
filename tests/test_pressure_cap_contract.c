#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"

static void init_loop(HYD_MotionSegment* segment,
                      HYD_PressureControllerState* state,
                      HYD_PressureControllerInput* input,
                      HYD_PressureControllerType strategy) {
    memset(segment, 0, sizeof(*segment));
    memset(input, 0, sizeof(*input));
    segment->mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment->endCondition = HYD_END_MANUAL;
    segment->direction = HYD_DIRECTION_HOLD;
    segment->targetPressure = 150.0;
    segment->pressureController = strategy;
    segment->pressureFilterAlpha = 1.0;
    segment->pressureKp = 0.2;
    segment->pressureKi = 0.1;
    segment->pressureCeiling = 250.0;

    input->targetPressure = 150.0;
    input->outputMin = 0.0;
    input->outputMax = 20.0;
    input->systemGain = 200.0;
    input->boostFlowLimitLmin = 12.0;
    input->boostBrakeFrac = 2.0;
    input->plantTauS = 1.0;
    input->loopOmega = 12.0;
    input->measuredPressure = 0.0;
    input->timestamp = 0.0;

    HYD_PressureController_InitState(state, 0.0, 0.0, 0.0);
}

static HYD_PressureControllerOutput step_at_error(
    const HYD_MotionSegment* segment,
    HYD_PressureControllerState* state,
    HYD_PressureControllerInput* input,
    HYD_REAL error,
    HYD_TIME timestamp) {
    HYD_PressureControllerOutput output;
    input->measuredPressure = input->targetPressure - error;
    input->timestamp = timestamp;
    HYD_PressureController_Execute(segment, state, input, &output);
    return output;
}

static void test_narrow_band_release_is_continuous(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput before;
    HYD_PressureControllerOutput after;

    init_loop(&segment, &state, &input, HYD_PRESSURE_CONTROLLER_FF_PI);
    before = step_at_error(&segment, &state, &input, 10.6, 0.001);
    after = step_at_error(&segment, &state, &input, 10.4, 0.002);

    assert(before.effectiveUpperCap > 0.0);
    assert(after.effectiveUpperCap > 0.0);
    assert(after.effectiveUpperCap <= input.outputMax + 1.0e-6);
    /* The old implementation jumped from roughly 1 L/min to hardMax here. */
    assert(fabs(after.effectiveUpperCap - before.effectiveUpperCap) <
           0.05 * input.targetPressure);
}

static void test_cap_is_shared_by_strategies(void) {
    const HYD_PressureControllerType strategies[] = {
        HYD_PRESSURE_CONTROLLER_PI,
        HYD_PRESSURE_CONTROLLER_FF_PI,
        HYD_PRESSURE_CONTROLLER_RBF_PI
    };
    size_t i;

    for (i = 0; i < sizeof(strategies) / sizeof(strategies[0]); ++i) {
        HYD_MotionSegment segment;
        HYD_PressureControllerState state;
        HYD_PressureControllerInput input;
        HYD_PressureControllerOutput output;

        init_loop(&segment, &state, &input, strategies[i]);
        output = step_at_error(&segment, &state, &input, 50.0, 0.001);
        assert(output.effectiveUpperCap < input.outputMax);
        assert(output.outputFlow <= output.effectiveUpperCap + 1.0e-5);
        assert(state.effectiveUpperCap == output.effectiveUpperCap);

        if (strategies[i] == HYD_PRESSURE_CONTROLLER_RBF_PI) {
            assert(state.rbfPid.effective_upper_cap_valid);
            assert(fabsf(state.rbfPid.effective_upper_cap -
                        (float)output.effectiveUpperCap) < 1.0e-5f);
        }
    }
}

int main(void) {
    test_narrow_band_release_is_continuous();
    test_cap_is_shared_by_strategies();
    puts("pressure cap contract: PASS");
    return 0;
}
