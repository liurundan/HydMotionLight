#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "toggle_hydraulic_sim.h"

static void test_pump_flow_reaches_template_through_toggle_inverse(void)
{
    HYD_ToggleHydraulicSimConfig config;
    HYD_ToggleHydraulicSimState state;
    HYD_ToggleHydraulicSimOutput output;
    HYD_TogglePreparedConfig toggle;
    HYD_ToggleGeometryConfig raw;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    raw = HYD_ToggleKinematics_DefaultConfig();
    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &toggle, &error));
    config = HYD_ToggleHydraulicSim_DefaultConfig();
    config.pumpDisplacementMlRev = 20.0f;
    config.pumpVolumetricEfficiency = 1.0f;
    config.areaExtendMm2 = 2000.0f;
    config.areaRetractMm2 = 1000.0f;
    state.templatePosition = 101.0f;
    state.actuatorPosition = 63.808094f;

    assert(HYD_ToggleHydraulicSim_Step(&config, &toggle, 600.0f,
                                        0.01f, &state, &output, &error));
    assert(fabsf(output.flowLpm - 12.0f) < 1.0e-4f);
    assert(output.actuatorVelocity > 0.0f);
    assert(output.templateVelocity > 0.0f);
    assert(fabsf(state.templatePosition - 101.0f -
                 output.templateVelocity * 0.01f) < 2.0e-3f);
}

int main(void)
{
    test_pump_flow_reaches_template_through_toggle_inverse();
    printf("toggle hydraulic simulator tests passed\n");
    return 0;
}
