#include "toggle_hydraulic_sim.h"

#include <math.h>
#include <string.h>

static void set_error(HYD_ToggleError *error, HYD_ToggleError value)
{
    if (error != NULL) {
        *error = value;
    }
}

HYD_ToggleHydraulicSimConfig HYD_ToggleHydraulicSim_DefaultConfig(void)
{
    HYD_ToggleHydraulicSimConfig config;

    config.pumpDisplacementMlRev = 20.0f;
    config.pumpVolumetricEfficiency = 0.95f;
    config.areaExtendMm2 = 1000.0f;
    config.areaRetractMm2 = 1000.0f;
    return config;
}

HYD_BOOL HYD_ToggleHydraulicSim_Step(
    const HYD_ToggleHydraulicSimConfig *config,
    const HYD_TogglePreparedConfig *toggle,
    HYD_REAL pumpSpeedRpm,
    HYD_REAL deltaTime,
    HYD_ToggleHydraulicSimState *state,
    HYD_ToggleHydraulicSimOutput *output,
    HYD_ToggleError *error)
{
    HYD_ToggleHydraulicSimOutput next;
    HYD_REAL area;
    HYD_REAL gain;
    HYD_REAL next_actuator_position;
    HYD_REAL next_template_position;

    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
    if (config == NULL || toggle == NULL || state == NULL || output == NULL) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return false;
    }
    if (!isfinite(pumpSpeedRpm) || !isfinite(deltaTime) || deltaTime <= 0.0f ||
        !isfinite(config->pumpDisplacementMlRev) ||
        !isfinite(config->pumpVolumetricEfficiency) ||
        config->pumpDisplacementMlRev <= 0.0f ||
        config->pumpVolumetricEfficiency <= 0.0f) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return false;
    }

    memset(&next, 0, sizeof(next));
    next.flowLpm = pumpSpeedRpm * config->pumpDisplacementMlRev *
                   config->pumpVolumetricEfficiency / 1000.0f;
    next.actuatorDirection = (next.flowLpm > 0.0f) ? HYD_DIRECTION_EXTEND :
                             (next.flowLpm < 0.0f) ? HYD_DIRECTION_RETRACT :
                                                       HYD_DIRECTION_HOLD;
    if (next.actuatorDirection == HYD_DIRECTION_HOLD) {
        *output = next;
        set_error(error, HYD_TOGGLE_ERROR_NONE);
        return true;
    }
    area = (next.actuatorDirection == HYD_DIRECTION_RETRACT)
        ? config->areaRetractMm2 : config->areaExtendMm2;
    gain = area * 6.0e-5f;
    if (!isfinite(area) || area <= 0.0f || gain <= 0.0f) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return false;
    }

    next.actuatorVelocity = next.flowLpm / gain;
    next_actuator_position = state->actuatorPosition +
                             next.actuatorVelocity * deltaTime;
    if (!HYD_ToggleKinematics_InversePosition(toggle, next_actuator_position,
                                               &next_template_position, error)) {
        return false;
    }
    next.templateVelocity = (next_template_position - state->templatePosition) /
                            deltaTime;
    state->actuatorPosition = next_actuator_position;
    state->templatePosition = next_template_position;
    *output = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return true;
}
