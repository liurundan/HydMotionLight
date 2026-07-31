#ifndef HYD_TOGGLE_HYDRAULIC_SIM_H
#define HYD_TOGGLE_HYDRAULIC_SIM_H

#include "toggle_kinematics.h"

/* Host/test-only plant model. This header is implemented in src/sim and is
 * linked only through HydroSimLib; it is not part of HydroMotionLib. */
typedef struct {
    HYD_REAL pumpDisplacementMlRev;
    HYD_REAL pumpVolumetricEfficiency;
    HYD_REAL areaExtendMm2;
    HYD_REAL areaRetractMm2;
} HYD_ToggleHydraulicSimConfig;

typedef struct {
    HYD_REAL templatePosition;
    HYD_REAL actuatorPosition;
} HYD_ToggleHydraulicSimState;

typedef struct {
    HYD_REAL flowLpm;
    HYD_REAL actuatorVelocity;
    HYD_REAL templateVelocity;
    HYD_MotionDirection actuatorDirection;
} HYD_ToggleHydraulicSimOutput;

HYD_ToggleHydraulicSimConfig HYD_ToggleHydraulicSim_DefaultConfig(void);
HYD_BOOL HYD_ToggleHydraulicSim_Step(
    const HYD_ToggleHydraulicSimConfig *config,
    const HYD_TogglePreparedConfig *toggle,
    HYD_REAL pumpSpeedRpm,
    HYD_REAL deltaTime,
    HYD_ToggleHydraulicSimState *state,
    HYD_ToggleHydraulicSimOutput *output,
    HYD_ToggleError *error);

#endif
