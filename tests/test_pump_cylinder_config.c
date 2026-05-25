#include "common_types.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_pump_config_gain_derivation(void) {
    HYD_PumpConfig cfg = {28.0f, 0.95f, 2000.0f};
    HYD_REAL gain = HYD_PumpConfig_GetFlowToSpeedGain(&cfg);
    HYD_REAL limit = HYD_PumpConfig_GetSpeedLimit(&cfg);
    /* 1000 / (28 * 0.95) = 37.594 rpm/(L/min) */
    assert(fabsf(gain - 37.594f) < 0.1f);
    assert(fabsf(limit - 2000.0f) < 0.01f);
    printf("  PASS: pump config gain derivation\n");
}

static void test_pump_config_zero_returns_zero(void) {
    HYD_PumpConfig cfg = {0.0f, 0.0f, 0.0f};
    HYD_REAL gain = HYD_PumpConfig_GetFlowToSpeedGain(&cfg);
    assert(gain == 0.0f);
    printf("  PASS: pump config zero returns zero\n");
}

static void test_cylinder_config_extend(void) {
    HYD_CylinderConfig cfg = {6362.0f, 3534.0f, 300.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_EXTEND);
    /* 6362 * 6e-5 = 0.38172 L/min per mm/s */
    assert(fabsf(gain - 0.38172f) < 0.001f);
    printf("  PASS: cylinder config extend gain\n");
}

static void test_cylinder_config_retract(void) {
    HYD_CylinderConfig cfg = {6362.0f, 3534.0f, 300.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_RETRACT);
    /* 3534 * 6e-5 = 0.21204 L/min per mm/s */
    assert(fabsf(gain - 0.21204f) < 0.001f);
    printf("  PASS: cylinder config retract gain\n");
}

static void test_cylinder_config_zero_returns_zero(void) {
    HYD_CylinderConfig cfg = {0.0f, 0.0f, 0.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_EXTEND);
    assert(gain == 0.0f);
    printf("  PASS: cylinder config zero returns zero\n");
}

int main(void) {
    printf("test_pump_cylinder_config:\n");
    test_pump_config_gain_derivation();
    test_pump_config_zero_returns_zero();
    test_cylinder_config_extend();
    test_cylinder_config_retract();
    test_cylinder_config_zero_returns_zero();
    printf("All pump/cylinder config tests passed.\n");
    return 0;
}
