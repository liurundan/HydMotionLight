#include "common_types.h"
#include "motion_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

static void test_pump_config_is_valid(void) {
    HYD_PumpConfig valid = {28.0f, 0.95f, 2000.0f};
    HYD_PumpConfig zero_disp = {0.0f, 0.95f, 2000.0f};
    HYD_PumpConfig zero_eff = {28.0f, 0.0f, 2000.0f};
    assert(HYD_PumpConfig_IsValid(&valid) == 1);
    assert(HYD_PumpConfig_IsValid(&zero_disp) == 0);
    assert(HYD_PumpConfig_IsValid(&zero_eff) == 0);
    assert(HYD_PumpConfig_IsValid(NULL) == 0);
    printf("  PASS: pump config IsValid\n");
}

static void test_cylinder_config_is_valid(void) {
    HYD_CylinderConfig valid = {6362.0f, 3534.0f, 300.0f};
    HYD_CylinderConfig zero_both = {0.0f, 0.0f, 300.0f};
    HYD_CylinderConfig extend_only = {6362.0f, 0.0f, 300.0f};
    assert(HYD_CylinderConfig_IsValid(&valid) == 1);
    assert(HYD_CylinderConfig_IsValid(&zero_both) == 0);
    assert(HYD_CylinderConfig_IsValid(&extend_only) == 1);
    assert(HYD_CylinderConfig_IsValid(NULL) == 0);
    printf("  PASS: cylinder config IsValid\n");
}

static void test_null_safety(void) {
    assert(HYD_PumpConfig_GetFlowToSpeedGain(NULL) == 0.0f);
    assert(HYD_PumpConfig_GetSpeedLimit(NULL) == 0.0f);
    assert(HYD_CylinderConfig_GetVelocityToFlowGain(NULL, HYD_DIRECTION_EXTEND) == 0.0f);
    printf("  PASS: null safety\n");
}

static void test_fb_pump_config_derivation(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    /* Configure pump: 28 mL/rev, eta=0.95, max 2000 rpm */
    fb.pumpConfig.displacementMlRev = 28.0f;
    fb.pumpConfig.volumetricEfficiency = 0.95f;
    fb.pumpConfig.maxSpeedRpm = 2000.0f;

    /* Load a simple segment */
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.3f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.002f;
    HYD_MotionControlFB_Cycle(&fb);

    /* pumpConfig active: pump speed must be within derived limit (2000 rpm) */
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.PUMP_SPEED <= 2000.0f);
    /* Poison legacy field — pumpConfig should still be used */
    fb.FLOW_TO_PUMP_SPEED_GAIN = 9999.0f;
    fb.AXIS_REF.timestamp = 0.003f;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fb.PUMP_SPEED <= 2000.0f);
    printf("  PASS: FB pump config derivation\n");
}

static void test_fb_pump_config_fallback_to_legacy(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    fb.FLOW_TO_PUMP_SPEED_GAIN = 80.0f;
    fb.PUMP_SPEED_LIMIT = 5000.0f;

    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.3f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.002f;
    HYD_MotionControlFB_Cycle(&fb);

    /* After two cycles the FB should be in RUNNING state (not FAULT) */
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.PUMP_SPEED <= 5000.0f);
    printf("  PASS: FB pump config fallback to legacy\n");
}

static void test_fb_cylinder_config_derivation(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    fb.cylinderConfig.areaExtendMm2 = 6362.0f;
    fb.cylinderConfig.areaRetractMm2 = 3534.0f;
    fb.cylinderConfig.strokeMm = 300.0f;

    fb.FLOW_TO_PUMP_SPEED_GAIN = 80.0f;
    fb.PUMP_SPEED_LIMIT = 5000.0f;

    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.38172f;  /* matches areaExtend 6362 * 6e-5 = 0.38172 */
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.002f;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.PUMP_SPEED > 0.0f);
    printf("  PASS: FB cylinder config derivation\n");
}

static void test_fb_cylinder_config_overrides_segment_gain(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    fb.cylinderConfig.areaExtendMm2 = 10000.0f;
    fb.cylinderConfig.areaRetractMm2 = 6000.0f;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 100.0f;
    seg.velocityToFlowGain = 0.2f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f));

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb._activeSegmentValid);
    assert(fabsf(fb._activeSegment.velocityToFlowGain - 0.6f) < 0.001f);
    printf("  PASS: FB cylinder config overrides segment gain\n");
}

static void test_fb_cylinder_config_keeps_segment_gain_when_direction_area_missing(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    fb.cylinderConfig.areaExtendMm2 = 10000.0f;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_RETRACT;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 100.0f;
    seg.velocityToFlowGain = 0.25f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f));

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb._activeSegmentValid);
    assert(fabsf(fb._activeSegment.velocityToFlowGain - 0.25f) < 0.001f);
    printf("  PASS: FB cylinder config keeps segment gain when direction area missing\n");
}

static void test_fb_cylinder_config_uses_resolved_current_direction(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    fb.cylinderConfig.areaRetractMm2 = 6000.0f;
    fb._lastActiveDirection = HYD_DIRECTION_NEGATIVE;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_CURRENT;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 100.0f;
    seg.velocityToFlowGain = 0.25f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f));

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb._activeSegmentValid);
    assert(fabsf(fb._activeSegment.velocityToFlowGain - 0.36f) < 0.001f);
    printf("  PASS: FB cylinder config uses resolved CURRENT direction\n");
}

static void test_fb_cylinder_config_uses_resolved_shortest_way_direction(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    HYD_MotionControlFB_Init(&fb);
    fb.cylinderConfig.areaRetractMm2 = 6000.0f;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.planner = HYD_PLANNER_POSITION_BASED;
    seg.mode = HYD_MODE_POSITION;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_SHORTEST_WAY;
    seg.targetPosition = -10.0f;
    seg.positionTolerance = 0.1f;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 100.0f;
    seg.velocityToFlowGain = 0.25f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &seg));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f));

    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb._activeSegmentValid);
    assert(fabsf(fb._activeSegment.velocityToFlowGain - 0.36f) < 0.001f);
    printf("  PASS: FB cylinder config uses resolved SHORTEST_WAY direction\n");
}

static void test_parameter_access(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_REAL val;

    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_DISPLACEMENT, 45.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_VOLUMETRIC_EFF, 0.93f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_MAX_SPEED, 1800.0f));

    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_DISPLACEMENT, &val));
    assert(fabsf(val - 45.0f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_VOLUMETRIC_EFF, &val));
    assert(fabsf(val - 0.93f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_MAX_SPEED, &val));
    assert(fabsf(val - 1800.0f) < 0.01f);

    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_AREA_EXTEND, 6362.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_AREA_RETRACT, 3534.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_STROKE, 300.0f));

    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_CYLINDER_AREA_EXTEND, &val));
    assert(fabsf(val - 6362.0f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_CYLINDER_AREA_RETRACT, &val));
    assert(fabsf(val - 3534.0f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_CYLINDER_STROKE, &val));
    assert(fabsf(val - 300.0f) < 0.01f);

    printf("  PASS: parameter access for pump/cylinder config\n");
}

int main(void) {
    printf("test_pump_cylinder_config:\n");
    test_pump_config_gain_derivation();
    test_pump_config_zero_returns_zero();
    test_cylinder_config_extend();
    test_cylinder_config_retract();
    test_cylinder_config_zero_returns_zero();
    test_pump_config_is_valid();
    test_cylinder_config_is_valid();
    test_null_safety();
    test_fb_pump_config_derivation();
    test_fb_pump_config_fallback_to_legacy();
    test_fb_cylinder_config_derivation();
    test_fb_cylinder_config_overrides_segment_gain();
    test_fb_cylinder_config_keeps_segment_gain_when_direction_area_missing();
    test_fb_cylinder_config_uses_resolved_current_direction();
    test_fb_cylinder_config_uses_resolved_shortest_way_direction();
    test_parameter_access();
    printf("All pump/cylinder config tests passed.\n");
    return 0;
}
