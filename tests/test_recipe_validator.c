#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "recipe_validator.h"
#include "segment_limits.h"
#include "test_recipe_rejection_helpers.h"

static HYD_MotionSegment make_valid_segment(void) {
    HYD_MotionSegment segment = {0};
    segment.segmentTag = 1;
    segment.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.targetFlow = 8.0;
    segment.targetPressure = 12.0;
    segment.maxAcceleration = 10.0;
    segment.maxVelocity = 6.0;
    segment.maxFlow = 15.0;
    segment.duration = 0.5;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = 1.0;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 2.0;
    return segment;
}

static void test_validate_recipe_success(void) {
    HYD_MotionSegment recipe[1];
    HYD_DiagnosticCode code = HYD_DIAG_CODE_INTERNAL_ERROR;

    printf("Testing recipe validator success path...\n");
    recipe[0] = make_valid_segment();

    assert(HYD_RecipeValidator_ValidateRecipe(recipe, 1U, &code));
    assert(code == HYD_DIAG_CODE_NONE);
    printf("✓ Recipe validator success path test passed\n");
}

static void test_validate_recipe_rejects_oversized_recipe(void) {
    HYD_MotionSegment recipe[2];

    printf("Testing oversized recipe rejection...\n");
    recipe[0] = make_valid_segment();
    recipe[1] = make_valid_segment();
    recipe[1].direction = HYD_DIRECTION_RETRACT;

    assert_oversized_recipe_validation_rejected(recipe, 2U);
    printf("✓ Oversized recipe rejection test passed\n");
}

static void test_validate_recipe_rejects_speed_ramp_non_time_planner(void) {
    HYD_MotionSegment recipe[1];
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("Testing speed-ramp/time-planner contract validation...\n");
    recipe[0] = make_valid_segment();
    recipe[0].planner = HYD_PLANNER_POSITION_BASED;

    assert(!HYD_RecipeValidator_ValidateRecipe(recipe, 1, &code));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);
    printf("✓ Speed-ramp/time-planner contract test passed\n");
}

static void test_validate_runtime_config(void) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("Testing runtime config validation...\n");
    assert(!HYD_RecipeValidator_ValidateRuntimeConfig(0.0, 3000.0, &code));
    assert(code == HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID);

    assert(HYD_RecipeValidator_ValidateRuntimeConfig(100.0, 3000.0, &code));
    assert(code == HYD_DIAG_CODE_NONE);
    printf("✓ Runtime config validation test passed\n");
}

static void test_validate_pressure_derivative_filter_alpha(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("Testing pressure derivative filter alpha validation...\n");
    segment = make_valid_segment();
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.velocityToFlowGain = 0.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PID;
    segment.pressureKp = 0.5;
    segment.pressureKi = 0.5;
    segment.pressureKd = 0.1;
    segment.pressureDerivativeFilterAlpha = 1.5;

    assert(!HYD_RecipeValidator_ValidateSegment(&segment, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);
    printf("✓ Pressure derivative filter alpha validation test passed\n");
}

static void test_validate_rbf_pi_pressure_controller(void) {
    HYD_MotionSegment segment = make_valid_segment();
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 40.0;
    segment.maxFlow = 20.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PI;

    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_NONE);

    segment.pressureRbfConfig.strategy.outputSlewRate = 12.0;
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_NONE);

    segment.pressureRbfConfig.strategy.outputSlewRate = -1.0;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&segment, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    segment.pressureRbfConfig.strategy.outputSlewRate = 0.0;
    segment.pressureRbfConfig.minKp = 2.0;
    segment.pressureRbfConfig.maxKp = 1.0;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&segment, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);
    printf("✓ RBF-PI pressure controller validation test passed\n");
}

static void test_validate_start_context_direction_conflict(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("Testing start context directional conflict validation...\n");
    segment = make_valid_segment();
    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 5.0;
    segment.direction = HYD_DIRECTION_EXTEND;
    axisRef.position = 10.0;

    assert(!HYD_RecipeValidator_ValidateStartContext(&segment, 0, &axisRef, &code));
    assert(code == HYD_DIAG_CODE_START_CONTEXT_INVALID);
    printf("✓ Start context directional conflict test passed\n");
}

static void test_invalid_ceiling_tolerance_rejected(void) {
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    seg.mode = HYD_MODE_POSITION;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;

    /* Valid ceiling: passes */
    seg.pressureCeiling = 5.0;
    seg.pressureCeilingTolerance = 0.2;
    seg.pressureCeilingPositionStart = 70.0;
    seg.pressureCeilingPositionEnd = 100.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));

    /* Negative tolerance: rejected */
    seg.pressureCeilingTolerance = -0.1;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* Zero ceiling disables check — other fields irrelevant */
    seg.pressureCeiling = 0.0;
    seg.pressureCeilingTolerance = -1.0;  /* invalid but ignored */
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));

    printf("test_invalid_ceiling_tolerance_rejected PASSED\n");
}

static void test_invalid_ceiling_value_rejected(void) {
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    seg.mode = HYD_MODE_POSITION;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;

    /* NaN ceiling: rejected (silent disablement is exactly what we forbid) */
    seg.pressureCeiling = (HYD_REAL)NAN;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* +Inf ceiling: rejected */
    seg.pressureCeiling = (HYD_REAL)INFINITY;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* -Inf ceiling: rejected */
    seg.pressureCeiling = (HYD_REAL)(-INFINITY);
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* Negative ceiling: rejected */
    seg.pressureCeiling = -5.0;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* Zero ceiling: passes (disabled) */
    seg.pressureCeiling = 0.0;
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_NONE);

    printf("test_invalid_ceiling_value_rejected PASSED\n");
}

static void test_pressure_ceiling_active_at(void) {
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    /* Ceiling disabled (ceiling <= 0): never active, regardless of position */
    seg.pressureCeiling = 0.0;
    seg.pressureCeilingPositionStart = 10.0;
    seg.pressureCeilingPositionEnd = 90.0;
    assert(!HYD_Segment_PressureCeilingActiveAt(&seg, 50.0));

    seg.pressureCeiling = -1.0;
    assert(!HYD_Segment_PressureCeilingActiveAt(&seg, 50.0));

    /* Ceiling enabled, degenerate window (end <= start): always active */
    seg.pressureCeiling = 5.0;
    seg.pressureCeilingPositionStart = 0.0;
    seg.pressureCeilingPositionEnd = 0.0;
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, -1000.0));
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 0.0));
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 1000.0));

    seg.pressureCeilingPositionStart = 50.0;
    seg.pressureCeilingPositionEnd = 50.0;  /* end == start: degenerate */
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 0.0));
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 100.0));

    seg.pressureCeilingPositionStart = 60.0;
    seg.pressureCeilingPositionEnd = 50.0;  /* end < start: degenerate (inverted) */
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 0.0));

    /* Ceiling enabled, proper window (end > start): active only inside [start, end] */
    seg.pressureCeilingPositionStart = 70.0;
    seg.pressureCeilingPositionEnd = 100.0;
    assert(!HYD_Segment_PressureCeilingActiveAt(&seg, 69.9));
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 70.0));   /* inclusive lower */
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 85.0));
    assert(HYD_Segment_PressureCeilingActiveAt(&seg, 100.0));  /* inclusive upper */
    assert(!HYD_Segment_PressureCeilingActiveAt(&seg, 100.1));

    /* NULL segment: returns false safely */
    assert(!HYD_Segment_PressureCeilingActiveAt(NULL, 50.0));

    printf("test_pressure_ceiling_active_at PASSED\n");
}

static void test_invalid_derate_ratio_rejected(void) {
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    seg.mode = HYD_MODE_POSITION;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;

    /* 0.0 = use default -> passes */
    seg.derateRatio = 0.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));

    /* Valid range (0,1) -> passes */
    seg.derateRatio = 0.3;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));

    /* Out of range -> rejected */
    seg.derateRatio = 1.5;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    seg.derateRatio = -0.1;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    printf("test_invalid_derate_ratio_rejected PASSED\n");
}

static void test_target_position_exceeds_stroke_rejected(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 100.0;
    cyl.softLimitRetractMm = 0.0;

    /* targetPosition > strokeMm → 拒绝 */
    seg.targetPosition = 105.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, &cyl) == false);
    assert(code == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED);

    /* targetPosition == strokeMm → 通过（边界值） */
    seg.targetPosition = 100.0;
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, &cyl) == true);

    printf("test_target_position_exceeds_stroke_rejected PASSED\n");
}

static void test_target_position_below_retract_limit_rejected(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 100.0;
    cyl.softLimitRetractMm = 5.0;

    /* targetPosition < softLimitRetractMm → 拒绝 */
    seg.targetPosition = 3.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, &cyl) == false);
    assert(code == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED);

    /* targetPosition == softLimitRetractMm → 通过 */
    seg.targetPosition = 5.0;
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, &cyl) == true);

    printf("test_target_position_below_retract_limit_rejected PASSED\n");
}

static void test_stroke_zero_skips_position_validation(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 0.0; /* 不启用 */

    seg.targetPosition = 9999.0; /* 任意大值 */
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, &cyl) == true);

    printf("test_stroke_zero_skips_position_validation PASSED\n");
}

int main(void) {
    printf("Running RecipeValidator tests...\n\n");

    test_validate_recipe_success();
    test_validate_recipe_rejects_oversized_recipe();
    test_validate_recipe_rejects_speed_ramp_non_time_planner();
    test_validate_runtime_config();
    test_validate_pressure_derivative_filter_alpha();
    test_validate_rbf_pi_pressure_controller();
    test_validate_start_context_direction_conflict();
    test_invalid_ceiling_tolerance_rejected();
    test_invalid_ceiling_value_rejected();
    test_pressure_ceiling_active_at();
    test_invalid_derate_ratio_rejected();
    test_target_position_exceeds_stroke_rejected();
    test_target_position_below_retract_limit_rejected();
    test_stroke_zero_skips_position_validation();

    printf("\n✅ All RecipeValidator tests passed successfully!\n");
    return 0;
}
