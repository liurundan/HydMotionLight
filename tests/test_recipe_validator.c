#include <assert.h>
#include <stdio.h>
#include "recipe_validator.h"

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
    HYD_MotionSegment recipe[2];
    HYD_DiagnosticCode code = HYD_DIAG_CODE_INTERNAL_ERROR;

    printf("Testing recipe validator success path...\n");
    recipe[0] = make_valid_segment();
    recipe[1] = make_valid_segment();
    recipe[1].direction = HYD_DIRECTION_RETRACT;

    assert(HYD_RecipeValidator_ValidateRecipe(recipe, 2, &code));
    assert(code == HYD_DIAG_CODE_NONE);
    printf("✓ Recipe validator success path test passed\n");
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

    assert(!HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);
    printf("✓ Pressure derivative filter alpha validation test passed\n");
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

int main(void) {
    printf("Running RecipeValidator tests...\n\n");

    test_validate_recipe_success();
    test_validate_recipe_rejects_speed_ramp_non_time_planner();
    test_validate_runtime_config();
    test_validate_pressure_derivative_filter_alpha();
    test_validate_start_context_direction_conflict();

    printf("\n✅ All RecipeValidator tests passed successfully!\n");
    return 0;
}
