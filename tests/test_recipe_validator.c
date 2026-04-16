#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "recipe_validator.h"

static HDY_MotionSegment make_valid_segment(void) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, "Valid", HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_INJECTION;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.mode = HDY_MODE_SPEED_RAMP;
    segment.endCondition = HDY_END_TIME;
    segment.direction = HDY_DIRECTION_EXTEND;
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
    HDY_MotionSegment recipe[2];
    HDY_DiagnosticCode code = HDY_DIAG_CODE_INTERNAL_ERROR;
    char message[HDY_MESSAGE_MAX] = {0};

    printf("Testing recipe validator success path...\n");
    recipe[0] = make_valid_segment();
    recipe[1] = make_valid_segment();
    recipe[1].direction = HDY_DIRECTION_RETRACT;

    assert(HDY_RecipeValidator_ValidateRecipe(recipe, 2, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_NONE);
    assert(message[0] == '\0');
    printf("✓ Recipe validator success path test passed\n");
}

static void test_validate_recipe_rejects_speed_ramp_non_time_planner(void) {
    HDY_MotionSegment recipe[1];
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    printf("Testing speed-ramp/time-planner contract validation...\n");
    recipe[0] = make_valid_segment();
    recipe[0].planner = HDY_PLANNER_POSITION_BASED;

    assert(!HDY_RecipeValidator_ValidateRecipe(recipe, 1, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(message, "TIME_BASED") != NULL);
    printf("✓ Speed-ramp/time-planner contract test passed\n");
}

static void test_validate_runtime_config(void) {
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    printf("Testing runtime config validation...\n");
    assert(!HDY_RecipeValidator_ValidateRuntimeConfig(0.0, 3000.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL);

    assert(HDY_RecipeValidator_ValidateRuntimeConfig(100.0, 3000.0, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_NONE);
    assert(message[0] == '\0');
    printf("✓ Runtime config validation test passed\n");
}

static void test_validate_start_context_direction_conflict(void) {
    HDY_MotionSegment segment;
    HDY_AxisRef axisRef = {0};
    HDY_DiagnosticCode code = HDY_DIAG_CODE_NONE;
    char message[HDY_MESSAGE_MAX] = {0};

    printf("Testing start context directional conflict validation...\n");
    segment = make_valid_segment();
    segment.mode = HDY_MODE_POSITION;
    segment.planner = HDY_PLANNER_POSITION_BASED;
    segment.endCondition = HDY_END_POSITION;
    segment.targetPosition = 5.0;
    segment.direction = HDY_DIRECTION_EXTEND;
    axisRef.position = 10.0;

    assert(!HDY_RecipeValidator_ValidateStartContext(&segment, 0, &axisRef, &code, message, sizeof(message)));
    assert(code == HDY_DIAG_CODE_START_CONTEXT_INVALID);
    assert(strstr(message, "conflicts") != NULL);
    printf("✓ Start context directional conflict test passed\n");
}

int main(void) {
    printf("Running RecipeValidator tests...\n\n");

    test_validate_recipe_success();
    test_validate_recipe_rejects_speed_ramp_non_time_planner();
    test_validate_runtime_config();
    test_validate_start_context_direction_conflict();

    printf("\n✅ All RecipeValidator tests passed successfully!\n");
    return 0;
}
