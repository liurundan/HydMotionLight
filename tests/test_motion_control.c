#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.h"

static void init_controller(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_Init(fb);
    fb->EN = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb->PUMP_SPEED_LIMIT = 3000.0;
}

static HDY_MotionSegment make_position_segment(const char* name,
                                               HDY_REAL targetPosition,
                                               HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_CLAMPING;
    segment.planner = HDY_PLANNER_POSITION_BASED;
    segment.mode = HDY_MODE_POSITION;
    segment.endCondition = HDY_END_POSITION;
    segment.direction = direction;
    segment.targetPosition = targetPosition;
    segment.targetFlow = 12.0;
    segment.targetPressure = 8.0;
    segment.maxAcceleration = 20.0;
    segment.maxVelocity = 15.0;
    segment.maxFlow = 18.0;
    segment.duration = 1.0;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = 2.0;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 5.0;
    return segment;
}

static HDY_MotionSegment make_time_segment(const char* name,
                                           HDY_TIME duration,
                                           HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_HOLDING;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.mode = HDY_MODE_SPEED_RAMP;
    segment.endCondition = HDY_END_TIME;
    segment.direction = direction;
    segment.targetPosition = 10.0;
    segment.targetFlow = 8.0;
    segment.targetPressure = 10.0;
    segment.maxAcceleration = 10.0;
    segment.maxVelocity = 6.0;
    segment.maxFlow = 10.0;
    segment.duration = duration;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = duration + 0.5;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 2.0;
    return segment;
}

static void test_load_recipe_requires_start_command(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing LoadRecipe idle semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("Clamp", 10.0, HDY_DIRECTION_EXTEND);

    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(fb.RECIPE_SIZE == 1U);
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(!fb.STATE.active);
    assert(!fb.FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.CURRENT_SEGMENT_NAME[0] == '\0');
    assert(fb.STATE.currentSegmentName[0] == '\0');
    assert(fb.STATE.status == HDY_STATUS_READY);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    printf("✓ LoadRecipe idle semantics test passed\n");
}

static void test_start_segment_and_segment_changed_pulse(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing StartSegment execution semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("Inject", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 2.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    assert(fb.STATUS == HDY_STATUS_RUNNING);

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.STATE.status == HDY_STATUS_RUNNING);
    assert(fb.SEGMENT_CHANGED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.STATE.plannedVelocity > 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(strcmp(fb.CURRENT_SEGMENT_NAME, "Inject") == 0);
    assert(strcmp(fb.STATE.currentSegmentName, "Inject") == 0);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    printf("✓ StartSegment execution semantics test passed\n");
}

static void test_start_segment_command_input(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing START_SEGMENT command input semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("CommandStart", 12.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.5;
    fb.AXIS_REF.timestamp = 0.0;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;

    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.START_SEGMENT);
    assert(fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(strcmp(fb.CURRENT_SEGMENT_NAME, "CommandStart") == 0);
    printf("✓ START_SEGMENT command input semantics test passed\n");
}

static void test_segment_completion_and_next_segment(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];

    printf("Testing segment completion and NextSegment behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("SegmentA", 1.0, HDY_DIRECTION_EXTEND);
    recipe[1] = make_time_segment("SegmentB", 0.5, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 2));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(!HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.currentSegmentIndex == 0U);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED);
    assert(strstr(fb.DIAGNOSTIC.message, "not completed") != NULL);

    fb.AXIS_REF.position = 1.0;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_SEGMENT_COMPLETE);
    assert(fb.PUMP_SPEED == 0.0);
    assert(strcmp(fb.CURRENT_SEGMENT_NAME, "SegmentA") == 0);

    assert(HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.STATE.currentSegmentIndex == 1U);
    assert(strcmp(fb.CURRENT_SEGMENT_NAME, "SegmentB") == 0);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);
    assert(strstr(fb.DIAGNOSTIC.message, "already finished") != NULL);
    printf("✓ Segment completion and NextSegment behavior test passed\n");
}

static void test_retract_position_directional_planning(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing retract position directional planning...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("RetractPosition", 2.0, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 10.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT);
    assert(fb.STATE.plannedVelocity < 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.PUMP_SPEED > 0.0);

    fb.AXIS_REF.position = 2.05;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    printf("✓ Retract position directional planning test passed\n");
}

static void test_speed_ramp_retract_directional_planning(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing speed-ramp retract directional planning...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("RetractRamp", 0.5, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 10.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT);
    assert(fb.STATE.plannedVelocity < 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.PUMP_SPEED > 0.0);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    printf("✓ Speed-ramp retract directional planning test passed\n");
}

static void test_disable_requires_restart(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing EN disable safety semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DisableCase", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED > 0.0);

    fb.EN = false;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ENO);
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.START_SEGMENT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);

    fb.EN = true;
    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ENO);
    assert(!fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);

    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.PUMP_SPEED > 0.0);
    printf("✓ EN disable safety semantics test passed\n");
}

static void test_abort_forces_safe_outputs(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort safety semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AbortCase", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED > 0.0);

    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    assert(HDY_MotionControlFB_Abort(&fb));
    assert(!fb.ACTIVE);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.START_SEGMENT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.CURRENT_SEGMENT_NAME[0] == '\0');
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_ABORTED);
    assert(strstr(fb.DIAGNOSTIC.message, "Aborted by caller") != NULL);

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.ACTIVE);
    printf("✓ Abort safety semantics test passed\n");
}

static void test_reset_performs_full_reinitialization(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing RESET full reinitialization semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("ResetCase", 6.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.RESET = true;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.EN);
    assert(fb.ENO);
    assert(!fb.RESET);
    assert(fb.RECIPE_SIZE == 0U);
    assert(fb.FLOW_TO_PUMP_SPEED_GAIN == 0.0);
    assert(fb.PUMP_SPEED_LIMIT == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.CURRENT_SEGMENT_NAME[0] == '\0');
    printf("✓ RESET full reinitialization semantics test passed\n");
}

static void test_typed_diagnostic_thresholds_override_legacy_tolerance(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing typed diagnostic thresholds override legacy tolerance...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DiagTyped", 20.0, HDY_DIRECTION_EXTEND);
    recipe[0].tolerance = 0.05;
    recipe[0].pressureTolerance = 0.5;
    recipe[0].flowTolerance = 0.2;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 11.85;
    fb.AXIS_REF.pressure = 8.4;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.DIAGNOSTIC.overPressure);
    assert(!fb.DIAGNOSTIC.underPressure);
    assert(!fb.DIAGNOSTIC.flowDeviation);
    assert(fb.DIAGNOSTIC.positionDeviation);
    assert(fb.DIAGNOSTIC.velocityDeviation);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_POSITION_DEVIATION);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_WARNING);
    printf("✓ Typed diagnostic thresholds test passed\n");
}

static void test_timeout_limit_stops_segment_safely(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing timeoutLimit safe-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("TimeoutCase", 100.0, HDY_DIRECTION_EXTEND);
    recipe[0].timeoutLimit = 0.25;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.DIAGNOSTIC.timeout);

    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.STATE.faultActive);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.DIAGNOSTIC.timeout);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_TIMEOUT);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(strstr(fb.DIAGNOSTIC.message, "timeout") != NULL);
    printf("✓ timeoutLimit safe-stop test passed\n");
}

static void test_runtime_validation_fault_latches_safe_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing runtime validation fault behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("RuntimeFault", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.FLOW_TO_PUMP_SPEED_GAIN = 0.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(strstr(fb.DIAGNOSTIC.message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL);
    printf("✓ Runtime validation fault test passed\n");
}

static void test_parameter_validation(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment badRecipe[1];
    HDY_MotionSegment goodRecipe[1];

    printf("Testing recipe and runtime parameter validation...\n");
    init_controller(&fb);

    badRecipe[0] = make_time_segment("BadDirection", 0.5, HDY_DIRECTION_AUTO);
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.RECIPE_SIZE == 0U);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "direction") != NULL);

    badRecipe[0] = make_time_segment("BadPlanner", 0.5, HDY_DIRECTION_EXTEND);
    badRecipe[0].planner = HDY_PLANNER_POSITION_BASED;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "TIME_BASED") != NULL);

    goodRecipe[0] = make_position_segment("Good", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, goodRecipe, 1));
    fb.FLOW_TO_PUMP_SPEED_GAIN = 0.0;
    fb.AXIS_REF.position = 0.0;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL);

    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.AXIS_REF.position = 10.0;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_START_CONTEXT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "conflicts") != NULL);
    printf("✓ Recipe and runtime parameter validation test passed\n");
}

int main(void) {
    printf("Running MotionControl tests...\n\n");

    test_load_recipe_requires_start_command();
    test_start_segment_and_segment_changed_pulse();
    test_start_segment_command_input();
    test_segment_completion_and_next_segment();
    test_retract_position_directional_planning();
    test_speed_ramp_retract_directional_planning();
    test_disable_requires_restart();
    test_abort_forces_safe_outputs();
    test_reset_performs_full_reinitialization();
    test_typed_diagnostic_thresholds_override_legacy_tolerance();
    test_timeout_limit_stops_segment_safely();
    test_runtime_validation_fault_latches_safe_state();
    test_parameter_validation();

    printf("\n✅ All MotionControl tests passed successfully!\n");
    return 0;
}
